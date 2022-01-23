#include <routingkit/osm_profile.h>
#include <routingkit/osm_graph_builder.h>
#include <routingkit/osm_parking.h>
#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/tag_map.h>
#include <routingkit/id_mapper.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/contraction_hierarchy.h>

#include <iostream>
#include <string>
#include <exception>
#include <experimental/filesystem>

using namespace RoutingKit;
using std::experimental::filesystem::path;

int main(int argc, char *argv[])
{

	if (argc != 3)
	{
		std::cout << "Usage:" << std::endl;
		std::cout << argv[0] << " pbf_file export_directory --hgv-only" << std::endl;
		std::cout << "geo_distance is in [m]" << std::endl;
		std::cout << "travel_time is in [s]" << std::endl;
		std::cout << "way_speed is in [km/h]" << std::endl;
		return 1;
	}

	const std::string pbf_file = argv[1];
	const path export_dir = path(argv[2]);
	const bool hgv_only = argv[3] == "--hgv-only";

	const std::string first_out_file = export_dir / "first_out";

	const std::string head_file = export_dir / "head";
	const std::string geo_distance_file = export_dir / "geo_distance";
	const std::string travel_time_file = export_dir / "travel_time";
	const std::string way_file = export_dir / "way";
	const std::string way_speed_file = export_dir / "way_speed";
	const std::string way_name_file = export_dir / "way_name";
	const std::string latitude_file = export_dir / "latitude";
	const std::string longitude_file = export_dir / "longitude";
	const std::string osm_node_id_file = export_dir / "osm_node_id";
	const std::string osm_way_file = export_dir / "osm_way";
	const std::string routing_parking_flags_file = export_dir / "routing_parking_flags";
	const std::string osm_parking_way_file = export_dir / "osm_parking_way";
	const std::string parking_info_file = export_dir / "parking_info.csv";

	long long complete_timer = -get_micro_time();

	std::function<void(const std::string &)> log_message = [](const std::string &msg)
	{
		std::cout << msg << std::endl;
	};

	std::function<bool(uint64_t, const TagMap &)> is_osm_way_used_for_routing =
		[&](uint64_t osm_way_id, const TagMap &tags)
	{
		return is_osm_way_used_by_cars(osm_way_id, tags, log_message);
	};

	std::function<bool(uint64_t, const TagMap &)>
		has_parking_node_criteria =
			[&](uint64_t osm_node_id, const TagMap &tags)
	{
		return is_osm_object_used_for_parking(osm_node_id, tags);
	};

	if (hgv_only)
	{
		has_parking_node_criteria = [&](uint64_t osm_node_id, const TagMap &tags)
		{
			return is_osm_object_used_for_hgv_parking(osm_node_id, tags);
		};
	}

	BitVector is_routing_node;
	BitVector routing_parking_flags;
	{
		auto mapping = load_osm_id_mapping_from_pbf(
			pbf_file,
			has_parking_node_criteria,
			is_osm_way_used_for_routing,
			log_message);

		unsigned routing_way_count = mapping.is_routing_way.population_count();
		std::vector<uint32_t> way_speed(routing_way_count);
		std::vector<std::string> way_name(routing_way_count);

		auto routing_graph = load_osm_routing_graph_from_pbf(
			pbf_file,
			mapping,
			[&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap &way_tags)
			{
				way_speed[routing_way_id] = get_osm_way_speed(osm_way_id, way_tags, log_message);
				way_name[routing_way_id] = get_osm_way_name(osm_way_id, way_tags, log_message);
				return get_osm_car_direction_category(osm_way_id, way_tags, log_message);
			},
			nullptr,
			log_message);

		unsigned arc_count = routing_graph.arc_count();

		std::vector<uint32_t> travel_time = routing_graph.geo_distance;
		for (unsigned a = 0; a < arc_count; ++a)
		{
			travel_time[a] *= 18000;
			travel_time[a] /= way_speed[routing_graph.way[a]];
			travel_time[a] /= 5;
		}

		std::vector<uint64_t> osm_node_ids;
		osm_node_ids.reserve(mapping.is_routing_node.count_true());

		for (uint64_t i = 0; i < mapping.is_routing_node.size(); ++i)
		{
			if (mapping.is_routing_node.is_set(i))
				osm_node_ids.push_back(i);
		}

		{
			log_message("Start saving routing graph");
			long long timer = -get_micro_time();

			if (!first_out_file.empty())
				save_vector(first_out_file, routing_graph.first_out);
			if (!head_file.empty())
				save_vector(head_file, routing_graph.head);
			// if (!geo_distance_file.empty())
			// 	save_vector(geo_distance_file, routing_graph.geo_distance);
			if (!travel_time_file.empty())
				save_vector(travel_time_file, travel_time);
			// if (!way_file.empty())
			// 	save_vector(way_file, routing_graph.way);
			// if (!way_name_file.empty())
			// 	save_vector(way_name_file, way_name);
			// if (!way_speed_file.empty())
			// 	save_vector(way_speed_file, way_speed);
			if (!latitude_file.empty())
				save_vector(latitude_file, routing_graph.latitude);
			if (!longitude_file.empty())
				save_vector(longitude_file, routing_graph.longitude);
			if (!osm_node_id_file.empty())
				save_vector(osm_node_id_file, osm_node_ids);
			// if (!osm_way_file.empty())
			// 	save_bit_vector(osm_way_file, mapping.is_routing_way);

			timer += get_micro_time();
			log_message("Finished saving, needed " + std::to_string(timer) + "musec.");
		}

		routing_parking_flags.make_large_enough_for(routing_graph.node_count());
		is_routing_node = mapping.is_routing_node;
	}

	log_message("Extracting parking.");

	const auto parking_mapping = load_osm_parking_id_mapping_from_pbf(
		pbf_file, has_parking_node_criteria, log_message);

	auto parking = load_osm_parking_from_pbf(
		pbf_file,
		parking_mapping,
		log_message);

	log_message("Constructing parking flags");
	LocalIDMapper routing_node_mapper(is_routing_node);
	LocalIDMapper parking_node_mapper(parking_mapping.is_parking_node);

	for (uint64_t i = 0; i < parking_mapping.is_parking_node.size(); ++i)
	{
		if (parking_mapping.is_parking_node.is_set(i))
		{
			routing_parking_flags.set(routing_node_mapper.to_local(i));
		}
	}
	log_message("Finished constructing parking flags");

	try
	{
		log_message("Start saving parking information");
		long long timer = -get_micro_time();

		if (!parking_info_file.empty())
			save_parking_tags_csv(parking_info_file, parking);
		if (!routing_parking_flags_file.empty())
			save_bit_vector(routing_parking_flags_file, routing_parking_flags);
		// if (!osm_parking_way_file.empty())
		// 	save_bit_vector(osm_parking_way_file, parking_mapping.is_parking_way);

		timer += get_micro_time();
		log_message("Finished saving, needed " + std::to_string(timer) + "musec.");
	}
	catch (std::exception &err)
	{
		std::cout << "Exception : " << err.what() << std::endl;
	}

	complete_timer += get_micro_time();
	log_message("Finished extraction, needed " + std::to_string(complete_timer) + "musec.");

	// log_message("Start building CH.");
	// long long timer = -get_micro_time();

	// auto ch = ContractionHierarchy::build(
	// 	routing_graph.node_count(),
	// 	invert_inverse_vector(routing_graph.first_out), routing_graph.head,
	// 	travel_time);

	// timer += get_micro_time();
	// log_message("Finished building CH, needed " + std::to_string(timer) + "musec.");

	// log_message("Saving CH and node ordering.");
	// std::string ch_node_rank_file = "ch_node_rank";
	// if (!ch_node_rank_file.empty())
	// 	save_vector(ch_node_rank_file, ch.rank);

	// std::string ch_file = "ch";
	// if (!ch_file.empty())
	// 	ch.save_file(ch_file);

	// ContractionHierarchyQuery ch_query(ch);
	// ch_query.add_source(7448645).add_target(6099730).run();
	// auto distance = ch_query.get_distance();
	// std::cout << "dist is " << distance << std::endl;
}
