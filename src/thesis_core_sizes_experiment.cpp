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
#include <iomanip>
#include <sstream>
#include <exception>
#include <experimental/filesystem>

using namespace RoutingKit;
namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[])
{
	if (argc != 3 && !(argc == 4 && (argv[3] == std::string("--hgv-only") || argv[3] == std::string("--hgv-speed"))) && !(argc == 5 && ((argv[3] == std::string("--hgv-only") && argv[4] == std::string("--hgv-speed")) || (argv[3] == std::string("--hgv-speed") && argv[4] == std::string("--hgv-only")))))
	{
		std::cout << "Usage:" << std::endl;
		std::cout << argv[0] << " pbf_file export_directory [--hgv-only] [--hgv-speed]" << std::endl;
		std::cout << "geo_distance is in [m]" << std::endl;
		std::cout << "travel_time is in [s]" << std::endl;
		std::cout << "way_speed is in [km/h]" << std::endl;
		return 1;
	}

	const std::string pbf_file = argv[1];

	if (!fs::is_directory(argv[2]) || !fs::exists(argv[2]))
	{
		fs::create_directory(argv[2]);
	}

	const fs::path export_dir = fs::path(argv[2]);

	bool hgv_only = false;
	bool hgv_speed = false;

	if (argc == 4)
	{
		hgv_only |= argv[3] == std::string("--hgv-only");
		hgv_speed |= argv[3] == std::string("--hgv-speed");
	}

	if (argc == 5)
	{
		hgv_only |= argv[3] == std::string("--hgv-only") || argv[4] == std::string("--hgv-only");
		hgv_speed |= argv[3] == std::string("--hgv-speed") || argv[4] == std::string("--hgv-speed");
	}

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
	const std::string is_routing_node_file = export_dir / "is_routing_node";
	const std::string osm_parking_way_file = export_dir / "osm_parking_way";
	const std::string parking_info_file = export_dir / "parking_info.csv";

	const fs::path ch_dir = export_dir / "ch";
	std::string ch_node_rank_file = ch_dir / "rank";
	std::string ch_node_order_file = ch_dir / "order";

	const fs::path ch_fw_graph_dir = ch_dir / "forward";
	const fs::path ch_bw_graph_dir = ch_dir / "backward";
	const std::string ch_fw_first_out_file = ch_fw_graph_dir / "first_out";
	const std::string ch_fw_head_file = ch_fw_graph_dir / "head";
	const std::string ch_fw_travel_time_file = ch_fw_graph_dir / "travel_time";
	const std::string ch_bw_first_out_file = ch_bw_graph_dir / "first_out";
	const std::string ch_bw_head_file = ch_bw_graph_dir / "head";
	const std::string ch_bw_travel_time_file = ch_bw_graph_dir / "travel_time";

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

	// std::function<bool(uint64_t, const TagMap &, std::function<void(const std::string &)>)>
	// 	get_car_or_truck_osm_way_speed =
	// 		[&](uint64_t osm_way_id, const TagMap &tags, std::function<void(const std::string &)> log_message)
	// {
	// 	return get_osm_way_speed(osm_way_id, tags, log_message);
	// };

	// if (hgv_speed)
	// {
	// 	get_car_or_truck_osm_way_speed =
	// 		[&](uint64_t osm_way_id, const TagMap &tags, std::function<void(const std::string &)> log_message)
	// 	{
	// 		return get_osm_way_truck_speed(osm_way_id, tags, log_message);
	// 	};
	// }

	BitVector is_parking_node;
	BitVector is_parking_modelling_node;
	{
		log_message("Extracting parking.");

		const auto parking_mapping = load_osm_parking_id_mapping_from_pbf(
			pbf_file, has_parking_node_criteria, log_message);

		auto parking = load_osm_parking_from_pbf(
			pbf_file,
			parking_mapping,
			log_message);

		try
		{
			log_message("Start saving parking information");
			long long timer = -get_micro_time();

			if (!parking_info_file.empty())
				save_parking_tags_csv(parking_info_file, parking);
			// if (!osm_parking_way_file.empty())
			// 	save_bit_vector(osm_parking_way_file, parking_mapping.is_parking_way);

			timer += get_micro_time();
			log_message("Finished saving, needed " + std::to_string(timer) + "musec.");
		}
		catch (std::exception &err)
		{
			std::cout << "Exception : " << err.what() << std::endl;
		}

		is_parking_node = parking_mapping.is_parking_node;
		is_parking_modelling_node = parking_mapping.is_parking_modelling_node;
	}

	OSMRoutingGraph routing_graph;
	BitVector routing_parking_flags;
	std::vector<uint32_t> travel_time;
	{
		auto mapping = load_osm_id_mapping_from_pbf(
			pbf_file,
			has_parking_node_criteria,
			is_osm_way_used_for_routing,
			log_message);

		unsigned routing_way_count = mapping.is_routing_way.population_count();
		std::vector<uint32_t> way_speed(routing_way_count);
		std::vector<std::string> way_name(routing_way_count);

		mapping.is_routing_node = mapping.is_routing_node | (mapping.is_modelling_node & is_parking_modelling_node);

		routing_graph = load_osm_routing_graph_from_pbf(
			pbf_file,
			mapping,
			[&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap &way_tags)
			{
				// way_speed[routing_way_id] = get_car_or_truck_osm_way_speed(osm_way_id, way_tags, log_message);
				way_speed[routing_way_id] = get_osm_way_speed(osm_way_id, way_tags, log_message);
				way_name[routing_way_id] = get_osm_way_name(osm_way_id, way_tags, log_message);
				return get_osm_car_direction_category(osm_way_id, way_tags, log_message);
			},
			nullptr,
			log_message);

		unsigned arc_count = routing_graph.arc_count();

		travel_time = routing_graph.geo_distance;
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

		log_message("Start saving routing graph");
		long long timer = -get_micro_time();

		if (!first_out_file.empty())
			save_vector(first_out_file, routing_graph.first_out);
		if (!head_file.empty())
			save_vector(head_file, routing_graph.head);
		if (!geo_distance_file.empty())
			save_vector(geo_distance_file, routing_graph.geo_distance);
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

		BitVector is_area_parking_node = mapping.is_modelling_node & is_parking_modelling_node;

		log_message("Constructing parking flags");
		LocalIDMapper routing_node_mapper(mapping.is_routing_node);
		routing_parking_flags.make_large_enough_for(routing_graph.node_count());

		for (uint64_t i = 0; i < is_parking_node.size(); ++i)
		{
			if (is_parking_node.is_set(i) || is_area_parking_node.is_set(i))
			{
				routing_parking_flags.set(routing_node_mapper.to_local(i));
			}
		}

		log_message("Finished constructing parking flags");

		if (!routing_parking_flags_file.empty())
			save_bit_vector(routing_parking_flags_file, routing_parking_flags);
		if (!is_routing_node_file.empty())
			save_bit_vector(is_routing_node_file, mapping.is_routing_node);
	}

	log_message("Start building CH.");

	long long timer = -get_micro_time();
	std::vector<unsigned int> ch_rank;
	{
		auto ch = ContractionHierarchy::build(
			routing_graph.node_count(),
			invert_inverse_vector(routing_graph.first_out), routing_graph.head,
			travel_time, log_message);

		timer += get_micro_time();

		check_contraction_hierarchy_for_errors(ch);
		log_message("Finished building CH, needed " + std::to_string(timer) + "musec.");

		log_message("Saving CH and node ordering.");

		if (!fs::is_directory(ch_dir) || !fs::exists(ch_dir))
		{
			fs::create_directory(ch_dir);
		}

		if (!fs::is_directory(ch_fw_graph_dir) || !fs::exists(ch_fw_graph_dir))
		{
			fs::create_directory(ch_fw_graph_dir);
		}

		if (!fs::is_directory(ch_bw_graph_dir) || !fs::exists(ch_bw_graph_dir))
		{
			fs::create_directory(ch_bw_graph_dir);
		}

		if (!ch_node_rank_file.empty())
			save_vector(ch_node_rank_file, ch.rank);
		if (!ch_node_order_file.empty())
			save_vector(ch_node_order_file, ch.order);

		if (!ch_fw_first_out_file.empty())
			save_vector(ch_fw_first_out_file, ch.forward.first_out);
		if (!ch_fw_head_file.empty())
			save_vector(ch_fw_head_file, ch.forward.head);
		if (!ch_fw_travel_time_file.empty())
			save_vector(ch_fw_travel_time_file, ch.forward.weight);

		if (!ch_bw_first_out_file.empty())
			save_vector(ch_bw_first_out_file, ch.backward.first_out);
		if (!ch_bw_head_file.empty())
			save_vector(ch_bw_head_file, ch.backward.head);
		if (!ch_bw_travel_time_file.empty())
			save_vector(ch_bw_travel_time_file, ch.backward.weight);

		ch_rank = std::move(ch.rank);
	}

	double rel_core_size_start = 0.2;
	double step_factor = 0.5;
	double n = 10;
	{
		log_message("Start core size experiment with relative core size targets");

		timer = -get_micro_time();

		std::vector<unsigned int> core;
		ContractionHierarchy core_ch;
		ContractionHierarchy::core_experiment(
			ch_rank, routing_parking_flags,
			invert_inverse_vector(routing_graph.first_out), routing_graph.head,
			travel_time, export_dir.string(), rel_core_size_start, step_factor, n, log_message);

		timer += get_micro_time();

		log_message("Finished core CH experiment, needed " + std::to_string(timer) + "musec.");
	}
}