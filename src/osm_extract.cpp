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

using namespace RoutingKit;
using namespace std;
#undef NDEBUG
#include <assert.h>
int main(int argc, char *argv[])
{
	try
	{
		std::string
			pbf_file,
			first_out_file,
			head_file,
			geo_distance_file,
			travel_time_file,
			way_file,
			way_speed_file,
			way_name_file,
			latitude_file,
			longitude_file,
			osm_node_file,
			osm_way_file,
			osm_parking_node_file,
			osm_parking_way_file,
			parking_info_file;

		bool with_parking = false;

		if (argc == 13)
		{
			pbf_file = argv[1];
			first_out_file = argv[2];
			head_file = argv[3];
			geo_distance_file = argv[4];
			travel_time_file = argv[5];
			way_file = argv[6];
			way_speed_file = argv[7];
			way_name_file = argv[8];
			latitude_file = argv[9];
			longitude_file = argv[10];
			osm_node_file = argv[11];
			osm_way_file = argv[12];
		}
		else if (argc == 16)
		{
			pbf_file = argv[1];
			first_out_file = argv[2];
			head_file = argv[3];
			geo_distance_file = argv[4];
			travel_time_file = argv[5];
			way_file = argv[6];
			way_speed_file = argv[7];
			way_name_file = argv[8];
			latitude_file = argv[9];
			longitude_file = argv[10];
			osm_node_file = argv[11];
			osm_way_file = argv[12];
			osm_parking_node_file = argv[13];
			osm_parking_way_file = argv[14];
			parking_info_file = argv[15];

			with_parking = true;
		}
		else if (argc == 8)
		{
			pbf_file = argv[1];
			first_out_file = argv[2];
			head_file = argv[3];
			geo_distance_file = argv[4];
			travel_time_file = argv[5];
			latitude_file = argv[6];
			longitude_file = argv[7];
		}
		else if (argc == 11)
		{
			pbf_file = argv[1];
			first_out_file = argv[2];
			head_file = argv[3];
			geo_distance_file = argv[4];
			travel_time_file = argv[5];
			latitude_file = argv[6];
			longitude_file = argv[7];
			osm_parking_node_file = argv[8];
			osm_parking_way_file = argv[9];
			parking_info_file = argv[10];

			with_parking = true;
		}
		else
		{
			cout << "Usage:" << endl;
			cout << argv[0] << " pbf_file first_out head geo_distance travel_time way way_speed way_name latitude longitude osm_node osm_way" << endl;
			cout << argv[0] << " pbf_file first_out head geo_distance travel_time latitude longitude" << endl;
			cout << "geo_distance is in [m]" << endl;
			cout << "travel_time is in [s]" << endl;
			cout << "way_speed is in [km/h]" << endl;
			return 1;
		}

		long long complete_timer = -get_micro_time();

		std::function<void(const std::string &)> log_message = [](const string &msg)
		{
			cout << msg << endl;
		};

		std::function<bool(uint64_t, const TagMap &)> is_osm_way_used_for_routing =
			[&](uint64_t osm_way_id, const TagMap &tags)
		{
			return is_osm_way_used_by_cars(osm_way_id, tags, log_message);
		};

		std::function<bool(uint64_t, const TagMap &)> is_osm_node_used_for_routing = nullptr;

		if (with_parking)
		{
			is_osm_node_used_for_routing =
				[&](uint64_t osm_node_id, const TagMap &tags)
			{
				return is_osm_object_used_for_parking(osm_node_id, tags);
			};
		}

		auto mapping = load_osm_id_mapping_from_pbf(
			pbf_file,
			is_osm_node_used_for_routing,
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

		// {
		// 	unsigned max_arc = invalid_id;
		// 	unsigned max_travel_time = 0;
		// 	for (unsigned a = 0; a < arc_count; ++a)
		// 	{
		// 		if (travel_time[a] >= max_travel_time)
		// 		{
		// 			max_arc = a;
		// 			max_travel_time = travel_time[a];
		// 		}
		// 	}

		// 	IDMapper way_mapper(mapping.is_routing_way);

		// 	cout << "Arc with maximum travel time : " << max_arc << endl;
		// 	cout << "Corresponding travel time : " << travel_time[max_arc] << " ms" << endl;
		// 	cout << "Corresponding geographic length : " << routing_graph.geo_distance[max_arc] << " m" << endl;
		// 	cout << "Speed : " << way_speed[routing_graph.way[max_arc]] << " km/h" << endl;
		// 	cout << "Corresponding OSM way : " << way_mapper.to_global(routing_graph.way[max_arc]) << endl;
		// }

		{
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
			if (!way_file.empty())
				save_vector(way_file, routing_graph.way);
			if (!way_name_file.empty())
				save_vector(way_name_file, way_name);
			if (!way_speed_file.empty())
				save_vector(way_speed_file, way_speed);
			if (!latitude_file.empty())
				save_vector(latitude_file, routing_graph.latitude);
			if (!longitude_file.empty())
				save_vector(longitude_file, routing_graph.longitude);
			if (!osm_node_file.empty())
				save_bit_vector(osm_node_file, mapping.is_routing_node);
			if (!osm_way_file.empty())
				save_bit_vector(osm_way_file, mapping.is_routing_way);

			timer += get_micro_time();
			log_message("Finished saving, needed " + std::to_string(timer) + "musec.");
		}

		if (with_parking)
		{
			log_message("Extracting with parking.");

			auto parking_mapping = load_osm_parking_id_mapping_from_pbf(
				pbf_file, log_message);

			auto parking = load_osm_parking_from_pbf(
				pbf_file,
				parking_mapping,
				log_message);

			LocalIDMapper routing_node_mapper(mapping.is_routing_node);
			LocalIDMapper parking_node_mapper(parking_mapping.is_parking_node);
			BitVector routing_parking_flags(routing_graph.node_count());

			for (uint64_t i = 0; i < parking_mapping.is_parking_node.size(); ++i)
			{
				if (parking_mapping.is_parking_node.is_set(i))
				{
					routing_parking_flags.set(routing_node_mapper.to_local(i));

					assert(parking.latitude[parking_node_mapper.to_local(i)] == routing_graph.latitude[routing_node_mapper.to_local(i)]);
					assert(parking.longitude[parking_node_mapper.to_local(i)] == routing_graph.longitude[routing_node_mapper.to_local(i)]);
				}
			}

			try
			{
				log_message("Start saving parking information");
				long long timer = -get_micro_time();

				if (!parking_info_file.empty())
					save_parking_tags(parking_info_file, parking.tags);
				if (!osm_parking_node_file.empty())
					save_bit_vector(osm_parking_node_file, routing_parking_flags); // parking_mapping.is_parking_node);
				// if (!osm_parking_way_file.empty())
				// 	save_bit_vector(osm_parking_way_file, parking_mapping.is_parking_way);

				timer += get_micro_time();
				log_message("Finished saving, needed " + std::to_string(timer) + "musec.");
			}
			catch (std::exception &err)
			{
				cout << "Exception : " << err.what() << endl;
			}

			// crucial for performance that parking_mapping is not used after extracting the routing
			// so do not use parking mapping in callbacks
			// is_osm_way_used_for_routing = [&](uint64_t osm_way_id, const TagMap &tags)
			// {
			// 	return is_osm_way_used_by_cars(osm_way_id, tags, log_message);
			// 	// 	   || parking_mapping.is_parking_way.is_set(osm_way_id);
			// };
		}

		complete_timer += get_micro_time();
		log_message("Finished extraction, needed " + std::to_string(complete_timer) + "musec.");

		log_message("Start building CH.");
		long long timer = -get_micro_time();

		auto ch = ContractionHierarchy::build(
			routing_graph.node_count(),
			invert_inverse_vector(routing_graph.first_out), routing_graph.head,
			travel_time);

		timer += get_micro_time();
		log_message("Finished building CH, needed " + std::to_string(timer) + "musec.");

		log_message("Saving CH and node ordering.");
		std::string ch_node_rank_file = "ch_node_rank";
		if (!ch_node_rank_file.empty())
			save_vector(ch_node_rank_file, ch.rank);

		std::string ch_file = "ch";
		if (!ch_file.empty())
			ch.save_file(ch_file);

		// ContractionHierarchyQuery ch_query(ch);
		// ch_query.add_source(7448645).add_target(6099730).run();
		// auto distance = ch_query.get_distance();
		// std::cout << "dist is " << distance << std::endl;
	}
	catch (std::exception &err)
	{
		cout << "Exception : " << err.what() << endl;
	}
}
