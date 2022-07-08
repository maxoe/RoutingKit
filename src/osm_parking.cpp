#include <routingkit/osm_graph_builder.h>
#include <routingkit/osm_profile.h>
#include <routingkit/osm_parking.h>

#include <routingkit/id_mapper.h>
#include <routingkit/filter.h>

#include <routingkit/timer.h>

#include <vector>
#include <stdint.h>
#include <string>
#include <iostream>
#include <algorithm>

namespace RoutingKit
{
	bool is_osm_object_used_for_parking(uint64_t osm_way_id, const TagMap &tags, std::function<void(const std::string &)> log_message)
	{
		return (tags["amenity"] != nullptr && strcmp(tags["amenity"], "parking") == 0) ||
			   (tags["parking"] != nullptr);
	}

	bool is_osm_object_used_for_hgv_parking(uint64_t osm_way_id, const TagMap &tags, std::function<void(const std::string &)> log_message)
	{
		return is_osm_object_used_for_parking(osm_way_id, tags, log_message) &&
			   ((tags["hgv"] != nullptr && (strcmp(tags["hgv"], "yes") == 0 || strcmp(tags["hgv"], "designated") == 0)) ||
				(tags["access"] != nullptr && strcmp(tags["access"], "hgv") == 0));
	}

	unsigned int get_osm_way_truck_speed(uint64_t osm_way_id, const TagMap &tags, std::function<void(const std::string &)> log_message)
	{
		return std::min(get_osm_way_truck_speed(osm_way_id, tags, log_message), 80U);
	}

	OSMParkingIDMapping load_osm_parking_id_mapping_from_pbf(
		const std::string &file_name, std::function<bool(uint64_t, const TagMap &)> is_parking_node, std::function<void(const std::string &)> log_message)
	{
		OSMParkingIDMapping mapping;

		long long timer = 0;

		if (log_message)
		{
			log_message("Scanning OSM PBF data to determine parking object IDs");
			timer = -get_micro_time();
		}

		if (is_parking_node == nullptr)
		{
			is_parking_node = [&](uint64_t osm_node_id, const TagMap &tags)
			{
				return is_osm_object_used_for_parking(osm_node_id, tags);
			};
		}

		std::function<void(uint64_t, double, double, const TagMap &)> node_callback = nullptr;
		node_callback = [&](uint64_t osm_node_id, double lat, double lon, const TagMap &tags)
		{
			if (is_parking_node(osm_node_id, tags))
			{
				mapping.is_parking_node.make_large_enough_for(osm_node_id);
				mapping.is_parking_node.set(osm_node_id);
			}
		};

		std::function<void(uint64_t, const std::vector<uint64_t> &, const RoutingKit::TagMap &)> way_callback;
		way_callback = [&](uint64_t osm_way_id, const std::vector<uint64_t> &osm_node_id_list, const TagMap &tags)
		{
			if (is_parking_node(osm_way_id, tags))
			{
				mapping.is_parking_way.make_large_enough_for(osm_way_id);
				mapping.is_parking_way.set(osm_way_id);

				for (const auto &osm_node_id : osm_node_id_list)
				{
					mapping.is_parking_modelling_node.make_large_enough_for(osm_node_id);
					mapping.is_parking_modelling_node.set(osm_node_id);
				}
			}
		};

		unordered_read_osm_pbf(file_name, node_callback, way_callback, nullptr, log_message);

		if (log_message)
		{
			timer += get_micro_time();
			log_message("Finished scan, needed " + std::to_string(timer) + " musec.");

			log_message("OSM ID range goes up to " + std::to_string(mapping.is_parking_node.size()) + " for parking nodes.");
			log_message("Found " + std::to_string(mapping.is_parking_node.population_count()) + " parking nodes.");
			log_message("OSM ID range goes up to " + std::to_string(mapping.is_parking_way.size()) + " for parking ways.");
			log_message("Found " + std::to_string(mapping.is_parking_way.population_count()) + " parking ways.");
			log_message("OSM ID range goes up to " + std::to_string(mapping.is_parking_modelling_node.size()) + " for parking modelling nodes.");
			log_message("Found " + std::to_string(mapping.is_parking_modelling_node.population_count()) + " parking modelling nodes.");
		}

		return mapping;
	}

	OSMExtractedParking
	load_osm_parking_from_pbf(
		const std::string &pbf_file,
		const OSMParkingIDMapping &mapping,
		std::function<void(const std::string &)> log_message,
		bool file_is_ordered_even_though_file_header_says_that_it_is_unordered)
	{
		IDMapper parking_node(mapping.is_parking_node);
		IDMapper parking_way(mapping.is_parking_way);
		IDMapper parking_modelling_node(mapping.is_parking_modelling_node);

		OSMExtractedParking extracted;
		const uint64_t num_parking_nodes = parking_node.local_id_count();
		const uint64_t num_parking_ways = parking_way.local_id_count();
		const uint64_t num_parking_objects = num_parking_nodes + num_parking_ways;
		extracted.latitude = std::vector<float>(num_parking_objects);
		extracted.longitude = std::vector<float>(num_parking_objects);
		extracted.tags = std::vector<OSMParkingTagsArray>(num_parking_objects);

		struct OSMTempExtractedParkingModelling
		{
			std::vector<float> latitude;
			std::vector<float> longitude;
		};

		OSMTempExtractedParkingModelling temp_extracted_modelling;
		temp_extracted_modelling.latitude = std::vector<float>(parking_modelling_node.local_id_count());
		temp_extracted_modelling.longitude = std::vector<float>(parking_modelling_node.local_id_count());

		long long timer = 0;

		if (log_message)
		{
			log_message("Start computing parking ID mapping");
			timer = -get_micro_time();
		}

		if (log_message)
		{
			timer += get_micro_time();
			log_message("Finished, needed " + std::to_string(timer) + " musec.");
		}

		if (log_message)
		{
			log_message("Scanning OSM PBF data to extract parking");
			timer = -get_micro_time();
		}

		// TagMap key_inverse_map;
		// key_inverse_map.build(
		// 	OSMParkingAdditionalTags_MAX + 1,
		// 	[&](uint64_t i)
		// 	{
		// 		return OSMParkingAdditionalTagsToString[i];
		// 	},
		// 	[&](uint64_t i)
		// 	{
		// 		return std::to_string(i);
		// 	});

		std::function<void(uint64_t, double, double, const TagMap &)> node_callback = nullptr;
		node_callback = [&](uint64_t osm_node_id, double lat, double lon, const TagMap &tags)
		{
			unsigned parking_node_id = parking_node.to_local(osm_node_id, invalid_id);
			if (parking_node_id != invalid_id)
			{
				extracted.latitude[parking_node_id] = lat;
				extracted.longitude[parking_node_id] = lon;

				for (unsigned x = 0; x < OSMParkingAdditionalTags_MAX + 1; ++x)
				{
					const char *tag_value = tags[OSMParkingAdditionalTagsToString[x].c_str()];
					if (tag_value != nullptr)
					{
						extracted.tags[parking_node_id][x] = std::string(tag_value == nullptr ? "" : tag_value);
					}
				}
			}

			unsigned parking_modelling_node_id = parking_modelling_node.to_local(osm_node_id, invalid_id);
			if (parking_modelling_node_id != invalid_id)
			{
				temp_extracted_modelling.latitude[parking_modelling_node_id] = lat;
				temp_extracted_modelling.longitude[parking_modelling_node_id] = lon;
			}
		};

		std::function<void(uint64_t, const std::vector<uint64_t> &, const RoutingKit::TagMap &)> way_callback;
		way_callback = [&](uint64_t osm_way_id, const std::vector<uint64_t> &osm_node_id_list, const TagMap &tags)
		{
			unsigned parking_way_id = parking_way.to_local(osm_way_id, invalid_id);
			if (parking_way_id != invalid_id)
			{
				// extract tags
				for (unsigned x = 0; x < OSMParkingAdditionalTags_MAX + 1; ++x)
				{
					const char *tag_value = tags[OSMParkingAdditionalTagsToString[x].c_str()];
					if (tag_value != nullptr)
					{
						extracted.tags[num_parking_nodes + parking_way_id][x] =
							std::string(tag_value == nullptr ? "" : tag_value);
					}
				}

				// calculate lat/lon
				// averaging WGS84 lat/long is not correct but sufficient here
				float lat = 0;
				float lon = 0;

				for (const auto &osm_node_id : osm_node_id_list)
				{
					unsigned local_modelling_node_id = parking_modelling_node.to_local(osm_node_id, invalid_id);

					lat += temp_extracted_modelling.latitude[local_modelling_node_id];
					lon += temp_extracted_modelling.longitude[local_modelling_node_id];
				}

				extracted.latitude[num_parking_nodes + parking_way_id] = lat / osm_node_id_list.size();
				extracted.longitude[num_parking_nodes + parking_way_id] = lon / osm_node_id_list.size();
			}
		};

		// std::function<void(uint64_t, const std::vector<OSMRelationMember> &, const TagMap &)> relation_callback;
		// relation_callback = [&](uint64_t osm_relation_id, const std::vector<OSMRelationMember> &member_list, const TagMap &tags)
		// {
		// 	bool has_parking_modelling_node = false;
		// 	bool has_osm_way_for_cars = false;
		// 	unsigned i = 0;

		// 	while (!(has_parking_modelling_node && has_osm_way_for_cars) && i < member_list.size())
		// 	{
		// 		const auto &member = member_list[i];
		// 		has_parking_modelling_node = has_parking_modelling_node ||
		// 									 (member.type == OSMIDType::node &&
		// 									  parking_modelling_node.to_local(member.id, invalid_id) != invalid_id);

		// 		has_osm_way_for_cars = has_osm_way_for_cars ||
		// 							   (member.type == OSMIDType::way &&
		// 								car_way.to_local(member.id, invalid_id) != invalid_id);
		// 		++i;
		// 	}

		// 	if (has_parking_modelling_node && has_osm_way_for_cars)
		// 	{
		// 		std::cout << "id: " << member_list[i].id << std::endl;
		// 		for (const auto &t : tags)
		// 		{
		// 			std::cout << t.key << ": " << t.value << std::endl;
		// 		}
		// 		std::cout << std::endl;
		// 	}
		// };

		// Three scans of the pbf for nodes first, then ways, then relations.
		// So when scanning ways, node data is already fully extracted
		ordered_read_osm_pbf(
			pbf_file,
			node_callback,
			way_callback,
			nullptr /*relation_callback*/,
			log_message,
			file_is_ordered_even_though_file_header_says_that_it_is_unordered);

		if (log_message)
		{
			timer += get_micro_time();
			log_message("Finished scan, needed " + std::to_string(timer) + " musec.");
		}

		return extracted;
	}

	OSMExtractedParking simple_load_osm_car_parking_routing_graph_from_pbf(
		const std::string &pbf_file,
		const std::function<void(const std::string &)> &log_message,
		bool file_is_ordered_even_though_file_header_says_that_it_is_unordered)
	{
		return load_osm_parking_from_pbf(
			pbf_file,
			load_osm_parking_id_mapping_from_pbf(
				pbf_file, nullptr, log_message),
			log_message,
			file_is_ordered_even_though_file_header_says_that_it_is_unordered);
	}

	void save_parking_tags(const std::string &file_name, const std::vector<OSMParkingTagsArray> &tags)
	{
		std::ofstream out(file_name, std::ios::binary);
		for (unsigned i = 0; i < tags.size(); ++i)
			for (unsigned j = 0; j < OSMParkingAdditionalTags_MAX + 1; ++j)
			{
				{
					const char *x = tags[i][j].c_str();
					out.write(x, tags[i][j].length() + 1);
				}
			}
	}

	void save_parking_tags_csv(const std::string &file_name, const OSMExtractedParking &parking)
	{
		std::ofstream out(file_name);

		out << "latitude,longitude";

		for (unsigned i = 0; i < OSMParkingAdditionalTags_MAX + 1; ++i)
		{
			out << "," << OSMParkingAdditionalTagsToString[i];
		}

		out << std::endl;

		for (unsigned i = 0; i < parking.tags.size(); ++i)
		{
			out << parking.latitude[i] << "," << parking.longitude[i];
			for (unsigned j = 0; j < OSMParkingAdditionalTags_MAX + 1; ++j)
			{
				{
					out << "," << parking.tags[i][j];
				}
			}
			out << std::endl;
		}
	}
} // RoutingKit
