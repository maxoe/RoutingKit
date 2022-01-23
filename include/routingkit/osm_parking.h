#ifndef ROUTING_KIT_OSM_SIMPLE_H
#define ROUTING_KIT_OSM_SIMPLE_H

#include <routingkit/tag_map.h>
#include <routingkit/bit_vector.h>

#include <vector>
#include <functional>
#include <string>
#include <fstream>

namespace RoutingKit
{
	// Additional tags for amenity=parking
	// from https://wiki.openstreetmap.org/wiki/Tag:amenity%3Dparking
	// Note that the tag "operator" has been replaced with "operator_name" in the enum
	enum OSMParkingAdditionalTags
	{
		name,
		ref,
		access,
		parking,
		park_ride,
		fee,
		supervised,
		capacity,
		capacity_disabled,
		capacity_parent,
		capacity_charging,
		surface,
		maxstay,
		opening_hours,
		operator_name,
		website,
		hgv,
		bus,
		OSMParkingAdditionalTags_MAX = bus
	};

	using OSMParkingTagsArray = std::array<std::string, OSMParkingAdditionalTags::OSMParkingAdditionalTags_MAX + 1>;

	const OSMParkingTagsArray OSMParkingAdditionalTagsToString =
		{
			"name",
			"ref",
			"access",
			"parking",
			"park_ride",
			"fee",
			"supervised",
			"capacity",
			"capacity_disabled",
			"capacity_parent",
			"capacity_charging",
			"surface",
			"maxstay",
			"opening_hours",
			"operator",
			"website",
			"hgv",
			"bus"};

	struct OSMParkingIDMapping
	{
		BitVector is_parking_node;
		BitVector is_parking_way;
		BitVector is_parking_modelling_node;
	};

	struct OSMExtractedParking
	{
		std::vector<float> latitude;
		std::vector<float> longitude;
		std::vector<OSMParkingTagsArray> tags;
	};

	bool is_osm_object_used_for_parking(uint64_t osm_way_id, const TagMap &tags, std::function<void(const std::string &)> log_message = nullptr);
	bool is_osm_object_used_for_hgv_parking(uint64_t osm_way_id, const TagMap &tags, std::function<void(const std::string &)> log_message = nullptr);

	OSMParkingIDMapping load_osm_parking_id_mapping_from_pbf(
		const std::string &file_name,
		std::function<bool(uint64_t, const TagMap &)> is_parking_node = nullptr,
		std::function<void(const std::string &)> log_message = nullptr);

	OSMExtractedParking load_osm_parking_from_pbf(
		const std::string &pbf_file,
		const OSMParkingIDMapping &mapping,
		std::function<void(const std::string &)> log_message = nullptr,
		bool file_is_ordered_even_though_file_header_says_that_it_is_unordered = false);

	OSMExtractedParking simple_load_osm_car_parking_routing_graph_from_pbf(
		const std::string &pbf_file,
		const std::function<void(const std::string &)> &log_message = nullptr,
		bool file_is_ordered_even_though_file_header_says_that_it_is_unordered = false);

	void save_parking_tags(const std::string &file_name, const std::vector<OSMParkingTagsArray> &tags);
	void save_parking_tags_csv(const std::string &file_name, const OSMExtractedParking &tags);

} // RoutingKit

#endif
