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
namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		std::cout << "Usage:" << std::endl;
		std::cout << argv[0] << " export_directory" << std::endl;
		return 1;
	}

	if (!fs::is_directory(argv[1]) || !fs::exists(argv[1]))
	{
		fs::create_directory(argv[1]);
	}

	const fs::path export_dir = fs::path(argv[1]);

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

	const fs::path ch_dir = export_dir / "ch";
	std::string ch_node_rank_file = ch_dir / "rank";

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

	log_message("Start building CH.");

	std::vector<unsigned> first_out{0, 1, 3, 4, 5, 5};
	std::vector<unsigned> head{1, 2, 3, 4, 4};
	std::vector<unsigned> travel_time{1, 4, 3, 2, 4};

	auto ch = ContractionHierarchy::build(
		first_out.size() - 1,
		invert_inverse_vector(first_out), head,
		travel_time);

	log_message("Finished building CH");
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

	check_contraction_hierarchy_for_errors(ch);

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

	if (!ch_node_rank_file.empty())
		save_vector(ch_node_rank_file, ch.rank);
}
