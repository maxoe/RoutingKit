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
		std::cout << argv[0] << " pbf_file export_directory" << std::endl;
		std::cout << "geo_distance is in [m]" << std::endl;
		std::cout << "travel_time is in [s]" << std::endl;
		std::cout << "way_speed is in [km/h]" << std::endl;
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

	const fs::path core_ch_dir = export_dir / "core_ch";
	std::string core_ch_node_order_file = core_ch_dir / "order";
	std::string core_ch_node_rank_file = core_ch_dir / "rank";
	std::string core_ch_core_file = core_ch_dir / "core";

	const fs::path core_ch_fw_graph_dir = core_ch_dir / "forward";
	const fs::path core_ch_bw_graph_dir = core_ch_dir / "backward";
	const std::string core_ch_fw_first_out_file = core_ch_fw_graph_dir / "first_out";
	const std::string core_ch_fw_head_file = core_ch_fw_graph_dir / "head";
	const std::string core_ch_fw_travel_time_file = core_ch_fw_graph_dir / "travel_time";
	const std::string core_ch_bw_first_out_file = core_ch_bw_graph_dir / "first_out";
	const std::string core_ch_bw_head_file = core_ch_bw_graph_dir / "head";
	const std::string core_ch_bw_travel_time_file = core_ch_bw_graph_dir / "travel_time";

	std::function<void(const std::string &)> log_message = [](const std::string &msg)
	{
		std::cout << msg << std::endl;
	};

	log_message("Start building CH.");

	// // core_instance
	// std::vector<unsigned> first_out{0, 1, 3, 5, 6, 6};
	// std::vector<unsigned> head{1, 2, 3, 3, 4, 4};
	// std::vector<unsigned> travel_time{1, 4, 3, 2, 2, 4};

	// // core_instance_2
	// std::vector<unsigned> first_out{0, 1, 3, 5, 6, 6};
	// std::vector<unsigned> head{1, 2, 3, 3, 4, 4};
	// std::vector<unsigned> travel_time{1, 1, 3, 1, 4, 1};

	// core_instance_3
	std::vector<unsigned> first_out{0, 2, 4, 5, 5};
	std::vector<unsigned> head{1, 2, 2, 3, 3};
	std::vector<unsigned> travel_time{2, 1, 1, 2, 1};

	if (!first_out_file.empty())
		save_vector(first_out_file, first_out);
	if (!head_file.empty())
		save_vector(head_file, head);
	if (!travel_time_file.empty())
		save_vector(travel_time_file, travel_time);

	auto ch = ContractionHierarchy::build(
		first_out.size() - 1,
		invert_inverse_vector(first_out), head,
		travel_time);

	for (auto &i : ch.rank)
	{
		std::cout << i << " ";
	}
	std::cout << std::endl;
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
	if (!ch_node_order_file.empty())
		save_vector(ch_node_order_file, ch.order);

	BitVector is_core = BitVector(5, false);
	is_core.set(2, true);
	is_core.set(3, true);

	log_message("Start building core CH.");

	std::vector<unsigned int> core;
	ContractionHierarchy core_ch;
	std::tie(core, core_ch) = ContractionHierarchy::build_excluding_core(
		ch.rank, is_core,
		invert_inverse_vector(first_out), head,
		travel_time);
	for (auto &i : core_ch.rank)
	{
		std::cout << i << " ";
	}
	log_message("Finished building core CH");
	log_message("Saving core CH and node ordering.");

	if (!fs::is_directory(core_ch_dir) || !fs::exists(core_ch_dir))
	{
		fs::create_directory(core_ch_dir);
	}

	if (!fs::is_directory(core_ch_fw_graph_dir) || !fs::exists(core_ch_fw_graph_dir))
	{
		fs::create_directory(core_ch_fw_graph_dir);
	}

	if (!fs::is_directory(core_ch_bw_graph_dir) || !fs::exists(core_ch_bw_graph_dir))
	{
		fs::create_directory(core_ch_bw_graph_dir);
	}

	if (!core_ch_node_order_file.empty())
		save_vector(core_ch_node_order_file, core_ch.order);

	if (!core_ch_node_rank_file.empty())
		save_vector(core_ch_node_rank_file, core_ch.rank);

	if (!core_ch_core_file.empty())
		save_vector(core_ch_core_file, core);

	if (!core_ch_fw_first_out_file.empty())
		save_vector(core_ch_fw_first_out_file, core_ch.forward.first_out);
	if (!core_ch_fw_head_file.empty())
		save_vector(core_ch_fw_head_file, core_ch.forward.head);
	if (!core_ch_fw_travel_time_file.empty())
		save_vector(core_ch_fw_travel_time_file, core_ch.forward.weight);

	if (!core_ch_bw_first_out_file.empty())
		save_vector(core_ch_bw_first_out_file, core_ch.backward.first_out);
	if (!core_ch_bw_head_file.empty())
		save_vector(core_ch_bw_head_file, core_ch.backward.head);
	if (!core_ch_bw_travel_time_file.empty())
		save_vector(core_ch_bw_travel_time_file, core_ch.backward.weight);
}
