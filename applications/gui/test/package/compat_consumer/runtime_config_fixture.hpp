#pragma once

#include <filesystem>
#include <fstream>

inline void WriteRuntimeConfigFixture(const std::filesystem::path& config,
                                      const std::filesystem::path& data,
                                      const std::filesystem::path& output) {
  namespace fs = std::filesystem;
  fs::create_directories(data / "agent1/Scans");
  fs::create_directories(config / "server");
  fs::create_directories(config / "core");
  std::ofstream(data / "agent1/poses.txt")
      << "1 0 0 0 0 1 0 0 0 0 1 0\n";
  std::ofstream(data / "agent1/Scans/000000.pcd")
      << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\n"
      << "HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n"
      << "10 0 0 1\n";
  std::ofstream(config / "config.json")
      << "{\"global\":{\"config_map_server\":\"server/map.json\","
      << "\"config_data_loader\":\"core/data.json\","
      << "\"config_loop_detector\":\"core/loop.json\","
      << "\"config_backend_optimizer\":\"core/optimizer.json\","
      << "\"config_dynamic_remover\":\"core/remover.json\"},"
      << "\"directory\":{\"root_dir_path\":\"" << data.string()
      << "\",\"sub_dir_list\":[\"agent1\"],\"root_save_dir\":\""
      << output.string() << "\"}}";
  std::ofstream(config / "server/map.json")
      << R"({"map_server":{"enable_map_updater":false,"anchor_agent_index":0,"save_voxel_size":0.2,"parallel_data_load":false,"parallel_map_update":false,"max_parallel_agents":1}})";
  std::ofstream(config / "core/data.json")
      << R"({"data_loader":{"data_loader_type":"file_based","pose_format":"kitti","pose_file_name":"poses.txt","extrinsic":[0,0,0,0,0,0,1],"scan_type":"pcd","scan_dir_name":"Scans","voxel_size":0.5,"min_range":1.0,"max_range":60.0,"delimiter":" "}})";
  std::ofstream(config / "core/loop.json")
      << R"({"loop_detector":{"loop_detector_type":"kdtree","model":"scan_context"},"database":{"descriptor_vector_dim":20,"distance_threshold":0.15,"num_candidates":3,"rebuild_threshold":50},"alignment":{"pcm_translation_threshold":10.0,"pcm_rotation_threshold_deg":20.0,"pcm_solver":"heuristic","pcm_threads":1,"pcm_max_candidates":0}})";
  std::ofstream(config / "core/optimizer.json")
      << R"({"backend_optimizer":{"backend_optimizer_type":"incremental","relinearizeThreshold":0.1,"relinearizeSkip":1,"isam_extra_updates":1,"min_loop_frame_gap":30,"icp_search_num":1}})";
  std::ofstream(config / "core/remover.json")
      << R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"free_dom"}})";
}
