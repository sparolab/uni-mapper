//
// Created by gil on 23. 11. 17.
//

#ifndef UNI_MAPPER_ROSPARAMSERVER_H
#define UNI_MAPPER_ROSPARAMSERVER_H

#include "utility.h"
#include "STDesc.h"

class RosParamServer {
public:
    ros::NodeHandle nh_;
    std::string data_root_path_;
    std::vector<std::string> data_dir_path_vec_;
    std::string save_dir_path_;
    int kf_dist_;
    int rs_dist_;
    int rs_dist_th_;

    ConfigSetting std_config_;

public:
    RosParamServer();
    void read_parameters();
};

#endif //UNI_MAPPER_ROSPARAMSERVER_H
