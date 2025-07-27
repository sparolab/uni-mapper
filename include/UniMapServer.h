//
// Created by gil on 23. 11. 13.
//

#ifndef UNI_MAPPER_UNIMAPSERVER_H
#define UNI_MAPPER_UNIMAPSERVER_H

#include "utility.h"
#include "RosParamServer.h"
#include "SingleMapDB.h"
#include "IntraLoopPGO.h"
#include "InterLoopPGO.h"

class UniMapServer : public RosParamServer {
public:
    std::vector<std::string> full_data_path_vec_;
    std::vector<SingleMapDB*> multi_map_db_;
    int curr_single_map_idx_;

    ros::Publisher pub_origin_path_;
    ros::Publisher pub_updated_path_;
    ros::Publisher pub_loop_edge_;

    std::vector<ros::Publisher> pub_updated_path_vec_;
    std::vector<ros::Publisher> pub_loop_marker_vec_;
    std::vector<ros::Publisher> pub_pcd_vec_;

public:
    UniMapServer();
    ~UniMapServer();

    void run();

    void singleMapOpt();
    void multiMapMerging();
    void pubPCL();
    void vizTrajectory();
    void saveUpdatedPoses();
};

#endif //UNI_MAPPER_UNIMAPSERVER_H
