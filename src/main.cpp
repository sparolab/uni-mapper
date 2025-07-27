//
// Created by gil on 23. 11. 13.
//

#include "UniMapServer.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uni_mapper");
    ROS_INFO("\033[1;32m----> Uni-Mapper Started.\033[0m");

    UniMapServer uni_map_server;

    std::thread run_thread(&UniMapServer::run, &uni_map_server);
    std::thread viz_thread(&UniMapServer::vizTrajectory, &uni_map_server);

    run_thread.join();
    viz_thread.join();

    ROS_INFO("\033[1;32m----> Uni-Mapper Finished.\033[0m");

    return 0;
}