#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include "zed_shm_structs.h"

using namespace boost::interprocess;

SHMHeader* shm_header = nullptr;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!shm_header) return;

    // Direct copy of all 18 positions
    // Order: [Arm1_7, Grip1_2, Arm2_7, Grip2_2]
    for (size_t i = 0; i < 18 && i < msg->position.size(); ++i) {
        shm_header->joints[i] = msg->position[i];
    }
    
    shm_header->joint_timestamp = ros::Time::now().toNSec();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_shm_relay");
    ros::NodeHandle nh;

    try {
        shared_memory_object shm(open_only, "zed_shm", read_write);
        mapped_region region(shm, read_write);
        shm_header = static_cast<SHMHeader*>(region.get_address());

        // Standard ROS subscriber
        ros::Subscriber sub = nh.subscribe("/panda_dual/joint_states", 1, jointCallback);

        ROS_INFO("Bimanual Relay Online: Mapping 18 ROS joints to 18 SHM slots.");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("SHM Link Failed: %s", e.what());
        return 1;
    }
    return 0;
}