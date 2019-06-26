
// ROS
#include "ros/ros.h"

#include "p4_ros/minAccXYWpPVA.h"
#include "p4_ros/min_time.h"
#include "p4_ros/p4_helper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "p4_client");
    ros::NodeHandle node("~");

    // ros::spin();
    ros::Rate loop_rate(10);
    loop_rate.sleep();


    ros::ServiceClient client = node.serviceClient<p4_ros::min_time>("/p4_services/min_time_solver");
    p4_ros::min_time req;

    // Simple takeoff
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 0.0));
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 0.75));
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 1.5));

    // Scan Example
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, -0.3, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 2, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 4, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 6.2, -0.7));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 6.2, -1.4));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 4, -1.4));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 2, -1.4));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.4, 0.2, -1.4));

    // // Scan example 2
    // req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 1.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 1.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 1.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 1.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 1.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 1.5));
    // req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 1.5));
    // req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 1.5));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 1.5));
    // req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 1.5));
    // req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 2.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 2.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 2.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 2.0));
    // req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 2.0));

    req.request.sampling_freq = 30;
    req.request.corridor_width = 0.1;
    req.request.max_vel = 0.3;
    req.request.max_acc = 0.3;
    req.request.max_jerk = 0.3;
    req.request.visualize_output = false;

    for (uint i = 0; i < 1000; i++) {
        ros::Time t0 = ros::Time::now();
        ROS_INFO("[p4_services] Calling service %s!", client.getService().c_str());
        std::cout << "attempt " << i << std::endl;
        if (!client.call(req)) {
            ROS_WARN("Error when calling the trajectory optimizer!");
        }
        ros::Time t1 = ros::Time::now();
        ROS_INFO("Solution time: %f", (t1 - t0).toSec());

        if(req.response.final_time <= 0) {
            ROS_WARN("Solution failed!");
            break;
        }
    }
    
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

  return 0;
}