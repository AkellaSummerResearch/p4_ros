
// ROS
#include "ros/ros.h"

#include "p4_ros/trapezoidal_p2p.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "p4_client");
    ros::NodeHandle node("~");

    // // Sleep wait for ROS connection to finish setting up
    // ros::Rate loop_rate(10);
    // loop_rate.sleep();


    ros::ServiceClient client = node.serviceClient<p4_ros::trapezoidal_p2p>("/p4_services/trapezoidal_solver");
    p4_ros::trapezoidal_p2p req;

    // Waypoints
    req.request.init_pos.x = 1.0;  req.request.init_pos.y = 0.0;  req.request.init_pos.z = 0.0;
    req.request.final_pos.x = 3.0; req.request.final_pos.y = 0.0; req.request.final_pos.z = 0.0;
    req.request.init_yaw = 0.0;
    req.request.final_yaw = M_PI/2.0;

    req.request.max_vel = 0.3;
    req.request.max_acc = 0.3;
    req.request.max_yaw_vel = M_PI/6;
    req.request.max_yaw_acc = M_PI/6;

    req.request.sampling_freq = 50;
    req.request.visualize_output = true;

    ros::Time t0 = ros::Time::now();
    ROS_INFO("[p4_services] Calling service %s!", client.getService().c_str());
    if (!client.call(req)) {
        ROS_WARN("Error when calling the trajectory optimizer!");
    }
    ros::Time t1 = ros::Time::now();
    ROS_INFO("Solution time: %f", (t1 - t0).toSec());

    if(req.response.final_time < 0) {
        ROS_WARN("Solution failed!");
    }
    
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

  return 0;
}