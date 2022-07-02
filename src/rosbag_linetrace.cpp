/*License
    SPDX-License-Identifier:MIT
    Copyright (C) 2022 Yusuke Yamasaki. All Rights Reserved.
*/

#include "ros/ros.h"
#include "intelligent_robot_b_2022/rosbagdata.h"
#include "std_msgs/String.h"

void CallBack(const intelligent_robot_b_2022::rosbagdata &data)
{
    ROS_INFO("Sub_Data: L_encoder[%d] R_encoder[%d] L_x[%f] R_x[%f] L_v[%f] R_v[%f] v[%f]", data.L_encoder
                                                                                          , data.R_encoder
                                                                                          , data.L_x
                                                                                          , data.R_x
                                                                                          , data.L_v
                                                                                          , data.R_v
                                                                                          , data.v);
}
// void Callback(const std_msgs::String &data)
// {
//     ROS_INFO("%d",data.data);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_linetrace");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 10, CallBack);

    ros::Rate loop_Rate(16000000);

    ros::spin();

    return 0;
}