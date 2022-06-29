/*License
    SPDX-License-Identifier:MIT
    Copyright (C) 2022 Yusuke Yamasaki. All Rights Reserved.
*/

#include "ros/ros.h"
#include "intelligent_robot_b_2022/rosbagdata.h"

void CallBack(const intelligent_robot_b_2022::rosbagdata &data)
{
    ROS_INFO("Sub_Data: L_encoder[%ld] R_encoder[%ld] L_x[%f] R_x[%f] L_v[%f] R_v[%f] v[%f]", data.L_encoder
                                                                                            , data.R_encoder
                                                                                            , data.L_x
                                                                                            , data.R_x
                                                                                            , data.L_v
                                                                                            , data.R_v
                                                                                            , data.v);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_linetrace");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("linetrace_data", 10, CallBack);

    ros::Rate loop_rate(115200); // 単位：Hz_ここの値が、arduinoとの同期周期の定義になる。

    ROS_INFO("NODE: rosbag_linetrace standby.");

    ros::spin();

    return 0;
}