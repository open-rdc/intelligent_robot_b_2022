/*License
    SPDX-License-Identifier:MIT
    Copyright (C) 2022 Yusuke Yamasaki. All Rights Reserved.
*/

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "intelligent_robot_b_2022/Int32_4.h"
#include "intelligent_robot_b_2022/Color.h"
#include "intelligent_robot_b_2022/Line_Wheel.h"

// pub_message
//std_msgs::Int32 pub_servo_msg;
//std_msgs::Int32 pub_fan_msg;
intelligent_robot_b_2022::Line_Wheel pub_linetrace_msg;
//intelligent_robot_b_2022::Int32_4 pub_encoder_msg;

// sub_color
//void colorCallback(const intelligent_robot_b_2022::Color &sub_color_msg)
//{
//    ROS_INFO("COLOR_DATA : R[ %u ], G[ %u ], B[ %u ], IR[ %u ]", sub_color_msg.data1, sub_color_msg.data2, sub_color_msg.data3, sub_color_msg.data4);
//}

// sub_distance_1
//void distance_1Callback(const std_msgs::Int32 &distance_1_msg)
//{
//  ROS_INFO("Distance: %d mm", distance_1_msg.data);
//}

// sub_encoder
//void encoderCallback(const intelligent_robot_b_2022::Int32_4 &sub_encoder_msg)
//{
//    ROS_INFO("ENCODER_DATA : L[ %d ], R[ %d ]", sub_encoder_msg.data1, sub_encoder_msg.data2);
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control_linetrace");

    ros::NodeHandle n;

//    ros::Publisher pub_servo = n.advertise<std_msgs::Int32>("Servo", 10);  // サーボの角度制御opic
//    ros::Publisher pub_fan = n.advertise<std_msgs::Int32>("Fan", 10);
    ros::Publisher pub_linetrace = n.advertise<intelligent_robot_b_2022::Line_Wheel>("LineTrace", 10);
//    ros::Publisher pub_encoder = n.advertise<intelligent_robot_b_2022::Int32_4>("Encoder", 10);

//    ros::Subscriber sub_color = n.subscribe("Color", 10, colorCallback);
//    ros::Subscriber sub_distance_1 = n.subscribe("Distance_1", 45, distance_1Callback);
//    ros::Subscriber sub_encoder = n.subscribe("encoder_situation", 10, encoderCallback);

    ros::Rate loop_rate(16000000); // 単位：Hz_ここの値が、arduinoとの同期周期の定義になる。

    ros::spin();

    return 0;
}