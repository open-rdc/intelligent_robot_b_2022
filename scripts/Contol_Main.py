"""License
    SPDX-License-Identifier:MIT
    Copyright (C) 2022 Yusuke Yamasaki. All Rights Reserved.
"""
#    ros::ServiceServer srv_main = n.advertiseService("Control_linetrace", Control_linetrace);
#    ros::ServiceServer srv_main = n.advertiseService("Control_boll", Control_boll);
#    ros::ServiceServer srv_main = n.advertiseService("Control_sensing", Control_sensing);

import rospy
import rosnode
# service message ( bool だけのやつがほしい。デフォルトで用意されてたはず )

def main():
    print("hogehoge")

if __name__ == '__main__':
    
    if not rospy.is_shutdown():
        main()
        