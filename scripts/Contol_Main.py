#!/usr/bin/env python3

"""License
    SPDX-License-Identifier:MIT
    Copyright (C) 2022 Yusuke Yamasaki. All Rights Reserved.
"""

import rospy
from std_srvs.srv import SetBool

def main():
    rospy.init_node('Control_Main')

    control_linetrace = rospy.ServiceProxy("Control_linetrace", SetBool)
    control_sensing = rospy.ServiceProxy("Control_sensing", SetBool)
    contorl_boll = rospy.ServiceProxy("Control_boll", SetBool)

    srv_name = ("Control_linetrace", "Control_sensing", "Control_boll")
    for srv_name in srv_name:
        rospy.wait_for_service(srv_name)

    

if __name__ == '__main__':
    
    if not rospy.is_shutdown():
        main()
