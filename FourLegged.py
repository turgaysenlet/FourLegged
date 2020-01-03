#!/usr/bin/env python

# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'James Diprose'


import rospy
from std_msgs.msg import Float32
from ros_pololu_servo.srv import MotorRange
from ros_pololu_servo.msg import MotorCommand
from sensor_msgs.msg import Joy

pi = 3.1415926
class LipSync():

    def __init__(self):
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_s                  ize=1)

        # Setup motor command
        self.cmd = MotorCommand()
        self.cmd.speed = 1.0
        self.cmd.acceleration = 1.0

        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self.cmd.position = 0
        self.cmd.joint_name = 'left_front'
        self.motor_pub.publish(self.cmd)
        self.cmd.joint_name = 'left_rear'
        self.motor_pub.publish(self.cmd)
        self.cmd.joint_name = 'right_front'
        self.motor_pub.publish(self.cmd)
        self.cmd.joint_name = 'right_rear'
        self.motor_pub.publish(self.cmd)

    def joy_callback(self, msg):
        y = msg.axes[0]
        x = msg.axes[1]
        x1 = 1-x
        self.cmd.position = x1 * pi / 2.0
        self.cmd.joint_name = 'left_front'
        self.motor_pub.publish(self.cmd)
        self.cmd.position = (x1+y) * pi / 2.0
        self.cmd.joint_name = 'left_rear'
        self.motor_pub.publish(self.cmd)
        self.cmd.position = x * pi / 2.0
        self.cmd.joint_name = 'right_front'
        self.motor_pub.publish(self.cmd)
        self.cmd.position = (x+y) * pi / 2.0
        self.cmd.joint_name = 'right_rear'
        self.motor_pub.publish(self.cmd)

if __name__ == "__main__":
    rospy.init_node('lip_sync')
    lip_sync = LipSync()
    rospy.spin()