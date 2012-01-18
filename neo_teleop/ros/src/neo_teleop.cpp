/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopNeo
{
public:
  TeleopNeo();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_x, linear_y, angular_z;
  double l_scale_x, l_scale_y, a_scale_z;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


TeleopNeo::TeleopNeo():
  linear_x(1), linear_y(0), angular_z(2)
{

  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("scale_linear_x", l_scale_x, l_scale_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("scale_linear_y", l_scale_y, l_scale_y);
  nh_.param("axis_angular_z", angular_z, angular_z);
  nh_.param("scale_angular_z", a_scale_z, a_scale_z);


  ROS_INFO("started joystick drive with ");

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopNeo::joyCallback, this);

}

void TeleopNeo::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_z*joy->axes[angular_z];
  vel.linear.x = l_scale_x*joy->axes[linear_x];
  vel.linear.y = l_scale_y*joy->axes[linear_y];
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_Neo");
  TeleopNeo teleop_Neo;

  ros::spin();
}
