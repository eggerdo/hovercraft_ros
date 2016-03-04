/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <hovercraft_node/HovercraftCommand.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class HovercraftTeleop
{
public:
  HovercraftTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  void updateMotors();

  ros::NodeHandle ph_, nh_;

  int left_, angular_, deadman_axis_, right_;
  int lift_up_, lift_down_, land_, take_off_;
  int forward_, backward_;

  double l_scale_left_, l_scale_right_, a_scale_, r_scale_;
  // ros::Publisher vel_pub_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;

  // geometry_msgs::Twist last_published_;
  hovercraft_node::HovercraftCommand last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_, deadman_disabled_;
  ros::Timer timer_;

  // ros::ServiceClient setMotorClient_;
  // int motoraccess_axis_, elevator_axis_, brush_axis_, vacuum_axis_, pump_axis_;
  // bool elevator_up_, elevator_on_, brush_on_, vacuum_on_, pump_on_;
  // bool motoraccess_pressed_;
  // bool elevator_pressed_, brush_pressed_, vacuum_pressed_, pump_pressed_;
  // bool last_elevator_pressed_, last_brush_pressed_, last_vacuum_pressed_, last_pump_pressed_;
  // boost::mutex setMotor_mutex_;
  // ros::Timer setMotor_timer_;

};

HovercraftTeleop::HovercraftTeleop():
  ph_("~"),
  left_(1),
  right_(3),
  forward_(13),
  backward_(12),
  lift_up_(11),
  lift_down_(9),
  land_(10),
  take_off_(8),
  angular_(0),
  deadman_axis_(4),
  l_scale_left_(0.3),
  l_scale_right_(0.3),
  a_scale_(0.9),
  deadman_disabled_(false)
  // ,
  // motoraccess_axis_(11),
  // elevator_axis_(13),
  // brush_axis_(14),
  // vacuum_axis_(15),
  // pump_axis_(12),
  // motoraccess_pressed_(false),
  // last_elevator_pressed_(false),
  // last_brush_pressed_(false),
  // last_vacuum_pressed_(false),
  // last_pump_pressed_(false),
  // elevator_up_(false),
  // elevator_on_(false),
  // brush_on_(false),
  // vacuum_on_(false),
  // pump_on_(false)
{
  ph_.param("axis_left", left_, left_);
  ph_.param("axis_right", right_, right_);
  ph_.param("axis_forward", forward_, forward_);
  ph_.param("axis_backward", backward_, backward_);
  ph_.param("axis_lift_up", lift_up_, lift_up_);
  ph_.param("axis_lift_down", lift_down_, lift_down_);
  ph_.param("axis_land", land_, land_);
  ph_.param("axis_take_off", take_off_, take_off_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("deadman_disabled", deadman_disabled_, deadman_disabled_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear_left", l_scale_left_, l_scale_left_);
  ph_.param("scale_linear_right", l_scale_right_, l_scale_right_);
  // ph_.param("btn_motoraccess", motoraccess_axis_, motoraccess_axis_);
  // ph_.param("btn_pump", pump_axis_, pump_axis_);
  // ph_.param("btn_elevator", elevator_axis_, elevator_axis_);
  // ph_.param("btn_brush", brush_axis_, brush_axis_);
  // ph_.param("btn_vacuum", vacuum_axis_, vacuum_axis_);

  // vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  cmd_pub_ = ph_.advertise<hovercraft_node::HovercraftCommand>("hovercraft_cmd", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &HovercraftTeleop::joyCallback, this);

  // setMotorClient_ = nh_.serviceClient<hovercraft_node::SetMotor>("hovercraft_node/set_motor", true);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&HovercraftTeleop::publish, this));
  // setMotor_timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&HovercraftTeleop::updateMotors, this));
}

void HovercraftTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  hovercraft_node::HovercraftCommand cmd;
  // vel.angular.z = a_scale_*joy->axes[angular_];
  cmd.leftSpeed = l_scale_left_*joy->axes[left_];
  cmd.rightSpeed = l_scale_right_*joy->axes[right_];

  if (joy->axes[forward_] != 0) {
    cmd.leftSpeed = -l_scale_left_*joy->axes[forward_];
    cmd.rightSpeed = -l_scale_right_*joy->axes[forward_];
  }

  if (joy->axes[backward_] != 0) {
    cmd.leftSpeed = l_scale_left_*joy->axes[backward_];
    cmd.rightSpeed = l_scale_right_*joy->axes[backward_];
  }

  if (joy->axes[take_off_] != 0) {
    cmd.liftThrust = 1.0;
  } else if (joy->axes[land_] != 0) {
    cmd.liftThrust = -1.0;
  } else if (joy->axes[lift_up_] != 0) {
    cmd.liftThrust = 0.5;
  } else if (joy->axes[lift_down_] != 0) {
    cmd.liftThrust = -0.5;
  } else {
    cmd.liftThrust = 0.0;
  }

  last_published_ = cmd;
  deadman_pressed_ = joy->buttons[deadman_axis_];

  // motoraccess_pressed_ = joy->buttons[motoraccess_axis_];
  // elevator_pressed_ = joy->buttons[elevator_axis_];
  // brush_pressed_ = joy->buttons[brush_axis_];
  // vacuum_pressed_ = joy->buttons[vacuum_axis_];
  // pump_pressed_ = joy->buttons[pump_axis_];
}

// void HovercraftTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
// {
//   geometry_msgs::Twist vel;
//   // vel.angular.z = a_scale_*joy->axes[angular_];
//   vel.linear.x = l_scale_left_*joy->axes[left_];
//   vel.linear.y = l_scale_right_*joy->axes[right_];

//   if (joy->axes[forward_] != 0) {
//     vel.linear.x = -l_scale_left_*joy->axes[forward_];
//     vel.linear.y = -l_scale_right_*joy->axes[forward_];
//   }

//   if (joy->axes[backward_] != 0) {
//     vel.linear.x = l_scale_left_*joy->axes[backward_];
//     vel.linear.y = l_scale_right_*joy->axes[backward_];
//   }

//   if (joy->axes[take_off_] != 0) {
//     vel.linear.z = 1.0;
//   } else if (joy->axes[land_] != 0) {
//     vel.linear.z = -1.0;
//   } else if (joy->axes[lift_up_] != 0) {
//     vel.linear.z = 0.5;
//   } else if (joy->axes[lift_down_] != 0) {
//     vel.linear.z = -0.5;
//   } else {
//     vel.linear.z = 0.0;
//   }

//   last_published_ = vel;
//   deadman_pressed_ = joy->buttons[deadman_axis_];

//   // motoraccess_pressed_ = joy->buttons[motoraccess_axis_];
//   // elevator_pressed_ = joy->buttons[elevator_axis_];
//   // brush_pressed_ = joy->buttons[brush_axis_];
//   // vacuum_pressed_ = joy->buttons[vacuum_axis_];
//   // pump_pressed_ = joy->buttons[pump_axis_];
// }

void HovercraftTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_ || deadman_disabled_)
  {
    // vel_pub_.publish(last_published_);
    cmd_pub_.publish(last_published_);
  }

}

/*void HovercraftTeleop::updateMotors()
{
  boost::mutex::scoped_lock lock(setMotor_mutex_);

  if (motoraccess_pressed_)
  {
    //ROS_INFO("motoraccess pressed");

    hovercraft_node::SetMotor motorCmd;

    if (!last_elevator_pressed_ && elevator_pressed_) {
      motorCmd.request.motor_id = 0;
      if (elevator_on_) {
        ROS_INFO("elevator stop");
        motorCmd.request.speed = 0;

        if (setMotorClient_.call(motorCmd)) {
          elevator_on_ = false;
        }
      } else {
        if (elevator_up_) {
          ROS_INFO("elevator down");
          motorCmd.request.speed = -255;
        } else {
          ROS_INFO("elevator up");
          motorCmd.request.speed = 255;
        }

        if (setMotorClient_.call(motorCmd)) {
          elevator_on_ = true;
          elevator_up_ = !elevator_up_;
        }
      }
    }
    last_elevator_pressed_ = elevator_pressed_;

    if (!last_brush_pressed_ && brush_pressed_) {
      motorCmd.request.motor_id = 3;
      if (brush_on_) {
        ROS_INFO("brush off");
        motorCmd.request.speed = 0;
      } else {
        ROS_INFO("brush on");
        motorCmd.request.speed = 255;
      }
      if (setMotorClient_.call(motorCmd)) {
        brush_on_ = !brush_on_;
      }
    }
    last_brush_pressed_ = brush_pressed_;

    if (!last_vacuum_pressed_ && vacuum_pressed_) {
      motorCmd.request.motor_id = 2;
      if (vacuum_on_) {
        ROS_INFO("vacuum off");
        motorCmd.request.speed = 0;
      } else {
        ROS_INFO("vacuum on");
        motorCmd.request.speed = 255;
      }
      if (setMotorClient_.call(motorCmd)) {
        vacuum_on_ = !vacuum_on_;
      }
    }
    last_vacuum_pressed_ = vacuum_pressed_;

    if (!last_pump_pressed_ && pump_pressed_) {
      motorCmd.request.motor_id = 1;
      if (pump_on_) {
        ROS_INFO("pump off");
        motorCmd.request.speed = 0;
      } else {
        ROS_INFO("pump on");
        motorCmd.request.speed = 255;
      }
      if (setMotorClient_.call(motorCmd)) {
        pump_on_ = !pump_on_;
      } else {
        ROS_INFO("pump failed");
      }
    }
    last_pump_pressed_ = pump_pressed_;

  } else {
    last_elevator_pressed_ = false;
    last_brush_pressed_ = false;
    last_vacuum_pressed_ = false;
    last_pump_pressed_ = false;
  }

}*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hovercraft_teleop");
  HovercraftTeleop hovercraft_teleop;

  ros::spin();
}
