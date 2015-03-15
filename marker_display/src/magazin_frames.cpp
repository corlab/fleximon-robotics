/*#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "my_marker";
  int_marker.description = "Simple 1-DOF Control";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  //box_marker.mesh_resource = "/home/luensel/catkin_ws/src/marker_display/meshes/ur5_iml.dae";
  box_marker.mesh_resource = "package://marker_display/meshes/Magazin.STL";
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}*/

/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;

double p_x=0.0 ,p_y=-0.0475,p_z=0.0;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%





// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t,t_temp,t_temp2,t1,t2,t3,t4;

  ros::Time time = ros::Time::now();

// %Transformation calculation for grasping frame
  t1.setRotation(tf::createQuaternionFromRPY(3.14, 0, 0));
  t2.setRotation(tf::createQuaternionFromRPY(0, -1.57, 0));
  t3.setRotation(tf::createQuaternionFromRPY(0, 0, 0.436332));
  t4.setRotation(tf::createQuaternionFromRPY(-2.335, 0, 0));

  t_temp.mult(t1,t2);
  t_temp2.mult(t_temp,t3);
  t.mult(t_temp2,t4);


  t.setOrigin(tf::Vector3(0.05605, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m3_a"));

  t.setOrigin(tf::Vector3(0.10475, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m3_b"));

  t.setOrigin(tf::Vector3(0.15325, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m3_c"));

  t.setOrigin(tf::Vector3(0.20175, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m3_d"));

  // ***************************

  t.setOrigin(tf::Vector3(0.28705 , 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m2_a"));

  t.setOrigin(tf::Vector3(0.33575, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m2_b"));

  t.setOrigin(tf::Vector3(0.38425, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m2_c"));

  t.setOrigin(tf::Vector3(0.43275, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m2_d"));

  // ***************************

  t.setOrigin(tf::Vector3(0.51405 , 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m5_a"));

  t.setOrigin(tf::Vector3(0.56275, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m5_b"));

  t.setOrigin(tf::Vector3(0.61125, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m5_c"));

  t.setOrigin(tf::Vector3(0.65975, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m5_d"));


  // ***************************

  t.setOrigin(tf::Vector3(0.74705 , 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m1_a"));

  t.setOrigin(tf::Vector3(0.79575, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m1_b"));

  t.setOrigin(tf::Vector3(0.84425, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m1_c"));

  t.setOrigin(tf::Vector3(0.89275, 0.35957, 0.13617));
  br.sendTransform(tf::StampedTransform(t, time, "magazin_frame", "m1_d"));

  t.setOrigin(tf::Vector3(p_x, p_y, p_z));
  t.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "magazin_frame"));


  counter++;
}
// %EndTag(frameCallback)%



int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);


  ros::Duration(0.1).sleep();


  ros::spin();
  //ros::Rate(50.0);
  server.reset();
}
// %EndTag(main)%

