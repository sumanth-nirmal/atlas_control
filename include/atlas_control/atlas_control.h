#pragma once

#include"ros/ros.h"
#include "atlas_msgs/AtlasSimInterfaceCommand.h"
#include "atlas_msgs/AtlasSimInterfaceState.h"
#include "atlas_msgs/AtlasCommand.h"
#include "atlas_msgs/AtlasState.h"
#include "std_msgs/String.h"


class AtlasControl
{

public:
  /**
   * @brief constructor
   */
  AtlasControl(ros::NodeHandle& nh);



  /**
   * @brief destructor
   */
  ~AtlasControl();

private:

  /// node handle
  ros::NodeHandle nh_;

  // subscribers
  ros::Subscriber sub_atlas_sim_state_;
  ros::Subscriber sub_atlas_state_;

  //publishers
  ros::Publisher pub_mode_, pub_atlas_sim_command_;

  //msgs
  atlas_msgs::AtlasSimInterfaceCommand  atlas_sim_command_;
  atlas_msgs::AtlasSimInterfaceState atlas_sim_state_;
  atlas_msgs::AtlasCommand atlas_command_;
  atlas_msgs::AtlasState atlas_state_;

  //call backs
  void callback_atlas_sim_state (const atlas_msgs::AtlasSimInterfaceState& msg_sim_state);
  void callback_atlas_state (const atlas_msgs::AtlasState& msg_state);

 // mehtods
  void walk_primitive(void);
  calculate_pose(long step_index);

  // mutex
  boost::mutex sim_state_mtx_, state_mtx_;

  long step_index_;
  float pos_x_, pos_y_, pos_z_;
};
