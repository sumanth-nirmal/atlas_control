#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include "math.h"
#include <tf/transform_datatypes.h>
#include "atlas_control/atlas_control.h"
#include "stdio.h"

AtlasControl::AtlasControl(ros::NodeHandle & nh):
     nh_(nh), step_index_(0)
{
    printf("start\n");

    pub_atlas_sim_command_ = nh_.advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command", 1);
    pub_mode_ = nh_.advertise<std_msgs::String>("/atlas/mode", 1);

    sub_atlas_sim_state_ = nh_.subscribe("/atlas/atlas_sim_interface_state", 10, &AtlasControl::callback_atlas_sim_state, this);
    sub_atlas_state_ = nh_.subscribe("/atlas/atlas_state", 1, &AtlasControl::callback_atlas_state, this);

    printf("con done\n");
}

AtlasControl::~AtlasControl()
{

}

void AtlasControl::callback_atlas_sim_state(const atlas_msgs::AtlasSimInterfaceState msg_sim_state)
{
    printf("sub\n");

    {
        boost::lock_guard<boost::mutex> guard(sim_state_mtx_);

        // fetch the message
        atlas_sim_state_ = msg_sim_state;

        // get the position
        pos_x_ = atlas_sim_state_.pos_est.position.x;
        pos_y_ = atlas_sim_state_.pos_est.position.y;
        pos_z_ = atlas_sim_state_.pos_est.position.z;
    }


    //call the walk_primitive
    walk_primitive();
}

void AtlasControl::callback_atlas_state(const atlas_msgs::AtlasState &msg_state)
{
    {
//       boost::lock_gaurd<boost::mutex> gaurd(state_mtx_);


    }
}

void AtlasControl::walk_primitive(void)
{
    int i,is_right_foot;
    long step_index;

    printf("walk\n");

    atlas_sim_command_.behavior = 4; //WALK

    for (i=0;i<28;i++)
    {
      atlas_sim_command_.k_effort.push_back(0);
    }

    {
        boost::lock_guard<boost::mutex> guard(sim_state_mtx_);
        step_index_ = atlas_sim_state_.walk_feedback.next_step_index_needed;
    }

    for (i = 0; i < 4; i++)
    {
        step_index = step_index_ + i;
        is_right_foot = step_index % 2;

        atlas_sim_command_.walk_params.step_queue[i].step_index = step_index;
        atlas_sim_command_.walk_params.step_queue[i].foot_index = is_right_foot;
        atlas_sim_command_.walk_params.step_queue[i].duration = 0.63;
        atlas_sim_command_.walk_params.step_queue[i].swing_height = 0.2;
        atlas_sim_command_.walk_params.step_queue[i].pose = calculate_pose(step_index);
    }

    // publish the message
    pub_atlas_sim_command_.publish(atlas_sim_command_);
}

geometry_msgs::Pose AtlasControl::calculate_pose (long step_index)
{
int is_right_foot, is_left_foot;
long current_step;
float theta, R_foot, X, Y;
int offset_dir;

int R = 2;
float W = 0.3;

is_right_foot = step_index % 2;
is_left_foot = 1 - is_right_foot;

current_step = step_index % 60;

theta = current_step * M_PI/ 30;

offset_dir = 1 - (2 * is_left_foot);

R_foot = R + offset_dir * W/2;

X = R_foot * sin(theta);
Y = (R - R_foot*cos(theta));

printf("step: %d\n",step_index);
printf ("X %f \n",X);
printf ("Y %f \n",Y);

tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, theta);

geometry_msgs::Pose pose;

pose.position.x = pos_x_ + X;
pose.position.y = pos_y_ + Y;
pose.position.z = 0;

pose.orientation.x = q.getX();
pose.orientation.y = q.getY();
pose.orientation.z = q.getZ();
pose.orientation.w = q.getW();

printf ("x %f \n",pose.position.x);
printf ("y %f \n",pose.position.y);

return pose;
}

int main(int argc, char* argv[]) {
  // init ros stuff
  ros::init(argc, argv, "atlas_control_node");
  ros::NodeHandle nh("~");

  AtlasControl Atlas(nh);

  ros::spin();

  return 0;
}
