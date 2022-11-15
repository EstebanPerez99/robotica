/* Author: Enrique Flores */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

#include <geometry_msgs/Point.h>
#include <iostream>

double global_errorX;
double global_errorY;

void callbackError(const geometry_msgs::Point& msg);

bool init_gazebo_engine(void);
void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();

bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Subscriber read_joint_sub;

ros::ServiceClient set_joint_module_client;

int control_module = None;
bool demo_ready = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "read_write");

  ros::NodeHandle nh(ros::this_node::getName());

  ros::Subscriber error_sub = nh.subscribe("/ball_error", 1, callbackError);

  bool is_sim = true;
  nh.getParam("gazebo_sim", is_sim);
  if(is_sim)
  {
    init_gazebo_engine();
    // we have to create publishers for each one of the joints, here we only show for l_el joint, 
    // complete for others...
    ros::Publisher l_el_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_el_position/command", 1);
    // .....
    // now we can send value, firs we create message data and then we publish
    while (ros::ok())
    {
      std_msgs::Float64 l_el_msg;
      l_el_msg.data = 1.7; // we send constant value of pi/2
      l_el_pub.publish(l_el_msg);
    }

  }else
  {

    init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
    write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);

    // service
    set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");
    ros::start();

    //set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // wait for starting of op3_manager
    std::string manager_name = "/op3_manager";
    while (ros::ok())
    {
      ros::Duration(1.0).sleep();

      if (checkManagerRunning(manager_name) == true)
      {
        break;
        ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
      }
      ROS_WARN("Waiting for op3 manager");
    }

    readyToDemo();

    //node loop
    sensor_msgs::JointState write_msg;
    write_msg.header.stamp = ros::Time::now();
    // write_msg.name.push_back("r_sho_pitch");
    // write_msg.position.push_back(0.0);
    // write_msg.name.push_back("r_sho_roll");
    // write_msg.position.push_back(0.0);
    // write_msg.name.push_back("r_el");
    // write_msg.position.push_back(0.0);
    // write_joint_pub.publish(write_msg);

    double counter = 0.0;
    double K_p = 0.05;

    double X_ant = 0;
    double Y_ant = 0;

    while (ros::ok())
    {
      ros::spinOnce();

      global_errorX = (global_errorX*3.141592)/180;
      global_errorY = (global_errorY*3.141592)/180;

      global_errorX = K_p*global_errorX + X_ant;
      global_errorY = K_p*global_errorY + Y_ant;

      std::cout << "ErrorX en rad: " << (global_errorX) << "\n";
      std::cout << "ErrorY en rad: " << (global_errorY) << "\n";

      // process
      write_msg.name.push_back("head_pan");
      write_msg.position.push_back(global_errorX);
      write_msg.name.push_back("head_tilt");
      write_msg.position.push_back(global_errorY);
      write_joint_pub.publish(write_msg);

      X_ant = global_errorX;
      Y_ant = global_errorY;

      counter += 0.01;

      //relax to fit output rate
      loop_rate.sleep();
    }
  }
  //exit program
  return 0;
}
void readyToDemo()
{
  ROS_INFO("Start Read-Write Demo");
  torqueOnAll();
  ROS_INFO("Torque on All joints");

  // send message for going init posture
  goInitPose();
  ROS_INFO("Go Init pose");

  // wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

bool checkManagerRunning(std::string& manager_name)
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }
  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }
  return ;
}

void torqueOnAll()
{
  std_msgs::String check_msg;
  check_msg.data = "check";
  dxl_torque_pub.publish(check_msg);
}

bool init_gazebo_engine(void)
{
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;
  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused)
  {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }
  if (!unpaused)
  {
    ROS_FATAL("Could not wake up Gazebo.");
    return false;
  }
  ROS_INFO("Unpaused the Gazebo simulation.");
  // Wait for Gazebo GUI show up.
  ros::Duration(10).sleep();
  return true;
}

void callbackError(const geometry_msgs::Point& msg)
{
  global_errorX = msg.x;
  global_errorY = msg.y;
}