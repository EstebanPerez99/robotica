/* Based on read_write by Leo Campos */

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <cmath>

bool init_gazebo_engine(void);
void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
double pos_r_sho_pitch(double actual, double fin);
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
  
  bool is_sim = false;
  
  const int row = 6751;
  int col = 16;
  float posiciones[row][col];
  
  std::string line;
  std::ifstream myfile ("/home/robotis/Nay_ws/src/op3_leo/data/TestDeg.txt");
  if (myfile.is_open()){
	std::cout << "El archivo se abrió";
	
		for (int idx = 0; idx < row; idx++){
			for (int idy = 0; idy < col; idy++){
				myfile >> posiciones[idx][idy];
			}	
		}
		myfile.close();
  }else{
  	std::cout << "El archivo no se abrió" << std::endl;
  }
  
  nh.getParam("gazebo_sim", is_sim);
  if(is_sim)
  {
    init_gazebo_engine();
    // we have to create publishers for each one of the joints 
    ros::Publisher l_el_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_el_position/command", 1);
    ros::Publisher l_sho_roll_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_sho_roll_position/command", 1);
    ros::Publisher l_sho_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_sho_pitch_position/command", 1);
    ros::Publisher r_el_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_el_position/command", 1);
    ros::Publisher r_sho_roll_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_sho_roll_position/command", 1);
    ros::Publisher r_sho_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_sho_pitch_position/command", 1);
    ros::Publisher head_pan_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/head_pan_position/command", 1);
    ros::Publisher head_tilt_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/head_tilt_position/command", 1);
    ros::Publisher r_ank_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_ank_pitch_position/command", 1);
    ros::Publisher l_ank_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_ank_pitch_position/command", 1);
    ros::Publisher r_hip_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_hip_pitch_position/command", 1);
    ros::Publisher l_hip_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_hip_pitch_position/command", 1);
    ros::Publisher r_hip_yaw_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_hip_yaw_position/command", 1);
    ros::Publisher l_hip_yaw_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_hip_yaw_position/command", 1);
    ros::Publisher r_knee_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_knee_position/command", 1);
    ros::Publisher l_knee_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_knee_position/command", 1);

    // now we can send value, firs we create message data and then we publish
    while (ros::ok())
    {
      std_msgs::Float64 r_el_msg;
      r_el_msg.data = 0.4363;
      r_el_pub.publish(r_el_msg);
      
      std_msgs::Float64 r_sho_roll_msg;
      r_sho_roll_msg.data =  -1.1694;
      r_sho_roll_pub.publish(r_sho_roll_msg);
      
      std_msgs::Float64 r_sho_pitch_msg;
      r_sho_pitch_msg.data =  0.1745;
      r_sho_pitch_pub.publish(r_sho_pitch_msg);
      
      std_msgs::Float64 l_sho_pitch_msg;
      l_sho_pitch_msg.data =  0.1745;
      l_sho_pitch_pub.publish(l_sho_pitch_msg);
      
      std_msgs::Float64 l_sho_roll_msg;
      l_sho_roll_msg.data =  1.1694;
      l_sho_roll_pub.publish(l_sho_roll_msg);
      
      std_msgs::Float64 l_el_msg;
      l_el_msg.data =  -0.4363;
      l_el_pub.publish(l_el_msg);
      
      std_msgs::Float64 head_pan_msg;
      head_pan_msg.data =  0.0;
      head_pan_pub.publish(head_pan_msg);
 
      std_msgs::Float64 head_tilt_msg;
      head_tilt_msg.data =  0.0;
      head_tilt_pub.publish(head_tilt_msg);
 	     
      std_msgs::Float64 r_ank_pitch_msg;
      r_ank_pitch_msg.data =  -0.1749;
      r_ank_pitch_pub.publish(r_ank_pitch_msg); 
      
      std_msgs::Float64 r_knee_msg;
      r_knee_msg.data =  -0.2516;
      r_knee_pub.publish(r_knee_msg); 
     
      std_msgs::Float64 r_hip_pitch_msg;
      r_hip_pitch_msg.data =  0.1611;
      r_hip_pitch_pub.publish(r_hip_pitch_msg); 
      
      std_msgs::Float64 l_ank_pitch_msg;
      l_ank_pitch_msg.data =  0.1658;
      l_ank_pitch_pub.publish(l_ank_pitch_msg); 
      
      std_msgs::Float64 l_knee_msg;
      l_knee_msg.data =  0.2410;
      l_knee_pub.publish(l_knee_msg); 
     
      std_msgs::Float64 l_hip_pitch_msg;
      l_hip_pitch_msg.data =  -0.1703;
      l_hip_pitch_pub.publish(l_hip_pitch_msg); 
     
      std_msgs::Float64 r_hip_yaw_msg;
      r_hip_yaw_msg.data =  0.0;
      r_hip_yaw_pub.publish(r_hip_yaw_msg);
      
      std_msgs::Float64 l_hip_yaw_msg;
      l_hip_yaw_msg.data =  -0.0031;
      l_hip_yaw_pub.publish(l_hip_yaw_msg);
      
    }


  }else
   {
    init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
    write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
    
    //service
    set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");
    ros::start();

    //set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    //wait for starting of op3_manager
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
    
    //baile :)
    ros::Duration(1).sleep();
    ros::Rate loop_rate_dance(60);
    
    for (int fila=0; fila<row; fila++){
		write_msg.name.push_back("r_el");
		write_msg.position.push_back(posiciones[fila][0]);
		write_msg.name.push_back("r_sho_roll");
		write_msg.position.push_back(posiciones[fila][1]);
		write_msg.name.push_back("r_sho_pitch");
		write_msg.position.push_back(posiciones[fila][2]);
		write_msg.name.push_back("l_sho_pitch");
		write_msg.position.push_back(posiciones[fila][3]);
		write_msg.name.push_back("l_sho_roll");
		write_msg.position.push_back(posiciones[fila][4]);
		write_msg.name.push_back("l_el");
		write_msg.position.push_back(posiciones[fila][5]);
		write_msg.name.push_back("head_tilt");
		write_msg.position.push_back(posiciones[fila][6]);
		write_msg.name.push_back("head_pan");
		write_msg.position.push_back(posiciones[fila][7]);
		write_msg.name.push_back("r_ank_pitch");
		write_msg.position.push_back(posiciones[fila][8]);
		write_msg.name.push_back("r_knee");
		write_msg.position.push_back(posiciones[fila][9]);
		write_msg.name.push_back("r_hip_pitch");
		write_msg.position.push_back(posiciones[fila][10]);
		write_msg.name.push_back("l_ank_pitch");
		write_msg.position.push_back(posiciones[fila][11]);
		write_msg.name.push_back("l_knee");
		write_msg.position.push_back(posiciones[fila][12]);
		write_msg.name.push_back("l_hip_pitch");
		write_msg.position.push_back(posiciones[fila][13]);
		write_msg.name.push_back("r_hip_yaw");
		write_msg.position.push_back(posiciones[fila][14]);
		write_msg.name.push_back("l_hip_yaw");
		write_msg.position.push_back(posiciones[fila][15]);
    
/*
		write_msg.velocity.push_back(posiciones[fila][16]);
		write_msg.velocity.push_back(posiciones[fila][17]);
		write_msg.velocity.push_back(posiciones[fila][18]);
		write_msg.velocity.push_back(posiciones[fila][19]);
		write_msg.velocity.push_back(posiciones[fila][20]);
		write_msg.velocity.push_back(posiciones[fila][21]);
		write_msg.velocity.push_back(posiciones[fila][22]);
		write_msg.velocity.push_back(posiciones[fila][23]);
		write_msg.velocity.push_back(posiciones[fila][24]);
		write_msg.velocity.push_back(posiciones[fila][25]);
		write_msg.velocity.push_back(posiciones[fila][26]);
		write_msg.velocity.push_back(posiciones[fila][27]);
		write_msg.velocity.push_back(posiciones[fila][28]);
		write_msg.velocity.push_back(posiciones[fila][29]);
		write_msg.velocity.push_back(posiciones[fila][30]);
		write_msg.velocity.push_back(posiciones[fila][31]);*/
		write_joint_pub.publish(write_msg);

    loop_rate_dance.sleep();
    
    }
  }
  return 0;
}

void readyToDemo()
{
  ROS_INFO("Start Read-Write Demo");
  torqueOnAll();
  ROS_INFO("Torque on All joints");

  //send message for going init posture
  goInitPose();
  ROS_INFO("Go Init pose");

  //wait while ROBOTIS-OP3 goes to the init posture.
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
  //trying to unpause Gazebo for 10 seconds.
  while (i <= 5 && !unpaused)
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
  //wait for Gazebo GUI show up.
  ros::Duration(5).sleep();
  return true;
}