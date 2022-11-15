/* Author: Leo Campos */

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

std::vector<double> motores;
void CallBack(const sensor_msgs::JointState& posicion);

double r_hip_pitch;
double l_hip_pitch;
double r_knee;
double l_knee;
double r_ank_pitch;
double l_ank_pitch;

double limite = 0.000005;

int ult_pos;
double rest_inc = 0.2618;

double t_ref_ang;
double t_ref;
double act_val = 0;

bool a = true;

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
  
  const int row = 5700;
  int col = 14;
  float posiciones[row][col];
  
  const int row2 = 40;
  int col2 = 6;
  float posiciones2[row2][col2];
  
  std::string line;
  /*std::ifstream myfile ("/home/robotis/nobios/src/op3_leo/data/FLDeg.txt");
  if (myfile.is_open()){
	std::cout << "El archivo se abriò";
	
		for (int idx = 0; idx < row; idx++){
			for (int idy = 0; idy < col; idy++){
				myfile >> posiciones[idx][idy];
			}	
		}
		myfile.close();
  }else{
  	std::cout << "no abriò";
  }*/
  
  std::ifstream myfile ("/home/robotis/nobios/src/op3_leo/data/Pararse.txt");
  if (myfile.is_open()){
	std::cout << "El archivo se abriò";
	
		for (int idx2 = 0; idx2 < row2; idx2++){
			for (int idy2 = 0; idy2 < col2; idy2++){
				myfile >> posiciones2[idx2][idy2];
			}
			
		}
		/*for (int idx = 0; idx < row; idx++){
			for (int idy = 0; idy < col; idy++){
				std::cout  << posiciones[idx][idy] << std::endl;
			}
			
		}*/
		myfile.close();
  }else{
  	std::cout << "no abriò";
  }
  
  nh.getParam("gazebo_sim", is_sim);
  if(is_sim)
  {
    init_gazebo_engine();
    // we have to create publishers for each one of the joints, here we only show for l_el joint, 
    // complete for others...
    ros::Publisher l_el_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_el_position/command", 1);
    ros::Publisher l_sho_roll_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_sho_roll_position/command", 1);
    ros::Publisher l_sho_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_sho_pitch_position/command", 1);
    ros::Publisher r_el_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_el_position/command", 1);
    ros::Publisher r_sho_roll_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_sho_roll_position/command", 1);
    ros::Publisher r_sho_pitch_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_sho_pitch_position/command", 1);
    ros::Publisher r_hip_yaw_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/r_hip_yaw_position/command", 1);
    ros::Publisher l_hip_yaw_pub = nh.advertise<std_msgs::Float64>("/robotis_op3/l_hip_yaw_position/command", 1);
    
    // .....
    // now we can send value, firs we create message data and then we publish
    while (ros::ok())
    {
      std_msgs::Float64 r_hip_yaw_msg;
      r_hip_yaw_msg.data =  0.5236;
      r_hip_yaw_pub.publish(r_hip_yaw_msg);
      
      std_msgs::Float64 l_hip_yaw_msg;
      l_hip_yaw_msg.data =  0.5236;
      l_hip_yaw_pub.publish(l_hip_yaw_msg);
      
      	ROS_INFO("esperando... y viendo...");
      

    }


  }else
  {

    init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
    write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
    
    //Subscriber
    ros::Subscriber lectura = nh.subscribe("/robotis/present_joint_states",1,CallBack);

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
    
    //Pararse en posiciòn para caminar
    /*ros::Duration(1).sleep();
    ros::Rate loop_rate_pararse(60);
    
    for (int fila2=0; fila2<row2; fila2++){
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(posiciones2[fila2][0]);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(posiciones2[fila2][1]);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(posiciones2[fila2][2]);
	write_msg.name.push_back("l_ank_pitch");
	write_msg.position.push_back(posiciones2[fila2][3]);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(posiciones2[fila2][4]);
	write_msg.name.push_back("l_hip_pitch");
	write_msg.position.push_back(posiciones2[fila2][5]);
     	write_joint_pub.publish(write_msg);
     	 
     	loop_rate_pararse.sleep();
     	ult_pos = fila2;
    
    }*/
//////////////////////////////////////////////// Levantarse Adelante ////////////////////////////////////////////////

	
	write_msg.name.push_back("l_sho_pitch");
	write_msg.position.push_back(-1.3963);
	write_msg.name.push_back("r_sho_pitch");
	write_msg.position.push_back(1.3963);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-2.3);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(2.3);
     	write_joint_pub.publish(write_msg);
     	
     	ros::Duration(0.5).sleep();
	/*write_msg.name.push_back("r_el");
	write_msg.position.push_back(0.3492);
	write_msg.name.push_back("r_sho_roll");
	write_msg.position.push_back(-1.2);
	write_msg.name.push_back("l_el");
	write_msg.position.push_back(-0.3492);
	write_msg.name.push_back("l_sho_roll");
	write_msg.position.push_back(1.2);*/
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-2.0944);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(2.0944);
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-1.4661);
	write_msg.name.push_back("l_ank_pitch");
	write_msg.position.push_back(1.4661);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(1.5708);
	write_msg.name.push_back("l_hip_pitch");
	write_msg.position.push_back(-1.5708);
     	write_joint_pub.publish(write_msg);
     	
     	ros::Duration(0.5).sleep();
     	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back((posiciones2[0][1])/2);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(posiciones2[0][1]);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(-(posiciones2[0][1])/2);
     	write_joint_pub.publish(write_msg);
     	
       ros::Duration(0.5).sleep();
     	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-1.1345);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(0);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(-0.2);
     	write_joint_pub.publish(write_msg);
     	
     	ros::Duration(0.5).sleep();
     	write_msg.name.push_back("l_ank_pitch");
	write_msg.position.push_back(-(posiciones2[0][1])/2);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(-(posiciones2[0][1]));
	write_msg.name.push_back("l_hip_pitch");
	write_msg.position.push_back((posiciones2[0][1])/2);
     	write_joint_pub.publish(write_msg);
     	
       ros::Duration(0.5).sleep();
     	write_msg.name.push_back("l_ank_pitch");
	write_msg.position.push_back(1.1345);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(0);
	write_msg.name.push_back("l_hip_pitch");
	write_msg.position.push_back(0.2);
     	write_joint_pub.publish(write_msg);
     	
     	ros::Duration(0.5).sleep();
     	write_msg.name.push_back("l_sho_pitch");
	write_msg.position.push_back(-0.8727);
	write_msg.name.push_back("r_sho_pitch");
	write_msg.position.push_back(0.8727);
     	write_joint_pub.publish(write_msg);
     	
     	ros::Duration(0.5).sleep();
     	write_msg.name.push_back("r_el");
	write_msg.position.push_back(1.7372);
	write_msg.name.push_back("r_sho_roll");
	write_msg.position.push_back(0);
	write_msg.name.push_back("l_el");
	write_msg.position.push_back(-1.7372);
	write_msg.name.push_back("l_sho_roll");
	write_msg.position.push_back(0);/*
     	write_msg.name.push_back("l_sho_pitch");
	write_msg.position.push_back(-0.9599);
	write_msg.name.push_back("r_sho_pitch");
	write_msg.position.push_back(0.9599);*/
     	write_joint_pub.publish(write_msg);
     	
     	ros::Duration(0.5).sleep();
     	write_msg.name.push_back("r_el");
	write_msg.position.push_back(2.4904);
	write_msg.name.push_back("r_sho_roll");
	write_msg.position.push_back(1);
	write_msg.name.push_back("l_el");
	write_msg.position.push_back(-2.4904);
	write_msg.name.push_back("l_sho_roll");
	write_msg.position.push_back(-1);/*
     	write_msg.name.push_back("l_sho_pitch");
	write_msg.position.push_back(-0.9599);
	write_msg.name.push_back("r_sho_pitch");
	write_msg.position.push_back(0.9599);*/
     	write_joint_pub.publish(write_msg);

    
//////////////////////////////////////////////// Acomodo de pies ////////////////////////////////////////////////
	/*ros::Duration(1).sleep();
	write_msg.name.push_back("l_hip_yaw");
	write_msg.position.push_back(-0.0873);
	write_msg.name.push_back("r_hip_yaw");
	write_msg.position.push_back(0.0873);
     	write_joint_pub.publish(write_msg);
     	
	/*ros::Duration(1).sleep();
	write_msg.name.push_back("l_hip_roll");
	write_msg.position.push_back(-0.0873);
	write_msg.name.push_back("r_hip_roll");
	write_msg.position.push_back(0.0873);
	write_msg.name.push_back("l_ank_roll");
	write_msg.position.push_back(-0.0873);
	write_msg.name.push_back("r_ank_roll");
	write_msg.position.push_back(0.0873);
     	write_joint_pub.publish(write_msg);*/
    
//////////////////////////////////////////////// Caminata ////////////////////////////////////////////////

    	//Paso 0 (derecho)
	/*ros::Duration(1).sleep();
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-0.7091);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-1.5287);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(0.9474+rest_inc);
	
	write_msg.name.push_back("r_hip_roll");
	write_msg.position.push_back(0.0436);
	write_msg.name.push_back("r_ank_roll");
	write_msg.position.push_back(0.0436);
	
	write_joint_pub.publish(write_msg);
	
	ros::Duration(0.1).sleep();
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-0.5486);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-1.1446);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(0.6618+rest_inc);
	write_msg.name.push_back("l_ank_pitch");	//Pie izquierdo se acomoda para que centro de masa quede en medio de ambos pies
	write_msg.position.push_back(0.5845);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(1.1453);
	write_msg.name.push_back("l_hip_pitch");
	write_msg.position.push_back(-0.5233-rest_inc);
	
	write_msg.name.push_back("r_hip_roll");
	write_msg.position.push_back(0);
	write_msg.name.push_back("r_ank_roll");
	write_msg.position.push_back(0);
	
	write_joint_pub.publish(write_msg);
	
	for (int i=0; i<11; i++){
		//Pie izquierdo
		ros::Duration(0.1).sleep();
		write_msg.name.push_back("l_ank_pitch");
		write_msg.position.push_back(0.7520);
		write_msg.name.push_back("l_knee");
		write_msg.position.push_back(1.5317);
		write_msg.name.push_back("l_hip_pitch");
		write_msg.position.push_back(-0.8143-rest_inc);
	
		write_msg.name.push_back("l_hip_roll");
		write_msg.position.push_back(-0.0436);
		write_msg.name.push_back("l_ank_roll");
		write_msg.position.push_back(-0.0436);
		
		write_joint_pub.publish(write_msg);
		
		ros::Duration(0.1).sleep();
		write_msg.name.push_back("l_ank_pitch");
		write_msg.position.push_back(0.5486);
		write_msg.name.push_back("l_knee");
		write_msg.position.push_back(1.1446);
		write_msg.name.push_back("l_hip_pitch");
		write_msg.position.push_back(-0.6618-rest_inc);
		write_msg.name.push_back("r_ank_pitch");	//Pie derecho se acomoda para que centro de masa quede en medio de ambos pies
		write_msg.position.push_back(-0.5845);
		write_msg.name.push_back("r_knee");
		write_msg.position.push_back(-1.1453);
		write_msg.name.push_back("r_hip_pitch");
		write_msg.position.push_back(0.5233+rest_inc);
	
		write_msg.name.push_back("l_hip_roll");
		write_msg.position.push_back(0);
		write_msg.name.push_back("l_ank_roll");
		write_msg.position.push_back(0);
	
		write_joint_pub.publish(write_msg);
		
		//Pie derecho
		ros::Duration(0.1).sleep();
		write_msg.name.push_back("r_ank_pitch");
		write_msg.position.push_back(-0.7520);
		write_msg.name.push_back("r_knee");
		write_msg.position.push_back(-1.5317);
		write_msg.name.push_back("r_hip_pitch");
		write_msg.position.push_back(0.8143+rest_inc);
	
		write_msg.name.push_back("r_hip_roll");
		write_msg.position.push_back(0.0436);
		write_msg.name.push_back("r_ank_roll");
		write_msg.position.push_back(0.0436);
	
		write_joint_pub.publish(write_msg);
		
		ros::Duration(0.1).sleep();
		write_msg.name.push_back("r_ank_pitch");	
		write_msg.position.push_back(-0.5486);		//-0.5486  <- valor matlab
		write_msg.name.push_back("r_knee");
		write_msg.position.push_back(-1.1446);
		write_msg.name.push_back("r_hip_pitch");
		write_msg.position.push_back(0.6618+rest_inc);
		write_msg.name.push_back("l_ank_pitch");	//Pie izquierdo se acomoda para que centro de masa quede en medio de ambos pies
		write_msg.position.push_back(0.5845);
		write_msg.name.push_back("l_knee");
		write_msg.position.push_back(1.1453);
		write_msg.name.push_back("l_hip_pitch");
		write_msg.position.push_back(-0.5233-rest_inc);
		
		write_msg.name.push_back("r_hip_roll");
		write_msg.position.push_back(0);
		write_msg.name.push_back("r_ank_roll");
		write_msg.position.push_back(0);
		
		write_joint_pub.publish(write_msg);
	}
	
//////////////////////////////////////////////// Patada ////////////////////////////////////////////////

	//Detenerse
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(posiciones2[ult_pos][0]);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(posiciones2[ult_pos][1]);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
	write_msg.name.push_back("l_ank_pitch");
	write_msg.position.push_back(posiciones2[ult_pos][3]);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(posiciones2[ult_pos][4]);
	write_msg.name.push_back("l_hip_pitch");
	write_msg.position.push_back(posiciones2[ult_pos][5] - rest_inc);
	write_joint_pub.publish(write_msg);
	
	//Inclinarse para alinear el centro de masa
	ros::Duration(1).sleep();
    	write_msg.name.push_back("l_hip_roll");
	write_msg.position.push_back(-0.17);
    	write_msg.name.push_back("r_hip_roll");
	write_msg.position.push_back(-0.17);
	write_msg.name.push_back("l_ank_roll");
	write_msg.position.push_back(0.15);
	write_msg.name.push_back("r_ank_roll");
	write_msg.position.push_back(0.45);
	write_joint_pub.publish(write_msg);
	
	//Posiciòn de seguridad
	ros::Duration(0.1).sleep();
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-0.7091);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-1.5287);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(0.9474 + rest_inc);
	write_msg.name.push_back("r_ank_roll");
	write_msg.position.push_back(0);
	write_joint_pub.publish(write_msg);
	
	//Patada
	ros::Duration(0.1).sleep();
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-0.0046);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-0.7420);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(1.2287 + rest_inc);
	write_joint_pub.publish(write_msg);
	
	//Posiciòn de seguridad
	ros::Duration(0.1).sleep();
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-0.7091);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-1.5287);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(0.9474 + rest_inc);
	write_msg.name.push_back("r_ank_roll");
	write_msg.position.push_back(0);
	write_joint_pub.publish(write_msg);
	
	//Regreso
	ros::Duration(0.1).sleep();
    	write_msg.name.push_back("l_hip_roll");
	write_msg.position.push_back(0);
    	write_msg.name.push_back("r_hip_roll");
	write_msg.position.push_back(0);
	write_msg.name.push_back("l_ank_roll");
	write_msg.position.push_back(0);
	write_msg.name.push_back("r_ank_roll");
	write_msg.position.push_back(0);
	
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(posiciones2[ult_pos][0]);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(posiciones2[ult_pos][1]);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(posiciones2[ult_pos][2] + rest_inc);
	
	/*write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(-0.6184);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(-1.1420);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(0.3388);*/
	
	/////////////////////////////////////////////write_joint_pub.publish(write_msg);

//////////////////////////////////////////////// Levantarse ////////////////////////////////////////////////

	

	
//////////////////////////////////////////////// Marcha en su lugar con giro ////////////////////////////////////////////////

	/*ros::Duration(1).sleep();
	
	t_ref_ang = 45;
	t_ref = t_ref_ang * (0.0174532925);

	while(act_val < abs(t_ref)){
	
		if (r_ref > 0){
	 
			//Levantar pie derecho 
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("r_ank_pitch");
			write_msg.position.push_back(-0.7091);
			write_msg.name.push_back("r_knee");
			write_msg.position.push_back(-1.4131);
			write_msg.name.push_back("r_hip_pitch");
			write_msg.position.push_back(0.7091);
			write_msg.name.push_back("r_hip_yaw");
			write_msg.position.push_back(0.1746);
			write_joint_pub.publish(write_msg);

			//Bajar pie derecho
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("r_ank_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][0]);
			write_msg.name.push_back("r_knee");
			write_msg.position.push_back(posiciones2[ult_pos][1]);
			write_msg.name.push_back("r_hip_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][2]);
			write_joint_pub.publish(write_msg);
			
			//Levantar pie izquierdo
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("l_ank_pitch");
			write_msg.position.push_back(0.7091);
			write_msg.name.push_back("l_knee");
			write_msg.position.push_back(1.4131);
			write_msg.name.push_back("l_hip_pitch");
			write_msg.position.push_back(-0.7091);
			write_msg.name.push_back("r_hip_yaw");
			write_msg.position.push_back(0);
			write_joint_pub.publish(write_msg);
			
			//Bajar pie izquierdo
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("l_ank_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][3]);
			write_msg.name.push_back("l_knee");
			write_msg.position.push_back(posiciones2[ult_pos][4]);
			write_msg.name.push_back("l_hip_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][5]);
			write_joint_pub.publish(write_msg);
			
			act_val += 0.1746;
		} else {
	 
			//Levantar pie derecho 
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("r_ank_pitch");
			write_msg.position.push_back(-0.7091);
			write_msg.name.push_back("r_knee");
			write_msg.position.push_back(-1.4131);
			write_msg.name.push_back("r_hip_pitch");
			write_msg.position.push_back(0.7091);
			write_msg.name.push_back("l_hip_yaw");
			write_msg.position.push_back(0);
			write_joint_pub.publish(write_msg);

			//Bajar pie derecho
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("r_ank_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][0]);
			write_msg.name.push_back("r_knee");
			write_msg.position.push_back(posiciones2[ult_pos][1]);
			write_msg.name.push_back("r_hip_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][2]);
			write_joint_pub.publish(write_msg);
			
			//Levantar pie izquierdo
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("l_ank_pitch");
			write_msg.position.push_back(0.7091);
			write_msg.name.push_back("l_knee");
			write_msg.position.push_back(1.4131);
			write_msg.name.push_back("l_hip_pitch");
			write_msg.position.push_back(-0.7091);
			write_msg.name.push_back("l_hip_yaw");
			write_msg.position.push_back(-0.1746);
			write_joint_pub.publish(write_msg);
			
			//Bajar pie izquierdo
			ros::Duration(0.1).sleep();
			write_msg.name.push_back("l_ank_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][3]);
			write_msg.name.push_back("l_knee");
			write_msg.position.push_back(posiciones2[ult_pos][4]);
			write_msg.name.push_back("l_hip_pitch");
			write_msg.position.push_back(posiciones2[ult_pos][5]);
			write_joint_pub.publish(write_msg);
			
			act_val += 0.1746;
			
		}
	}
	
	act_val = 0;
	
    
    /*ros::Duration(1).sleep();
    ros::Rate loop_rate_sentarse(60);
    
    for (int fila2=0; fila2<row2; fila2++){
	write_msg.name.push_back("r_ank_pitch");
	write_msg.position.push_back(posiciones2[fila2][0]);
	write_msg.name.push_back("r_knee");
	write_msg.position.push_back(posiciones2[fila2][1]);
	write_msg.name.push_back("r_hip_pitch");
	write_msg.position.push_back(posiciones2[fila2][2]);
	write_msg.name.push_back("l_ank_pitch");
	write_msg.position.push_back(posiciones2[fila2][3]);
	write_msg.name.push_back("l_knee");
	write_msg.position.push_back(posiciones2[fila2][4]);
	write_msg.name.push_back("l_hip_pitch");
	write_msg.position.push_back(posiciones2[fila2][5]);
     	write_joint_pub.publish(write_msg);
     	 
     	loop_rate_sentarse.sleep();
    
    }
    
    //Baile :)
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
     	 write_joint_pub.publish(write_msg);
     	 
     	 loop_rate_dance.sleep();
    
    }*/
  }
  return 0;
}

void CallBack(const sensor_msgs::JointState& posicion){
	r_hip_pitch = posicion.position[14];
	l_hip_pitch = posicion.position[5];
	r_knee = posicion.position[17];
	l_knee = posicion.position[8];
	r_ank_pitch = posicion.position[11];
	l_ank_pitch = posicion.position[2];
	
	ROS_INFO("esperando... y viendo...");
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
  // Wait for Gazebo GUI show up.
  ros::Duration(5).sleep();
  return true;
}