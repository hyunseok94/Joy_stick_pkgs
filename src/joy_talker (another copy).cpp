#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

#define OFF 0
#define ON 1

double v_x, v_y, v_z;
double w_x, w_y, w_z;
double max_x, max_y, max_z;
double min_x, min_y, min_z;
double des_x, des_y, des_z;
double des_pitch, des_roll, des_yaw;

int operation=OFF;
//std_msgs::Float64MultiArray stick_msg0;
//std_msgs::Float64MultiArray button_msg0;
//std_msgs::Float64MultiArray stick_msg1;
//std_msgs::Float64MultiArray button_msg1;

std_msgs::Float64MultiArray joy0_msg;
std_msgs::Float64MultiArray joy1_msg;

void Callback0(const sensor_msgs::Joy &msg)
{
    v_x=msg.axes[0];
    v_y=msg.axes[1];
    v_z=msg.axes[2];
    w_x=msg.axes[4];
    w_y=msg.axes[5];
    w_z=msg.axes[3];
    
    joy0_msg.data[1]=msg.axes[0];      //v_x
  joy0_msg.data[2]=msg.axes[1];      //v_y
  joy0_msg.data[3]=msg.axes[2];      //v_z
  joy0_msg.data[4]=msg.axes[4];      //w_x(roll)
  joy0_msg.data[5]=msg.axes[5];      //w_y(pitch)
  joy0_msg.data[6]=msg.axes[3];      //w_z(yaw)
 
  joy0_msg.data[1]=msg.buttons[0];   //button1
  joy0_msg.data[2]=msg.buttons[1];   //button2
  joy0_msg.data[3]=msg.buttons[2];   //button3
  joy0_msg.data[4]=msg.buttons[3];   //button4
  joy0_msg.data[5]=msg.buttons[4];   //button5
  joy0_msg.data[6]=msg.buttons[5];   //button6
  joy0_msg.data[7]=msg.buttons[6];   //button7
}

void Callback1(const sensor_msgs::Joy &msg)
{
  joy1_msg.data[1]=msg.axes[0];      //v_x
  joy1_msg.data[2]=msg.axes[1];      //v_y
  joy1_msg.data[3]=msg.axes[2];      //v_z
  joy1_msg.data[4]=msg.axes[4];      //w_x(roll)
  joy1_msg.data[5]=msg.axes[5];      //w_y(pitch)
  joy1_msg.data[6]=msg.axes[3];      //w_z(yaw)
 
  joy1_msg.data[1]=msg.buttons[0];   //button1
  joy1_msg.data[2]=msg.buttons[1];   //button2
  joy1_msg.data[3]=msg.buttons[2];   //button3
  joy1_msg.data[4]=msg.buttons[3];   //button4
  joy1_msg.data[5]=msg.buttons[4];   //button5
  joy1_msg.data[6]=msg.buttons[5];   //button6
  joy1_msg.data[7]=msg.buttons[6];   //button7
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_talker"); //node name
  ros::NodeHandle nh;
  nh.setParam("operation", OFF);
    
    v_x = 0;
    v_y = 0;
    v_z = 0;
    w_x = 0;
    w_y = 0;
    w_z = 0 
            
    max_x = 2.58;
    min_x = 1.88;
    max_y = 1.2;
    min_y = -1.2;
    max_z = 1.2;
    min_z = -0.679;
    
    des_x = 0;
    des_y = 0;
    des_z = 0;
    des_roll = 0;
    des_pitch = 0;
    des_yaw = 0;    
  //Set array size
//  stick_msg0.data.resize(5);
//  button_msg0.data.resize(8);
//
//  stick_msg1.data.resize(5);
//  button_msg1.data.resize(8);
    joy0_msg.data.resize(13);
    joy1_msg.data.resize(13);
    
  //Publish stick & button message
  ros::Subscriber S_instruction0 = nh.subscribe("j0", 1000, &Callback0);
  ros::Subscriber S_instruction1 = nh.subscribe("j1", 1000, &Callback1);
//  ros::Publisher P_stick_instruction0 = nh.advertise<std_msgs::Float64MultiArray>("stick_msg0", 1000); //message name
//  ros::Publisher P_button_instruction0 = nh.advertise<std_msgs::Float64MultiArray>("button_msg0", 1000);
  ros::Publisher P_joy0_instruction = nh.advertise<std_msgs::Float64MultiArray>("joy0_msg", 1000); //message name
//  ros::Publisher P_stick_instruction1 = nh.advertise<std_msgs::Float64MultiArray>("stick_msg1", 1000); //message name
//  ros::Publisher P_button_instruction1 = nh.advertise<std_msgs::Float64MultiArray>("button_msg1", 1000);
  ros::Publisher P_joy1_instruction = nh.advertise<std_msgs::Float64MultiArray>("joy1_msg", 1000); //message name
  ros::Rate loop_rate(100);

  //Loop
  while (ros::ok())   //if operate "Ctrl+C", it becomes false
  {
    //nh.getParam("operation", operation);
    //if(operation==ON){
      //P_stick_instruction0.publish(stick_msg0);
      //P_button_instruction0.publish(button_msg0);
      P_joy0_instruction.publish(joy0_msg);

      //P_stick_instruction1.publish(stick_msg1);
      //P_button_instruction1.publish(button_msg1);
      P_joy1_instruction.publish(joy1_msg);
      ros::spinOnce();
      loop_rate.sleep();
    //}
  }
  return 0;
}
