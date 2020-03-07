#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

//using namespace std;

//Variables of Joy 0
double joy0_v_x, joy0_v_y, joy0_v_z;
double joy0_w_x, joy0_w_y, joy0_w_z;

//Variables of Joy 1
double joy1_v_x, joy1_v_y, joy1_v_z;
double joy1_w_x, joy1_w_y, joy1_w_z;

//VectorXd target_vel = VectorXd::Zero(13);
double target_vel[12];

//Joy message
std_msgs::Float64MultiArray joy0_msg;
std_msgs::Float64MultiArray joy1_msg;
std_msgs::Float64MultiArray tmp_msg;

//Ros time
ros::Time last_time;

//Define Parabola functions
void joy0_Parabola_signal(void);
void joy1_Parabola_signal(void);

void Callback0(const sensor_msgs::Joy &msg)
{
    joy0_v_x = msg.axes[1];
    joy0_v_y = msg.axes[0];
    joy0_v_z = msg.axes[2];
    joy0_w_x = msg.axes[4];
    joy0_w_y = msg.axes[5];
    joy0_w_z = msg.axes[3];
    joy0_msg.data[6] = msg.buttons[0]; //button1
    joy0_msg.data[7] = msg.buttons[1]; //button2
    joy0_msg.data[8] = msg.buttons[2]; //button3
    joy0_msg.data[9] = msg.buttons[3]; //button4
    joy0_msg.data[10] = msg.buttons[4]; //button5
    joy0_msg.data[11] = msg.buttons[5]; //button6
    joy0_msg.data[12] = msg.buttons[6]; //button7
}

void Callback1(const sensor_msgs::Joy &msg)
{
    joy1_v_x = msg.axes[1];
    joy1_v_y = msg.axes[0];
    joy1_v_z = msg.axes[2];
    joy1_w_x = msg.axes[4];
    joy1_w_y = msg.axes[5];
    joy1_w_z = msg.axes[3];
    joy1_msg.data[6] = msg.buttons[0]; //button1
    joy1_msg.data[7] = msg.buttons[1]; //button2
    joy1_msg.data[8] = msg.buttons[2]; //button3
    joy1_msg.data[9] = msg.buttons[3]; //button4
    joy1_msg.data[10] = msg.buttons[4]; //button5
    joy1_msg.data[11] = msg.buttons[5]; //button6
    joy1_msg.data[12] = msg.buttons[6]; //button7
}

int main(int argc, char **argv)
{
    //Node init
    ros::init(argc, argv, "joy_talker");
    ros::NodeHandle nh;

    //Define initial values
    joy0_v_x = 0;
    joy0_v_y = 0;
    joy0_v_z = 0;
    joy0_w_x = 0;
    joy0_w_y = 0;
    joy0_w_z = 0;

    joy1_v_x = 0;
    joy1_v_y = 0;
    joy1_v_z = 0;
    joy1_w_x = 0;
    joy1_w_y = 0;
    joy1_w_z = 0;

    //Resize Message
    joy0_msg.data.resize(13);
    joy1_msg.data.resize(13);
    tmp_msg.data.resize(20);

    //Setting for subscribing joy stick messages from the original package
    ros::Subscriber S_joy0_instruction = nh.subscribe("j0", 1000, &Callback0);
    ros::Subscriber S_joy1_instruction = nh.subscribe("j1", 1000, &Callback1);

    //Setting for publishing Our own joy stick messages
    ros::Publisher P_joy0_instruction = nh.advertise<std_msgs::Float64MultiArray>("joy0_msg", 1000);
    ros::Publisher P_joy1_instruction = nh.advertise<std_msgs::Float64MultiArray>("joy1_msg", 1000);
    ros::Publisher P_tmp_data = nh.advertise<std_msgs::Float64MultiArray>("tmp_data", 1000);

    //Loop rate
    ros::Rate loop_rate(1000);

    //Loop
    while (ros::ok()) //if operate "Ctrl+C", it becomes false
    {
        joy0_Parabola_signal();
        joy1_Parabola_signal();

        //Put above calculated data into messages
        joy0_msg.data[0] = target_vel[0]; // x axis's velocity of joy0
        joy0_msg.data[1] = target_vel[1]; // y axis's veclocity of joy0
        joy0_msg.data[2] = target_vel[2]; // z axis's velocity of joy0
        joy0_msg.data[3] = target_vel[3]; // roll axis's velocity of joy0
        joy0_msg.data[4] = target_vel[4]; // pitch axis's velcity of joy0
        joy0_msg.data[5] = target_vel[5]; // yaw axis's velocity of joy0

        joy1_msg.data[0] = target_vel[6]; // x axis's velocity of joy1
        joy1_msg.data[1] = target_vel[7]; // y axis's velocity of joy1
        joy1_msg.data[2] = target_vel[8]; // z axis's velocity of joy1
        joy1_msg.data[3] = target_vel[9]; // roll axis's velocity of joy1
        joy1_msg.data[4] = target_vel[10]; // pitch axis's velocity of joy1
        joy1_msg.data[5] = target_vel[11]; // yaw axis's velocity of joy1

        //If the mode is "Home Return Mode"(=Button3 is On), All the desired values go to zero.
        if (joy0_msg.data[8] == 1) {
            target_vel[0] = 0;
            target_vel[1] = 0;
            target_vel[2] = 0;
            target_vel[3] = 0;
            target_vel[4] = 0;
            target_vel[5] = 0;
        }

        if (joy1_msg.data[8] == 1) {
            target_vel[6] = 0;
            target_vel[7] = 0;
            target_vel[8] = 0;
            target_vel[9] = 0;
            target_vel[10] = 0;
            target_vel[11] = 0;
        }

        // ROS message
        tmp_msg.data[0] = target_vel[0];
        tmp_msg.data[1] = target_vel[1];
        tmp_msg.data[2] = target_vel[2];
        tmp_msg.data[3] = target_vel[3];
        tmp_msg.data[4] = target_vel[4];
        tmp_msg.data[5] = target_vel[5];
        tmp_msg.data[6] = target_vel[6];
        tmp_msg.data[7] = target_vel[7];
        tmp_msg.data[8] = target_vel[8];
        tmp_msg.data[9] = target_vel[9];
        tmp_msg.data[10] = target_vel[10];
        tmp_msg.data[11] = target_vel[11];

        //Print out
        std::cout<<"joy"<<joy0_v_x<<std::endl;
        std::cout<<"target"<<target_vel[0]<<std::endl;
        std::cout<<"A"<<std::endl;
        std::cout<<"_____________________________"<<std::endl;

        //Publish joy stick messages
        P_joy0_instruction.publish(joy0_msg);
        P_joy1_instruction.publish(joy1_msg);

        //RQT message
        P_tmp_data.publish(tmp_msg);

        //Functions responsible to handle communication events(e.g arriving messages).
        ros::spinOnce();
        loop_rate.sleep();

        //last_time=current_time;
    }
    return 0;
}

void joy0_Parabola_signal(void)
{
    if (joy0_v_x > 0.0) {
        target_vel[0] = target_vel[0] + 0.001;

        if (target_vel[0] >= 1.0) {
            target_vel[0] = 1.0;
        }
    }
    else if (joy0_v_x == 0.0) {
        if (target_vel[0] > 0.0) {
            target_vel[0] = target_vel[0] - 0.001;
        }
        else if(abs(target_vel[0])<=0.001){
            target_vel[0] = 0.0;
        }
        else if (target_vel[0] < 0.0) {
            target_vel[0] = target_vel[0] + 0.001;
        }
    }
    else {
        target_vel[0] = target_vel[0] - 0.001;
        if (target_vel[0] <= -1.0) {
            target_vel[0] = -1.0;
        }
    }

    if (joy0_v_y > 0.0) {
        target_vel[1] = target_vel[1] + 0.001;

        if (target_vel[1] >= 1.0) {
            target_vel[1] = 1.0;
        }
    }
    else if (joy0_v_y == 0.0) {
        if (target_vel[1] > 0.0) {
            target_vel[1] = target_vel[1] - 0.001;
        }
        else if(abs(target_vel[1])<=0.001){
            target_vel[1] = 0.0;
        }
        else if (target_vel[1] < 0.0) {
            target_vel[1] = target_vel[1] + 0.001;
        }
    }
    else {
        target_vel[1] = target_vel[1] - 0.001;
        if (target_vel[1] <= -1.0) {
            target_vel[1] = -1.0;
        }
    }

    if (joy0_v_z > 0.0) {
        target_vel[2] = target_vel[2] + 0.001;
        if (target_vel[2] >= 1.0) {
            target_vel[2] = 1.0;
        }
    }
    else if (joy0_v_z == 0.0) {
        if (target_vel[2] > 0.0) {
            target_vel[2] = target_vel[2] - 0.001;
        }
        else if(abs(target_vel[2])<=0.001){
            target_vel[2] = 0.0;
        }
        else if (target_vel[2] < 0.0) {
            target_vel[2] = target_vel[2] + 0.001;
        }
    }
    else {
        target_vel[2] = target_vel[2] - 0.001;
        if (target_vel[2] <= -1.0) {
            target_vel[2] = -1.0;
        }
    }

    //**********************POS & ORI Boundary*************************//
    if (joy0_w_x > 0.0) {
        target_vel[3] = target_vel[3] + 0.001;
        if (target_vel[3] >= 1.0) {
            target_vel[3] = 1.0;
        }
    }
    else if (joy0_w_x == 0.0) {
        if (target_vel[3] > 0.0) {
            target_vel[3] = target_vel[3] - 0.001;
        }
        else if(abs(target_vel[3])<=0.001){
            target_vel[3] = 0.0;
        }
        else if (target_vel[3] < 0.0) {
            target_vel[3] = target_vel[3] + 0.001;
        }
    }
    else {
        target_vel[3] = target_vel[3] - 0.001;
        if (target_vel[3] <= -1.0) {
            target_vel[3] = -1.0;
        }
    }

    if (joy0_w_y > 0.0) {
        target_vel[4] = target_vel[4] + 0.001;

        if (target_vel[4] >= 1.0) {
            target_vel[4] = 1.0;
        }
    }
    else if (joy0_w_y == 0.0) {
        if (target_vel[4] > 0.0) {
            target_vel[4] = target_vel[4] - 0.001;
        }
        else if(abs(target_vel[4])<=0.001){
            target_vel[4] = 0.0;
        }
        else if (target_vel[4] < 0.0) {
            target_vel[4] = target_vel[4] + 0.001;
        }
    }
    else {
        target_vel[4] = target_vel[4] - 0.001;
        if (target_vel[4] <= -1.0) {
            target_vel[4] = -1.0;
        }
    }

    if (joy0_w_z > 0.0) {
        target_vel[5] = target_vel[5] + 0.001;

        if (target_vel[5] >= 1.0) {
            target_vel[5] = 1.0;
        }
    }
    else if (joy0_w_z == 0.0) {
        if (target_vel[5] > 0.0) {
            target_vel[5] = target_vel[5] - 0.001;
        }
        else if(abs(target_vel[5])<=0.001){
            target_vel[5] = 0.0;
        }
        else if (target_vel[5] < 0.0) {
            target_vel[5] = target_vel[5] + 0.001;
        }
    }
    else {
        target_vel[5] = target_vel[5] - 0.001;
        if (target_vel[5] <= -1.0) {
            target_vel[5] = -1.0;
        }
    }
}

void joy1_Parabola_signal(void)
{
    if (joy1_v_x > 0.0) {
        target_vel[6] = target_vel[6] + 0.001;

        if (target_vel[6] >= 1.0) {
            target_vel[6] = 1.0;
        }
    }
    else if (joy1_v_x == 0.0) {
        if (target_vel[6] > 0.0) {
            target_vel[6] = target_vel[6] - 0.001;
        }
        else if(abs(target_vel[6])<=0.001){
            target_vel[6] = 0.0;
        }
        else if (target_vel[6] < 0.0) {
            target_vel[6] = target_vel[6] + 0.001;
        }
    }
    else {
        target_vel[6] = target_vel[6] - 0.001;
        if (target_vel[6] <= -1.0) {
            target_vel[6] = -1.0;
        }
    }

    if (joy1_v_y > 0.0) {
        target_vel[7] = target_vel[7] + 0.001;

        if (target_vel[7] >= 1.0) {
            target_vel[7] = 1.0;
        }
    }
    else if (joy1_v_y == 0.0) {
        if (target_vel[7] > 0.0) {
            target_vel[7] = target_vel[7] - 0.001;
        }
        else if(abs(target_vel[7])<=0.001){
            target_vel[7] = 0.0;
        }
        else if (target_vel[7] < 0.0) {
            target_vel[7] = target_vel[7] + 0.001;
        }
    }
    else {
        target_vel[7] = target_vel[7] - 0.001;
        if (target_vel[7] <= -1.0) {
            target_vel[7] = -1.0;
        }
    }

    if (joy1_v_z > 0.0) {
        target_vel[8] = target_vel[8] + 0.001;

        if (target_vel[8] >= 1.0) {
            target_vel[8] = 1.0;
        }
    }
    else if (joy1_v_z == 0.0) {
        if (target_vel[8] > 0.0) {
            target_vel[8] = target_vel[8] - 0.001;
        }
        else if(abs(target_vel[8])<=0.001){
            target_vel[8] = 0.0;
        }
        else if (target_vel[8] < 0.0) {
            target_vel[8] = target_vel[8] + 0.001;
        }
    }
    else {
        target_vel[8] = target_vel[8] - 0.001;
        if (target_vel[8] <= -1.0) {
            target_vel[8] = -1.0;
        }
    }
    //**********************POS & ORI Boundary*************************//
    if (joy1_w_x > 0.0) {
        target_vel[9] = target_vel[9] + 0.001;

        if (target_vel[9] >= 1.0) {
            target_vel[9] = 1.0;
        }
    }
    else if (joy1_w_x == 0.0) {
        if (target_vel[9] > 0.0) {
            target_vel[9] = target_vel[9] - 0.001;
        }
        else if(abs(target_vel[9])<=0.001){
            target_vel[9] = 0.0;
        }
        else if (target_vel[9] < 0.0) {
            target_vel[9] = target_vel[9] + 0.001;
        }
    }
    else {
        target_vel[9] = target_vel[9] - 0.001;
        if (target_vel[9] <= -1.0) {
            target_vel[9] = -1.0;
        }
    }

    if (joy1_w_y > 0.0) {
        target_vel[10] = target_vel[10] + 0.001;

        if (target_vel[10] >= 1.0) {
            target_vel[10] = 1.0;
        }
    }
    else if (joy1_w_y == 0.0) {
        if (target_vel[10] > 0.0) {
            target_vel[10] = target_vel[10] - 0.001;
        }
        else if(abs(target_vel[10])<=0.001){
            target_vel[10] = 0.0;
        }
        else if (target_vel[10] < 0.0) {
            target_vel[10] = target_vel[10] + 0.001;
        }
    }
    else {
        target_vel[10] = target_vel[10] - 0.001;
        if (target_vel[10] <= -1.0) {
            target_vel[10] = -1.0;
        }
    }

    if (joy1_w_z > 0.0) {
        target_vel[11] = target_vel[11] + 0.001;

        if (target_vel[11] >= 1.0) {
            target_vel[11] = 1.0;
        }
    }
    else if (joy1_w_z == 0.0) {
        if (target_vel[11] > 0.0) {
            target_vel[11] = target_vel[11] - 0.001;
        }
        else if(abs(target_vel[11])<=0.001){
            target_vel[11] = 0.0;
        }
        else if (target_vel[11] < 0.0) {
            target_vel[11] = target_vel[11] + 0.001;
        }
    }
    else {
        target_vel[11] = target_vel[11] - 0.001;
        if (target_vel[11] <= -1.0) {
            target_vel[11] = -1.0;
        }
    }
}
