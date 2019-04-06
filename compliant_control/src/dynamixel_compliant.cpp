#include "dynamixel_sdk.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <math.h>

#define ADDR_MX_TORQUE_ENABLE           64                  // Control table address is different in Dynamixel model
#define ADDR_MX_DRIVE_MODE              11
#define ADDR_MX_GOAL_VELOCITY           104
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PRESENT_POSITION        132
#define ADDR_MX_GOAL_CURRENT            102

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default settings
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller

// Values to send to change settings
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define VELOCITY_CTRL_MODE              1                   // Value to set drive mode to wheel mode
#define POSITION_CTRL_MODE              3                   // Value to set drive mode to JOINT mode
#define CURRENT_CTRL_MODE               0                   // Value to set drive mode to TORQUE mode

uint16_t max_torque;
int goal;
int err;
double K;
double I;
double D;

uint16_t pos;
ros::Time p_time;
float p_err = 0;
float tot_err = 0;

class Dynamixel {
private:
  ros::Subscriber goal_pos_sub ;

public:
  Dynamixel();
  void posCallback(const std_msgs::Int64ConstPtr& pos);
  ros::NodeHandle nh;
};

Dynamixel::Dynamixel() {
	//create publisher for motor commands
  goal_pos_sub = nh.subscribe("goal_pos", 1, &Dynamixel::posCallback, this);

};

//Publishes raw position value to ROS
void Dynamixel::posCallback(const std_msgs::Int64ConstPtr& pos) {
  goal = pos->data;
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"dxl_torque_test");
  Dynamixel d;
  d.nh.param("K", K, 10.0);
  d.nh.param("D", D, 2.0);
  d.nh.param("I", I, 0.0);

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  //general variables
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0; // Dynamixel error

  // Open port
  portHandler->openPort();
  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE);
  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  // Change Operating Mode to Joint
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_DRIVE_MODE, POSITION_CTRL_MODE, &dxl_error);
  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  //Set position to 0
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, 512+135, &dxl_error);
  ros::Duration(1).sleep();
  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  // Change Operating Mode to torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_DRIVE_MODE, CURRENT_CTRL_MODE, &dxl_error);
  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  while(ros::ok() && dxl_error == 0)
  {
    // Read present position
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &pos, &dxl_error);
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, 38, &max_torque, &dxl_error);

    err = pos - goal;

    if(!p_time.toSec())
    {
        p_time = ros::Time::now();
    }

    ros::Time time = ros::Time::now();

    //float dt = time.toSec() - p_time.toSec();

    //tot_err += err*dt;

    //float d_err = (err - p_err)/dt;

    uint16_t tau = floor(K*err); //+ D*d_err + I*tot_err; //

    if(abs(tau)>max_torque)
    {
      tau = (tau)/abs(tau)*max_torque;
    }

    p_err = err;
    p_time = time;

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_CURRENT, tau, &dxl_error);

    ros::spinOnce();
  }
}
