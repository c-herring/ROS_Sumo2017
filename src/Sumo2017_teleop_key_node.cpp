#include "ros/ros.h"
#include <ros/console.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>


struct termios cooked, raw;
int kfd;
bool batman;

class Sumo2017TeleopKey {
public:
	// Public functions
	Sumo2017TeleopKey();
	void spin(void);
	void keyLoop();

private:

	// Private functions
	//void ReadIRSensors();
	//void ReadLineSensors();
	void PublishMotorTwist(void);


	// Callbacks
	void IR_sensor_Callback		(const std_msgs::UInt32MultiArray msg);
	void Line_sensor_Callback	(const std_msgs::UInt16MultiArray msg);

	// ROS nodehandle. Main point of  for all ROS shennanigans
	ros::NodeHandle nh;

	// Publishers
	ros::Publisher vel_set_pub;

	// Subscribers
	ros::Subscriber IR_sensor_sub;
	ros::Subscriber Line_sensor_sub;

  float vel;



};

/**
** Constructor
**/
Sumo2017TeleopKey::Sumo2017TeleopKey()
{
	// Get parameters
	// --- No params
  vel = 1.0;

	// Attach Advertisers
	vel_set_pub 		= nh.advertise<geometry_msgs::Twist>("vel_set", 1);

	// Attach Subscribers
	IR_sensor_sub 		= nh.subscribe<std_msgs::UInt32MultiArray>("IR_sensor", 1, &Sumo2017TeleopKey::IR_sensor_Callback, this);
	Line_sensor_sub 	= nh.subscribe<std_msgs::UInt16MultiArray>("Line_sensor", 1, &Sumo2017TeleopKey::Line_sensor_Callback, this);
}

/**
** 	Called once every ros spin. Do logic in here.
**/
void Sumo2017TeleopKey::spin()
{
	//std::cout << "spinning" << std::endl;
	PublishMotorTwist();
}

/**
** 	Callback for the IR sensor message
**/
void Sumo2017TeleopKey::IR_sensor_Callback(const std_msgs::UInt32MultiArray msg)
{
	//std::cout << "Got an IR sensor message" << std::endl;
}

/**
** 	Callback for the Line sensor message
**/
void Sumo2017TeleopKey::Line_sensor_Callback(const std_msgs::UInt16MultiArray msg)
{

	//std::cout << "Got a Line sensor message" << std::endl;
}

void Sumo2017TeleopKey::PublishMotorTwist()
{
	//std::cout << "Publishing a motor twist" << std::endl;
}



#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
void Sumo2017TeleopKey::keyLoop()
{  
  kfd = 0;
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");



    // get the next event from the keyboard  
  if(read(kfd, &c, 1) < 0)
  {
    perror("read():");
    exit(-1);
  }

  ROS_DEBUG("value: 0x%02X\n", c);
  
  geometry_msgs::Twist twist_msg;

  switch(c)
  {
    case KEYCODE_L:
      ROS_DEBUG("LEFT");
      std::cout << "Left at: " << vel << std::endl;
      twist_msg.linear.x = -vel;
      twist_msg.linear.y = vel;
      dirty = true;
      break;
    case KEYCODE_R:
      ROS_DEBUG("RIGHT");
      std::cout << "Right at: " << vel << std::endl;
      twist_msg.linear.x = vel;
      twist_msg.linear.y = -vel;
      dirty = true;
      break;
    case KEYCODE_U:
      ROS_DEBUG("UP");
      std::cout << "Fwd at: " << vel << std::endl;
      twist_msg.linear.x = vel;
      twist_msg.linear.y = vel;
      dirty = true;
      break;
    case KEYCODE_D:
      ROS_DEBUG("DOWN");
      std::cout << "Back at: " << vel << std::endl;
      twist_msg.linear.x = -vel;
      twist_msg.linear.y = -vel;
      dirty = true;
      break;
    case '+':
      ROS_DEBUG("+");
      std::cout << "Vel set to " << vel << std::endl;
      vel += 0.1;
      break;
    case '-':
      ROS_DEBUG("-");
      std::cout << "Vel set to " << vel << std::endl;
      vel -= 0.1;
      break;
    case 'q':
      batman = true;
      break;
  }
  if (vel < -1.0)     vel = -1.0;
  else if(vel > 1.0)  vel = 1.0;
   

  if(dirty ==true)
  {
    vel_set_pub.publish(twist_msg) ; 
    dirty=false;
  }
  return;
}

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}



/**
** 	Main node entry point
**/
int main(int argc, char** argv)
{
  batman = false;
  
	// Intiialise the ros node.
	ros::init(argc, argv, "Sumo2017_node");

	// Instantiate the class
	Sumo2017TeleopKey sumo2017;
  signal(SIGINT,quit);

	std::cout << "Starting Sumo2017 main control node.\n";

	// 50ms loop rate
	ros::Rate loop_rate(50);
	while (ros::ok() & !batman)
	{
		sumo2017.keyLoop();
		// Spin it the logic
		sumo2017.spin();

		ros::spinOnce();
		loop_rate.sleep();

	}
}
