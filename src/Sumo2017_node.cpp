#include "ros/ros.h"
#include <ros/console.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>

#define IR_N 	0
#define IR_NE 	1
#define IR_E 	2
#define IR_SE 	3
#define IR_S 	4
#define IR_SW 	5
#define IR_W 	6
#define IR_NW 	7

#define LINE_NE 0
#define	LINE_SE 1
#define LINE_SW 2
#define LINE_NW 3

#define IR_TriggerThreshold 	300
#define Line_TriggerThreshold 	600

class Sumo2017 {
public:
	// Public functions
	Sumo2017();
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

	// ROS nodehandle. Main point of access for all ROS shennanigans
	ros::NodeHandle nh;

	// Publishers
	ros::Publisher vel_set_pub;

	// Subscribers
	ros::Subscriber IR_sensor_sub;
	ros::Subscriber Line_sensor_sub;


	// IR sensor state variables
	uint8_t 	triggered_IR;
	uint32_t 	triggeredVal_IR;
	uint8_t 	lastTriggered_IR;
	uint32_t 	lastTriggeredVal_IR;

	// Line Sensor State Variable
	uint8_t 	lineSensorState;
	uint8_t 	lastLineSensorState;

	uint8_t 	defaultSeekDir;



};

/**
** Constructor
**/
Sumo2017::Sumo2017()
{
	// Get parameters
	// --- No params

	defaultSeekDir = 1;
	triggered_IR = -1;
	lastTriggered_IR = -1;
	lineSensorState = 0;
	lastLineSensorState = 0;


	// Attach Advertisers
	vel_set_pub 		= nh.advertise<geometry_msgs::Twist>("vel_set", 1);

	// Attach Subscribers
	IR_sensor_sub 		= nh.subscribe<std_msgs::UInt32MultiArray>("IR_sensor", 1, &Sumo2017::IR_sensor_Callback, this);
	Line_sensor_sub 	= nh.subscribe<std_msgs::UInt16MultiArray>("Line_sensor", 1, &Sumo2017::Line_sensor_Callback, this);
}

/**
** 	Called once every ros spin. Do logic in here.
**/
void Sumo2017::spin()
{
	std::cout << "spinning" << std::endl;

	geometry_msgs::Twist twist_msg;

	// Check line sensor state
	if (lineSensorState > 0)
	{
		// Handle line sensor stuff

		// If it was one of the north sensors then set default seek direction to be reverse
		if ( (lineSensorState >> LINE_NE) & 0x01 | (lineSensorState >> LINE_NW) & 0x01)
		{
			defaultSeekDir = -1
		}
		else
		{
			defaultSeekDir = 1;
		}
	}
	else
	{
		// Otherwise drive from IR sensors
		switch (triggered_IR)
		{
			case IR_N:
				break;
			case IR_NE:
				break;
			case IR_E:
				break;
			case IR_SE:
				break;
			case IR_S:
				break;
			case IR_SW:
				break;
			case IR_W:
				break;
			case IR_NW:
				break;
		}

		//If north sensor then drive forward
			// If north sensor above attack threshold then drive at attack speed, otherwise drive at seek speed
		
		// else if south sensor, do same as north sensor but backwards

		// If East sensor then we turn hard in direction based on criteria:
			// If last sensor was North/Northeast then CCW
			// If lst sensor was South/Southeast then CW
			// Default CW
		
		// Same deal with West sensor

		// If NE or SW then turn CCW

		// If NW or SE then turn CW

		// Default: forward or backwards in the current default direction

	}

	PublishMotorTwist();
}

/**
** 	Callback for the IR sensor message
**/
void Sumo2017::IR_sensor_Callback(const std_msgs::UInt32MultiArray msg)
{
	std::cout << "Got an IR sensor message" << std::endl;
	uint32_t tempMax = 0;
	uint32_t tempMaxIndex = -1;
	// Search for maximum value in sensor values (as long as it is above the detection threshold)
	int i;
	for (i =0; i < 8; i++)
	{
		if (msg.data[i] > IR_TriggerThreshold & msg.data[i] > tempMax)
		{
			tempMax = msg.data[i];
			tempMaxIndex = i;
		}
	}
	// If we found one (ie index is not -1) then update the triggered and last triggered states.
	if (tempMaxIndex >= 0)
	{
		lastTriggered_IR = triggered_IT;
		lastTriggeredVal_IR = triggeredVal_IR;
		triggered_IR = i;
		triggeredVal_IR = msg.data[i];
	}
}

/**
** 	Callback for the Line sensor message
**/
void Sumo2017::Line_sensor_Callback(const std_msgs::UInt16MultiArray msg)
{
	std::cout << "Got a Line sensor message" << std::endl;

	uint8_t tempState = 0;
	// Set the bit for each of the line sensors, 1 if they exceede the trigger and 0 if not.
	for (int i = 0; i < 4; i++)
	{
		tempState |= (msg.data[i] > Line_TriggerThreshold) ? 1 << i : 0x00;
	}

	// If this state is different to the last one then save it and set the old one to the last state
	if (tempState != lineSensorState)
	{
		lastLineSensorState = lineSensorState;
		lineSensorState = tempState;
	}

}

void Sumo2017::PublishMotorTwist()
{
	std::cout << "Publishing a motor twist" << std::endl;
}


/**
** 	Main node entry point
**/
int main(int argc, char** argv)
{
	// Intiialise the ros node.
	ros::init(argc, argv, "Sumo2017_node");

	// Instantiate the class
	Sumo2017 sumo2017;

	std::cout << "Starting Sumo2017 main control node.\n";

	// 50ms loop rate
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		sumo2017.keyLoop();
		// Spin it the logic
		sumo2017.spin();

		ros::spinOnce();
		loop_rate.sleep();

	}
}
