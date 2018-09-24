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

#define IR_TriggerThreshold 	1200
#define IR_ATTACK_THRESH 		3600
#define Line_TriggerThreshold 	500

#define HARD_TURN_SPEED 	0.8
#define SOFT_TURN_SPEED 	0.5
#define ATTACK_SPEED 		1.0
#define RETREAT_SPEED		0.5
#define SEEK_SPEED 			0.68

class Sumo2017 {
public:
	// Public functions
	Sumo2017();
	void spin(void);

private:

	// Private functions
	//void ReadIRSensors();
	//void ReadLineSensors();
	void PublishMotorTwist(geometry_msgs::Twist twist_msg);


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

	int8_t 	defaultSeekDir;

	bool attackMode;



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

	attackMode = false;

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
	//std::cout << "spinning" << std::endl;

	geometry_msgs::Twist twist_msg;

	// Check line sensor state
	if (lineSensorState > 0)
	{
		// Handle line sensor stuff

		// If it was one of the north sensors then set default seek direction to be reverse
		if ( (lineSensorState >> LINE_NE) & 0x01 | (lineSensorState >> LINE_NW) & 0x01)
		{
			defaultSeekDir = -1;
			std::cout << "I see NE or NW line";
		}
		else
		{
			defaultSeekDir = 1;
			std::cout << "I see SE or SW line";
		}
		// retreat from line
		twist_msg.linear.x = defaultSeekDir * RETREAT_SPEED;
		twist_msg.linear.y = defaultSeekDir * RETREAT_SPEED;
	}
	else
	{
		// Otherwise drive from IR sensors
		switch (triggered_IR)
		{
			case IR_N:
				// If we are in attack mode then gung ho it
				twist_msg.linear.x = (attackMode) ? ATTACK_SPEED : SEEK_SPEED;
				twist_msg.linear.y = (attackMode) ? ATTACK_SPEED : SEEK_SPEED;
				break;

			case IR_S:
				// If we are in attack mode then gung ho it
				twist_msg.linear.x = -((attackMode) ? ATTACK_SPEED : SEEK_SPEED);
				twist_msg.linear.y = -((attackMode) ? ATTACK_SPEED : SEEK_SPEED);
				break;

			case IR_NE:
			case IR_SW:
				//If we saw it on the Northeast or southwest sensor then turn CW
				twist_msg.linear.x =  SOFT_TURN_SPEED;
				twist_msg.linear.y = -SOFT_TURN_SPEED;
				break;

			case IR_E:
				// If east sensor is triggered then turn based on the following criteria:
					// If last sensor was North/Northeast then CCW
					// If lst sensor was South/Southeast then CW
					// Default CW
				twist_msg.linear.x =  HARD_TURN_SPEED;
				twist_msg.linear.y = -HARD_TURN_SPEED;
				break;

			case IR_SE:
			case IR_NW:
				//If we saw it on the northwest or southeast sensor then turn CCW
				twist_msg.linear.x = -SOFT_TURN_SPEED;
				twist_msg.linear.y = SOFT_TURN_SPEED;

			case IR_W:
				
				twist_msg.linear.x = -HARD_TURN_SPEED;
				twist_msg.linear.y = HARD_TURN_SPEED;
				break;

			default:
			// Seek
				twist_msg.linear.x = defaultSeekDir * SEEK_SPEED;
				twist_msg.linear.y = defaultSeekDir * SEEK_SPEED;
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

	std::cout << "Pub x: " << twist_msg.linear.x << " y = " << twist_msg.linear.y << std::endl;
	PublishMotorTwist(twist_msg);
}

/**
** 	Callback for the IR sensor message
**/
void Sumo2017::IR_sensor_Callback(const std_msgs::UInt32MultiArray msg)
{
	//std::cout << "Got an IR sensor message" << std::endl;
	uint32_t tempMax = 0;
	uint32_t tempMaxIndex = -1;
	// Search for maximum value in sensor values (as long as it is above the detection threshold)
	for (int i =0; i < 8; i++)
	{
		if (msg.data[i] > IR_TriggerThreshold & msg.data[i] > tempMax)
		{
			tempMax = msg.data[i];
			tempMaxIndex = i;
		}
	}
	// If we found one (ie index is not -1) then update the triggered and last triggered states.
	//if (tempMaxIndex >= 0)
	//{
	// If this new one is different to last then update the last
	if (lastTriggered_IR != tempMaxIndex)
	{
		lastTriggered_IR = triggered_IR;
		lastTriggeredVal_IR = triggeredVal_IR;
	}
	// Update the sensor value
	triggered_IR = tempMaxIndex;
	if (tempMaxIndex >= 0)
	{
		triggeredVal_IR = msg.data[tempMaxIndex];
	}

	// Check if this puts us in attack mode or not
	if (triggeredVal_IR > IR_ATTACK_THRESH & (triggered_IR == IR_N | triggered_IR == IR_S))
		attackMode = true;
	else
		attackMode = false;
	
	
	
}

/**
** 	Callback for the Line sensor message
**/
void Sumo2017::Line_sensor_Callback(const std_msgs::UInt16MultiArray msg)
{
	//std::cout << "Got a Line sensor message" << std::endl;

	uint8_t tempState = 0;
	// Set the bit for each of the line sensors, 1 if they exceede the trigger and 0 if not.
	for (int i = 0; i < 4; i++)
	{
		tempState |= (msg.data[i] < Line_TriggerThreshold) ? 1 << i : 0x00;
	}

	// If this state is different to the last one then save it and set the old one to the last state
	if (tempState != lineSensorState)
	{
		lastLineSensorState = lineSensorState;
		lineSensorState = tempState;
	}
	printf("LineSensorState = %x\n", lineSensorState);

}

void Sumo2017::PublishMotorTwist(geometry_msgs::Twist twist_msg)
{
	//std::cout << "Publishing a motor twist" << std::endl;
	vel_set_pub.publish(twist_msg); 
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

	std::cout << "Press any key to start 5 second sleep." << std::endl;
	std::cin.ignore();
	std::cout << "Sleeping for 5 seconds ..." << std::endl;
	ros::Duration(5.0).sleep();
	std::cout << "Done Sleeping! Ecexuting node." << std::endl;

	// 50ms loop rate
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		// Spin it the logic
		sumo2017.spin();

		ros::spinOnce();
		loop_rate.sleep();

	}
}
