#include "ros/ros.h"
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <ros/console.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <std_msgs/UInt32MultiArray.h>

#define SERIAL_PORT "/dev/ttyACM0"
extern "C" const uint8_t RXFooter[3] = {'e', 'n', 'd'};
extern "C" const uint8_t RXHeader[5] = {'s', 't', 'a', 'r', 't'};


class SumoTest {

public:
	SumoTest();
	void RecieveData(void);
	void InitialseVariables(void);

private:
	void PublishIRSensor(void);
	//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


	int hserial;

	// Ros nodehandle. Main access point for all ROS comms
	ros::NodeHandle nh;

	// Serial Comms Stuff
	uint8_t newChar;
	int RXDataLen;
	int RXHeaderLen;
	int RXFooterLen;

	int RXLen;
	int RXStrState;
	uint32_t sensorData[8];

	// Publishers
	ros::Publisher sensorPublisher;

};

void SumoTest::InitialseVariables(void)
{
	RXDataLen = 4*8+8;
	RXFooterLen = 3;	
	RXHeaderLen = 5;	
	RXLen = RXHeaderLen + RXDataLen + RXFooterLen;
	RXStrState = 0;
	for (int i = 0; i < 8; i++) sensorData[i] = 0;
}

void SumoTest::PublishIRSensor(void)
{
	//PointCloud::Prt msg (new PointCloud);
	//msg->header.frame_id = "IR_sensor_frame"
	//msg->height = 1;
	//msg->width = 8;

	std_msgs::UInt32MultiArray array;
	array.data.clear();

	for (int i = 0; i < 8; i++)
	{
		std::cout << i << "= " << sensorData[i] << "\t";
		//msg->points.push_back (pcl::PointXYZ(4-i, sensorData[i], 0.0));
		array.data.push_back(sensorData[i]);
	}
	std::cout << std::endl;
	sensorPublisher.publish(array);
}

SumoTest::SumoTest()
{

	InitialseVariables();

	// Attach advertisers
	//sensorPublisher = nh.advertise<PointCloud>("IR_Sensor", 1)
	sensorPublisher = nh.advertise<std_msgs::UInt32MultiArray>("IR_sensor", 1);

	// -------- Set up wiringPi --------
	wiringPiSetupSys();
	if ( (hserial = serialOpen(SERIAL_PORT, 115200)) == -1)
	{
		printf("Failed to connect to device - Exiting\n");
	}
	else
	{
		std::cout << "Opened serial\n";
	}
	
}


/**
*!
*! Recieve Sensor data
*!
**/
void SumoTest::RecieveData()
{
	while (serialDataAvail(hserial) > 0)
	{
		newChar = (uint8_t)serialGetchar(hserial);
		
		//std::cout << "-" << RXStrState << "-";
		//std::cout << newChar;
		//if (RXStrState == 0) std::cout << std::endl;
		// If we are still looking through header, make sure it matches the expected character, otherwise reset
		if (RXStrState < RXHeaderLen)
		{
			(newChar == RXHeader[RXStrState]) ? RXStrState++ : RXStrState = 0;
		}
		// If we are now looking at footer, make sure it matches the expected character, otherwise reset
		else if (RXStrState >= RXHeaderLen + RXDataLen)
		{
			(newChar == RXFooter[RXStrState - RXHeaderLen - RXDataLen]) ? RXStrState++ : RXStrState = 0;
		}
		else
		{
			if ((RXStrState - RXHeaderLen) % 5 == 0)
				((uint8_t)((RXStrState - RXHeaderLen) / 5) + '0' == newChar) ? RXStrState++ : RXStrState = 0;
			else
			{
				sensorData[(int)((RXStrState - RXHeaderLen) / 5)] |= newChar << 8*(RXStrState - RXHeaderLen) % 5;
				RXStrState++;
			}
		}
		if (RXStrState >= RXLen)
		{
			PublishIRSensor();
			for (int i = 0; i < 8; i++) sensorData[i] = 0;
			RXStrState = 0;
			
		}
	}
}






int main(int argc, char** argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "motor_controller_node");

	// Instantiate the class
	SumoTest sumoTest;


	std::cout << "Hello\n" << std::endl;

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		sumoTest.RecieveData();

		// Spin then sleep
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

















