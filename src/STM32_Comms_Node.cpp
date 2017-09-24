#include "ros/ros.h"
#include <wiringPi.h>
#include <stdio.h>
#include <wiringSerial.h>
#include <ros/console.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>


//#define SERIAL_PORT "/dev/ttyACM1"
extern "C" const uint8_t RXFooter[3] = {'e', 'n', 'd'};
extern "C" const uint8_t RXHeader[5] = {'s', 't', 'a', 'r', 't'};

class STM32_COMMS {

public:
	STM32_COMMS();
	void RecieveData(void);
	void setVelocities(void);
	void DeInit(void);
	

private:
	void PublishIRSensor(void);
	void PublishLineSensor(void);
	void InitialseVariables(void);

	
	// Callbacks
	void set_vel_callback(const geometry_msgs::Twist msg);

	// Ros nodehandle. Main access point for all ROS comms
	ros::NodeHandle nh;

	// Wiring PI serial handle
	std::string serialPort;
	int hserial;

	// Serial Comms Stuff
	uint8_t newChar;
	int RXDataLen;
	int RXHeaderLen;
	int RXFooterLen;

	int velTXbufferLen;
	int velTXbufferHeaderLen;
	uint8_t TXbuffer[19];

	// Message Objects
	geometry_msgs::Twist vel_msg;
	uint32_t maxPWMPulseWidth;
	int32_t L_Motor;
	int32_t R_Motor;


	int RXLen;
	int RXStrState;
	uint32_t sensorData[8];
	uint8_t lineSensorData;

	// Pubs
	ros::Publisher sensor_IR_Publisher;
	ros::Publisher sensor_Line_Publisher;

	// Subs
	ros::Subscriber set_vel; // Set velocities
	
};


void STM32_COMMS::InitialseVariables(void)
{
	// RX data length is eight uint32_t, eight ascii numbers identifying IT sensor data. Then an 'l' character and then the Line sensor state
	RXDataLen = 4*8+8+2;
	RXFooterLen = 3;	
	RXHeaderLen = 5;	
	RXLen = RXHeaderLen + RXDataLen + RXFooterLen;
	RXStrState = 0;
	lineSensorData = 0x00;
	maxPWMPulseWidth = 7999;
	velTXbufferLen = 19;
	velTXbufferHeaderLen = 6;
	for (int i = 0; i < 8; i++) sensorData[i] = 0;
}

void STM32_COMMS::PublishIRSensor(void)
{
	std_msgs::UInt32MultiArray array;
	array.data.clear();

	for (int i = 0; i < 8; i++)
	{
		//std::cout << i << "= " << sensorData[i] << "\t";
		//msg->points.push_back (pcl::PointXYZ(4-i, sensorData[i], 0.0));
		array.data.push_back(sensorData[i]);
	}
	//("\t[%d, %d, %d, %d", lineSensorData & 0x01, (lineSensorData >> 1) & 0x01, (lineSensorData >> 2) & 0x01, (lineSensorData >> 3) & 0x01);
	//std::cout << std::endl;
	sensor_IR_Publisher.publish(array);
}

STM32_COMMS::STM32_COMMS()
{
	// Get parameters
	nh.param<std::string>("SerialPort", serialPort, "/dev/ttyACM0");
	std::cout << "Using serial port: " << serialPort << std::endl;

	InitialseVariables();

	// Attach advertisers
	sensor_IR_Publisher = nh.advertise<std_msgs::UInt32MultiArray>("IR_sensor", 1);

	// -------- Set up wiringPi --------
	wiringPiSetupSys();
	if ( (hserial = serialOpen(serialPort.c_str(), 115200)) == -1)
	{
		printf("Failed to connect to device - Exiting\n");
	}
	else
	{
		std::cout << "Opened serial\n";
	}

	// Subscribe to topics
	set_vel = nh.subscribe<geometry_msgs::Twist>("vel_set", 100, &STM32_COMMS::set_vel_callback, this);
	
}

void STM32_COMMS::DeInit(void)
{
	serialClose(hserial);
}

/**
*!
*! Recieve Sensor data
*!
**/
void STM32_COMMS::RecieveData()
{
	while (serialDataAvail(hserial) > 0)
	{
		newChar = (uint8_t)serialGetchar(hserial);
		
		//std::cout << "-" << RXStrState << "-";
		std::cout << newChar;
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
		// If this is the last databyte then it is the line sensor state
		else if (RXStrState == RXHeaderLen + RXDataLen - 1)
		{
			lineSensorData = newChar;
			RXStrState++;
		}
		// If we are looking at the second last data bytes then make sure we get an 'l' followed by the line sensors byte
		else if (RXStrState == RXHeaderLen + RXDataLen - 2)
		{
			(newChar == 'l') ? RXStrState++ : RXStrState = 0;
		}
		else
		{
			if ((RXStrState - RXHeaderLen) % 5 == 0)
				((uint8_t)((RXStrState - RXHeaderLen) / 5) + '0' == newChar) ? RXStrState++ : RXStrState = 0;
			else
			{
				sensorData[(int)((RXStrState - RXHeaderLen) / 5)] |= newChar << 8*((RXStrState - RXHeaderLen) % 5-1);
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


/**
*
*	Set the velocity of Left and Right motors.
* 	Currently not using twist properly here. Just using x for L and y for R for testing.
*
**/
void STM32_COMMS::set_vel_callback(const geometry_msgs::Twist msg)
{
	L_Motor = (int32_t)(msg.linear.x*maxPWMPulseWidth);
	R_Motor = (int32_t)(msg.linear.y*maxPWMPulseWidth);
}

/**
*	Actually set the velocities.
*
**/
void STM32_COMMS::setVelocities()
{
	//printf("Setting L_motor = %f\tR_motor = %f\n", L_motor, R_motor);
	L_Motor += 100;
	R_Motor += 100;
	L_Motor %= 7999;
	R_Motor %= 7999;
	serialFlush(hserial);
	//memcpy(TXbuffer, "start!LaaaaRaaaaend", 19);
	memcpy(TXbuffer, "start!", 6);
	memcpy(TXbuffer+velTXbufferLen-3, "end", 3);
	TXbuffer[6] = 'L';
	for (int i=0; i < 4; i++)
	{
		TXbuffer[7+i] = (uint8_t)((L_Motor >> 8*i)&0xFF);
	}
	TXbuffer[11] = 'R';
	for (int i=0; i < 4; i++)
	{
		TXbuffer[12+i] = (uint8_t)((R_Motor >> 8*i)&0xFF);
	}
	printf("L = %ld, R = %ld, %s\n", L_Motor, R_Motor, TXbuffer);
	//serialFlush(hserial);
	for (int i = 0; i < 19; i++)
	{
		serialPutchar (hserial, (unsigned char)TXbuffer[i]) ;
	}
	//write(hserial, (const char*)TXbuffer, 19);

}


int main(int argc, char** argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "STM32_comms_node");

	// Instantiate the class
	STM32_COMMS STM32_Comms;


	std::cout << "Hello\n" << std::endl;

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		STM32_Comms.RecieveData();
		STM32_Comms.setVelocities();

		// Spin then sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	STM32_Comms.DeInit();

	return 0;
}
