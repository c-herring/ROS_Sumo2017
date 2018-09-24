#include "ros/ros.h"
#include <wiringPi.h>
#include <stdio.h>
#include <wiringSerial.h>
#include <ros/console.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
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
	std::string serialPort_stm;
	std::string serialPort_ard;
	int hserial_stm;
	int hserial_ard;

	// Serial Comms Stuff
	uint8_t newChar;

	// For the STM32
	int RXDataLen_stm;
	int RXHeaderLen_stm;
	int RXFooterLen_stm;
	int RXLen_stm;
	int RXStrState_stm;

	// For the Arduino
	int RXDataLen_ard;
	int RXHeaderLen_ard;
	int RXFooterLen_ard;
	int RXLen_ard;
	int RXStrState_ard;

	int velTXbufferLen;
	int velTXbufferHeaderLen;
	uint8_t TXbuffer[19];

	// Message Objects
	geometry_msgs::Twist vel_msg;
	uint32_t maxPWMPulseWidth;
	int32_t L_Motor;
	int32_t R_Motor;


	uint32_t sensorData[8];
	uint16_t lineSensorData[4];

	// Pubs
	ros::Publisher sensor_IR_Publisher;
	ros::Publisher sensor_Line_Publisher;

	// Subs
	ros::Subscriber set_vel; // Set velocities
	
};


void STM32_COMMS::InitialseVariables(void)
{
	// STM RX data length is eight uint32_t, eight ascii numbers identifying IT sensor data. Then an 'l' character and then the Line sensor state
	RXDataLen_stm = 4*8+8;
	RXFooterLen_stm = 3;	
	RXHeaderLen_stm = 5;	
	RXLen_stm = RXHeaderLen_stm + RXDataLen_stm + RXFooterLen_stm;
	RXStrState_stm = 0;

	// Arduino data length is four uint32_t and four ascii numbers idnetifying them
	RXDataLen_ard = 2*4+4;
	RXHeaderLen_ard = 5;
	RXFooterLen_ard = 3;
	RXLen_ard = RXDataLen_ard + RXHeaderLen_ard + RXFooterLen_ard;
	RXStrState_ard = 0;

	maxPWMPulseWidth = 7999;
	velTXbufferLen = 19;
	velTXbufferHeaderLen = 6;
	L_Motor = 0;
	R_Motor = 0;
	for (int i = 0; i < 8; i++) sensorData[i] = 0;
	for (int i = 0; i < 4; i++) lineSensorData[i] = 0;
}

void STM32_COMMS::PublishIRSensor(void)
{
	std_msgs::UInt32MultiArray array;
	array.data.clear();


	// Swap elements 4 and 5, and 6 and 7 because for some reason they are scanned in wrong order..
	uint32_t tempVal = sensorData[4];
	sensorData[4] = sensorData[5];
	sensorData[5] = tempVal;
	tempVal = sensorData[6];
	sensorData[6] = sensorData[7];
	sensorData[7] = tempVal;

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

/**
** Publish analog readongs of the four line sensor data (NE, SE, SW, NW)
**/
void STM32_COMMS::PublishLineSensor(void)
{
	std_msgs::UInt16MultiArray array;

	for (int i = 0; i < 4; i++)
	{
		array.data.push_back(lineSensorData[i]);
	}
	
	sensor_Line_Publisher.publish(array);
}

STM32_COMMS::STM32_COMMS()
{
	// Get parameters
	nh.param<std::string>("serialPort_stm", serialPort_stm, "/dev/ttyACM0");
	nh.param<std::string>("serialPort_ard", serialPort_ard, "/dev/ttyUSB0");
	std::cout << "STM on serial port: " << serialPort_stm << std::endl;
	std::cout << "Arduino on serial port: " << serialPort_ard << std::endl;

	InitialseVariables();

	// Attach advertisers
	sensor_IR_Publisher 	= nh.advertise<std_msgs::UInt32MultiArray>("IR_sensor", 1);
	sensor_Line_Publisher	= nh.advertise<std_msgs::UInt16MultiArray>("Line_sensor", 1);

	// -------- Set up wiringPi --------
	wiringPiSetupSys();
	if ( (hserial_stm = serialOpen(serialPort_stm.c_str(), 115200)) == -1)
	{
		printf("Failed to connect to STM device - Exiting\n");
	}
	else
	{
		std::cout << "Opened STM serial\n";
	}
	if ( (hserial_ard = serialOpen(serialPort_ard.c_str(), 115200)) == -1)
	{
		printf("Failed to connect to Arduino device - Exiting\n");
	}
	else
	{
		std::cout << "Opened Arduino serial\n";
	}

	// Subscribe to topics
	set_vel = nh.subscribe<geometry_msgs::Twist>("vel_set", 100, &STM32_COMMS::set_vel_callback, this);
	
}

void STM32_COMMS::DeInit(void)
{
	serialClose(hserial_stm);
	serialClose(hserial_ard);
}

/**
*!
*! Recieve Sensor data
*!
**/
void STM32_COMMS::RecieveData()
{
	// Get data from stm
	while (serialDataAvail(hserial_stm) > 0)
	{
		newChar = (uint8_t)serialGetchar(hserial_stm);
		
		//std::cout << "-" << RXStrState_stm << "-";
		//std::cout << newChar;
		//if (RXStrState_stm == 0) std::cout << std::endl;
		// If we are still looking through header, make sure it matches the expected character, otherwise reset
		if (RXStrState_stm < RXHeaderLen_stm)
		{
			(newChar == RXHeader[RXStrState_stm]) ? RXStrState_stm++ : RXStrState_stm = 0;
		}
		// If we are now looking at footer, make sure it matches the expected character, otherwise reset
		else if (RXStrState_stm >= RXHeaderLen_stm + RXDataLen_stm)
		{
			(newChar == RXFooter[RXStrState_stm - RXHeaderLen_stm - RXDataLen_stm]) ? RXStrState_stm++ : RXStrState_stm = 0;
		}
		// If this is the last databyte then it is the line sensor state
		//else if (RXStrState_stm == RXHeaderLen_stm + RXDataLen_stm - 1)
		//{
		//	//lineSensorData = newChar;
		//	RXStrState_stm++;
		//}
		// If we are looking at the second last data bytes then make sure we get an 'l' followed by the line sensors byte
		//else if (RXStrState_stm == RXHeaderLen_stm + RXDataLen_stm - 2)
		//{
		//	(newChar == 'l') ? RXStrState_stm++ : RXStrState_stm = 0;
		//}
		else
		{
			if ((RXStrState_stm - RXHeaderLen_stm) % 5 == 0)
				((uint8_t)((RXStrState_stm - RXHeaderLen_stm) / 5) + '0' == newChar) ? RXStrState_stm++ : RXStrState_stm = 0;
			else
			{
				sensorData[(int)((RXStrState_stm - RXHeaderLen_stm) / 5)] |= newChar << 8*((RXStrState_stm - RXHeaderLen_stm) % 5-1);
				RXStrState_stm++;
			}
		}
		if (RXStrState_stm >= RXLen_stm)
		{
			PublishIRSensor();
			for (int i = 0; i < 8; i++) sensorData[i] = 0;
			RXStrState_stm = 0;
			
		}
	}

	// Get data from Arduino
	while (serialDataAvail(hserial_ard) > 0)
	{
		newChar = (uint8_t)serialGetchar(hserial_ard);

		// If we are still looking through header, make sure it matches the expected character, otherwise reset
		if (RXStrState_ard < RXHeaderLen_ard)
		{
			(newChar == RXHeader[RXStrState_ard]) ? RXStrState_ard++ : RXStrState_ard = 0;
		}
		// If we are now looking at footer, make sure it matches the expected character, otherwise reset
		else if (RXStrState_ard >= RXHeaderLen_ard + RXDataLen_ard)
		{
			(newChar == RXFooter[RXStrState_ard - RXHeaderLen_ard - RXDataLen_ard]) ? RXStrState_ard++ : RXStrState_ard = 0;
		}
		// Parse data byte
		else
		{
			if ((RXStrState_ard - RXHeaderLen_ard) % 3 == 0)
				((uint8_t)((RXStrState_ard - RXHeaderLen_ard) / 3) + '0' == newChar) ? RXStrState_ard++ : RXStrState_ard = 0;
			else
			{
				lineSensorData[(int)((RXStrState_ard - RXHeaderLen_ard) / 3)] |= newChar << 8*((RXStrState_ard - RXHeaderLen_ard) % 3-1);
				RXStrState_ard++;
			}
		}

		if (RXStrState_ard >= RXLen_ard)
		{
			PublishLineSensor();
			for (int i = 0; i < 4; i++) lineSensorData[i] = 0;
			RXStrState_ard = 0;
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
	L_Motor =  (int32_t)(msg.linear.x*maxPWMPulseWidth);
	// Invert the right motor velocity now.
	R_Motor = -(int32_t)(msg.linear.y*maxPWMPulseWidth);
	setVelocities();
}

/**
*	Actually set the velocities.
*
**/
void STM32_COMMS::setVelocities()
{
	//printf("Setting L_motor = %f\tR_motor = %f\n", L_motor, R_motor);
	//L_Motor += 100;
	//R_Motor += 100;
	//L_Motor %= 7999;
	//R_Motor %= 7999;
	serialFlush(hserial_stm);
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
	//printf("L = %ld, R = %ld, %s\n", L_Motor, R_Motor, TXbuffer);
	//serialFlush(hserial_stm);
	for (int i = 0; i < 19; i++)
	{
		serialPutchar (hserial_stm, (unsigned char)TXbuffer[i]) ;
	}
	//write(hserial_stm, (const char*)TXbuffer, 19);

}


int main(int argc, char** argv)
{
	// First, initialise the ros node. Pass argc and argv in and give the node a name
	ros::init(argc, argv, "STM32_comms_node");

	// Instantiate the class
	STM32_COMMS STM32_Comms;


	std::cout << "Starting Comms Node Loop.\n" << std::endl;

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		// Check if there is data to recieve. If there is then recieve it and parse it
		STM32_Comms.RecieveData();
		//STM32_Comms.setVelocities();

		// Spin then sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	STM32_Comms.DeInit();

	return 0;
}
