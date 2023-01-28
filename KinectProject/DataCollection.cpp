// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to log EMG and IMU data. EMG streaming is only supported for one Myo at a time, and this entire sample is geared to one armband
// This is a C++ file

#define _USE_MATH_DEFINES
#include "DataCollection.h"
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stdbool.h>
#include <cstdlib>
#include <string>
#include <fstream>
#include <time.h>
#include <chrono>

#include "stdafx.h"
#include <Windows.h>
#include <NuiApi.h>


#include <myo/myo.hpp>

class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
	{
		openFiles();
	}


	void openFiles() {
		time_t timestamp = std::time(0);


		// Open file for EMG log
		if (emgFile.is_open()) {
			emgFile.close();
		}
		std::ostringstream emgFileString;
		emgFileString << "emg" << ".csv";
		emgFile.open(emgFileString.str(), std::ios::out);
		emgFile << "timestamp,emg1,emg2,emg3,emg4,emg5,emg6,emg7,emg8" << std::endl;

		// Open file for gyroscope log
		if (gyroFile.is_open()) {
			gyroFile.close();
		}
		std::ostringstream gyroFileString;
		gyroFileString << "gyro" << ".csv";
		gyroFile.open(gyroFileString.str(), std::ios::out);
		gyroFile << "timestamp,x,y,z" << std::endl;

		// Open file for accelerometer log
		if (accelerometerFile.is_open()) {
			accelerometerFile.close();
		}
		std::ostringstream accelerometerFileString;
		accelerometerFileString << "accelerometer" << ".csv";
		accelerometerFile.open(accelerometerFileString.str(), std::ios::out);
		accelerometerFile << "timestamp,x,y,z" << std::endl;

		// Open file for orientation log
		if (orientationFile.is_open()) {
			orientationFile.close();
		}
		std::ostringstream orientationFileString;
		orientationFileString << "orientation" << ".csv";
		orientationFile.open(orientationFileString.str(), std::ios::out);
		orientationFile << "timestamp,x,y,z,w" << std::endl;

		// Open file for orientation (Euler angles) log
		if (orientationEulerFile.is_open()) {
			orientationEulerFile.close();
		}
		std::ostringstream orientationEulerFileString;
		orientationEulerFileString << "orientationEuler" << ".csv";
		orientationEulerFile.open(orientationEulerFileString.str(), std::ios::out);
		orientationEulerFile << "timestamp,roll,pitch,yaw" << std::endl;



		// Open file for orientation log
		if (KienctRightHandFile.is_open()) {
			KienctRightHandFile.close();
		}
		std::ostringstream KienctRightHandFileString;
		KienctRightHandFileString << "KinectHandsData" << ".csv";
		KienctRightHandFile.open(KienctRightHandFileString.str(), std::ios::out);
		KienctRightHandFile << "timestamp,RH X ,RH Y,RH Z,RS X, RS Y,RS Z,RE X,RE Y,RE Z,RW X,RW Y,RW Z,LS X,LS Y,LS Z,LC X,LC Y,LC Z" << std::endl;




	}

	// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
	{

		emgFile << timestamp;
		for (size_t i = 0; i < 8; i++) {
			emgFile << ',' << static_cast<int>(emg[i]);

		}
		emgFile << std::endl;
	}

	// onOrientationData is called whenever new orientation data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onOrientationData(myo::Myo *myo, uint64_t timestamp, const myo::Quaternion< float > &rotation) {
		orientationFile << timestamp
			<< ',' << rotation.x()
			<< ',' << rotation.y()
			<< ',' << rotation.z()
			<< ',' << rotation.w()
			<< std::endl;

		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (rotation.w() * rotation.x() + rotation.y() * rotation.z()),
						   1.0f - 2.0f * (rotation.x() * rotation.x() + rotation.y() * rotation.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (rotation.w() * rotation.y() - rotation.z() * rotation.x()))));
		float yaw = atan2(2.0f * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
						  1.0f - 2.0f * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));

		orientationEulerFile << timestamp
			<< ',' << roll
			<< ',' << pitch
			<< ',' << yaw
			<< std::endl;
	}

	// onAccelerometerData is called whenever new acceleromenter data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel) {

		printVector(accelerometerFile, timestamp, accel);

	}

	// onGyroscopeData is called whenever new gyroscope data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
		printVector(gyroFile, timestamp, gyro);

	}

	void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
		//Reneable streaming
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		openFiles();
	}

	// Helper to print out accelerometer and gyroscope vectors
	void printVector(std::ofstream &file, uint64_t timestamp, const myo::Vector3< float > &vector) {
		file << timestamp
			<< ',' << vector.x()
			<< ',' << vector.y()
			<< ',' << vector.z()
			<< std::endl;
	}

	bool blIsMovementStopped(NUI_SKELETON_FRAME CurrentFrame, NUI_SKELETON_FRAME PreviousFrame, int i)
	{

		static int TimeCounter = 0;
		static int MovementStopCounter = 0;
		float MotionDifferenceMargin[10] = {};

		if (TimeCounter < TimeToWaitBeforeMotionStopDetectionStarts)
		{

			TimeCounter++;

			return false;
		}
		else
		{

			MotionDifferenceMargin[0] = ((abs(PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x))		*	PercentageToDetectMovementStop);
			MotionDifferenceMargin[1] = ((abs(PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y))		*	PercentageToDetectMovementStop);
			MotionDifferenceMargin[2] = ((abs(PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z))		*	PercentageToDetectMovementStop);

			if ( /* there is  movement in X,Y and Z for the hand
					by comparing the current value with the previous as follow:
					(Previous value + Motion Difference Margin) <= Current Value <= (Previous value - Motion Difference Margin) */

				(CurrentFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x >= (PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x + MotionDifferenceMargin[0])
				 ||
				 CurrentFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x <= (PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x - MotionDifferenceMargin[0])
				 )
				||
				(CurrentFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y >= (PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y + MotionDifferenceMargin[1])
				 ||
				 CurrentFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y <= (PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y - MotionDifferenceMargin[1])
				 )
				||
				(CurrentFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z >= (PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z + MotionDifferenceMargin[2])
				 ||
				 CurrentFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z <= (PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z - MotionDifferenceMargin[2])
				 )

				)
			{
				if (MovementStopCounter > 0)
				{
					MovementStopCounter--;
				}
			}
			else
			{
				if (MovementStopCounter < CountTimeToDetectMotionStop)
				{
					MovementStopCounter++;
				}
				else
				{
					return true;
				}
			}

			return false;
		}
	}



	void RetrieveKinectData(void)
	{
		NUI_SKELETON_FRAME ourframe;
		static NUI_SKELETON_FRAME PreviousFrame;

		time_t FrameTimeStamp_MilliSeconds;

		static bool IsMovementStopped	= false;
		static bool FirstTimeSound		= true;


		NuiSkeletonGetNextFrame(0, &ourframe); //Get a frame and stuff it into ourframe
		for (int i = 0; i < 6; i++) //Six times, because the Kinect has space to track six people
		{
			if (ourframe.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED) //See more on this line below
			{
				if (ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x != 0)
				{
					if (FirstTimeSound == true)
					{
						std::cout << "****************************************************** Started Logging Kinect Data*************************************************" << std::endl;
						Beep(523, 500);
						FirstTimeSound = false;
					}

					//Get Timestamp of frame
					//time_t FrameTimeStamp_MilliSeconds = std::time(0);
					FrameTimeStamp_MilliSeconds = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();


					//Start writing to CSV file

					//Timestamp
					KienctRightHandFile << FrameTimeStamp_MilliSeconds;

					//Right Hand
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z;

					//Right Shoulder 
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT].z;

					//Right Elbow 
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT].x;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT].y;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT].z;

					//Right Wirst 
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT].x;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT].y;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT].z;

					//Left Shoulder 
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT].x;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT].y;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT].z;

					//Right Wirst 
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].x;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].y;
					KienctRightHandFile << ',' << ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].z;


					std::cout << (ourframe.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y) << std::endl;
					std::cout << (PreviousFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y) << std::endl;

					KienctRightHandFile << std::endl;



					/*************************************************************************************************************************/
					/************************************************    Detect Motion stop   ************************************************/
					/*************************************************************************************************************************/

					if (blIsMovementStopped(ourframe, PreviousFrame, i) == true)
					{
						throw std::exception();
					}
					//}

					/* Update previuos Frame Value*/
					PreviousFrame = ourframe;

				}
			}

		}


		return;
	}





	// The files we are logging to
	std::ofstream emgFile;
	std::ofstream gyroFile;
	std::ofstream orientationFile;
	std::ofstream orientationEulerFile;
	std::ofstream accelerometerFile;
	std::ofstream KienctRightHandFile;

};


int main(int argc, char** argv)
{

	// We catch any exceptions that might occur below -- see the catch statement for more details.
	
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.undercoveryeti.myo-data-capture");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband! Logging to the file system. Check your home folder or the folder this application lives in." << std::endl << std::endl;

		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		
		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
	DataCollector collector;

	// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
	// Hub::run() to send events to all registered device listeners.
	hub.addListener(&collector);


	/******************************		Kinect init			************************************/
	NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
	NuiSkeletonTrackingEnable(NULL, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);


	// Finally we enter our main loop.
	while (1)
	{
		try
		{
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 50 times a second, so we run for 1000/20 milliseconds.
			hub.run(1);


			//Kinect Part
			collector.RetrieveKinectData();

		}
		catch (const std::exception& e)
		{
			return EXIT_SUCCESS;
		}
	}
	
	 //If a standard exception occurred, we print out its message and exit.
}
catch (const std::exception& e) {
	std::cerr << "Error: " << e.what() << std::endl;
	std::cerr << "Press enter to continue.";
	std::cin.ignore();
	return 1;
}

}
