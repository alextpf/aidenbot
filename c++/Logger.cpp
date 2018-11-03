#include <iostream>
#include <fstream>

#include "Logger.h"

//======================================
Logger::Logger()
{
}

//============================================================================
void Logger::WriteTableCorners(
	cv::Point& TopLeft,
	cv::Point& TopRight,
	cv::Point& LowerLeft,
	cv::Point& LowerRight )
{
	std::ofstream logFile;

	logFile.open( "Log.txt", std::ios_base::out ); // fresh file
	logFile << "Table corners: \n";
	logFile << TopLeft.x << " " << TopLeft.y << std::endl;
	logFile << TopRight.x << " " << TopRight.y << std::endl;
	logFile << LowerLeft.x << " " << LowerLeft.y << std::endl;
	logFile << LowerRight.x << " " << LowerRight.y << std::endl;

	logFile.close();
}//WriteTableCorners

 //============================================================================
void Logger::LogStatus( long numFrame, unsigned int dt, unsigned int fps )
{
	std::ofstream logFile;
	logFile.open( "Log.txt", std::ios_base::app ); // append
	logFile << "frame number: \n";
	logFile << numFrame << std::endl;
	logFile << "frame time: \n";
	logFile << dt << std::endl;
	logFile << "fps: \n";
	logFile << fps << std::endl;
	logFile << "puck pos: \n";
	logFile << "-1" << std::endl; // indicate that the puck can't be found for this frame

	logFile.close();
}