#include <iostream>
#include <fstream>

#include "Logger.h"

//======================================
Logger::Logger()
{
}

//============================================================================
void Logger::WriteTableCorners(
	const cv::Point& TopLeft,
	const cv::Point& TopRight,
	const cv::Point& LowerLeft,
	const cv::Point& LowerRight ) const
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
void Logger::LogStatus(
	const long numFrame,
	const unsigned int dt,
	const unsigned int fps ) const
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

//============================================================================
void Logger::LogStatus(
	const long numFrame,
	const unsigned int dt,
	const unsigned int fps,
	const cv::Point& puckPos,
	const cv::Point& bouncePos,
	const cv::Point& botPos,
	const int botSpeed,
	const int predictStatus,
	const unsigned int botStatus ) const
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
	logFile << puckPos.x << " " << puckPos.y << std::endl;
	logFile << "bounce pos: \n";
	logFile << bouncePos.x << " " << bouncePos.y << std::endl;
	logFile << "desired bot pos: \n";
	logFile << botPos.x << " " << botPos.y << std::endl;
	logFile << "desired bot speed: \n";
	logFile << botSpeed << std::endl;
	logFile << "predict status: \n";
	logFile << predictStatus << std::endl;
	logFile << "bot status: \n";
	logFile << botStatus << std::endl;

	logFile.close();
}