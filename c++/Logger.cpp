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
} // LogStatus

//============================================================================
void Logger::LogStatus(
	const long numFrame,
	const unsigned int dt,
	const unsigned int fps,
	const cv::Point& puckPos,
	const cv::Point& bouncePos,
	const cv::Point& predPos,
	const cv::Point& prevPos,
	const cv::Point& botPos,
	const cv::Point& puckSpeed,
	const int predictTimeDefence,
	const unsigned int predictBounceStatus,
	const int botSpeed,
	const int predictStatus,
	const unsigned int botStatus,
	const unsigned int attackStatus
) const
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
	logFile << "predict pos: \n";
	logFile << predPos.x << " " << predPos.y << std::endl;
	logFile << "previous pos: \n";
	logFile << prevPos.x << " " << prevPos.y << std::endl;
	logFile << "desired bot pos: \n";
	logFile << botPos.x << " " << botPos.y << std::endl;
	logFile << "current puck speed: \n";
	logFile << puckSpeed.x << " " << puckSpeed.y << std::endl;
	logFile << "predict time defence: \n";
	logFile << predictTimeDefence << std::endl;
	logFile << "predict Bounce Status: \n";
	logFile << predictBounceStatus << std::endl;
	logFile << "desired bot speed: \n";
	logFile << botSpeed << std::endl;
	logFile << "predict status: \n";
	logFile << predictStatus << std::endl;
	logFile << "bot status: \n";
	logFile << botStatus << std::endl;
	logFile << "attack status: \n";
	logFile << attackStatus << std::endl;

	logFile.close();
} // LogStatus