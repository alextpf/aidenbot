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
	const unsigned int fps,
	const cv::Point& puckPos,
	const cv::Point& bouncePos,
	const cv::Point& predPos,
	const cv::Point& prevPos,
	const cv::Point& botPos,
	const cv::Point& detectedBotPos,
	const cv::Point& puckSpeed,
	const int predictTimeDefence,
	const int predictTimeAttack,
	const unsigned int numBounce,
    const int botXSpeed,
    const int botYSpeed,
	const int predictStatus,
	const unsigned int botStatus,
	const unsigned int attackStatus,
	const clock_t attackTime,
	const cv::Point2f& avgPuckSpeed,
    const bool correctMissingSteps ) const
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
	logFile << "detected bot pos: \n";
	logFile << detectedBotPos.x << " " << detectedBotPos.y << std::endl;
	logFile << "current puck speed: \n";
	logFile << puckSpeed.x << " " << puckSpeed.y << std::endl;
	logFile << "predict time defence: \n";
	logFile << predictTimeDefence << std::endl;
	logFile << "predict time attack (only applicable in direct impact): \n";
	logFile << predictTimeAttack << std::endl;
	logFile << "predicted number of Bounce: \n";
	logFile << numBounce << std::endl;
	logFile << "desired bot X/Y speed: \n";
	logFile << botXSpeed << " " << botYSpeed << std::endl;
	logFile << "predict status ( -1 : error, 0 : No risk, 1. direct impact, 2. 1 bounce ): \n";
	logFile << predictStatus << std::endl;
	logFile << "bot status ( 0: Init, 1: Defense, 2: Defense+Atack, 3: Atack (only when predict status = no risk) ): \n";
	logFile << botStatus << std::endl;
	logFile << "attack status (0: wait for attack, 1: ready to attack, 2: after firing attack ) only useful in BOT_STATUS::ATTACK mode: \n";
	logFile << attackStatus << std::endl;
	logFile << "attack time: \n";
	logFile << attackTime << std::endl;
	logFile << "puck avg speed: \n";
	logFile << avgPuckSpeed.x << " " << avgPuckSpeed.y << std::endl;
    logFile << "correct missing steps? \n";
    if( correctMissingSteps )
    {
        logFile << "yes" << std::endl;
    }
    else
    {
        logFile << "no" << std::endl;
    }

	logFile.close();
} // LogStatus