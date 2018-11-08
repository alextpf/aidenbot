#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

class Camera
{
public:
    // -1 : error: noise
    // 0 : No risk,
    // 1 : Puck is moving to our field directly fast, with no bounce
    // 2 : Puck is moving to our field fast, with a bounce
    enum PREDICT_STATUS { ERROR = -1, NO_RISK, DIRECT_IMPACT, ONE_BOUNCE };

	Camera();
	~Camera();

	// Current Puck Position
	void SetCurrPuckPos( const cv::Point& pos );

	cv::Point GetCurrPuckPos() const;

	// Previous Puck Position
	void SetPrevPuckPos( const cv::Point& pos );

	cv::Point GetPrevPuckPos() const;

	// Main function
	void CamProcess( int dt /*ms*/ );

	unsigned int GetCurrNumPredictBounce();
	unsigned int GetPrevNumPredictBounce();

	int GetPredictTimeDefence() const;

	int GetPredictTimeAttack() const;

	cv::Point PredictPuckPos( int predictTime );

    PREDICT_STATUS GetPredictStatus() const;

	cv::Point GetCurrPredictPos() const;

	void SetCurrPredictPos( const cv::Point& pos );

	cv::Point2f GetPuckAvgSpeed() const;

	cv::Point2f GetCurrPuckSpeed() const;

	int GetPredictXAttack();

	void SetRobotPos(const cv::Point& pos);
	cv::Point GetRobotPos() const;
	cv::Point GetBouncePos() const;

private:

	bool HasBounce( const int x ) const;

	/////////////////////////////
	// Puck
	/////////////////////////////

	//////////////
	// position
	//////////////
	cv::Point		m_CurrPuckPos;        // current pos. mm. Corresponds to puckCoordX/Y.
	cv::Point		m_PrevPuckPos;        // previous pos. mm. Corresponds to puckOldCoordX. Updated in reading from the camera, when new puck pos comes in
	cv::Point		m_CurrPredictPos;
	cv::Point		m_PrevPredictPos;
	cv::Point		m_BouncePos;
	int				m_PredictXAttack;     // predicted X coordinate for attack

	//////////////
	// speed
	//////////////
	cv::Point2f		m_CurrPuckSpeed;      // current speed. dm/ms
	cv::Point2f		m_PrevPuckSpeed;      // previous speed. dm/ms
	cv::Point2f		m_AverageSpeed;

	//////////////
	// Bounce
	//////////////
	unsigned int	m_CurrNumPredictBounce;     // number of bounce predicted
	unsigned int	m_PrevNumPredictBounce;     // number of bounce predicted

    //////////////
	// status
	//////////////
    PREDICT_STATUS	m_PredictStatus;

	//////////////
	// Time
	//////////////
	int				m_PredictTimeDefence;
	int				m_PredictTimeAttack;

	/////////////////////////////
	// Robot
	/////////////////////////////
	cv::Point		m_RobotPos;        // robot pos captured by camera. mm

};// Camera

