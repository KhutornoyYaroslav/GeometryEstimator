#pragma once
#include <memory>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "HungarianAlgorithm.h"

class LPTracker
{
private:

	//std::unique_ptr<cv::KalmanFilter> p_kalman;

public:
	LPTracker();
	~LPTracker();

	void process_frame();
};

