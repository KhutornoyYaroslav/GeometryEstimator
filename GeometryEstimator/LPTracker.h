#pragma once

#include <memory>
#include <algorithm>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "LPRecognizer.h"
#include "HungarianAlgorithm.h"

#define DEGREE_IN_RADIAN 57.295779513

struct LPTrack
{
	std::vector<cv::Rect> points;
	bool finished = false;
};

class LPTracker
{
private:
	std::unique_ptr<LPRecognizer> p_recognizer;
	std::vector<LPTrack> m_tracks;

public:
	LPTracker();
	~LPTracker() = default;

	void process_frame(const cv::Mat& frame, cv::Mat debug);
};

