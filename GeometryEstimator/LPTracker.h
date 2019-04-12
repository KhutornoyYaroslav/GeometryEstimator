#pragma once

#include <memory>
#include <algorithm>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "LPRecognizer.h"
#include "LPTrack.h"
#include "munkres.h"

#define DEGREE_IN_RADIAN 57.295779513

// TODOS:
// Сделать динамически считаемыми: время жизни трека
// Создать отдеьный класс для трека и описать функциональность более подробно (среднее число потеряных кадров и т. п.)
// 
// 
// 
// 
// 
// 

class LPTracker
{
private:
	std::unique_ptr<LPRecognizer> p_recognizer;
	std::vector<LPTrack> m_process_tracks;
	std::vector<LPTrack> m_finished_tracks;
	int m_track_age = 5;

public:
	LPTracker();
	~LPTracker() = default;

	void process_frame(const cv::Mat& frame, cv::Mat debug);
};

