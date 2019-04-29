#pragma once

#include <memory>
#include <algorithm>
#include <thread>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "LPRecognizer.h"
#include "LPTrack.h"
#include "LPPlate.h"

#include "munkres.h"

#define DEGREE_IN_RADIAN 57.295779513

#define MAX_MISSED_FRAMES 5

// TODOS:
// Сделать динамически считаемыми: время жизни трека
// Создать отдеьный класс для трека и описать функциональность более подробно (среднее число потеряных кадров и т. п.)

class LPTracker
{
private:

	// Recognizer
	std::unique_ptr<LPRecognizer> p_recognizer;

	// Image capturing	
	bool m_is_new_image;
	cv::Mat m_gray_image;
	std::mutex m_input_mutex;

	// Tracks
	std::mutex m_finished_tracks_mutex;
	std::vector<LPTrack> m_finished_tracks;
	std::vector<LPTrack> m_process_tracks;

	// Processing
	std::thread m_process_thread;
	std::atomic<bool> m_process_interruption;
	std::atomic<bool> m_is_process_finished;

public:
	LPTracker();
	~LPTracker() = default;

	//bool init();
	void clear();
	bool start_process();
	bool stop_process();
	bool capture_frame(const cv::Mat& frame);
	void pull_tracks(std::vector<LPTrack>& tracks);
	void process_plates(const cv::Mat& frame, const std::vector<cv::Rect>& plates);

private:
	void process_thread_function();
	void process(const cv::Mat& frame, const std::vector<cv::Rect>& plates, std::vector<LPTrack>& tracks);
};

