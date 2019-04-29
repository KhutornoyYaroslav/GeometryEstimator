#pragma once

#include <list>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <fstream>

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "rapidjson/document.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/prettywriter.h"

#include "LPRecognizerZone.h"

#define DEBUG_PRINT
#define POINTS_TO_CALIBRATE 75
#define MIN_POINTS_TO_CALIBRATE 25
#define MAX_STOP_WEIGHT 100.0
#define PLATE_RESIZE_SCALE 1.3

// TODO: add CLEAR() method.

class LPRecognizer
{
private:

	// Zones
	mutable std::mutex m_zones_mutex;
	std::list<LPRecognizerZone> m_zones;

	// Detector
	mutable std::mutex m_detector_mutex;
	std::unique_ptr<cv::CascadeClassifier> p_plate_detector;

	// Plate sizes
	cv::Size m_min_plate_size; 
	cv::Size m_max_plate_size;
	mutable std::mutex m_plate_size_mutex;

	// Image capturing
	cv::Mat m_gray_image;
	std::mutex m_input_mutex;
	bool m_is_new_image_detection;
	bool m_is_new_image_calibration;

	// Calibration
	enum class CalibrationState
	{
		init,
		up_scale,
		up_search,
		down_scale,
		down_search,
		finished
	};

	std::thread m_calibration_thread;
	std::atomic<bool> m_is_calibration_finished;
	std::atomic<bool> m_calibration_interruption;

public:
	LPRecognizer();
	~LPRecognizer();

	bool init();
	bool stop_calibration();
	bool start_calibration();
	bool is_calibration_finished() const;
	bool capture_frame(const cv::Mat& frame);
	bool detect(std::vector<cv::Rect>& plates);

	cv::Size min_plate_size() const;
	cv::Size max_plate_size() const;
	void set_min_plate_size(const cv::Size& size);
	void set_max_plate_size(const cv::Size& size);

	bool load_from_json(const std::string& filename);
	bool save_to_json(const std::string& filename) const;

private:
	void calibration_function();
	void correct_zones(const cv::Size& frame_size, std::list<LPRecognizerZone>& zones) const;
	std::vector<cv::Rect> detect_plates(const cv::Mat& gray_frame, const cv::Rect& ROI, const cv::Size& plate_size, const int& min_neighbor) const;
};

