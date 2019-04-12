#pragma once
#include <memory>
#include <list>

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "LPRecognizerZone.h"
// TODO:
// 1) Возможность настраивать вручную макс и мин размеры номеров
// 2) Возможность накопления статистики и автомат. вычисления мин и макс размеров номеров

class LPRecognizer
{
private:

	std::unique_ptr<cv::CascadeClassifier> p_plate_detector;
	cv::Size m_cascade_wnd_size;



	//std::list<LPRecognizerZone> m_zones;
	std::list<std::pair<LPRecognizerZone,double>> m_zones;

	LPRecognizerZone m_cur_zone;
	double m_resized_k = 1.0;

public:
	LPRecognizer();
	~LPRecognizer();
	bool initCascadeClassifier();
	std::vector<cv::Rect> process_frame(const cv::Mat& frame, cv::Mat& debug_frame);
	void calibrate_zones(const cv::Mat& frame, cv::Mat& debug_frame);
};

