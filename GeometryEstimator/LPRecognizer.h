#pragma once
#include <memory>
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// TODO:
// 1) Возможность настраивать вручную макс и мин размеры номеров
// 2) Возможность накопления статистики и автомат. вычисления мин и макс размеров номеров

class LPRecognizer
{
private:

	std::unique_ptr<cv::CascadeClassifier> p_plate_detector;
	cv::Size m_cascade_wnd_size;

public:
	LPRecognizer();
	~LPRecognizer();

	std::vector<cv::Rect> process_frame(const cv::Mat& frame, cv::Mat& debug_frame);

private:

	bool initCascadeClassifier();
};

