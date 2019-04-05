#include "LPRecognizer.h"

LPRecognizer::LPRecognizer()
{
	p_plate_detector = std::make_unique<cv::CascadeClassifier>();
	initCascadeClassifier();
}

LPRecognizer::~LPRecognizer()
{
}

bool LPRecognizer::initCascadeClassifier()
{
	if (!p_plate_detector)
		return false;

	if (p_plate_detector->load(cv::String("haarcascade_russian_plate_number.xml")))
	{
		m_cascade_wnd_size = p_plate_detector->getOriginalWindowSize();
		return true;
	}
	else
	{
		return false;
	}
};

std::vector<cv::Rect> LPRecognizer::process_frame(const cv::Mat& frame, cv::Mat& debug_frame)
{
	cv::Mat gray_frame;

	if (frame.empty() || frame.size().area() == 0)
		return {};

	if (frame.type() != CV_8UC1)
		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(gray_frame);

	if (p_plate_detector->empty())
	{
		printf("Can't load classifier cascade file \r\n");
		return {};
	}

	std::vector<cv::Rect> plates;
	p_plate_detector->detectMultiScale(gray_frame, plates, 1.1, 3, 0, m_cascade_wnd_size, m_cascade_wnd_size * 3);

	//if (plates.empty())
		//printf("No detect! \r\n");

	// Print debug
	if (!debug_frame.empty() && frame.size().area() != 0)
	{
		for (auto& p : plates)
			cv::rectangle(debug_frame, p, cv::Scalar(0, 255, 0), 2, CV_AA);
	}

	return plates;
};
