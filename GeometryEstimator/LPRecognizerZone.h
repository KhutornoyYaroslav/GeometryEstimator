#pragma once
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// TODO:
// оптимизировать работу с точками (обработанные старые сохранять, второй раз не пересчитывать)

class LPRecognizerZone
{
private:
	cv::Rect m_zone;
	cv::Point m_frame_size;
	std::vector<cv::Rect> m_rects;

public:
	LPRecognizerZone();
	LPRecognizerZone(const cv::Rect& zone, const cv::Point& frame_size);
	~LPRecognizerZone() = default;

	void clear();
	void calibrate();

	void set_zone(const cv::Rect& zone);
	void add_plates(const std::vector<cv::Rect>& plates);
	void set_frame_size(const cv::Point& frame_size);

	size_t size() const;
	cv::Rect rect() const;
	void print(cv::Mat image) const;

private:
	cv::Rect bound_plates();
};

