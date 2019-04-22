#pragma once
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// TODO:
// 1. Add methods "bound by height, bound by width"...
//

class LPRecognizerZone
{
private:
	cv::Rect m_zone;
	cv::Size m_frame_size;
	cv::Size m_plate_size;	
	double m_points_density;
	std::vector<std::pair<cv::Point, size_t>> m_points;
	cv::Scalar m_color;

public:
	LPRecognizerZone();
	LPRecognizerZone(const cv::Rect& zone, const cv::Size& frame_size, const cv::Size& plate_size);
	~LPRecognizerZone() = default;

	// Processing methods
	void clear();
	void calibrate();
	void add_points(const std::vector<cv::Point>& points);
	void add_points(const std::vector<cv::Rect>& rects);

	// Setters
	void set_zone(const cv::Rect& zone);
	void set_color(const cv::Scalar& color);
	void set_plate_size(const cv::Size& size);
	void set_frame_size(const cv::Size& frame_size);
	
	// Getters
	cv::Rect zone() const;
	cv::Scalar color() const;
	size_t points_size() const;	
	cv::Size plate_size() const;

	// Debug methods
	void print(cv::Mat image) const;
	
private:
	cv::Rect bound_points();
};

