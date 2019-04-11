#pragma once
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

class LPlate
{
private:
	cv::Rect rect;

public:
	LPlate() = default;
	~LPlate() = default;
	LPlate(const cv::Rect& rect) { this->rect = rect; };

	cv::Point get_center() const
	{
		return cv::Point(rect.x + (rect.width / 2), rect.y + (rect.height / 2));
	};

	const cv::Rect* get_rect() const
	{
		return &rect;
	};
};

class LPTrack
{
public:
	LPTrack();
	LPTrack(const LPlate& plate);
	LPTrack(const LPlate& plate, cv::Scalar color);
	~LPTrack();

	void add_plate(const LPlate& plate);
	const std::vector<LPlate>* get_plates() const;

	int get_average_age() const;

	int lost_frames() const;
	void inc_lost_frames();
	void clean_lost_frames();

	cv::Scalar color() const;
	void set_color(cv::Scalar color);
	
	int average_plate_width() const;
	int average_plate_height() const;

	double lenght() const;

private:
	std::vector<LPlate> m_plates;
	cv::Scalar m_color;
	int m_lost_frames;
};

