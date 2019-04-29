#pragma once
//#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"

#include "LPPlate.h"

class LPTrack
{
private:
	std::vector<LPPlate> m_plates;
	cv::Scalar m_color;
	int m_lost_frames;

public:
	LPTrack();
	LPTrack(const LPPlate& plate);
	LPTrack(const LPPlate& plate, cv::Scalar color);
	~LPTrack();

	void add_plate(const LPPlate& plate);
	const std::vector<LPPlate>* get_plates() const;

	int get_average_age() const;

	int lost_frames() const;
	void inc_lost_frames();
	void clean_lost_frames();

	cv::Scalar color() const;
	void set_color(cv::Scalar color);
	
	int average_plate_width() const;
	int average_plate_height() const;

	double lenght() const;
};

