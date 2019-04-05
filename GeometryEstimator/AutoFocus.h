#pragma once
#include "opencv2/imgproc.hpp"

class AutoFocus
{
public:
	AutoFocus() = default;
	~AutoFocus() = default;

	double process_frame(const cv::Mat frame);
	double estimate_focus(const cv::Mat frame);
	void gaussianFilter(std::vector<double>& data, double sigma);
	void medianFilter(std::vector<double>& data, uint32_t wnd_size);
};

