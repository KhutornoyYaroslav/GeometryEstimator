#include "AutoFocus.h"

double AutoFocus::process_frame(const cv::Mat frame)
{
	return this->estimate_focus(frame);
};

double AutoFocus::estimate_focus(const cv::Mat frame)
{
	cv::Mat gray_frame;
	cv::Mat Gx, Gy, grad_frame;

	if (frame.empty() || frame.size().area() == 0)
		return 0.0;

	if (frame.type() != CV_8UC1)
		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(gray_frame);

	cv::Sobel(gray_frame, Gx, CV_64F, 1, 0);
	cv::Sobel(gray_frame, Gy, CV_64F, 0, 1);
	cv::sqrt(Gx.mul(Gx) + Gy.mul(Gy), grad_frame);

	return cv::mean(grad_frame).val[0];
};

void AutoFocus::gaussianFilter(std::vector<double>& data, double sigma)
{
	// Create filter response window
	int window_size = ceil(abs(sigma) * 3);
	double *imp = new double[2 * window_size + 1];
	const double alpha = 1 / (sqrt(2.0*3.14) * sigma);

	for (int i = 0; i < 2 * window_size + 1; ++i)
	{
		int x = i - window_size;
		imp[i] = alpha * exp(-(x*x) / (2 * sigma*sigma));
	}

	// Filter input data
	double *input = new double[2 * window_size + 1];
	for (int i = 0; i < 2 * window_size + 1; ++i)
		input[i] = 0.0;

	for (size_t i = 0; i < data.size(); ++i)
	{
		// get new data item
		input[0] = data[i];

		// compute result
		double result = 0.0;
		for (int j = 0; j < 2 * window_size + 1; ++j)
			result = result + imp[j] * input[j];

		// save result
		data[i] = result;

		// shift input data
		for (int s = 2 * window_size; s >= 0; --s)
			input[s] = input[s - 1];
	}

	delete[] imp;
	delete[] input;
};

void AutoFocus::medianFilter(std::vector<double>& data, uint32_t wnd_size)
{
	uint32_t window_size = 0;
	std::vector<double> window;
	std::vector<double> tmp;

	// check input data
	if (data.empty())
		return;

	if (wnd_size % 2 == 0)
		window_size = wnd_size + 1;
	else
		window_size = wnd_size;

	// initializing window
	auto it_wnd_new = data.begin();
	window.resize(window_size, *it_wnd_new);
	tmp.resize(window_size);

	for (size_t i = (window_size - 1) / 2; i < window.size(); ++i)
	{
		window[i] = *it_wnd_new;
		it_wnd_new = (++it_wnd_new == data.end()) ? --it_wnd_new : it_wnd_new;
	}

	// filter data
	for (size_t shift_id = 0; shift_id < data.size(); ++shift_id)
	{
		// Compute median
		std::copy(window.begin(), window.end(), tmp.begin());
		std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());

		// Save result
		data[shift_id] = tmp[tmp.size() / 2];

		// Shift window
		for (size_t i = 0; i < window.size() - 1; ++i)
			window[i] = window[i + 1];

		window[window.size() - 1] = *it_wnd_new;
		it_wnd_new = (++it_wnd_new == data.end()) ? --it_wnd_new : it_wnd_new;
	}
};