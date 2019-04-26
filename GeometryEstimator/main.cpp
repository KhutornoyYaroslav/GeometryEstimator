#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <chrono>
#include <thread>

#include "GeometryCommon.h"
#include "OpticalFlowTracker.h"
#include "AutoFocus.h"
#include "LPRecognizer.h"
#include "LPTracker.h"

using namespace cv;
using namespace std;

size_t counter = 0;
long long time_sum = 0;

int main(int argc, char** argv)
{
	cv::Mat frame;
	size_t frame_counter = 0;

	VideoCapture cap;
	LPRecognizer lprecognizer;
	LPTracker lptracker;

	cap.open("..\\videos\\5.mp4");

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing.. \r\n";
		cin.get();
		return 0;
	}

	if (!lprecognizer.init())
	{
		cerr << "Error while initing LPRecognizer. \r\n";
		cin.get();
		return 0;
	}

	//if (!lprecognizer.load_from_json("config_5.json"))
	//{
	//	cerr << "Error while reading config.json file. \r\n";
	//	cin.get();
	//	return 0;
	//}

	if (!lprecognizer.start_calibration())
	{
		cerr << "Error while starting calibration. \r\n";
		cin.get();
		return 0;
	}

	lprecognizer.set_min_plate_size(cv::Size(25, 9));

	while(true)
	{		
		// Capture new frame
		if (cap.read(frame))
		{
			++frame_counter;

			if (frame_counter == static_cast<size_t>(cap.get(CV_CAP_PROP_FRAME_COUNT)))
			{
				frame_counter = 0;
				cap.set(CV_CAP_PROP_POS_FRAMES, 0);
			}
		}


		/*cv::Mat frame_resize;
		cv::resize(frame, frame_resize, cv::Size(), 2.0, 2.0);	
		cv::Mat debug_resize;
		frame_resize.copyTo(debug_resize);*/

		auto start = std::chrono::high_resolution_clock::now();
		lprecognizer.capture_frame(frame);

		std::vector<cv::Rect> result;
		lprecognizer.detect(result);

		for (auto& p : result)
			cv::rectangle(frame, p, cv::Scalar(0, 255, 0), 2, CV_AA);

		//printf("plates detected %u \r\n", result.size());

		auto end = std::chrono::high_resolution_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(start - end).count();

		++counter;
		if (counter >= 100)
		{

			auto avr = (double)time_sum / counter;

			std::cout << "FPS = " << abs(1000.0 / avr) << std::endl;
			time_sum = 0;
			counter = 0;
			
			if (lprecognizer.is_calibration_finished())
			{
				//lprecognizer.save_to_json("config_5.json");
				std::cout << "Calibration finisehd \r\n";
			}
		}
		else
		{
			time_sum += time;
		}

		//cv::Mat debug_resize_;
		//cv::resize(debug_resize, debug_resize_, cv::Size(), 0.35, 0.35);
		//imshow("Debug", debug_resize_);
		//cvWaitKey(1);


		imshow("Frame", frame);
		cvWaitKey(1);






	}

	system("pause");
	return 0;
}