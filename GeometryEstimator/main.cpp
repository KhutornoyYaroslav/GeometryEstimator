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
	OpticalFlowTracker tracker;
	LPRecognizer lprecognizer;
	LPTracker lptracker;

	//cap.open("..\\videos\\6.mp4");
	//cap.open("..\\videos\\23.avi");
	cap.open("..\\videos\\ufa2.mkv");

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	for (;;)
	{
		++counter;
			
		if (cap.read(frame))
		{
			++frame_counter;

			if (frame_counter == static_cast<size_t>(cap.get(CV_CAP_PROP_FRAME_COUNT)))
			{
				printf("****************** RESTART VIDEO! ******************\r\n");
				frame_counter = 0;
				cap.set(CV_CAP_PROP_POS_FRAMES, 0);
			}
		}


		//cv::Mat frame_resize;
		//cv::resize(frame, frame_resize, cv::Size(), 2.0, 2.0);	
		cv::Mat debug_resize;
		frame.copyTo(debug_resize);

		auto start = std::chrono::high_resolution_clock::now();

		lprecognizer.calibrate_zones(frame, debug_resize);
		auto plates = lprecognizer.process_frame(frame, debug_resize);
		
		//lptracker.process_frame(frame, debug_resize);

		auto end = std::chrono::high_resolution_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(start - end).count();

		if (counter >= 100)
		{
			auto avr = (double)time_sum / counter;

			std::cout << "FPS = " << abs(1000.0 / avr) << std::endl;
			time_sum = 0;
			counter = 0;
		}
		else
		{
			time_sum += time;
		}

		cv::Mat debug_resize_;
		cv::resize(debug_resize, debug_resize_, cv::Size(), 0.75, 0.75);
		imshow("Debug", debug_resize_);
		cvWaitKey(1);
	}

	system("pause");
	return 0;
}