#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <chrono>

#include "LPRecognizer.h"
#include "LPTracker.h"
#include "LPTrack.h"
#include "FrameCapture.h"

using namespace std;

size_t counter = 0;
long long time_sum = 0;
bool is_calib_finished = false;
std::vector<LPTrack> track_history;

int main(int argc, char** argv)
{
	cv::Mat videoframe;
	FrameCapture debugger;
	LPTracker lptracker;
	LPRecognizer lprecognizer;

	if (!debugger.open_video("..\\videos\\5_x2.mp4"))
	{
		cout << "Could not open video file. \r\n";
		cin.get();
		return -1;
	}

	if (!lprecognizer.init())
	{
		cout << "Could not init recognizer. \r\n";
		cin.get();
		return -1;
	}

	//lprecognizer.set_min_plate_size(cv::Size(60, 20));
	lprecognizer.set_min_plate_size(cv::Size(30, 10));

	if (!lprecognizer.load_from_json("config.json"))
	{
		cout << "Could not load recognizer config file. \r\n";
		cout << "Start automatical calibration. \r\n";

		if (!lprecognizer.start_calibration())
		{
			cout << "Could not start recognizer calibration. \r\n";
		}
	}

	while(true)
	{		
		if (!debugger.next_frame(videoframe, 0.5))
		{
			cout << "Could not read next video frame. \r\n";
			break;
		}
		
		//printf("counter = %u \r\n", counter_);
		auto start = std::chrono::high_resolution_clock::now();	

		// ********* MAIN SECTION *********
		if (!lprecognizer.capture_frame(videoframe))
			continue;
		
		if (lprecognizer.is_calibration_finished() && !is_calib_finished)
			if (!lprecognizer.save_to_json("config.json"))
				cout << "Could not save recognizer to file. \r\n";
			else
				cout << "Calibration finished. \r\n";
				

		is_calib_finished = lprecognizer.is_calibration_finished();


		std::vector<cv::Rect> detected_plates;
		lprecognizer.detect(detected_plates);

		//if (lprecognizer.detect(detected_plates))
		//	lptracker.process_plates(videoframe, detected_plates);

		//lptracker.pull_tracks(track_history);



		//cout << "Finished tracks size = " << track_history.size() << endl;

		//for (size_t i = 0; i < track_history.size(); ++i)
		//{
		//	auto& track = track_history[i];

		//	for (size_t j = 0; j < track.get_plates()->size(); ++j)
		//	{
		//		const auto &t_plate = track.get_plates()->at(j);

		//		cv::Mat plate_debug;
		//		t_plate.get_image()->copyTo(plate_debug);

		//		imshow("Plate", plate_debug);
		//		cvWaitKey(1);
		//	}
		//}


	/*	for (size_t i = 0; i < track_history.size(); ++i)
		{
			const auto& color = track_history[i].color();
			auto& track = track_history[i];

			for (size_t j = 0; j < track.get_plates()->size() - 1; ++j)
			{
				const auto rect = track.get_plates()->at(j).get_rect();
				const auto rect_center = track.get_plates()->at(j).center_position();

				const auto rect_next = track.get_plates()->at(j + 1).get_rect();
				const auto rect_center_next = track.get_plates()->at(j + 1).center_position();
				cv::circle(videoframe, rect_center, 2, color, 2);

				cv::line(videoframe, rect_center, rect_center_next, color, 1, CV_AA);
			}

			const auto rect = track.get_plates()->back().get_rect();
			const auto rect_center = track.get_plates()->back().center_position();
			cv::circle(videoframe, rect_center, 2, color, 2);
		}*/

		// ********************************

		auto end = std::chrono::high_resolution_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(start - end).count();

		++counter;
		if (counter >= 100)
		{
			auto avr = (double)time_sum / counter;
			std::cout << "FPS = " << abs(1000.0 / avr) << std::endl;
			time_sum = 0;
			counter = 0;
		}
		else
			time_sum += time;

		//cv::Mat frame_resize_;
		//cv::resize(videoframe, frame_resize_, cv::Size(), 0.45, 0.45);
		//imshow("Debug", frame_resize_);
		//cvWaitKey(1);
	}

	cin.get();
	return 0;
}