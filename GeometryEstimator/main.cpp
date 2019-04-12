#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <ctype.h>
#include <algorithm>

#include "GeometryCommon.h"
#include "OpticalFlowTracker.h"
#include "AutoFocus.h"

#include "LPRecognizer.h"
#include "LPTracker.h"
//#include "HungarianAlgorithm.h"

#include "munkres.h"

using namespace cv;
using namespace std;

#define TRACK_COUNT 1

int main(int argc, char** argv)
{
	//cv::Mat scores(5, 5, CV_32FC1);
	//scores.setTo(cv::Scalar(1)); // 1 ?

	//scores.at<float>(0, 0) = 7; scores.at<float>(0, 1) = 3; scores.at<float>(0, 2) = 6; scores.at<float>(0, 3) = 9; scores.at<float>(0, 4) = 5;
	//scores.at<float>(1, 0) = 7; scores.at<float>(1, 1) = 5; scores.at<float>(1, 2) = 7; scores.at<float>(1, 3) = 5; scores.at<float>(1, 4) = 6;
	//scores.at<float>(2, 0) = 7; scores.at<float>(2, 1) = 6; scores.at<float>(2, 2) = 8; scores.at<float>(2, 3) = 8; scores.at<float>(2, 4) = 9;
	//scores.at<float>(3, 0) = 3; scores.at<float>(3, 1) = 1; scores.at<float>(3, 2) = 6; scores.at<float>(3, 3) = 5; scores.at<float>(3, 4) = 7;
	//scores.at<float>(4, 0) = 2; scores.at<float>(4, 1) = 4; scores.at<float>(4, 2) = 9; scores.at<float>(4, 3) = 9; scores.at<float>(4, 4) = 5;

	//std::cout << "Data: \n" << scores << std::endl;

	////scores = 1 / scores;

	//Munkres<float> hunharian;
	//cv::Mat res = hunharian.solve(scores);
	//

	//std::cout << "Res: \n" << res << std::endl;
	//system("pause");


	std::vector<double> results;
	std::vector<std::vector<cv::Point2f>> good_tracks;

	cv::Point2f dir_vpoint;

	VideoCapture cap;
	OpticalFlowTracker tracker;
	LPRecognizer lprecognizer;
	LPRecognizer lprecognizer2;
	LPRecognizer lprecognizer3;


	LPTracker lptracker;
	cap.open("..\\videos\\5.mp4");
	//cap.open("..\\videos\\16.avi");
	//cap.open("..\\videos\\ufa2.mkv");

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	cv::Mat frame;

	for (;;)
	{
		cap >> frame;

		cv::Mat frame_resize;
		cv::resize(frame, frame_resize, cv::Size(), 1.0, 1.0);	
		cv::Mat debug_resize(frame_resize);
		//auto plates = lprecognizer.process_frame(frame_resize, debug_resize);
		//lprecognizer.calibrate_zones(frame_resize, debug_resize);
		//auto plates = lprecognizer.process_frame(frame_resize, debug_resize);
		lptracker.process_frame(frame_resize, debug_resize);

		//cv::Mat debug_resize_;
		//cv::resize(debug_resize, debug_resize_, cv::Size(), 1.25, 1.25);
		imshow("Debug", debug_resize);
		cvWaitKey(1);




		/*cv::Mat frame_resize2;
		cv::resize(frame, frame_resize2, cv::Size(), 2.0, 2.0);
		cv::Mat debug_resize2(frame_resize2);
		plates = lprecognizer2.process_frame(frame_resize2, debug_resize2);
		cv::Mat debug_resize2_;
		cv::resize(debug_resize2, debug_resize2_, cv::Size(), 0.25, 0.25);
		imshow("Debug2", debug_resize2_);
		cvWaitKey(1);




		cv::Mat frame_resize3;
		cv::resize(frame, frame_resize3, cv::Size(), 1.0, 1.0);
		cv::Mat debug_resize3(frame_resize3);
		plates = lprecognizer3.process_frame(frame_resize3, debug_resize3);
		cv::Mat debug_resize3_;
		cv::resize(debug_resize3, debug_resize3_, cv::Size(), 0.5, 0.5);
		imshow("Debug3", debug_resize3_);
		cvWaitKey(1);*/




		//if (!tracker.process_frame(frame_resize))
		//{
		//	printf("some error! \r\n");
		//}

		//tracker.process_frame3(frame_resize);

		//if (tracker.tracks_count() > TRACK_COUNT)
		//{
		//	std::vector<std::vector<cv::Point2f>> tracks;
		//	tracker.get_tracks(tracks);
		//	//tracker.clear();

		//	std::vector<LineF> rother_lines;	

		//	for (size_t i = 0; i != tracks.size(); ++i)
		//	{
		//		LineF line_result;	
		//		fitLineRansac(5.0, tracks[i].size(), tracks[i], line_result);
		//		rother_lines.push_back(line_result);
		//	}
		//			
		//	EstimateRotherVP(rother_lines, dir_vpoint, { 0, 0 }, false);
		//	
		//	for (size_t i = 0; i != tracks.size(); ++i)
		//	{
		//		LineF line_result;
		//		fitLineRansac(5.0, tracks[i].size(), tracks[i], line_result);

		//		auto dist = abs(line_result.A() * dir_vpoint.x + line_result.B() * dir_vpoint.y + line_result.C()) / sqrt(line_result.A()*line_result.A() + line_result.B() * line_result.B());

		//		if (dist < 50.0)
		//		{
		//			good_tracks.push_back(tracks[i]);
		//		}
		//	}

		//	const double pixs_in_deg = static_cast<double>(frame_resize.cols) / 24.0;
		//	double dx = (frame_resize.cols / 2) - dir_vpoint.x;
		//	double dy = (frame_resize.rows / 2) - dir_vpoint.y;
		//	printf("lines = %u, rotate = %.1f, slope = %.1f \n", rother_lines.size(), dx / pixs_in_deg, dy / pixs_in_deg);
		//	//system("pause");
		//}

	/*	for (size_t i = 0; i < good_tracks.size(); ++i)
		{
			for (size_t j = 0; j < good_tracks[i].size() - 1; ++j)
			{
				cv::line(frame_resize, good_tracks[i][j], good_tracks[i][j + 1], cv::Scalar(0, 0, 225), 1, CV_AA);
				cv::circle(frame_resize, good_tracks[i][j], 1, cv::Scalar(155, 155, 225), 1, CV_AA);
			}
		}
		
		cv::circle(frame_resize, { (int)dir_vpoint.x, (int)dir_vpoint.y }, 1, cv::Scalar(0, 0, 255), 10, CV_AA);*/


		/*std::vector<std::vector<cv::Point2f>> tracks;
		tracker.get_tracks(tracks);

		for (size_t i = 0; i < tracks.size(); ++i)
		{
			for (size_t j = 0; j < tracks[i].size() - 1; ++j)
			{
				cv::line(frame_resize, tracks[i][j], tracks[i][j + 1], cv::Scalar(0, 0, 225), 1, CV_AA);
				cv::circle(frame_resize, tracks[i][j], 1, cv::Scalar(155, 155, 225), 1, CV_AA);
			}
		}


		printf("tracks count = %u \r\n", tracker.tracks_count());
		*/
		/*imshow("LK Demo", frame_resize);
		cvWaitKey(1);*/

	}

	system("pause");
	return 0;
}