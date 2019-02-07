#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <ctype.h>

#include "GeometryCommon.h"
#include "GeometryTracker.h"


using namespace cv;
using namespace std;

#define TRACK_COUNT 250

int main(int argc, char** argv)
{

	std::vector<std::vector<cv::Point2f>> good_tracks;

	cv::Point2f dir_vpoint;

	VideoCapture cap;
	GeometryTracker tracker;

	cap.open("..\\videos\\6.mp4");
	//cap.open("..\\videos\\18.avi");
	//cap.open("..\\videos\\ufa2.mkv");

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}
	namedWindow("LK Demo", 1);

	cv::Mat frame;

	for (;;)
	{
		cap >> frame;
		cv::Mat frame_resize;
		cv::resize(frame, frame_resize, cv::Size(frame.cols * 0.5, frame.rows * 0.5));

		if (!tracker.process_frame(frame_resize))
		{
			printf("some error! \r\n");
		}

		if (tracker.tracks_count() > TRACK_COUNT)
		{
			std::vector<std::vector<cv::Point2f>> tracks;
			tracker.get_tracks(tracks);
			tracker.clear();

			std::vector<LineF> rother_lines;	

			for (size_t i = 0; i != tracks.size(); ++i)
			{
				LineF line_result;	
				fitLineRansac(5.0, tracks[i].size(), tracks[i], line_result);
				rother_lines.push_back(line_result);
			}
					
			EstimateRotherVP(rother_lines, dir_vpoint, { 0, 0 }, false);
			
			for (size_t i = 0; i != tracks.size(); ++i)
			{
				LineF line_result;
				fitLineRansac(5.0, tracks[i].size(), tracks[i], line_result);

				auto dist = abs(line_result.A() * dir_vpoint.x + line_result.B() * dir_vpoint.y + line_result.C()) / sqrt(line_result.A()*line_result.A() + line_result.B() * line_result.B());

				if (dist < 50.0)
				{
					good_tracks.push_back(tracks[i]);
				}
			}

			const double pixs_in_deg = static_cast<double>(frame_resize.cols) / 25.5;
			double dx = (frame_resize.cols / 2) - dir_vpoint.x;
			double dy = (frame_resize.rows / 2) - dir_vpoint.y;
			printf("lines = %u, rotate = %.1f, slope = %.1f \n", rother_lines.size(), dx / pixs_in_deg, dy / pixs_in_deg);
			//system("pause");
		}

		for (size_t i = 0; i < good_tracks.size(); ++i)
		{
			for (size_t j = 0; j < good_tracks[i].size() - 1; ++j)
			{
				cv::line(frame_resize, good_tracks[i][j], good_tracks[i][j + 1], cv::Scalar(0, 0, 225), 1, CV_AA);
				cv::circle(frame_resize, good_tracks[i][j], 1, cv::Scalar(155, 155, 225), 1, CV_AA);
			}
		}
		
		cv::circle(frame_resize, { (int)dir_vpoint.x, (int)dir_vpoint.y }, 1, cv::Scalar(0, 0, 255), 10, CV_AA);

		printf("tracks count = %u \r\n", tracker.tracks_count());

		imshow("LK Demo", frame_resize);
		cvWaitKey(1);
	}

	system("pause");
	return 0;
}