#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <ctype.h>

#include "GeometryTracker.h"

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
	VideoCapture cap;
	GeometryTracker tracker;

	//cap.open("20.mp4");
	//cap.open("18.avi");
	cap.open("..\\videos\\ufa.mkv");

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
		cv::resize(frame, frame_resize, cv::Size(frame.cols * 0.75, frame.rows * 0.75));

		tracker.process_frame(frame_resize);
		imshow("LK Demo", frame_resize);

		cvWaitKey(1);
	}

	return 0;
}