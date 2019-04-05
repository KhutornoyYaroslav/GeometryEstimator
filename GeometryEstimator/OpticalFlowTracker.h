#pragma once
#include <memory>
#include <vector>
#include <algorithm>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"

#include "GeometryCommon.h"

#define MIN_TRACK_SIZE 8
#define MAX_MISSED_FRAMES 15

#define MAX_FEATURES 100
#define LK_WIN_SIZE_SCALE 0.025
#define FEAUTERS_DIST_MIN 0.01

class OpticalFlowTracker
{
private:
	struct Track
	{
		std::vector<cv::Point2f> points;
		size_t missed_frames = 0;
	};

	cv::Mat m_gray_prev;
	bool m_initial_loop;
	std::vector<Track> m_process_tracks;
	std::vector<Track> m_finished_tracks;
	std::vector<cv::Point2f> m_points_prev;
	std::unique_ptr<cv::TermCriteria> p_termcrit;

public:
	OpticalFlowTracker();
	~OpticalFlowTracker() = default;

	void clear();
	size_t tracks_count() const;
	bool process_frame(const cv::Mat& frame);	
	void get_tracks(std::vector<std::vector<cv::Point2f>>& tracks) const;
};



