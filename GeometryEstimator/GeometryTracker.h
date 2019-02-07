#pragma once
#include <memory>
#include <vector>
#include <algorithm>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#define MIN_TRACK_SIZE 5
#define MAX_MISSED_FRAMES 15

#define MAX_FEATURES 200
#define LK_WIN_SIZE_SCALE 0.025
#define FEAUTERS_DIST_MIN 0.01

#define ROTHER_MAX_ANG 5.0
#define ROTHER_COEF_LEN 0.8
#define ROTHER_COEF_ANG 0.2

//#define DEGRAD  57.295779513082320876798154814105 // from deg to rad coeff



class GeometryTracker
{
private:
	struct Track
	{
		std::vector<cv::Point2f> points;
		size_t missed_frames = 0;
	};

	int m_frame_width;
	int m_frame_height;

	cv::Mat m_gray_prev;
	bool m_initial_loop;
	cv::Size m_lk_win_size;

	std::vector<Track> m_process_tracks;
	std::vector<Track> m_finished_tracks;
	std::vector<cv::Point2f> m_points_prev;

	std::unique_ptr<cv::TermCriteria> p_termcrit;

	// tmp
	cv::Point2f m_vpoint;

public:
	GeometryTracker();
	~GeometryTracker();

	void clear();
	bool process_frame(const cv::Mat &frame);

private:
	void filter_finished_tracks();
};



