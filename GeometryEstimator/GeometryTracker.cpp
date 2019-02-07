#include "GeometryTracker.h"

GeometryTracker::GeometryTracker()
{
	p_termcrit = std::make_unique<cv::TermCriteria>(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
	clear();
};

GeometryTracker::~GeometryTracker()
{
};

void GeometryTracker::clear()
{
	m_initial_loop = true;
	m_points_prev.clear();
	m_process_tracks.clear();
	m_finished_tracks.clear();
	m_gray_prev.release();
};

size_t GeometryTracker::tracks_count() const
{
	return m_finished_tracks.size();
};

void GeometryTracker::get_tracks(std::vector<std::vector<cv::Point2f>>& tracks) const
{
	tracks.resize(m_finished_tracks.size());

	for (size_t i = 0; i < m_finished_tracks.size(); ++i)
	{
		tracks[i] = m_finished_tracks[i].points;
	}
};

bool GeometryTracker::process_frame(const cv::Mat &frame)
{
	cv::Mat gray_frame;
	std::vector<cv::Point2f> points;

	if (frame.empty() || frame.size().area() == 0)
		return false;

	if (frame.type() != CV_8UC1)
		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(gray_frame);

	cv::Size lk_win_size = { static_cast<int>(std::max(std::round(gray_frame.cols * LK_WIN_SIZE_SCALE), 3.0)),
							 static_cast<int>(std::max(std::round(gray_frame.rows * LK_WIN_SIZE_SCALE), 3.0)) };

	if (m_initial_loop)
	{
		const double min_dist = std::ceil(FEAUTERS_DIST_MIN * sqrt(gray_frame.cols * gray_frame.rows));
		cv::goodFeaturesToTrack(gray_frame, points, MAX_FEATURES, 0.01, min_dist);
		cv::cornerSubPix(gray_frame, points, cv::Size(min_dist, min_dist), cv::Size(-1, -1), *p_termcrit);

		m_process_tracks.clear();
		m_process_tracks.resize(points.size());

		for (size_t i = 0; i < points.size(); ++i)
		{
			m_process_tracks[i].points.push_back(points[i]);
			m_process_tracks[i].missed_frames = 0;
		}

		m_initial_loop = false;
	}
	else
	{
		if (!m_points_prev.empty())
		{
			if (m_gray_prev.size() != gray_frame.size())				
				return false;

			std::vector<float> err;
			std::vector<uchar> status;
			cv::calcOpticalFlowPyrLK(m_gray_prev, gray_frame, m_points_prev, points, status, err, lk_win_size, 3, *p_termcrit, 0, 0.005);

			size_t i, k;
			for (i = k = 0; i < points.size(); ++i)
			{
				if (status[i] == 0 || m_process_tracks[i].missed_frames > MAX_MISSED_FRAMES)
				{
					if (m_process_tracks[i].points.size() > MIN_TRACK_SIZE)
						m_finished_tracks.push_back(m_process_tracks[i]);

					continue;
				}
			
				LineF line_result;
				std::vector<cv::Point2f> line_points;
				line_points.insert(std::end(line_points), std::begin(m_process_tracks[i].points), std::end(m_process_tracks[i].points));
				line_points.push_back(points[i]);

				if (fitLineRansac(1.0, line_points.size(), line_points, line_result))
				{
					if (cv::norm(m_process_tracks[i].points.back() - points[i]) < 5.0)
					{
						++m_process_tracks[i].missed_frames;
					}
					else
					{
						m_process_tracks[i].points.push_back(points[i]);
						m_process_tracks[i].missed_frames = 0;
					}
				}
				else
				{
					if (m_process_tracks[i].points.size() > MIN_TRACK_SIZE)
						m_finished_tracks.push_back(m_process_tracks[i]);

					continue;
				}

				points[k] = points[i];
				m_process_tracks[k] = m_process_tracks[i];
				++k;
			}

			points.resize(k);
			m_process_tracks.resize(k);
		}
		else
		{
			m_initial_loop = true;
		}
	}

	m_points_prev.clear();
	m_points_prev.insert(std::end(m_points_prev), std::begin(points), std::end(points));
	gray_frame.copyTo(m_gray_prev);
};
