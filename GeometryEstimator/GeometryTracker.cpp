#include "GeometryTracker.h"
#include "GeometryCommon.h"




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
	m_frame_width = 0;
	m_frame_height = 0;
	m_lk_win_size = {};
	m_initial_loop = true;
	m_points_prev.clear();
	m_process_tracks.clear();
	m_finished_tracks.clear();

	// TODO: m_gray_prev
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

	if (m_initial_loop)
	{
		/// to init function ?
		m_frame_width = gray_frame.cols;
		m_frame_height = gray_frame.rows;

		m_lk_win_size.width = static_cast<int>(std::max(std::round(m_frame_width * LK_WIN_SIZE_SCALE), 3.0));
		m_lk_win_size.height = static_cast<int>(std::max(std::round(m_frame_height * LK_WIN_SIZE_SCALE), 3.0));

		const double min_dist = std::ceil(FEAUTERS_DIST_MIN * sqrt(m_frame_width * m_frame_height));
		/// ...


		cv::goodFeaturesToTrack(gray_frame, points, MAX_FEATURES, 0.01, min_dist);
		cv::cornerSubPix(gray_frame, points, cv::Size(min_dist, min_dist), cv::Size(-1, -1), *p_termcrit);

		//m_points_prev.clear();
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
			//if (m_gray_prev.size() != frame.size())
			//{
			//	return false;
			//}

			std::vector<float> err;
			std::vector<uchar> status;
			cv::calcOpticalFlowPyrLK(m_gray_prev, gray_frame, m_points_prev, points, status, err, m_lk_win_size, 3, *p_termcrit, 0, 0.005);

			size_t i, k;
			for (i = k = 0; i < points.size(); ++i)
			{
				if (status[i] == 0 || m_process_tracks[i].missed_frames >= MAX_MISSED_FRAMES)
				{
					if (m_process_tracks[i].points.size() > MIN_TRACK_SIZE)
						m_finished_tracks.push_back(m_process_tracks[i]);

					continue;
				}

				LineF line_result;
				std::vector<cv::Point2f> line_points;
				line_points.insert(std::end(line_points), std::begin(m_process_tracks[i].points), std::end(m_process_tracks[i].points));
				line_points.push_back(points[i]);

				if (fitLineRansac(1.5, line_points.size(), line_points, line_result))
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
			filter_finished_tracks();
		}
	}

	m_points_prev.clear();
	m_points_prev.insert(std::end(m_points_prev), std::begin(points), std::end(points));
	cv::swap(m_gray_prev, gray_frame);



	// PRINT RESULT TRACKS

	for (size_t i = 0; i < m_finished_tracks.size(); ++i)
	{
		for (size_t j = 0; j < m_finished_tracks[i].points.size() - 1; ++j)
		{
			cv::line(frame, m_finished_tracks[i].points.at(j), m_finished_tracks[i].points.at(j + 1), cv::Scalar(0, 0, 225), 1, CV_AA);
			cv::circle(frame, m_finished_tracks[i].points[j], 1, cv::Scalar(155, 155, 225), 1, CV_AA);
		}
	}
	cv::circle(frame, { (int)m_vpoint.x, (int)m_vpoint.y }, 1, cv::Scalar(0, 0, 255), 10, CV_AA);


	//for (auto it = m_finished_tracks.begin(); it != m_finished_tracks.end(); ++it)
	//{
	//	std::vector<graphics::PointF> points_;
	//	graphics::LineF l;
	//	for (auto &p : it->points)
	//		points_.push_back({ p.x, p.y });

	//	fitLineRansac(5.0, points_.size(), points_, l);

	//	if (abs(sqrt(l.A() * l.A() + l.B() * l.B())) <= DBL_EPSILON)
	//	{
	//		continue;
	//	}

	//	double dist2point = abs(l.A() * (int)m_vpoint.x + l.B() * (int)m_vpoint.y + l.C()) / sqrt(l.A()*l.A() + l.B() * l.B());

	//	if (dist2point < 50.0)
	//	{
	//		for (size_t j = 0; j < it->points.size() - 1; ++j)
	//		{
	//			cv::line(frame, it->points.at(j), it->points.at(j + 1), cv::Scalar(0, 0, 225), 1, CV_AA);
	//			cv::circle(frame, it->points[j], 1, cv::Scalar(155, 155, 225), 1, CV_AA);
	//		}
	//	}
	//}


};

void GeometryTracker::filter_finished_tracks()
{
	/*std::vector<graphics::LineF> rother_lines;

	for (auto it = m_finished_tracks.begin(); it != m_finished_tracks.end(); ++it)
	{
		std::vector<graphics::PointF> points_;
		graphics::LineF l;
		for (auto &p : it->points)
			points_.push_back({ p.x, p.y });

		fitLineRansac(5.0, points_.size(), points_, l);

		rother_lines.push_back(l);
	}

	EstimateRotherVP(rother_lines, m_vpoint, graphics::Point(m_frame_width, m_frame_height), true);

	const double pixs_in_deg = static_cast<double>(m_frame_width) / 10.5;
	double dx = (m_frame_width / 2) - m_vpoint.x;
	double dy = (m_frame_height / 2) - m_vpoint.y;
	printf("lines = %u, rotate = %.1f, slope = %.1f \n", rother_lines.size(), dx / pixs_in_deg, dy / pixs_in_deg);*/

};
