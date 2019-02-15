#include "GeometryTracker.h"

GeometryTracker::GeometryTracker()
{
	p_termcrit = std::make_unique<cv::TermCriteria>(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
	p_plate_detector = std::make_unique<cv::CascadeClassifier>("haarcascade_russian_plate_number.xml");
	p_backgnd_sub = cv::createBackgroundSubtractorMOG2(512, 64.0, true);
	//p_backgnd_sub->setBackgroundRatio(0.99);
	//p_backgnd_sub->setComplexityReductionThreshold(0.15);
	//p_backgnd_sub->setNMixtures(120);
	//p_backgnd_sub->setShadowThreshold(100.00001);
	p_backgnd_sub->setShadowValue(0);

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


bool GeometryTracker::process_frame2(const cv::Mat& frame)
{
	cv::Mat debug_frame(frame);
	cv::Mat gray_frame;

	if (frame.empty() || frame.size().area() == 0)
		return false;

	if (frame.type() != CV_8UC1)
		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(gray_frame);



	if (p_plate_detector->empty())
	{
		printf("Can't load classifier cascade file \r\n");
		return false;
	}

	cv::Size wnd = p_plate_detector->getOriginalWindowSize();

	std::vector<cv::Rect> plates;
	p_plate_detector->detectMultiScale(gray_frame, plates, 1.1, 3, 0, wnd, cv::Size(wnd.width * 5, wnd.height * 5));
	
	if (plates.empty())
	{
		printf("No detect! \r\n");
	}

	for (auto& p : plates)
	{
		cv::rectangle(debug_frame, p, cv::Scalar(0, 255, 0), 2, CV_AA);
	}

	cv::Mat debug_frame_resize;
	cv::resize(debug_frame, debug_frame_resize, cv::Size(frame.cols * 1.0, frame.rows * 1.0));
	imshow("Debug", debug_frame_resize);
	cvWaitKey(1);

	return true;
};

void GeometryTracker::get_contour_lines(const cv::Mat& gray_frame, std::vector<cv::Vec4i>& lines)
{
	cv::Mat foreground;
	cv::Mat motionmask;
	cv::Mat contourframe;
	cv::Mat contourmask;

	// Get foreground frame
	p_backgnd_sub->apply(gray_frame, foreground);

	// Compute motion mask frame
	cv::blur(foreground, motionmask, cv::Size(3, 3));
	//threshold(motionmask, motionmask, 0, 255, CV_THRESH_BINARY + CV_THRESH_OTSU);

	int dilation_size = 6;
	cv::Mat element = getStructuringElement(2, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));

	dilate(motionmask, motionmask, element, cv::Point(-1, -1), 3);
	erode(motionmask, motionmask, cv::Mat(), cv::Point(-1, -1), 3);

	// Get contours from source frame
	gray_frame.copyTo(contourframe);
	//cv::blur(contourframe, contourframe, cv::Size(3, 3));
	//cv::Canny(contourframe, contourframe, 225, 255, 3);
	cv::blur(contourframe, contourframe, cv::Size(3, 3));
	cv::threshold(contourframe, contourframe, 0, 255, CV_THRESH_BINARY + CV_THRESH_OTSU);
	cv::Sobel(contourframe, contourframe, 0, 0, 1, 1);


	// Make gray image with mask
	contourframe.copyTo(contourmask, motionmask);

	// Get Hough lines
	HoughLinesP(contourmask, lines, 1, CV_PI / 720, 85, 85, 10);
};

bool GeometryTracker::process_frame3(const cv::Mat& frame)
{
	cv::Mat gray_frame;
	cv::Mat debugframe;

	frame.copyTo(debugframe);

	if (frame.empty() || frame.size().area() == 0)
		return false;

	if (frame.type() != CV_8UC1)
		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(gray_frame);

	// Get foreground frame
	std::vector<cv::Vec4i> lines;
	get_contour_lines(gray_frame, lines);

	// Print results
	const double resize = 0.75;
	cv::Mat debug_frame_resize;
	cv::resize(debugframe, debug_frame_resize, cv::Size(), resize, resize);
	imshow("contourmask", debug_frame_resize);
	cvWaitKey(1);

	return true;
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
		}
	}

	m_points_prev.clear();
	m_points_prev.insert(std::end(m_points_prev), std::begin(points), std::end(points));
	gray_frame.copyTo(m_gray_prev);
};
