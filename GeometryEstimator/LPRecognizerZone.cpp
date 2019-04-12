#include "LPRecognizerZone.h"

LPRecognizerZone::LPRecognizerZone()
{
	set_zone({});
};

LPRecognizerZone::LPRecognizerZone(const cv::Rect& zone, const cv::Point& frame_size)
{
	set_zone(zone);
	set_frame_size(frame_size);
};

void LPRecognizerZone::set_frame_size(const cv::Point& frame_size)
{
	m_frame_size = frame_size;
};

void LPRecognizerZone::set_zone(const cv::Rect& zone)
{
	m_zone = zone;
};

cv::Rect LPRecognizerZone::rect() const
{
	return m_zone;
};

size_t LPRecognizerZone::size() const
{
	return m_rects.size();
};

void LPRecognizerZone::clear()
{
	m_zone = {};
	m_frame_size = {};
	m_rects.clear();
};

void LPRecognizerZone::add_plates(const std::vector<cv::Rect>& plates)
{
	m_rects.insert(m_rects.end(), plates.begin(), plates.end());
};

void LPRecognizerZone::calibrate()
{
	if (m_rects.empty())
		return; 

	m_zone = bound_plates();
	auto bound_zone = bound_plates();

	int new_center_x = bound_zone.x + bound_zone.width / 2;
	int new_center_y = (bound_zone.y + bound_zone.height / 2);

	int new_height = std::max(200.0, m_zone.height * 1.3); // TODO: 200.0 ?

	m_zone.x = 0;
	m_zone.y = std::max(0, (bound_zone.y + bound_zone.height / 2) - new_height / 2);

	m_zone.width = m_frame_size.x;
	m_zone.height = std::min(new_height, m_frame_size.y - m_zone.y);
};

cv::Rect LPRecognizerZone::bound_plates()
{
	int amount = 0;
	double radius = 0;
	std::vector<std::pair<cv::Point, int>> data;

	if (m_rects.empty())
		return {};

	for (size_t i = 0; i < m_rects.size(); ++i)
		radius += cv::norm(m_rects[i].tl() - m_rects[i].br());

	radius = 0.5 * (radius / m_rects.size());

	for (size_t i = 0; i < m_rects.size(); ++i)
	{	
		int counter = 0;
		cv::Point point(m_rects[i].x + m_rects[i].width / 2, m_rects[i].y + m_rects[i].height / 2);

		for (size_t j = 0; j < m_rects.size(); ++j)
		{
			cv::Point pair_point(m_rects[j].x + m_rects[j].width / 2, m_rects[j].y + m_rects[j].height / 2);
			if (cv::norm(point - pair_point) <= radius)
				++counter;
		}

		amount += counter;
		data.push_back(std::pair<cv::Point, int>(point, counter));
	}

	std::vector<cv::Point> filtered_points;
	const double average_amount = static_cast<double>(amount) / (data.size() + DBL_EPSILON);

	for (size_t i = 0; i < data.size(); ++i)
		if (data[i].second >= 0.5 * average_amount)
			filtered_points.push_back(data[i].first);

	return cv::boundingRect(filtered_points);
};

void LPRecognizerZone::print(cv::Mat image) const
{
	auto color = cv::Scalar(150 + rand() % 155, 0 + rand() % 255, 100 + rand() % 155);

	cv::rectangle(image, m_zone, color, 2, CV_AA);

	for (size_t i = 0; i < m_rects.size(); ++i)
	{
		cv::Point point(m_rects[i].x + m_rects[i].width / 2, m_rects[i].y + m_rects[i].height / 2);
		//cv::circle(image, point, 3, cv::Scalar(0, 0, 200), 3, CV_AA);
	}
};