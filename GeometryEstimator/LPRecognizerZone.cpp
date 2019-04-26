#include "LPRecognizerZone.h"

LPRecognizerZone::LPRecognizerZone()
{
	clear();
};

LPRecognizerZone::LPRecognizerZone(const cv::Rect& zone, const cv::Size& frame_size, const cv::Size& plate_size)
{
	clear();
	set_zone(zone);
	set_plate_size(plate_size);
	set_frame_size(frame_size);
};

void LPRecognizerZone::set_color(const cv::Scalar& color)
{
	m_color = color;
};

void LPRecognizerZone::set_zone(const cv::Rect& zone)
{
	m_zone = zone;
};

void LPRecognizerZone::set_plate_size(const cv::Size& size)
{
	m_plate_size = size;
};

void LPRecognizerZone::set_frame_size(const cv::Size& frame_size)
{
	m_frame_size = frame_size;
};

cv::Rect LPRecognizerZone::zone() const
{
	return m_zone;
};

size_t LPRecognizerZone::points_size() const
{
	return m_points.size();
};

cv::Scalar LPRecognizerZone::color() const
{
	return m_color;
};

cv::Size LPRecognizerZone::plate_size() const
{
	return m_plate_size;
};

void LPRecognizerZone::clear()
{
	m_zone = {};
	m_frame_size = {};
	m_plate_size = {};
	m_color = {};
	m_points.clear();
	m_points_density = {};
};

void LPRecognizerZone::calibrate()
{
	if (m_points.empty())
		return; 

	const auto bound_zone = bound_points();
	const int new_height = static_cast<int>(std::max(m_frame_size.height * 0.01, bound_zone.height * 1.2));

	m_zone.x = 0;
	m_zone.y = std::max(0, (bound_zone.y + bound_zone.height / 2) - new_height / 2);

	m_zone.width = m_frame_size.width;
	m_zone.height = std::min(new_height, m_frame_size.height - m_zone.y);
};

void LPRecognizerZone::add_points(const std::vector<cv::Rect>& rects)
{
	std::vector<cv::Point> points(rects.size());

	for (size_t i = 0; i < points.size(); ++i)
		points[i] = cv::Point(rects[i].x + rects[i].width / 2, rects[i].y + rects[i].height / 2);

	add_points(points);
};

void LPRecognizerZone::add_points(const std::vector<cv::Point>& points)
{
	if (points.empty())
		return;

	m_points_density = 0.0;	
	const double min_radius = 5.0;
	const double max_radius = 0.5 * cv::norm(cv::Point(0, 0) - cv::Point(m_plate_size.width, m_plate_size.height)); // TODO: if m_plate_size == {} ?

	// Re-compute metrics for old points:
	for (size_t i = 0; i < m_points.size(); ++i)
	{
		for (size_t j = 0; j < points.size(); ++j)
		{
			const double dist = cv::norm(points[j] - m_points[i].first);
			if (dist < max_radius && dist > min_radius)
				++m_points[i].second;
		}

		m_points_density += static_cast<double>(m_points[i].second);
	}

	// Compute metrics for new points
	std::vector<std::pair<cv::Point, size_t>> new_points(points.size());

	for (size_t i = 0; i < new_points.size(); ++i)
	{
		new_points[i].first = points[i];
		new_points[i].second = 0;

		for (size_t j = 0; j < m_points.size(); ++j)
		{
			const double dist = cv::norm(new_points[i].first - m_points[j].first);
			if (dist < max_radius && dist > min_radius)
				++new_points[i].second;
		}

		for (size_t j = 0; j < points.size(); ++j)
		{
			if (i == j) continue;

			const double dist = cv::norm(new_points[i].first - new_points[j].first);
			if (dist < max_radius && dist > min_radius)
				++new_points[i].second;
		}

		m_points_density += static_cast<double>(new_points[i].second);
	}

	m_points.insert(m_points.end(), new_points.begin(), new_points.end());

	// Remove duplicate points
	std::unordered_set<cv::Point> pointset;
	auto itor = m_points.begin();
	while (itor != m_points.end())
	{
		if (pointset.find(itor->first) != pointset.end())
		{
			m_points_density -= static_cast<double>(itor->second);
			itor = m_points.erase(itor);
		}
		else
		{
			pointset.insert(itor->first);
			itor++;
		}
	}

	// Compute points density
	assert(!m_points.empty());
	m_points_density /= m_points.size();
};

cv::Rect LPRecognizerZone::bound_points()
{
	if (m_points.empty())
		return {};

	std::vector<cv::Point> filtered_points;
	filtered_points.reserve(m_points.size());

	for (size_t i = 0; i < m_points.size(); ++i)
		if (m_points[i].second >= 0.65 * m_points_density)
			filtered_points.push_back(m_points[i].first);

	return cv::boundingRect(filtered_points);
};

void LPRecognizerZone::print(cv::Mat image) const
{
	cv::rectangle(image, m_zone, m_color, 2, CV_AA);

	for (size_t i = 0; i < m_points.size(); ++i)
		cv::circle(image, m_points[i].first, 3, m_color, 3, CV_AA);
}