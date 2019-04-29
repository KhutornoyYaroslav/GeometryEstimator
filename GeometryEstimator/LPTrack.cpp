#include "LPTrack.h"

LPTrack::LPTrack()
{
	clean_lost_frames();
};

LPTrack::~LPTrack()
{
};

LPTrack::LPTrack(const LPPlate& plate)
{
	clean_lost_frames();
	add_plate(plate);
};

LPTrack::LPTrack(const LPPlate& plate, cv::Scalar color)
{
	clean_lost_frames();
	add_plate(plate);
	set_color(color);
}

void LPTrack::add_plate(const LPPlate& plate)
{
	m_plates.push_back(plate);
};

void LPTrack::inc_lost_frames()
{
	++m_lost_frames;
};

int LPTrack::lost_frames() const
{
	return m_lost_frames;
};

void LPTrack::clean_lost_frames()
{
	m_lost_frames = 0;
};

void LPTrack::set_color(cv::Scalar color)
{
	m_color = color;
};

cv::Scalar LPTrack::color() const
{
	return m_color;
};

const std::vector<LPPlate>* LPTrack::get_plates() const
{
	return &m_plates;
};

int LPTrack::average_plate_width() const
{
	int result = 0;

	for (size_t i = 0; i < m_plates.size(); ++i)
		result += m_plates[i].get_rect()->width;

	if (m_plates.size() != 0)
		result /= m_plates.size();
	
	return result;
};

int LPTrack::average_plate_height() const
{
	int result = 0;

	for (size_t i = 0; i < m_plates.size(); ++i)
		result += m_plates[i].get_rect()->height;

	if (m_plates.size() != 0)
		result /= m_plates.size();

	return result;
};

double LPTrack::lenght() const
{
	double result = 0.0;

	for (auto it = std::next(m_plates.begin()); it != m_plates.end(); ++it)
		result += cv::norm(it->center_position() - std::prev(it)->center_position());

	return result;
};

int LPTrack::get_average_age() const
{
	// TODO:
	return 0;
};

