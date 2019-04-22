#include "LPRecognizer.h"

LPRecognizer::LPRecognizer()
{
	p_plate_detector = std::make_unique<cv::CascadeClassifier>();
	initCascadeClassifier();

	m_state = CalibrationState::init;
	m_min_plate_size = {};
	m_max_plate_size = {};
}

LPRecognizer::~LPRecognizer()
{
}

bool LPRecognizer::initCascadeClassifier()
{
	if (!p_plate_detector)
		return false;

	if (p_plate_detector->load(cv::String("haarcascade_russian_plate_number.xml")))
		return true;
	else
		return false;
};

void LPRecognizer::set_min_plate_size(const cv::Size& size)
{
	m_min_plate_size = size;
};

void LPRecognizer::set_max_plate_size(const cv::Size& size)
{
	m_max_plate_size = size;
};

std::vector<cv::Rect> LPRecognizer::detect_plates(const cv::Mat& gray_frame, const cv::Rect& ROI, const cv::Size& plate_size, const int& min_neighbor) const
{
	if (!p_plate_detector || p_plate_detector->empty() || p_plate_detector->getOriginalWindowSize().empty())
		return {};

	if (gray_frame.empty() || gray_frame.size().area() == 0)
		return {};

	if (plate_size.empty())
		return {};

	cv::Rect roi(0, 0, gray_frame.cols, gray_frame.rows);
	if (!ROI.empty() && !(ROI.x < roi.x || ROI.y < roi.y || ROI.x + ROI.width > roi.width || ROI.y + ROI.height > roi.height))
		roi = ROI;

	std::vector<cv::Rect> plates;
	const cv::Size orig_wnd = p_plate_detector->getOriginalWindowSize();

	const double k_rsz = sqrt(static_cast<double>(orig_wnd.area()) / plate_size.area());
	if (abs(k_rsz) < DBL_EPSILON) return {};
	
	if (k_rsz > 1.0) //(k_rsz - 1.0) >= -DBL_EPSILON
	{
		cv::Mat resized_frame;
		cv::resize(gray_frame(roi), resized_frame, cv::Size(), k_rsz, k_rsz);

		p_plate_detector->detectMultiScale(resized_frame, plates, 1.1, min_neighbor, 0, orig_wnd, orig_wnd);

		for (auto& plate : plates)
		{
			plate.x /= k_rsz;
			plate.y /= k_rsz;
			plate.width /= k_rsz;
			plate.height /= k_rsz;
		}
	}
	else
	{
		p_plate_detector->detectMultiScale(gray_frame(roi), plates, 1.1, min_neighbor, 0, plate_size, plate_size);
	}

	for (auto& plate : plates)
	{
		plate.x += roi.x;
		plate.y += roi.y;
	}

	return plates;
};

void LPRecognizer::calibrate_zones(const cv::Mat& frame, cv::Mat& debug_frame)
{
	if (p_plate_detector->empty() || frame.empty() || frame.size().area() == 0)
		return;

	if (is_calib_finished)
		return;

	// ----------------- DEBUG PRINT -----------------
	for (auto it = m_zones.begin(); it != m_zones.end(); ++it)
		it->print(debug_frame);

	m_cur_zone.print(debug_frame);
	// ----------------- ----------- -----------------

	// SM variables
	double p_width = 0.0, p_height = 0.0;

	// SM parameters:
	const size_t max_zero_zone_count = 2;
	const int min_dist_to_border = frame.rows / 20;


	this->set_max_plate_size(cv::Size(80, 30)); // DEBUG
	this->set_min_plate_size(cv::Size(25, 8)); // DEBUG

	// Detect plates, update missed frames counter
	auto plates = detect_plates(frame, m_cur_zone.zone(), m_cur_zone.plate_size(), 5);

	if (plates.empty())
	{
		if (m_zones.empty())
			m_cur_stop_weight += 0.05;
		else
			if (!process_frame(frame, debug_frame).empty())
				m_cur_stop_weight += 1.0;
	}
	else
	{
		m_cur_zone.add_points(plates);
		m_cur_stop_weight = std::max(0.0, m_cur_stop_weight - 5.0);
	}

	printf("Current zone info: stop weight = %.1f, point size = %u \r\n", m_cur_stop_weight, m_cur_zone.points_size());

	// State machine processing
	switch (m_state)
	{
	case CalibrationState::init:

		m_cur_zone.clear();		
		m_cur_zone.set_frame_size(cv::Size(frame.cols, frame.rows));
		m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, frame.rows));
		m_cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
		m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());

		m_cur_stop_weight = 0.0;

		m_state = CalibrationState::up_search;
		break;

	case CalibrationState::up_search:

		if (m_cur_zone.points_size() >= POINTS_TO_CALIBRATE || m_cur_stop_weight >= MAX_STOP_WEIGHT)
		{
			m_cur_stop_weight = 0.0;

			if (m_cur_zone.points_size() >= std::min(MIN_POINTS_TO_CALIBRATE, POINTS_TO_CALIBRATE)) // Save zone and scale it
			{
				// Estimate zone rectangle and save zone
				m_cur_zone.calibrate();
				m_zones.push_back(m_cur_zone);
				correct_zones(cv::Size(frame.cols, frame.rows));

				// Check distance to border (min dist to bottom or top)			
				if (std::min((frame.rows - m_cur_zone.zone().br().y), (m_cur_zone.zone().y)) < min_dist_to_border)
				{
					m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());
					m_state = CalibrationState::down_scale;
				}
				else
				{
					m_state = CalibrationState::up_scale;
				}
			}
			else // Here is only zone scaling
			{
				m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());
				m_state = CalibrationState::down_scale;
			}
		}

		break;

	case CalibrationState::up_scale:

		p_width = static_cast<double>(m_cur_zone.plate_size().width) * PLATE_RESIZE_SCALE;
		p_height = static_cast<double>(m_cur_zone.plate_size().height) * PLATE_RESIZE_SCALE;

		m_cur_zone.clear();
		m_cur_zone.set_frame_size(cv::Size(frame.cols, frame.rows));
		m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, frame.rows));
		m_cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
		m_cur_zone.set_plate_size(cv::Size(std::round(p_width), std::round(p_height)));

		if (!m_zones.empty())
		{
			m_zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
			{
				return (z1.zone().y + z1.zone().height / 2) > (z2.zone().y + z2.zone().height / 2);
			});

			m_cur_zone.set_zone(cv::Rect(0, m_zones.front().zone().y, frame.cols, frame.rows - m_zones.front().zone().y));
		}

		if (((m_cur_zone.plate_size().area() > m_max_plate_size.area()) && !m_max_plate_size.empty()) ||
			((m_cur_zone.plate_size().area() < m_min_plate_size.area()) && !m_min_plate_size.empty()))
		{
			m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());
			m_state = CalibrationState::down_scale;
		}
		else
		{
			m_state = CalibrationState::up_search;
		}
		break;

	case CalibrationState::down_search:

		if (m_cur_zone.points_size() >= POINTS_TO_CALIBRATE || m_cur_stop_weight >= MAX_STOP_WEIGHT)
		{
			m_cur_stop_weight = 0.0;

			if (m_cur_zone.points_size() >= std::min(MIN_POINTS_TO_CALIBRATE, POINTS_TO_CALIBRATE)) // Save zone and scale it
			{
				// Estimate zone rectangle and save zone
				m_cur_zone.calibrate();
				m_zones.push_back(m_cur_zone);
				correct_zones(cv::Size(frame.cols, frame.rows));

				// Check distance to border (min dist to bottom or top)			
				if (std::min((frame.rows - m_cur_zone.zone().br().y), (m_cur_zone.zone().y)) < min_dist_to_border)
				{
					m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());
					m_state = CalibrationState::finished;
				}
				else
				{
					m_state = CalibrationState::down_scale;
				}
			}
			else // Here is only zone scaling
			{
				m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());
				m_state = CalibrationState::finished;
			}
		}
		break;

	case CalibrationState::down_scale:

		p_width = static_cast<double>(m_cur_zone.plate_size().width) / PLATE_RESIZE_SCALE;
		p_height = static_cast<double>(m_cur_zone.plate_size().height) / PLATE_RESIZE_SCALE;

		m_cur_zone.clear();
		m_cur_zone.set_frame_size(cv::Size(frame.cols, frame.rows));
		m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, frame.rows));
		m_cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
		m_cur_zone.set_plate_size(cv::Size(std::round(p_width), std::round(p_height)));

		if (!m_zones.empty())
		{
			m_zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
			{
				return (z1.zone().y + z1.zone().height / 2) > (z2.zone().y + z2.zone().height / 2);
			});

			m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, m_zones.back().zone().br().y));
		}

		if (((m_cur_zone.plate_size().area() > m_max_plate_size.area()) && !m_max_plate_size.empty()) ||
			((m_cur_zone.plate_size().area() < m_min_plate_size.area()) && !m_min_plate_size.empty()))
		{
			m_state = CalibrationState::finished;
		}
		else
		{
			m_state = CalibrationState::down_search;
		}
		break;

	case CalibrationState::finished:

		correct_zones(cv::Size(frame.cols, frame.rows));
		is_calib_finished = true;
		break;

	default: break;
	}
};

//void LPRecognizer::calibrate_zones(const cv::Mat& frame, cv::Mat& debug_frame)
//{
//	 TODO: доделать следующие критерии останова масштабировани€:
//	 1) –азмер окна меньше, чем мин. допустимый размер номера (задаетс€ пользователем)
//	 ... 
//	 TODO: 
//	 если missed frames > thresh, то не забывать область, а все-таки сделать bound rect вокруг хот€ бы тех точек, которые успели накопитьс€..
//	 ƒл€ начальной области missed frames считать без учета уже сущ. областей...
//
//	if (is_calib_finished)
//		return;
//
//	for (auto it = m_zones.begin(); it != m_zones.end(); ++it)
//		it->print(debug_frame);
//
//	m_cur_zone.print(debug_frame);
//
//	if (frame.empty() || frame.size().area() == 0)
//		return;
//
//	if (p_plate_detector->empty())
//		return;
//	
//	 SM variables
//	int dist_to_border = 0;
//	double p_width = 0.0, p_height = 0.0;
//
//	 SM parameters:
//	const double resize_coef_scale = 1.3;
//	const int max_missed_frames = 100;
//	const size_t max_zero_zone_count = 2;
//	const size_t points_to_calibrate = 75;
//	const int min_dist_to_border = frame.rows / 20;
//
//	printf("Current zone points size = %u \r\n", m_cur_zone.points_size());
//
//	std::vector<cv::Rect> plates;
//
//	 ќбрабатываем
//	switch (m_state)
//	{
//	case CalibrationState::none:
//
//		m_cur_zone.clear();
//		p_width = static_cast<double>(p_plate_detector->getOriginalWindowSize().width) / resize_coef_scale;
//		p_height = static_cast<double>(p_plate_detector->getOriginalWindowSize().height) / resize_coef_scale;
//		m_cur_zone.set_plate_size(cv::Size(std::round(p_width), std::round(p_height)));
//
//		m_cur_missed_frames = 0;
//		m_state = CalibrationState::up_scale_init;
//		break;
//
//
//	case CalibrationState::up_scale_init:
//
//		p_width = static_cast<double>(m_cur_zone.plate_size().width) * resize_coef_scale;
//		p_height = static_cast<double>(m_cur_zone.plate_size().height) * resize_coef_scale;
//
//		m_cur_zone.clear();
//		m_cur_zone.set_plate_size(cv::Size(std::round(p_width), std::round(p_height)));
//		m_cur_zone.set_frame_size(cv::Size(frame.cols, frame.rows));
//		m_cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
//		
//		m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, frame.rows)); // TODO	
//
//
//		if (!m_zones.empty())
//		{
//			 Sort zones by position from bottom to top
//			m_zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
//			{
//				const auto z1_center = z1.zone().y + z1.zone().height / 2;
//				const auto z2_center = z2.zone().y + z2.zone().height / 2;
//				return z1_center > z2_center;
//			});
//
//			const auto& zone = m_zones.front().zone();
//			m_cur_zone.set_zone(cv::Rect(0, zone.y, frame.cols, frame.rows - zone.y));
//		}
//
//		printf("cur zone plate w = %i h = %i \r\n", m_cur_zone.plate_size().width, m_cur_zone.plate_size().height);
//		m_state = CalibrationState::up_scale;
//		break;
//
//	case CalibrationState::up_scale:
//
//		plates = detect_plates(frame, m_cur_zone.zone(), m_cur_zone.plate_size(), 5);
//		m_cur_zone.add_points(plates);
//
//		 ѕроверка на отсутствие номеров 
//		if (plates.empty())
//		{
//			 “ут еще провер€ть, есть ли в других зонах номера
//			if (!this->process_frame(frame, debug_frame).empty())
//			{
//				++m_cur_missed_frames;				
//				printf("Current missed frames count = %u \r\n", m_cur_missed_frames);
//			}
//		}
//		else
//		{
//			printf("Current zone points size = %u \r\n", m_cur_zone.points_size());
//			m_cur_missed_frames = std::max(0, m_cur_missed_frames - 2);
//		}
//
//		 TODO:
//		 ≈сли у нас набралось нужное кол-во точек, то калибруем, сохран€ем, переходим к следующей зоне
//		 ≈сли превысилс€ счетчик пропущенных кадров ?
//
//		if (m_cur_missed_frames > max_missed_frames)
//		{
//			m_cur_missed_frames = 0;
//			++m_zero_zones_count;
//
//			if (m_cur_zone.points_size() > 15)
//			{
//				 Estimate zone rectangle and save zone
//				m_cur_zone.calibrate();
//				m_zones.push_back(m_cur_zone);
//				correct_zones(cv::Size(frame.cols, frame.rows));
//
//				 Check distance to border (min dist to bottom or top)			
//				dist_to_border = std::min((frame.rows - m_cur_zone.zone().br().y), (m_cur_zone.zone().y));
//
//				if (dist_to_border < min_dist_to_border)
//				{
//					m_state = CalibrationState::down_scale_init;
//					break; // !
//				}
//				else
//				{
//					m_state = CalibrationState::up_scale_init;
//				}
//				printf("Distance to border = %i \r\n", dist_to_border);
//			}
//			
//			if (m_zero_zones_count >= max_zero_zone_count)
//			{
//				m_zero_zones_count = 0;
//				m_cur_zone.clear();
//				m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());
//				m_state = CalibrationState::down_scale_init;
//			}
//			else
//			{
//				m_state = CalibrationState::up_scale_init;
//			}
//
//			printf("zero zones count = %u \r\n", m_zero_zones_count);
//		}
//
//		if (m_cur_zone.points_size() >= points_to_calibrate)
//		{
//			 Estimate zone rectangle and save zone
//			m_cur_zone.calibrate();
//			m_zones.push_back(m_cur_zone);
//			correct_zones(cv::Size(frame.cols, frame.rows));
//
//			 Check distance to border (min dist to bottom or top)			
//			dist_to_border = std::min((frame.rows - m_cur_zone.zone().br().y), (m_cur_zone.zone().y));
//
//			if (dist_to_border < min_dist_to_border)
//			{
//				m_cur_zone.set_plate_size(p_plate_detector->getOriginalWindowSize());
//				m_state = CalibrationState::down_scale_init;
//			}
//			else
//			{
//				m_state = CalibrationState::up_scale_init;
//			}
//			printf("Distance to border = %i \r\n", dist_to_border);
//		}
//
//		break;
//
//
//	case CalibrationState::down_scale_init:
//
//		p_width = static_cast<double>(m_cur_zone.plate_size().width) / resize_coef_scale;
//		p_height = static_cast<double>(m_cur_zone.plate_size().height) / resize_coef_scale;
//
//		m_cur_zone.clear();
//		m_cur_zone.set_plate_size(cv::Size(std::round(p_width), std::round(p_height)));
//		m_cur_zone.set_frame_size(cv::Size(frame.cols, frame.rows));
//
//		 TODO	
//		m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, frame.rows));
//		if (!m_zones.empty())
//		{
//			 Sort zones by position from bottom to top
//			m_zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
//			{
//				const auto z1_center = z1.zone().y + z1.zone().height / 2;
//				const auto z2_center = z2.zone().y + z2.zone().height / 2;
//				return z1_center > z2_center;
//			});
//
//			const auto& zone = m_zones.back().zone();
//			m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, zone.br().y));
//		}
//
//
//		m_cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
//
//		printf("cur zone plate w = %i h = %i \r\n", m_cur_zone.plate_size().width, m_cur_zone.plate_size().height);
//		m_state = CalibrationState::down_scale;
//		break;
//
//	case CalibrationState::down_scale:
//
//		plates = detect_plates(frame, m_cur_zone.zone(), m_cur_zone.plate_size(), 5);
//		m_cur_zone.add_points(plates);
//
//		 ѕроверка на отсутствие номеров 
//		if (plates.empty())
//		{
//			 “ут еще провер€ть, есть ли в других зонах номера
//			if (!this->process_frame(frame, debug_frame).empty())
//			{
//				++m_cur_missed_frames;
//				printf("Current missed frames count = %u \r\n", m_cur_missed_frames);
//			}
//		}
//		else
//		{
//			printf("Current zone points size = %u \r\n", m_cur_zone.points_size());
//			m_cur_missed_frames = std::max(0, m_cur_missed_frames - 2);
//		}
//
//		if (m_cur_missed_frames > max_missed_frames)
//		{
//			m_cur_missed_frames = 0;
//			++m_zero_zones_count;
//
//			if (m_cur_zone.points_size() > 15)
//			{
//				 Estimate zone rectangle and save zone
//				m_cur_zone.calibrate();
//				m_zones.push_back(m_cur_zone);
//				correct_zones(cv::Size(frame.cols, frame.rows));
//
//				 Check distance to border (min dist to bottom or top)			
//				dist_to_border = std::min((frame.rows - m_cur_zone.zone().br().y), (m_cur_zone.zone().y));
//
//				if (dist_to_border < min_dist_to_border)
//				{
//					m_state = CalibrationState::correction;
//					break; // !
//				}
//				else
//				{
//					m_state = CalibrationState::down_scale_init;
//				}
//				printf("Distance to border = %i \r\n", dist_to_border);
//			}
//
//
//			if (m_zero_zones_count >= max_zero_zone_count)
//			{
//				m_zero_zones_count = 0;
//				m_state = CalibrationState::correction;
//			}
//			else
//			{
//				m_state = CalibrationState::down_scale_init;
//			}
//
//			printf("zero zones count = %u \r\n", m_zero_zones_count);
//		}
//
//		if (m_cur_zone.points_size() >= points_to_calibrate)
//		{
//			 Estimate zone rectangle and save zone
//			m_cur_zone.calibrate();
//			m_zones.push_back(m_cur_zone);
//			correct_zones(cv::Size(frame.cols, frame.rows)); // TODO ___________________________
//
//			 Check distance to border (min dist to bottom or top)
//			dist_to_border = std::min((frame.rows - m_cur_zone.zone().br().y), (m_cur_zone.zone().y));
//
//			if (dist_to_border < min_dist_to_border)
//			{
//				m_state = CalibrationState::correction;
//			}
//			else
//			{
//				m_state = CalibrationState::down_scale_init;
//			}
//
//			printf("Distance to border = %i \r\n", dist_to_border);
//		}
//		
//		break;
//
//
//
//
//
//	case CalibrationState::correction:
//
//		correct_zones(cv::Size(frame.cols, frame.rows));
//		m_state = CalibrationState::finished;
//		break;
//
//	case CalibrationState::finished:
//		is_calib_finished = true;
//		break;
//
//	default: break;
//	}
//};

void LPRecognizer::correct_zones(const cv::Size& frame_size)
{
	// TODO: вдруг попали пустые зоны? удалить их.

	// Sort zones by position from bottom to top
	m_zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
	{
		const auto z1_center = z1.zone().y + z1.zone().height / 2;
		const auto z2_center = z2.zone().y + z2.zone().height / 2;
		return z1_center > z2_center;
	});

	// Remove zones that fully covered by another 
	for (auto z1 = m_zones.begin(); z1 != m_zones.end(); ++z1)
		for (auto z2 = m_zones.begin(); z2 != m_zones.end(); ++z2)
		{
			if (z1 == z2) continue;

			if (z1->zone().tl().y >= z2->zone().tl().y &&
				z1->zone().br().y <= z2->zone().br().y)
			{
				z1->set_zone(cv::Rect());
				break;
			}
		}
	m_zones.erase(std::remove_if(m_zones.begin(), m_zones.end(), [](const LPRecognizerZone& z1) { return z1.zone().empty(); }), m_zones.end());

	// Correct zone sizes
	for (auto z_down = m_zones.begin(); z_down != std::prev(m_zones.end()); ++z_down)
	{
		auto z_up = std::next(z_down);		
		const int mid_y = ((z_up->zone().y + z_up->zone().height) + z_down->zone().y) / 2;
		
		if (mid_y >= z_down->zone().y) // перекрываютс€ // TODO: тут все-таки надо считать, что перекрываютс€ с каким-то допуском...
		{
			//  оррекци€ нижней зоны
			cv::Rect zone_bot_rect = z_down->zone();
			zone_bot_rect.y = std::max(0, mid_y - z_down->plate_size().height);
			zone_bot_rect.height = (z_down->zone().y + z_down->zone().height) - zone_bot_rect.y;

			//  оррекци€ верхней зоны
			cv::Rect zone_top_rect = z_up->zone();
			zone_top_rect.height = std::min(frame_size.height, (mid_y + z_up->plate_size().height)) - z_up->zone().y;

			// —охран€ем скорректированные размеры зон
			z_down->set_zone(zone_bot_rect);
			z_up->set_zone(zone_top_rect);
		}
		else // не перекрываемс€ - интерполируем
		{
			cv::Rect new_rect;
			new_rect.y = std::max(0, mid_y - 2 * z_up->plate_size().height);
			new_rect.height = std::min(frame_size.height, 2 * (z_up->plate_size().height + z_down->plate_size().height));
			new_rect.x = 0;
			new_rect.width = frame_size.width;

			auto new_plate_w = (z_up->plate_size().width + z_down->plate_size().width) / 2;
			auto new_plate_h = (z_up->plate_size().height + z_down->plate_size().height) / 2;

			LPRecognizerZone new_zone;
			new_zone.set_zone(new_rect);
			new_zone.set_frame_size(frame_size);
			new_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
			new_zone.set_plate_size(cv::Size(new_plate_w, new_plate_h));
			m_zones.push_back(new_zone);

			correct_zones(frame_size);
		}	
	}
};

std::vector<cv::Rect> LPRecognizer::process_frame(const cv::Mat& frame, cv::Mat& debug_frame)
{
	if (frame.empty() || frame.size().area() == 0)
		return {};

	//cv::Mat gray_frame;
	//if (frame.type() != CV_8UC1)
	//	cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	//else
	//	frame.copyTo(gray_frame);

	std::vector<cv::Rect> plates_all;

	for (auto it = m_zones.begin(); it != m_zones.end(); ++it)
	{
		std::vector<cv::Rect> plates = detect_plates(frame, it->zone(), it->plate_size(), 3);
		plates_all.insert(plates_all.end(), plates.begin(), plates.end());
	}

	// Group idential rects
	plates_all.insert(std::end(plates_all), std::begin(plates_all), std::end(plates_all));
	cv::groupRectangles(plates_all, 1, 0.5);

	for (auto& p : plates_all)
		cv::rectangle(debug_frame, p, cv::Scalar(0, 255, 0), 2, CV_AA);

	return plates_all;
};