#include "LPRecognizer.h"

LPRecognizer::LPRecognizer()
{
	p_plate_detector = std::make_unique<cv::CascadeClassifier>();
	initCascadeClassifier();
}

LPRecognizer::~LPRecognizer()
{
}

bool LPRecognizer::initCascadeClassifier()
{
	if (!p_plate_detector)
		return false;

	if (p_plate_detector->load(cv::String("haarcascade_russian_plate_number.xml")))
	{
		m_cascade_wnd_size = p_plate_detector->getOriginalWindowSize();
		return true;
	}
	else
	{
		return false;
	}
};

void LPRecognizer::calibrate_zones(const cv::Mat& frame, cv::Mat& debug_frame)
{
	cv::Mat gray_frame;

	if (frame.empty() || frame.size().area() == 0)
		return;

	if (frame.type() != CV_8UC1)
		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(gray_frame);

	if (p_plate_detector->empty())
	{
		printf("Can't load classifier cascade file \r\n");
		return;
	}

	// TODO:
	// Сначала грубый шаг, поток мелкий, чтобы дыры подзакрыть ?

	if (m_zones.size() > 4)
		return;

	cv::Mat gray_frame_resized;
	cv::resize(frame, gray_frame_resized, cv::Size(), m_resized_k, m_resized_k);

	// Detector parameters
	const double precision = 1.1;
	const double min_neighbors = 5;

	// Начинаем с размера по-умолчанию
	m_cur_zone.set_frame_size(cv::Point(gray_frame_resized.cols, gray_frame_resized.rows));

	std::vector<cv::Rect> plates;
	p_plate_detector->detectMultiScale(gray_frame_resized, plates, precision, min_neighbors, 0, m_cascade_wnd_size, m_cascade_wnd_size);

	printf("zones = %u \r\n", m_cur_zone.size());

	// Считаем зону (пока грубо очень)
	if (m_cur_zone.size() < 100)
	{
		m_cur_zone.add_plates(plates);
	}
	else
	{
		m_cur_zone.calibrate();

		// Сохраняем зону
		//m_zones.push_back(m_cur_zone);

		auto x = m_cur_zone.rect().x / m_resized_k;
		auto y = m_cur_zone.rect().y / m_resized_k;
		auto w = m_cur_zone.rect().width / m_resized_k;
		auto h = m_cur_zone.rect().height / m_resized_k;

		std::pair<LPRecognizerZone, double> tmp;
		//tmp.first = LPRecognizerZone(m_cur_zone.rect(), cv::Point(frame.cols, frame.rows));
		tmp.first = LPRecognizerZone(cv::Rect(x, y, w, h), cv::Point(frame.cols, frame.rows));
		tmp.second = m_resized_k;

		m_zones.push_back(tmp);

		//m_zones.push_back(LPRecognizerZone(cv::Rect(x,y,w,h), cv::Point(frame.cols, frame.rows)));

		m_cur_zone.clear();
		m_resized_k = m_resized_k * 1.3;
	}

	// Print zones
	for (auto it = m_zones.begin(); it != m_zones.end(); ++it)
	{
		//it->first.print(debug_frame);
	}
};




std::vector<cv::Rect> LPRecognizer::process_frame(const cv::Mat& frame, cv::Mat& debug_frame)
{
	cv::Mat gray_frame;

	if (frame.empty() || frame.size().area() == 0)
		return {};

	if (frame.type() != CV_8UC1)
		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(gray_frame);

	if (p_plate_detector->empty())
	{
		printf("Can't load classifier cascade file \r\n");
		return {};
	}

	// Params
	const double precision = 1.1;
	const double min_neighbors = 5;

	std::vector<cv::Rect> plates_all;

	for (auto it = m_zones.begin(); it != m_zones.end(); ++it)
	{
		// Detect plates
		auto frame_ = frame(it->first.rect());
		cv::Mat gray_frame_resized;
		cv::resize(frame_, gray_frame_resized, cv::Size(), it->second, it->second);

		std::vector<cv::Rect> plates;
		p_plate_detector->detectMultiScale(gray_frame_resized, plates, precision, min_neighbors, 0, m_cascade_wnd_size, m_cascade_wnd_size);

		// Group idential rects
		plates.insert(std::end(plates), std::begin(plates), std::end(plates));
		cv::groupRectangles(plates, 1, 0.5);

		for (auto& p : plates)
		{
			auto x = it->first.rect().x + (p.x  / it->second);
			auto y = it->first.rect().y + (p.y  / it->second);
			auto w = p.width / it->second;
			auto h = p.height / it->second;

			plates_all.push_back(cv::Rect(x, y, w, h));
			
		}

	}

	// Group idential rects
	plates_all.insert(std::end(plates_all), std::begin(plates_all), std::end(plates_all));
	cv::groupRectangles(plates_all, 1, 0.5);

	for (auto& p : plates_all)
		cv::rectangle(debug_frame, p, cv::Scalar(0, 255, 0), 2, CV_AA);

	return plates_all;
};






//std::vector<cv::Rect> LPRecognizer::process_frame(const cv::Mat& frame, cv::Mat& debug_frame)
//{
//	cv::Mat gray_frame;
//
//	if (frame.empty() || frame.size().area() == 0)
//		return {};
//
//	if (frame.type() != CV_8UC1)
//		cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
//	else
//		frame.copyTo(gray_frame);
//
//	if (p_plate_detector->empty())
//	{
//		printf("Can't load classifier cascade file \r\n");
//		return {};
//	}
//
//	// Params
//	const double precision = 1.1;
//	const double min_neighbors = 5;
//
//	// Detect plates
//	std::vector<cv::Rect> plates;
//	p_plate_detector->detectMultiScale(gray_frame(m_cur_zone.rect()), plates, precision, min_neighbors, 0, m_cascade_wnd_size * 1, m_cascade_wnd_size * 1);
//
//	// Group idential rects
//	plates.insert(std::end(plates), std::begin(plates), std::end(plates));
//	cv::groupRectangles(plates, 1, 0.5);
//
//	// Add plates to zone
//	std::vector<cv::Rect> plates_(plates.size());
//	for (size_t i = 0; i < plates.size(); ++i)
//	{
//		plates_[i].width = plates[i].width;
//		plates_[i].height = plates[i].height;
//		plates_[i].x = plates[i].x + m_cur_zone.rect().x;
//		plates_[i].y = plates[i].y + m_cur_zone.rect().y;
//	}
//
//	m_cur_zone.add_plates(plates_);
//	m_cur_zone.set_frame_size(cv::Point(frame.cols, frame.rows));
//	m_cur_zone.set_zone(cv::Rect(0, 0, frame.cols, frame.rows));
//	m_cur_zone.calibrate();
//
//	// Print debug
//	if (!debug_frame.empty() && frame.size().area() != 0)
//	{
//		// Print current plates
//		for (auto& p : plates)
//			cv::rectangle(debug_frame, cv::Rect(m_cur_zone.rect().x + p.x, m_cur_zone.rect().y + p.y, p.width, p.height), cv::Scalar(0, 255, 0), 2, CV_AA);
//
//		// Print zones
//		m_cur_zone.print(debug_frame);
//	}
//
//	return plates;
//};