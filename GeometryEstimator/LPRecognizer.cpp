#include "LPRecognizer.h"

LPRecognizer::LPRecognizer()
{
	set_min_plate_size(cv::Size(0, 0));
	set_max_plate_size(cv::Size(0, 0));

	m_is_calibration_finished.store(true);
	m_calibration_interruption.store(false);

	p_plate_detector = std::make_unique<cv::CascadeClassifier>();
}

LPRecognizer::~LPRecognizer()
{
}

bool LPRecognizer::init()
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
	std::lock_guard<std::mutex> lock(m_plate_size_mutex);
	m_min_plate_size = size;
};

void LPRecognizer::set_max_plate_size(const cv::Size& size)
{
	std::lock_guard<std::mutex> lock(m_plate_size_mutex);
	m_max_plate_size = size;
};

cv::Size LPRecognizer::min_plate_size() const
{
	std::lock_guard<std::mutex> lock(m_plate_size_mutex);
	return m_min_plate_size;
};

cv::Size LPRecognizer::max_plate_size() const
{
	std::lock_guard<std::mutex> lock(m_plate_size_mutex);
	return m_max_plate_size;
};

bool LPRecognizer::save_to_json(const std::string& filename) const
{
	// Open the file or create if it don't exist in filesystem
	std::fstream file(filename.c_str(), std::fstream::out | std::fstream::app);
	file.close();
	file.open(filename.c_str(), std::fstream::out | std::fstream::in);

	if (!file.is_open())
		return false;
	
	// Read file and parse to document
	rapidjson::Document doc;
	rapidjson::IStreamWrapper in_stream(file);	
	doc.ParseStream(in_stream);

	if (doc.GetParseError() == rapidjson::kParseErrorNone || doc.GetParseError() == rapidjson::kParseErrorDocumentEmpty)
	{
		if (!doc.IsObject())
			doc.SetObject();
		
		rapidjson::Value recognizer_parameters(rapidjson::kObjectType);
		rapidjson::Value plate_size_min(rapidjson::kObjectType);
		rapidjson::Value plate_size_max(rapidjson::kObjectType);
		rapidjson::Value zones(rapidjson::kArrayType);

		// Minimal plate size
		{
			rapidjson::Value width(rapidjson::kNumberType);
			width.SetInt(min_plate_size().width);

			plate_size_min.AddMember("width", width, doc.GetAllocator());

			rapidjson::Value height(rapidjson::kNumberType);
			height.SetInt(min_plate_size().height);
			plate_size_min.AddMember("height", height, doc.GetAllocator());
		}

		// Maximal plate size
		{
			rapidjson::Value width(rapidjson::kNumberType);
			width.SetInt(max_plate_size().width);
			plate_size_max.AddMember("width", width, doc.GetAllocator());

			rapidjson::Value height(rapidjson::kNumberType);
			height.SetInt(max_plate_size().height);
			plate_size_max.AddMember("height", height, doc.GetAllocator());
		}

		// Zones
		{
			std::lock_guard<std::mutex> lock(m_zones_mutex);

			for (auto z = m_zones.begin(); z != m_zones.end(); ++z)
			{
				rapidjson::Value zone(rapidjson::kObjectType);

				rapidjson::Value x(rapidjson::kNumberType);
				rapidjson::Value y(rapidjson::kNumberType);
				rapidjson::Value width(rapidjson::kNumberType);
				rapidjson::Value height(rapidjson::kNumberType);

				x.SetInt(z->zone().x);
				y.SetInt(z->zone().y);
				width.SetInt(z->zone().width);
				height.SetInt(z->zone().height);

				rapidjson::Value plate_size(rapidjson::kObjectType);
				{
					rapidjson::Value width(rapidjson::kNumberType);
					width.SetInt(z->plate_size().width);
					plate_size.AddMember("width", width, doc.GetAllocator());

					rapidjson::Value height(rapidjson::kNumberType);
					height.SetInt(z->plate_size().height);
					plate_size.AddMember("height", height, doc.GetAllocator());
				}

				zone.AddMember("x", x, doc.GetAllocator());
				zone.AddMember("y", y, doc.GetAllocator());
				zone.AddMember("width", width, doc.GetAllocator());
				zone.AddMember("height", height, doc.GetAllocator());
				zone.AddMember("plateSize", plate_size, doc.GetAllocator());

				zones.PushBack(zone, doc.GetAllocator());
			}
		}

		recognizer_parameters.AddMember("plateSizeMin", plate_size_min, doc.GetAllocator());
		recognizer_parameters.AddMember("plateSizeMax", plate_size_max, doc.GetAllocator());
		recognizer_parameters.AddMember("zones", zones, doc.GetAllocator());

		if (doc.HasMember("recognizerParameters"))
			doc.RemoveMember("recognizerParameters");

		doc.AddMember("recognizerParameters", recognizer_parameters, doc.GetAllocator());

		file.close();
		file.open(filename.c_str(), std::fstream::out);
		rapidjson::OStreamWrapper out_stream(file);
		rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(out_stream);

		bool write_result = doc.Accept(writer);
		file.close();
		return write_result;
	}

	return false;
};

bool LPRecognizer::load_from_json(const std::string& filename)
{
	std::fstream file(filename.c_str(), std::fstream::in);
	if (!file.is_open())
		return false;

	rapidjson::Document doc;
	rapidjson::IStreamWrapper in_stream(file);

	doc.ParseStream(in_stream);

	if (doc.HasParseError() || !doc.IsObject())
		return false;

	if (doc.HasMember("recognizerParameters"))
	{
		rapidjson::Value recognizer_parameters;
		recognizer_parameters = doc["recognizerParameters"];
		
		if (recognizer_parameters.IsObject())
		{
			if(recognizer_parameters.HasMember("plateSizeMin"))
			{
				rapidjson::Value plate_size_min;
				plate_size_min = recognizer_parameters["plateSizeMin"];

				if (plate_size_min.IsObject())
				{
					if (plate_size_min.HasMember("width") && plate_size_min.HasMember("height"))
					{
						rapidjson::Value width, height;
						width = plate_size_min["width"];
						height = plate_size_min["height"];

						cv::Size size;

						if (width.IsInt() && height.IsInt())
						{
							size.width = width.GetInt();
							size.height = height.GetInt();
							set_min_plate_size(size);
						}
					}
				}
			}

			if(recognizer_parameters.HasMember("plateSizeMax"))
			{
				rapidjson::Value plate_size_max;
				plate_size_max = recognizer_parameters["plateSizeMax"];

				if (plate_size_max.IsObject())
				{
					if (plate_size_max.HasMember("width") && plate_size_max.HasMember("height"))
					{
						rapidjson::Value width, height;
						width = plate_size_max["width"];
						height = plate_size_max["height"];

						cv::Size size;

						if (width.IsInt() && height.IsInt())
						{
							size.width = width.GetInt();
							size.height = height.GetInt();
							set_max_plate_size(size);
						}
					}
				}
			}

			if(recognizer_parameters.HasMember("zones"))
			{
				rapidjson::Value zones;
				zones = recognizer_parameters["zones"];

				if (zones.IsArray())
				{
					for (size_t i = 0; i < zones.Size(); ++i)
					{
						auto& zone = zones[i];

						if (zone.IsObject())
						{
							if (zone.HasMember("x") && 
								zone.HasMember("y") && 
								zone.HasMember("width") && 
								zone.HasMember("height"))
							{
								rapidjson::Value x, y, width, height;
								x = zone["x"]; y = zone["y"];
								width = zone["width"]; height = zone["height"];

								LPRecognizerZone lpzone;
								lpzone.clear();

								if (x.IsInt() && y.IsInt() && width.IsInt() && height.IsInt())
								{
									cv::Rect zone_rect;

									zone_rect.x = x.GetInt();
									zone_rect.y = y.GetInt();
									zone_rect.width = width.GetInt();
									zone_rect.height = height.GetInt();

									lpzone.set_zone(zone_rect);
								}

								if (zone.HasMember("plateSize"))
								{
									rapidjson::Value plate_size;
									plate_size = zone["plateSize"];

									if (plate_size.HasMember("width") && plate_size.HasMember("height"))
									{
										rapidjson::Value width, height;
										width = plate_size["width"];
										height = plate_size["height"];

										if (width.IsInt() && height.IsInt())
										{
											cv::Size size;

											size.width = width.GetInt();
											size.height = height.GetInt();

											lpzone.set_plate_size(size);
										}
									}
								}

								if (!lpzone.plate_size().empty() && !lpzone.zone().empty())
								{
									std::lock_guard<std::mutex> lock(m_zones_mutex);
									m_zones.push_back(lpzone);
								}													
							}
						}
					}
				}
			}

			return true;
		}
	}

	return false;
};

bool LPRecognizer::capture_frame(const cv::Mat& frame)
{
	if (frame.empty() || frame.size().area() == 0)
		return false;

	m_input_mutex.lock();

	if (frame.type() != CV_8UC1)
		cvtColor(frame, m_gray_image, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(m_gray_image);

	m_is_new_image_detection = true;
	m_is_new_image_calibration = true;
	m_input_mutex.unlock();
	return true;
};

bool LPRecognizer::is_calibration_finished() const
{
	return m_is_calibration_finished.load();
}

bool LPRecognizer::stop_calibration()
{
	m_calibration_interruption.store(true);

	// Wait thread finish for one second
	for (size_t i = 0; i < 40; ++i)
	{
		if (m_is_calibration_finished.load())
		{
			if (m_calibration_thread.joinable())
				m_calibration_thread.join();

			return true;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(25));
	}

	return false;
};

bool LPRecognizer::start_calibration()
{	
	if (m_is_calibration_finished.load())
	{
		if (m_calibration_thread.joinable())
			m_calibration_thread.join();

		m_is_calibration_finished.store(false);
		m_calibration_interruption.store(false);
		m_calibration_thread = std::thread(&LPRecognizer::calibration_function, this);

		return true;
	}

	return false;
};


bool LPRecognizer::detect(std::vector<cv::Rect>& plates)
{
	cv::Mat img_working;

	// Capture new frame
	m_input_mutex.lock();

	if (!m_is_new_image_detection ||
		m_gray_image.empty() ||
		m_gray_image.size().area() == 0)
	{
		m_input_mutex.unlock();
		return false;
	}

	m_gray_image.copyTo(img_working);
	m_is_new_image_detection = false;
	m_input_mutex.unlock();

	// Detect plates
	{
		std::lock_guard<std::mutex> lock(m_zones_mutex);

		for (auto it = m_zones.begin(); it != m_zones.end(); ++it)
		{
			if ((it->plate_size().area() < min_plate_size().area() && !min_plate_size().empty())|| 
				(it->plate_size().area() > max_plate_size().area() && !max_plate_size().empty()))
				continue;

			std::vector<cv::Rect> plates_ = detect_plates(img_working, it->zone(), it->plate_size(), 3);
			plates.insert(plates.end(), plates_.begin(), plates_.end());
		}
	}

	// Group idential rects
	plates.insert(std::end(plates), std::begin(plates), std::end(plates));
	cv::groupRectangles(plates, 1, 0.5);

	if (plates.empty())
		return false;

	return true;
};

std::vector<cv::Rect> LPRecognizer::detect_plates(const cv::Mat& gray_frame, const cv::Rect& ROI, const cv::Size& plate_size, const int& min_neighbor) const
{
	std::lock_guard<std::mutex> lock(m_detector_mutex);

	if (!p_plate_detector || p_plate_detector->empty() || p_plate_detector->getOriginalWindowSize().empty())
		return {};

	if (plate_size.empty() || gray_frame.empty() || gray_frame.size().area() == 0)
		return {};

	cv::Rect roi(0, 0, gray_frame.cols, gray_frame.rows);
	if (!ROI.empty() && !(ROI.x < roi.x || ROI.y < roi.y || ROI.x + ROI.width > roi.width || ROI.y + ROI.height > roi.height))
		roi = ROI;
	else
		return {};

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

void LPRecognizer::calibration_function()
{
	cv::Mat img_working;

	int min_dist_to_border = 0;
	double cur_stop_weight = 0.0;
	double p_width = 0.0, p_height = 0.0;
	
	double orig_plate_resize = 1.0;
	cv::Size orig_plate_size = {};
	cv::Size min_ps = min_plate_size();
	cv::Size max_ps = max_plate_size();

	std::list<std::vector<cv::Rect>> history_plates;
	CalibrationState state = CalibrationState::init;

	LPRecognizerZone cur_zone;
	std::list<LPRecognizerZone> zones;
	std::list<LPRecognizerZone>::iterator it_detect_zone = zones.end();

	while (!m_calibration_interruption.load())
	{		
		// Capture new frame
		m_input_mutex.lock();

		if (!m_is_new_image_calibration ||
			m_gray_image.empty() ||
			m_gray_image.size().area() == 0)
		{
			m_input_mutex.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		
		m_gray_image.copyTo(img_working);
		m_is_new_image_calibration = false;
		m_input_mutex.unlock();
		
		// Detect movements in frame
		bool is_movement = false;

		if (it_detect_zone != zones.end())
		{
			auto plate_tmp = detect_plates(img_working, it_detect_zone->zone(), it_detect_zone->plate_size(), 3);
			for (auto p1 = plate_tmp.begin(); p1 != plate_tmp.end(); ++p1)
			{
				bool is_staing = false;
				const double thresh = 0.3 * cv::norm(p1->tl() - p1->br());
				const cv::Point p1_center(p1->x + p1->width / 2, p1->y + p1->height / 2);

				for (auto vect = history_plates.begin(); vect != history_plates.end(); ++vect)
					for (auto p2 = vect->begin(); p2 != vect->end(); ++p2)
					{
						const cv::Point p2_center(p2->x + p2->width / 2, p2->y + p2->height / 2);

						if (cv::norm(p1_center - p2_center) < thresh)
						{
							is_staing = true;
							vect = std::prev(history_plates.end());
							break;
						}
					}

				if (!is_staing)
				{
					is_movement = true;
					break;
				}
			}

			if (history_plates.size() >= 5)
				history_plates.pop_front();

			history_plates.push_back(plate_tmp);
		}

		// Detect plates
		auto plates = detect_plates(img_working, cur_zone.zone(), cur_zone.plate_size(), 5);

		size_t old_points_count = cur_zone.points_size();
		cur_zone.add_points(plates);

		// Update missed frames counter
		if (cur_zone.points_size() <= old_points_count)
		{
			cur_stop_weight += 0.01;
			if (is_movement)
				cur_stop_weight += 0.99;
		}
		else
		{
			cur_stop_weight = std::max(0.0, cur_stop_weight - 3.0);
		}

		//printf("Current zone info: stop weight = %.1f, point size = %u \r\n", cur_stop_weight, cur_zone.points_size());

		// Process new frame
		switch (state)
		{
		case CalibrationState::init:

			cur_stop_weight = 0.0;
			min_dist_to_border = img_working.rows / 20;

			it_detect_zone = zones.end();
	
			cur_zone.set_frame_size(cv::Size(img_working.cols, img_working.rows));
			cur_zone.set_zone(cv::Rect(0, 0, img_working.cols, img_working.rows));
			cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));

			// Compute initial plate size		
			{
				std::lock_guard<std::mutex> lock(m_detector_mutex);
				orig_plate_size = p_plate_detector->getOriginalWindowSize();
			}

			if (!min_ps.empty() && !max_ps.empty() && min_ps.area() < max_ps.area())
				orig_plate_resize = sqrt(sqrt(min_ps.area()) * sqrt(max_ps.area())) / sqrt(orig_plate_size.area());	
			else if (!min_ps.empty() && min_ps.area() > orig_plate_size.area())
				orig_plate_resize = sqrt(min_ps.area()) / sqrt(orig_plate_size.area());
			else if (!max_ps.empty() && max_ps.area() < orig_plate_size.area())
				orig_plate_resize = sqrt(max_ps.area()) / sqrt(orig_plate_size.area());

			orig_plate_size.width *= orig_plate_resize;
			orig_plate_size.height *= orig_plate_resize;
			cur_zone.set_plate_size(orig_plate_size);
			
			printf("start plate_size: width = %u, height = %u \r\n", orig_plate_size.width, orig_plate_size.height);
			state = CalibrationState::up_search;
			break;

		case CalibrationState::up_search:

			if (cur_zone.points_size() >= POINTS_TO_CALIBRATE || cur_stop_weight >= MAX_STOP_WEIGHT)
			{
				cur_stop_weight = 0.0;

				// Save zone and scale it
				if (cur_zone.points_size() >= std::min(MIN_POINTS_TO_CALIBRATE, POINTS_TO_CALIBRATE)) 
				{
					// Estimate zone rectangle and save zone
					cur_zone.calibrate();
					zones.push_back(cur_zone);
					correct_zones(cv::Size(img_working.cols, img_working.rows), zones);

					// Find zone for detection movements
					it_detect_zone = std::find_if(zones.begin(), zones.end(), [&](const LPRecognizerZone& z)
					{
						return z.plate_size() == cur_zone.plate_size();
					});

					// Check distance to border (min dist to bottom or top)			
					if (std::min((img_working.rows - cur_zone.zone().br().y), (cur_zone.zone().y)) < min_dist_to_border)
					{
						{
							std::lock_guard<std::mutex> lock(m_detector_mutex);
							cur_zone.set_plate_size(orig_plate_size);
						}
						state = CalibrationState::down_scale;
					}
					else
					{
						state = CalibrationState::up_scale;
					}
				}
				// Here is only zone scaling
				else 
				{
					{
						std::lock_guard<std::mutex> lock(m_detector_mutex);
						cur_zone.set_plate_size(orig_plate_size);
					}
					state = CalibrationState::down_scale;
				}
			}

			break;

		case CalibrationState::up_scale:

			p_width = static_cast<double>(cur_zone.plate_size().width) * PLATE_RESIZE_SCALE;
			p_height = static_cast<double>(cur_zone.plate_size().height) * PLATE_RESIZE_SCALE;

			cur_zone.clear();
			cur_zone.set_frame_size(cv::Size(img_working.cols, img_working.rows));
			cur_zone.set_zone(cv::Rect(0, 0, img_working.cols, img_working.rows));
			cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
			cur_zone.set_plate_size(cv::Size(std::round(p_width), std::round(p_height)));

			if (!zones.empty())
			{
				zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
				{
					return (z1.zone().y + z1.zone().height / 2) > (z2.zone().y + z2.zone().height / 2);
				});

				cur_zone.set_zone(cv::Rect(0, zones.front().zone().y, img_working.cols, img_working.rows - zones.front().zone().y));
			}

			if (((cur_zone.plate_size().area() > max_plate_size().area()) && !max_plate_size().empty()) ||
				((cur_zone.plate_size().area() < min_plate_size().area()) && !min_plate_size().empty()))
			{
				{
					std::lock_guard<std::mutex> lock(m_detector_mutex);
					cur_zone.set_plate_size(orig_plate_size);
				}
				state = CalibrationState::down_scale;
			}
			else
			{
				state = CalibrationState::up_search;
			}
			break;

		case CalibrationState::down_search:

			if (cur_zone.points_size() >= POINTS_TO_CALIBRATE || cur_stop_weight >= MAX_STOP_WEIGHT)
			{
				cur_stop_weight = 0.0;

				// Save zone and scale it
				if (cur_zone.points_size() >= std::min(MIN_POINTS_TO_CALIBRATE, POINTS_TO_CALIBRATE)) 
				{
					// Estimate zone rectangle and save zone
					cur_zone.calibrate();
					zones.push_back(cur_zone);
					correct_zones(cv::Size(img_working.cols, img_working.rows), zones);

					// Find zone for detection movements
					it_detect_zone = std::find_if(zones.begin(), zones.end(), [&](const LPRecognizerZone& z)
					{
						return z.plate_size() == cur_zone.plate_size();
					});

					// Check distance to border (min dist to bottom or top)			
					if (std::min((img_working.rows - cur_zone.zone().br().y), (cur_zone.zone().y)) < min_dist_to_border)
					{
						state = CalibrationState::finished;
					}
					else
					{
						state = CalibrationState::down_scale;
					}
				}
				// Here is only zone scaling
				else 
				{
					{
						std::lock_guard<std::mutex> lock(m_detector_mutex);
						cur_zone.set_plate_size(orig_plate_size);
					}
					state = CalibrationState::finished;
				}
			}
			break;

		case CalibrationState::down_scale:
	
			p_width = static_cast<double>(cur_zone.plate_size().width) / PLATE_RESIZE_SCALE;
			p_height = static_cast<double>(cur_zone.plate_size().height) / PLATE_RESIZE_SCALE;

			cur_zone.clear();
			cur_zone.set_frame_size(cv::Size(img_working.cols, img_working.rows));
			cur_zone.set_zone(cv::Rect(0, 0, img_working.cols, img_working.rows));
			cur_zone.set_color(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
			cur_zone.set_plate_size(cv::Size(std::round(p_width), std::round(p_height)));

			if (!zones.empty())
			{
				zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
				{
					return (z1.zone().y + z1.zone().height / 2) > (z2.zone().y + z2.zone().height / 2);
				});

				cur_zone.set_zone(cv::Rect(0, 0, img_working.cols, zones.back().zone().br().y));
			}

			if (((cur_zone.plate_size().area() > max_plate_size().area()) && !max_plate_size().empty()) ||
				((cur_zone.plate_size().area() < min_plate_size().area()) && !min_plate_size().empty()))
			{
				state = CalibrationState::finished;
			}
			else
			{
				state = CalibrationState::down_search;
			}
			break;

		case CalibrationState::finished:

			{
				std::lock_guard<std::mutex> lock(m_zones_mutex);
				m_zones.clear();
				m_zones.insert(m_zones.end(), zones.begin(), zones.end());
			}

			m_is_calibration_finished.store(true);
			return;

		default: 
			break;
		}
	}

	m_is_calibration_finished.store(true);
};


void LPRecognizer::correct_zones(const cv::Size& frame_size, std::list<LPRecognizerZone>& zones) const
{
	// Delete empty zones
	zones.erase(std::remove_if(zones.begin(), zones.end(), [](const LPRecognizerZone& z) { return z.zone().empty(); }), zones.end());

	// Sort zones by position from bottom to top
	zones.sort([](const LPRecognizerZone& z1, const LPRecognizerZone& z2)
	{
		const auto z1_center = z1.zone().y + z1.zone().height / 2;
		const auto z2_center = z2.zone().y + z2.zone().height / 2;
		return z1_center > z2_center;
	});

	// Remove zones that fully covered by another 
	for (auto z1 = zones.begin(); z1 != zones.end(); ++z1)
		for (auto z2 = zones.begin(); z2 != zones.end(); ++z2)
		{
			if (z1 == z2) continue;

			if (z1->zone().tl().y >= z2->zone().tl().y &&
				z1->zone().br().y <= z2->zone().br().y)
			{
				z1->set_zone(cv::Rect());
				break;
			}
		}
	zones.erase(std::remove_if(zones.begin(), zones.end(), [](const LPRecognizerZone& z1) { return z1.zone().empty(); }), zones.end());

	// Correct zone sizes
	for (auto z_down = zones.begin(); z_down != std::prev(zones.end()); ++z_down)
	{
		auto z_up = std::next(z_down);		
		const int mid_y = ((z_up->zone().y + z_up->zone().height) + z_down->zone().y) / 2;
		
		// Zones cross with each other
		if (mid_y >= z_down->zone().y)
		{
			// Correct bottom zone
			cv::Rect zone_bot_rect = z_down->zone();
			zone_bot_rect.y = std::max(0, mid_y - z_down->plate_size().height);
			zone_bot_rect.height = (z_down->zone().y + z_down->zone().height) - zone_bot_rect.y;

			// Correct top zone
			cv::Rect zone_top_rect = z_up->zone();
			zone_top_rect.height = std::min(frame_size.height, (mid_y + z_up->plate_size().height)) - z_up->zone().y;

			// Save corrected zones
			z_down->set_zone(zone_bot_rect);
			z_up->set_zone(zone_top_rect);
		}
		// Zones don't cross with each other
		else
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
			zones.push_back(new_zone);

			correct_zones(frame_size, zones);
		}	
	}
};