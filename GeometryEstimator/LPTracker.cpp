#include "LPTracker.h"

LPTracker::LPTracker()
{
	m_is_process_finished.store(true);
	m_process_interruption.store(false);

	p_recognizer = std::make_unique<LPRecognizer>();
}

bool LPTracker::capture_frame(const cv::Mat& frame)
{
	if (frame.empty() || frame.size().area() == 0)
		return false;

	m_input_mutex.lock();

	if (frame.type() != CV_8UC1)
		cvtColor(frame, m_gray_image, cv::COLOR_BGR2GRAY);
	else
		frame.copyTo(m_gray_image);

	m_is_new_image = true;
	m_input_mutex.unlock();
	return true;
};

void LPTracker::pull_tracks(std::vector<LPTrack>& tracks)
{
	std::lock_guard<std::mutex> lock(m_finished_tracks_mutex);
	tracks.insert(tracks.end(), m_finished_tracks.begin(), m_finished_tracks.end());
	m_finished_tracks.clear();
};

void LPTracker::clear()
{
	m_process_tracks.clear();

	{
		std::lock_guard<std::mutex> lock(m_finished_tracks_mutex);
		m_finished_tracks.clear();
	}
};

bool LPTracker::start_process()
{
	if (m_is_process_finished.load())
	{
		if (m_process_thread.joinable())
			m_process_thread.join();

		m_is_process_finished.store(false);
		m_process_interruption.store(false);
		m_process_thread = std::thread(&LPTracker::process_thread_function, this);

		return true;
	}

	return false;
};

bool LPTracker::stop_process()
{
	m_process_interruption.store(true);

	// Wait thread finish for one second
	for (size_t i = 0; i < 40; ++i)
	{
		if (m_is_process_finished.load())
		{
			if (m_process_thread.joinable())
				m_process_thread.join();

			return true;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(25));
	}

	return false;
};

void LPTracker::process_thread_function()
{
	cv::Mat img_working;
	std::vector<LPTrack> working_tracks;
	std::vector<cv::Rect> plates_positions;

	while (!m_process_interruption.load())
	{
		// Capture new frame
		m_input_mutex.lock();

		if (!m_is_new_image ||
			m_gray_image.empty() ||
			m_gray_image.size().area() == 0)
		{
			m_input_mutex.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		m_gray_image.copyTo(img_working);
		m_is_new_image = false;
		m_input_mutex.unlock();

		// TODO: rec calibration ??

		// Recognizer

		if (!p_recognizer->capture_frame(img_working))
			continue;

		plates_positions.clear();

		if (!p_recognizer->detect(plates_positions))
			continue;

		process(img_working, plates_positions, working_tracks);		
	}

	m_is_process_finished.store(true);
};

void LPTracker::process_plates(const cv::Mat& frame, const std::vector<cv::Rect>& plates)
{
	process(frame, plates, m_process_tracks);
};

void LPTracker::process(const cv::Mat& frame, const std::vector<cv::Rect>& plates, std::vector<LPTrack>& tracks)
{
	// Assign new plates to exisiting tracks
	std::vector<bool> is_plate_assigned(plates.size(), false);
	std::vector<bool> is_track_assigned(tracks.size(), false);

	if (tracks.size() > 0 && plates.size() > 0)
	{
		const double assign_table_init_val = 1.0;
		cv::Mat assign_table(tracks.size(), plates.size(), CV_64FC1);
		assign_table.setTo(cv::Scalar(assign_table_init_val));

		for (size_t i = 0; i < tracks.size(); ++i)
		{
			const auto& track = tracks[i];
			assert(!track.get_plates()->empty());
			const auto& track_end_plate = track.get_plates()->back();

			//// Estimate maximum distance from new plate to ends of tracks
			//double max_distance = 0.0;
			//for (size_t j = 0; j < plates.size(); ++j)
			//{
			//	cv::Point plate_center(plates[j].x + (plates[j].width / 2), plates[j].y + (plates[j].height / 2));
			//	max_distance = std::max(max_distance, cv::norm(track_end_plate.get_center() - plate_center));
			//}
			//max_distance += DBL_EPSILON;

			// Compute weights for all pairs (track <-> plate)
			for (size_t j = 0; j < plates.size(); ++j)
			{
				const auto& plate = plates[j];
				cv::Point plate_center(plate.x + (plate.width / 2), plate.y + (plate.height / 2));

				double angle = 0.0;
				double distance = 0.0;

				// Estimate distance
				{
					distance = cv::norm(track_end_plate.center_position() - plate_center);
				}

				// Estimate vector angle
				{
					const double min_dist = track.lenght() / 5.0;

					cv::Point tr_ref_point = {};
					cv::Point tr_end_point = track.get_plates()->rbegin()->center_position();

					for (auto rev_it = track.get_plates()->rbegin(); rev_it != track.get_plates()->rend(); ++rev_it)
					{
						tr_ref_point = rev_it->center_position();
						if (cv::norm(tr_end_point - tr_ref_point) >= min_dist)
							break;
					}

					cv::Point ref_vec = tr_end_point - tr_ref_point;
					cv::Point tar_vec = plate_center - tr_ref_point;

					const double ref_vec_len = cv::norm(ref_vec - cv::Point(0, 0));
					const double tar_vec_len = cv::norm(tar_vec - cv::Point(0, 0));
					const double dot = ref_vec.x * tar_vec.x + ref_vec.y * tar_vec.y;

					const double vec_len_mult = ref_vec_len * tar_vec_len;

					if (abs(vec_len_mult) > DBL_EPSILON)
					{
						angle = dot / vec_len_mult;
						angle = (angle >= 1.0) ? 0.0 : ((angle <= -1.0) ? 180.0 : DEGREE_IN_RADIAN * acos(angle));
					}
				}

				// Estimate result weight
				// NOTICE: чем меньше вес - тем лучше
				// TODO: каждый вес нормировать от 0 до 1 

				const double k_dist = 0.4;
				const double k_angle = 0.4;
				const double k_age = 0.2;

				const double max_angle = 30.0;
				const double max_dist = 2.0 * cv::norm(track.get_plates()->back().get_rect()->br() - track.get_plates()->back().get_rect()->tl());


				//printf("angle = %.1f \r\n", angle);

				//double weight = k_dist * (distance / max_dist) + k_angle * (angle / max_angle) + k_age * (track.lost_frames() / m_track_age);
				double weight = k_dist * (distance / max_dist) + k_angle * (angle / max_angle) + k_age * (track.lost_frames() / MAX_MISSED_FRAMES);

				if (distance > 1.5 * max_dist)
					weight = 100.0;

				//if (angle > 1.5 * max_angle)
				//	weight = 100.0;

				assign_table.at<double>(i, j) = weight + assign_table_init_val; // TODO: можно написать кроче: += weight;
			}
		}

		Munkres<double> hungarian;
		cv::Mat res = hungarian.solve(assign_table);

		// Здесь мы проверяеm трешхолды, условия добавления новых треков
		for (int i = 0; i < res.rows; ++i)
			for (int j = 0; j < res.rows; ++j)
				if (res.at<int>(i, j) == 0 && assign_table.at<double>(i, j) >= 2.0) // TODO: какой трешхолд здесь выбрать?
					res.at<int>(i, j) = -1;

		// Дополняем треки найденными соответсвтвиями
		cv::Point maxLoc;
		double max;
		for (int i = 0; i < res.rows; i++)
		{
			cv::minMaxLoc(res(cv::Rect(0, i, res.cols, 1)), nullptr, &max, nullptr, &maxLoc, cv::Mat());
			if (assign_table.at<float>(i, maxLoc.x) != assign_table_init_val && max == 0)
			{
				is_track_assigned[i] = true;
				is_plate_assigned[maxLoc.x] = true;

				LPPlate new_plate;
				new_plate.set_rect(plates[maxLoc.x]);
				new_plate.set_image(frame(plates[maxLoc.x]));

				tracks[i].add_plate(new_plate);
			}
		}
	}

	/////update unassigned tracks
	for (auto it = tracks.begin(); it != tracks.end(); )
	{
		if (!is_track_assigned[std::distance(tracks.begin(), it)])
		{
			it->inc_lost_frames();

			if (it->lost_frames() > MAX_MISSED_FRAMES)
			{
				{
					std::lock_guard<std::mutex> lock(m_finished_tracks_mutex);
					m_finished_tracks.push_back(*it);
				}

				it = tracks.erase(it);
			}
			else
				++it;
		}
		else
		{
			it->clean_lost_frames();
			++it;
		}
	}

	// Create new tracks
	for (size_t i = 0; i < plates.size(); i++)
		if (!is_plate_assigned[i])
		{
			cv::RNG rng;
			auto color = cv::Scalar(150 + rand() % 155, 0 + rand() % 255, 100 + rand() % 155);

			LPPlate new_plate;
			new_plate.set_rect(plates[i]);
			new_plate.set_image(frame(plates[i]));

			tracks.emplace_back(LPTrack(new_plate, color));
		}
};












/*
void LPTracker::process_frame(const cv::Mat& frame, cv::Mat debug)
{
	// Detect new plates
	//p_recognizer->calibrate_zones(frame, debug); // TODO;

	//static bool is_try_loaded = false;
	//static bool is_loaded = false;

	//if (!is_try_loaded)
	//{
	//	is_try_loaded = true;
	//	if (p_recognizer->load_from_json("zones_config_6.json"))
	//		is_loaded = true;
	//	
	//}
	//
	//if(!is_loaded)
	//	p_recognizer->calibrate_zones(frame, debug);


	std::vector<cv::Rect> plates = {};// p_recognizer->process_frame(frame, debug);

	// Assign new plates to exisiting tracks
	std::vector<bool> is_plate_assigned(plates.size(), false);
	std::vector<bool> is_track_assigned(m_process_tracks.size(), false);

	if (m_process_tracks.size() > 0 && plates.size() > 0)
	{
		const double assign_table_init_val = 1.0;
		cv::Mat assign_table(m_process_tracks.size(), plates.size(), CV_64FC1);
		assign_table.setTo(cv::Scalar(assign_table_init_val));

		for (size_t i = 0; i < m_process_tracks.size(); ++i)
		{
			const auto& track = m_process_tracks[i];
			assert(!track.get_plates()->empty());
			const auto& track_end_plate = track.get_plates()->back();

			//// Estimate maximum distance from new plate to ends of tracks
			//double max_distance = 0.0;
			//for (size_t j = 0; j < plates.size(); ++j)
			//{
			//	cv::Point plate_center(plates[j].x + (plates[j].width / 2), plates[j].y + (plates[j].height / 2));
			//	max_distance = std::max(max_distance, cv::norm(track_end_plate.get_center() - plate_center));
			//}
			//max_distance += DBL_EPSILON;
		
			// Compute weights for all pairs (track <-> plate)
			for (size_t j = 0; j < plates.size(); ++j)
			{
				const auto& plate = plates[j];
				cv::Point plate_center(plate.x + (plate.width / 2), plate.y + (plate.height / 2));

				double angle = 0.0;
				double distance = 0.0;

				// Estimate distance
				{
					distance = cv::norm(track_end_plate.get_center() - plate_center);
				}

				// Estimate vector angle
				{
					const double min_dist = track.lenght() / 5.0;

					cv::Point tr_ref_point = {};
					cv::Point tr_end_point = track.get_plates()->rbegin()->get_center();

					for (auto rev_it = track.get_plates()->rbegin(); rev_it != track.get_plates()->rend(); ++rev_it)
					{
						tr_ref_point = rev_it->get_center();
						if (cv::norm(tr_end_point - tr_ref_point) >= min_dist)
							break;
					}

					cv::Point ref_vec = tr_end_point - tr_ref_point;
					cv::Point tar_vec = plate_center - tr_ref_point;
				
					const double ref_vec_len = cv::norm(ref_vec - cv::Point(0, 0));
					const double tar_vec_len = cv::norm(tar_vec - cv::Point(0, 0));
					const double dot = ref_vec.x * tar_vec.x + ref_vec.y * tar_vec.y;

					const double vec_len_mult = ref_vec_len * tar_vec_len;

					if (abs(vec_len_mult) > DBL_EPSILON)
					{
						angle = dot / vec_len_mult;
						angle = (angle >= 1.0) ? 0.0 : ((angle <= -1.0) ? 180.0 : DEGREE_IN_RADIAN * acos(angle));
					}
				}

				// Estimate result weight
				// NOTICE: чем меньше вес - тем лучше
				// TODO: каждый вес нормировать от 0 до 1 

				const double k_dist = 0.4;
				const double k_angle = 0.4;
				const double k_age = 0.2;

				const double max_angle = 30.0;
				//const double max_dist = 100.0;
				const double max_dist = 2.0 * track.get_plates()->back().get_rect()->width;

				//printf("angle = %.1f \r\n", angle);

				//double weight = k_dist * (distance / max_dist) + k_angle * (angle / max_angle) + k_age * (track.lost_frames() / m_track_age);
				double weight = k_dist * (distance / max_dist) + k_angle * (angle / max_angle) + k_age * (track.lost_frames() / m_track_age);

				if (distance > 1.5 * max_dist)
					weight = 100.0;

				//if (angle > 1.5 * max_angle)
				//	weight = 100.0;

				assign_table.at<double>(i, j) = weight + assign_table_init_val; // TODO: можно написать кроче: += weight;
			}
		}

		//std::cout << "Assign table = " << assign_table << std::endl;
		Munkres<double> hungarian;
		cv::Mat res = hungarian.solve(assign_table);
		//std::cout << "Res = " << res << std::endl;
		//std::cout << std::endl;

		// Здесь мы проверяеm трешхолды, условия добавления новых треков
		for (int i = 0; i < res.rows; ++i)
			for (int j = 0; j < res.rows; ++j)
				if (res.at<int>(i, j) == 0 && assign_table.at<double>(i, j) >= 2.0) // TODO: какой трешхолд здесь выбрать?
					res.at<int>(i, j) = -1;

		// Дополняем треки найдеными соответсвтвиями
		cv::Point maxLoc;
		double max;
		for (int i = 0; i < res.rows; i++)
		{
			cv::minMaxLoc(res(cv::Rect(0, i, res.cols, 1)), nullptr, &max, nullptr, &maxLoc, cv::Mat());
			if (assign_table.at<float>(i, maxLoc.x) != assign_table_init_val && max == 0)
			{
				is_track_assigned[i] = true;
				is_plate_assigned[maxLoc.x] = true;
				m_process_tracks[i].add_plate(LPlate(plates[maxLoc.x]));
			}
		}

		cvWaitKey(1);
	}


	/////update unassigned tracks
	/////TODO: should be placed after creating new!!!!
	for (auto it = m_process_tracks.begin(); it != m_process_tracks.end(); )
	{
		if (!is_track_assigned[std::distance(m_process_tracks.begin(), it)])
		{
			it->inc_lost_frames();

			if (it->lost_frames() > m_track_age)
			{
				if (it->get_plates()->size() > 5)
				{
					m_finished_tracks.push_back(*it);
					printf("finished track size = %u \r\n", m_finished_tracks.size());
				}
					

				it = m_process_tracks.erase(it);
			}
			else
				++it;
		}
		else
		{
			it->clean_lost_frames();
			++it;
		}
	}

	// Create new tracks
	for (size_t i = 0; i < plates.size(); i++)
		if (!is_plate_assigned[i])
		{
			cv::RNG rng;
			auto color = cv::Scalar(150 + rand() % 155, 0 + rand() % 255, 100 + rand() % 155);
			m_process_tracks.emplace_back(LPTrack(LPlate(plates[i]), color));
		}



	// ********************** DEBUG SECTION ********************** 



	//for (size_t i = 0; i < m_process_tracks.size(); ++i)
	//{
	//	const auto& color = m_process_tracks[i].color();
	//	auto& track = m_process_tracks[i];
	//	
	//	for (size_t j = 0; j < track.get_plates()->size() - 1; ++j)
	//	{		
	//		const auto rect = track.get_plates()->at(j).get_rect();
	//		const auto rect_center = track.get_plates()->at(j).get_center();

	//		const auto rect_next = track.get_plates()->at(j+1).get_rect();
	//		const auto rect_center_next = track.get_plates()->at(j+1).get_center();
	//		cv::circle(debug, rect_center, 2, color, 2);

	//		cv::line(debug, rect_center, rect_center_next, color, 1, CV_AA);
	//	}

	//	const auto rect = track.get_plates()->back().get_rect();
	//	const auto rect_center = track.get_plates()->back().get_center();
	//	cv::circle(debug, rect_center, 2, color, 2);


	//	//// VECTOR OF TRACK DIRECTION
	//	//{
	//	//	double track_lenght = 0.0;
	//	//	for (auto it = std::next(track.plates.begin()); it != track.plates.end(); ++it)
	//	//	{
	//	//		track_lenght += cv::norm(it->get_center() - std::prev(it)->get_center());
	//	//	}
	//	//	const double min_dist = track_lenght / 5.0; // PARAMETER
	//	//
	//	//	cv::Point ref_point = {};
	//	//	cv::Point end_point = track.plates.rbegin()->get_center();

	//	//	for (auto rev_it = track.plates.rbegin(); rev_it != track.plates.rend(); ++rev_it)
	//	//	{
	//	//		ref_point = rev_it->get_center();
	//	//		if (cv::norm(end_point - ref_point) >= min_dist)
	//	//			break;
	//	//	}

	//	//	cv::line(debug, end_point, ref_point, cv::Scalar(255, 255, 255), 2, CV_AA);
	//	//}


	//}


	for (size_t i = 0; i < m_finished_tracks.size(); ++i)
	{
		const auto& color = m_finished_tracks[i].color();
		auto& track = m_finished_tracks[i];

		for (size_t j = 0; j < track.get_plates()->size() - 1; ++j)
		{
			const auto rect = track.get_plates()->at(j).get_rect();
			const auto rect_center = track.get_plates()->at(j).get_center();

			const auto rect_next = track.get_plates()->at(j + 1).get_rect();
			const auto rect_center_next = track.get_plates()->at(j + 1).get_center();
			cv::circle(debug, rect_center, 2, color, 2);

			cv::line(debug, rect_center, rect_center_next, color, 1, CV_AA);
		}

		const auto rect = track.get_plates()->back().get_rect();
		const auto rect_center = track.get_plates()->back().get_center();
		cv::circle(debug, rect_center, 2, color, 2);
	}
};
*/