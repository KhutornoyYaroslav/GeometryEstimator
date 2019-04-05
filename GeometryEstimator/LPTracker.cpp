#include "LPTracker.h"

LPTracker::LPTracker()
{
	p_recognizer = std::make_unique<LPRecognizer>();
	p_recognizer->initCascadeClassifier(); // check to error
}

void LPTracker::process_frame(const cv::Mat& frame, cv::Mat debug)
{
	std::vector<cv::Rect> plates = p_recognizer->process_frame(frame, debug);

	const int height = plates.size(); // Распознанные номера на текущем кадре 
	const int width = m_tracks.size();//std::count_if(m_tracks.begin(), m_tracks.end(), [](const LPTrack& track) {return !track.finished; }); // Существующие треки в процессе

	// Два случая:
	// height <= width - разрешенный
	// height > width - запрещенный

	const size_t padding = std::max(0, height - width);

	// Высота матрицы должна быть не больше ширины.
	std::vector<std::vector<int>> matrix;

	for (size_t i = 0; i < height; ++i)
	{
		auto& new_plate = plates[i];
		cv::Point2d new_plate_center(new_plate.x + (new_plate.width / 2), new_plate.y + (new_plate.height / 2));


		std::vector<int> row; 
		// Заполняем row
		for (size_t j = 0; j < width; ++j)
		{
			auto& old_plate = m_tracks[j].points.back();
			cv::Point2d old_plate_center(old_plate.x + (old_plate.width / 2), old_plate.y + (old_plate.height / 2));

			const double dx = new_plate_center.x - old_plate_center.x;
			const double dy = new_plate_center.y - old_plate_center.y;
			/*double angle = 0.0;
			const double dx = new_plate_center.x - old_plate_center.x;
			if(abs(dx) < DBL_EPSILON)
				angle = 90.0;
			else
				angle = DEGREE_IN_RADIAN * atan((new_plate_center.y - old_plate_center.y) / dx);*/

			double dist = sqrt(dx * dx + dy * dy);

			row.push_back(-dist);
		}
		//Дозаполняем, если надо
		// TODO:
		for (size_t j = 0; j < padding; ++j)
		{
			row.push_back(0);
		}

		// Сохраняем строку в матрицу 
		matrix.push_back(row);
	}

	//// Решаем венгерским алгоритмом
	auto solution = HungarianAlgorithm::solve(matrix);

	for (size_t i = 0; i < solution.size(); ++i)
	{
		printf("solution[%u]: col[%i] = %i \r\n", i, solution[i].second, solution[i].first);
	}
	printf("\r\n");

	//printf("solution size = %u \r\n", solution.size());
	//// Дополняем треки
	size_t size = std::min(m_tracks.size(), solution.size());

	for (size_t i = 0; i < size; ++i)
	{
		m_tracks[solution[i].second].points.push_back(plates[solution[i].first]);
	}

	for (size_t i = 0; i < solution.size() - size; ++i)
	{
		LPTrack new_track;
		new_track.points.push_back(plates[solution[i].first]);
		m_tracks.push_back(new_track);
	}

	// Draw tracks
	for (size_t i = 0; i < m_tracks.size(); ++i)
	{
		auto& track = m_tracks[i];

		for (size_t j = 0; j < track.points.size() - 1; ++j)
		{
			auto& rect = track.points[j];
			cv::Point rect_center(rect.x + (rect.width / 2), rect.y + (rect.height / 2));

			auto& rect_next = track.points[j+1];
			cv::Point rect_center_next(rect_next.x + (rect_next.width / 2), rect_next.y + (rect_next.height / 2));
			cv::circle(debug, rect_center, 2, cv::Scalar(0, 0, 255), 2);

			cv::line(debug, rect_center, rect_center_next, cv::Scalar(0, 0, 255), 1, CV_AA);
		}

		auto& rect = track.points.back();
		cv::Point rect_center(rect.x + (rect.width / 2), rect.y + (rect.height / 2));
		cv::circle(debug, rect_center, 2, cv::Scalar(0, 0, 255), 2);
	}


	//tODO: убрать это потом
	//m_tracks.clear();

	cvWaitKey(0);
};