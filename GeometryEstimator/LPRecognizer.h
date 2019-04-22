#pragma once
#include <memory>
#include <list>

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "LPRecognizerZone.h"
// TODO:
// 1) Возможность настраивать вручную макс и мин размеры номеров
// 2) Возможность накопления статистики и автомат. вычисления мин и макс размеров номеров
// 3) Сранвить производиельность при мастш. изображении и при масшт. номеров. Возможно, на этапе распознавания
//    пластин не стоит гнаться за качеством изображения, главное - получить координаты номера, а кач. изображение номера
//    потом взять из исходного изображения.

// TODO: СРОЧНО!
// А если номера на одном месте ? ? Движения нет, а мы думаем, что есть...
// -----------------------------> Бороться со стоячими номерами (точками) <-----------------------------

#define POINTS_TO_CALIBRATE 125
#define MIN_POINTS_TO_CALIBRATE 30
#define MAX_STOP_WEIGHT 100.0
#define PLATE_RESIZE_SCALE 1.3

class LPRecognizer
{
private:

	enum class CalibrationState
	{
		init,
		up_scale,
		up_search,
		down_scale,
		down_search,
		finished
	};

	CalibrationState m_state;
	std::list<LPRecognizerZone> m_zones;
	std::unique_ptr<cv::CascadeClassifier> p_plate_detector;

	cv::Size m_min_plate_size; 
	cv::Size m_max_plate_size;



	bool is_calib_finished = false;	
	LPRecognizerZone m_cur_zone;
	double m_cur_stop_weight = 0.0; //int m_cur_missed_frames = 0;
	size_t m_zero_zones_count = 0;

public:
	LPRecognizer();
	~LPRecognizer();

	bool initCascadeClassifier();	
	void correct_zones(const cv::Size& frame_size);
	void calibrate_zones(const cv::Mat& frame, cv::Mat& debug_frame);
	std::vector<cv::Rect> process_frame(const cv::Mat& frame, cv::Mat& debug_frame);

	void set_min_plate_size(const cv::Size& size);
	void set_max_plate_size(const cv::Size& size);

private:
	std::vector<cv::Rect> detect_plates(const cv::Mat& gray_frame, const cv::Rect& ROI, const cv::Size& plate_size, const int& min_neighbor) const;
};

