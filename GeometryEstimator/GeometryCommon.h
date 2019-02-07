#pragma once
#include "opencv2/imgproc.hpp"

#define DEGRAD  57.295779513082320876798154814105

#define ROTHER_MAX_ANG 5.0
#define ROTHER_COEF_LEN 0.8
#define ROTHER_COEF_ANG 0.2

template <typename T>
struct Line_
{
	cv::Point_<T> p1 = {}, p2 = {};
	Line_() = default;
	Line_(const cv::Point_<T>& p1, const cv::Point_<T>& p2) : p1(p1), p2(p2) {}
	Line_(T x1, T y1, T x2, T y2) : Line_(cv::Point_<T>(x1, y1), cv::Point_<T>(x2, y2)) {}

	friend bool operator==(const Line_& lhs, const Line_& rhs)
	{
		return lhs.p1.x == rhs.p1.x &&
			lhs.p1.y == rhs.p1.y &&
			lhs.p2.x == rhs.p2.x &&
			lhs.p2.y == rhs.p2.y;
	};

	double length() const
	{
		T dx = p1.x - p2.x;
		T dy = p1.y - p2.y;
		return sqrt(dx * dx + dy * dy);
	};

	T A() const { return p1.y - p2.y; };
	T B() const { return p2.x - p1.x; };
	T C() const { return (p1.x * p2.y) - (p2.x * p1.y); };
};

typedef Line_<int> Line;
typedef Line_<float> LineF;
typedef Line_<double> LineD;

bool fitLineRansac(double thresh, size_t inliers_min, const std::vector<cv::Point2f>& points, LineF& line_result);
void EstimateRotherVP(const std::vector<LineF> &lines, cv::Point2f &vpoint, cv::Point2i frame_size, bool ontop);