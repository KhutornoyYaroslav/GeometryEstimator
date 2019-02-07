#pragma once
#include "opencv2/imgproc.hpp"

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

bool fitLineRansac(double thresh, size_t inliers_min, const std::vector<cv::Point2f>& points, LineF& line_result)
{
	size_t inliers_best = 0;

	for (auto it_p1 = points.begin(); it_p1 != points.end(); ++it_p1)
	{
		for (auto it_p2 = std::next(it_p1); it_p2 != points.end(); ++it_p2)
		{
			size_t inliers_count = 0;
			const LineF line(*it_p1, *it_p2);

			const double denominator = sqrt(line.A() * line.A() + line.B() * line.B());
			if (abs(denominator) < DBL_EPSILON)
			{
				continue;
			}

			for (size_t i = 0; i < points.size(); ++i)
			{
				const double rho = abs(line.A() * points[i].x + line.B() * points[i].y + line.C()) / denominator;
				if (rho < thresh)
				{
					++inliers_count;
				}
			}

			if (inliers_count < inliers_min)
			{
				continue;
			}

			if (inliers_count > inliers_best)
			{
				inliers_best = inliers_count;
				line_result = line;
			}
		}
	}

	if (inliers_best != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
};

//void EstimateRotherVP(const std::vector<graphics::LineF> &lines, graphics::PointF &vpoint, graphics::Point frame_size, bool ontop)
//{
//	if (lines.empty())
//	{
//		return;
//	}
//
//	double weight_max = 0.0;
//	graphics::PointF result_point = {};
//	std::vector<graphics::PointF> intersections;
//
//	// Compute lines intersection points
//	for (auto it_line_a = lines.begin(); it_line_a != lines.end(); it_line_a++)
//	{
//		for (auto it_line_b = std::next(it_line_a); it_line_b != lines.end(); it_line_b++)
//		{
//			double d = (it_line_a->A() * it_line_b->B()) - (it_line_a->B() * it_line_b->A());
//
//			if (abs(d) < DBL_EPSILON)
//			{
//				continue;
//			}
//
//			graphics::PointF point{ -((it_line_a->C() * it_line_b->B()) - (it_line_a->B() * it_line_b->C())) / d ,
//				-((it_line_a->A() * it_line_b->C()) - (it_line_a->C() * it_line_b->A())) / d };
//
//			// (x-x1)(y2-y1) = (y-y1)(x2-x1)
//			const double dx_point_a = (point.x - it_line_a->p1.x) * (it_line_a->p2.y - it_line_a->p1.y);
//			const double dy_point_a = (point.y - it_line_a->p1.y) * (it_line_a->p2.x - it_line_a->p1.x);
//			const double dx_point_b = (point.x - it_line_b->p1.x) * (it_line_b->p2.y - it_line_b->p1.y);
//			const double dy_point_b = (point.y - it_line_b->p1.y) * (it_line_b->p2.x - it_line_b->p1.x);
//
//			if (!(abs(dx_point_b - dy_point_b) <= DBL_EPSILON || abs(dx_point_a - dy_point_a) <= DBL_EPSILON))
//			{
//				const double frame_center_y = static_cast<double>(frame_size.y) / 2.0;
//
//				if (ontop)
//				{
//					if (point.y < frame_center_y)
//					{
//						intersections.push_back(point);
//					}
//				}
//				else
//				{
//					intersections.push_back(point);
//				}
//			}
//		}
//	}
//
//	// Find max length of lines
//	auto id_max = std::max_element(lines.begin(), lines.end(), [](const graphics::LineF &line_a, const graphics::LineF &line_b)
//	{
//		return line_a.length() < line_b.length();
//	});
//
//	double length_max = id_max->length();
//
//	if (abs(length_max) < DBL_EPSILON || abs(ROTHER_MAX_ANG) < DBL_EPSILON)
//	{
//		return;
//	}
//
//	// Find vanishing point
//	for (auto &point : intersections)
//	{
//		double weight = 0.0;
//		for (auto &line : lines)
//		{
//			graphics::PointF midPoint;
//			midPoint.x = (line.p1.x + line.p2.x) / 2.0;
//			midPoint.y = (line.p1.y + line.p2.y) / 2.0;
//
//			graphics::LineF lineToVp(midPoint, point);
//
//			double ch = line.A() * lineToVp.A() + line.B() * lineToVp.B();
//			double zn = sqrt(lineToVp.A() * lineToVp.A() + lineToVp.B() * lineToVp.B()) * sqrt(line.A() * line.A() + line.B() * line.B());
//
//			if (abs(zn) < DBL_EPSILON)
//			{
//				continue;
//			}
//
//			double dangle = abs(acos(ch / zn)) * DEGRAD;
//
//			if (dangle > 90.0)
//			{
//				dangle = 180.0 - dangle;
//			}
//
//			if (dangle < ROTHER_MAX_ANG)
//			{
//				double w = ROTHER_COEF_ANG * (1.0 - (dangle / ROTHER_MAX_ANG)) + ROTHER_COEF_LEN * (line.length() / length_max);
//				weight = weight + w;
//			}
//		}
//
//		if (weight > weight_max)
//		{
//			weight_max = weight;
//			result_point = point;
//		}
//	}
//
//	vpoint = result_point;
//};