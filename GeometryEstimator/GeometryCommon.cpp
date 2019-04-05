#include "GeometryCommon.h"

bool fitLineRansac(double thresh, size_t inliers_min, const std::vector<cv::Point2f>& points, LineF& line_result)
{
	size_t inliers_best = 0;

	for (auto p1 = points.begin(); p1 != points.end(); ++p1)
	{
		for (auto p2 = std::next(p1); p2 != points.end(); ++p2)
		{
			size_t inliers_count = 0;
			const LineF line(*p1, *p2);

			const float denominator = sqrt(line.A() * line.A() + line.B() * line.B());
			if (abs(denominator) < FLT_EPSILON)
				continue;

			for (size_t i = 0; i < points.size(); ++i)
			{
				const float rho = abs(line.A() * points[i].x + line.B() * points[i].y + line.C()) / denominator;
				if (rho < thresh)
					++inliers_count;
			}

			if (inliers_count < inliers_min)
				continue;

			if (inliers_count > inliers_best)
			{
				inliers_best = inliers_count;
				line_result = line;
			}
		}
	}

	if (inliers_best != 0)
		return true;
	else
		return false;
};

void EstimateRotherVP(const std::vector<LineF> &lines, cv::Point2f &vpoint, cv::Point2i frame_size, bool ontop)
{
	if (lines.empty())
	{
		return;
	}

	double weight_max = 0.0;
	cv::Point2f result_point = {};
	std::vector<cv::Point2f> intersections;

	// Compute lines intersection points
	for (auto it_line_a = lines.begin(); it_line_a != lines.end(); it_line_a++)
	{
		for (auto it_line_b = std::next(it_line_a); it_line_b != lines.end(); it_line_b++)
		{
			float d = (it_line_a->A() * it_line_b->B()) - (it_line_a->B() * it_line_b->A());

			if (abs(d) < FLT_EPSILON)
			{
				continue;
			}

			cv::Point2f point{ -((it_line_a->C() * it_line_b->B()) - (it_line_a->B() * it_line_b->C())) / d ,
				-((it_line_a->A() * it_line_b->C()) - (it_line_a->C() * it_line_b->A())) / d };

			// Does intersection point lie on either of two vanishing lines ?
			// To answer this quiestion we should to check two conditions:
			// 1) (x-x1)(y2-y1) - (y-y1)(x2-x1) = 0; the point lies on line. In our case this condition is always true.
			// 2) min(x1, x2) < x < max(x1, x2); the point lies between ends of line segment.
			const bool line_a_btw = (point.x > std::min(it_line_a->p1.x, it_line_a->p2.x)) && (point.x < std::max(it_line_a->p1.x, it_line_a->p2.x));
			const bool line_b_btw = (point.x > std::min(it_line_b->p1.x, it_line_b->p2.x)) && (point.x < std::max(it_line_b->p1.x, it_line_b->p2.x));

			if (!line_a_btw && !line_b_btw)
			{
				const float frame_center_y = static_cast<float>(frame_size.y) / 2.0f;

				if (ontop)
				{
					if (point.y < frame_center_y)
					{
						intersections.push_back(point);
					}
				}
				else
				{
					intersections.push_back(point);
				}
			}
		}
	}

	// Find max length of lines
	auto id_max = std::max_element(lines.begin(), lines.end(), [](const LineF &line_a, const LineF &line_b)
	{
		return line_a.length() < line_b.length();
	});

	double length_max = id_max->length();

	if (abs(length_max) < DBL_EPSILON || abs(ROTHER_MAX_ANG) < DBL_EPSILON)
	{
		return;
	}

	// Find vanishing point
	for (auto &point : intersections)
	{
		double weight = 0.0;
		for (auto &line : lines)
		{
			cv::Point2f midPoint;
			midPoint.x = (line.p1.x + line.p2.x) / 2.0f;
			midPoint.y = (line.p1.y + line.p2.y) / 2.0f;

			LineF lineToVp(midPoint, point);

			double ch = line.A() * lineToVp.A() + line.B() * lineToVp.B();
			double zn = sqrt(lineToVp.A() * lineToVp.A() + lineToVp.B() * lineToVp.B()) * sqrt(line.A() * line.A() + line.B() * line.B());

			if (abs(zn) < DBL_EPSILON)
			{
				continue;
			}

			double dangle = abs(acos(ch / zn)) * DEGRAD;

			if (dangle > 90.0)
			{
				dangle = 180.0 - dangle;
			}

			if (dangle < ROTHER_MAX_ANG)
			{
				double w = ROTHER_COEF_ANG * (1.0 - (dangle / ROTHER_MAX_ANG)) + ROTHER_COEF_LEN * (line.length() / length_max);
				weight = weight + w;
			}
		}

		if (weight > weight_max)
		{
			weight_max = weight;
			result_point = point;
		}
	}

	vpoint = result_point;
};