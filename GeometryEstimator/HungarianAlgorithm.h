#pragma once
#include <vector>
#include <limits>

class HungarianAlgorithm
{
public:
	HungarianAlgorithm() = default;
	~HungarianAlgorithm() = default;
	static std::vector<std::pair<int, int>> solve(const std::vector<std::vector<int>> &matrix);
};