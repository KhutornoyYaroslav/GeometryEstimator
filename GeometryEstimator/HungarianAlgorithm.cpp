#include "HungarianAlgorithm.h"

/*
* ������ ������ � ����������� ���������� �������.
* matrix: ������������� ������� �� ����� ����� (�� ����������� �������������).
*         ������ ������� ������ ���� �� ������ ������.
* ����������: ������ ��������� ���������, �� ������ �� ������ ������ �������.
*/

//http://acm.mipt.ru/twiki/bin/view/Algorithms/HungarianAlgorithmCPP

 std::vector<std::pair<int, int>> HungarianAlgorithm::solve(const std::vector<std::vector<int>> &matrix) 
 {
	// ������� �������
	int height = matrix.size();
	int width = matrix[0].size();

	if (height > width)
		return {};

	// ��������, ���������� �� ����� (u) � �������� (v)
	std::vector<int> u(height, 0);
	std::vector<int> v(width, 0);

	// ������ ���������� ������ � ������ �������
	std::vector<int> markIndices(width, -1);

	// ����� ��������� ������ ������� ���� �� ������
	for (int i = 0; i < height; i++) 
	{
		std::vector<int> links(width, -1);
		std::vector<int> mins(width, std::numeric_limits<int>::max());
		std::vector<int> visited(width, 0);

		// ���������� �������� (�������� "������������ �������" �� ������� ���������)
		int markedI = i, markedJ = -1, j;
		while (markedI != -1) 
		{
			// ������� ���������� � ��������� � ���������� ������� ������������ ��������
			// ������ �������� � j ������ ������������� ������� � ����� ��������� �� ���
			j = -1;
			for (int j1 = 0; j1 < width; j1++)
				if (!visited[j1]) 
				{
					if (matrix[markedI][j1] - u[markedI] - v[j1] < mins[j1]) 
					{
						mins[j1] = matrix[markedI][j1] - u[markedI] - v[j1];
						links[j1] = markedJ;
					}
					if (j == -1 || mins[j1] < mins[j])
						j = j1;
				}

			// ������ ��� ���������� ������� � ��������� (markIndices[links[j]], j)
			// ���������� ����������� �� �������� � ��������� ���, ����� �� ���������
			int delta = mins[j];
			for (int j1 = 0; j1 < width; j1++)
				if (visited[j1]) 
				{
					u[markIndices[j1]] += delta;
					v[j1] -= delta;
				}
				else 
				{
					mins[j1] -= delta;
				}

			u[i] += delta;

			// ���� �������� �� ��������� - �������� � ��������� ��������
			visited[j] = 1;
			markedJ = j;
			markedI = markIndices[j];
		}

		// ������� �� ��������� ������������ ������� ������, ������ ������� �
		// ���������� ������ � �������� ������� �� ������������
		for (; links[j] != -1; j = links[j])
			markIndices[j] = markIndices[links[j]];

		markIndices[j] = i;
	}

	// ������ ��������� � ������������ �����
	std::vector<std::pair<int, int>> result;
	for (int j = 0; j < width; j++)
		if (markIndices[j] != -1)
			result.push_back(std::pair<int, int>(markIndices[j], j));

	return result;
}
