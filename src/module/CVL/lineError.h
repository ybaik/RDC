#include "parameters.h"

#include <opencv2/opencv.hpp>

class LineError
{
public:
	cv::Point2f points[POINTNUM];

	void inputPt(cv::Point2f *pts);
	double measureError(int xSize, int ySize, bool outerior = false);
};