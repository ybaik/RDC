#include "lineError.h"
#include "rdclass.h"
#include "parameters.h"

#define MINPTNUM 3

void LineError::inputPt(cv::Point2f *pts)
{
	for(int i = 0 ; i <POINTNUM ; i++)
	{
		points[i] = pts[i];
	}
}

double LineError::measureError(int xSize, int ySize, bool outerior)
{
	cv::Mat A = cv::Mat(POINTNUM, 2, CV_64F, cv::Scalar(0));
	cv::Mat b = cv::Mat(POINTNUM, 1, CV_64F);

	// 3rd parameter of a line is 1.
	// matL is a line parameter.
	double error = 0;

	int count = 0;
	for (int i = 0; i < POINTNUM; i++)
	{
		if (outerior == true && points[i].x != -1)
		{
			A.at<double>(i, 0) = points[i].x;
			A.at<double>(i, 1) = points[i].y;
			b.at<double>(i) = -1;
			count++;
		}
		else if (outerior == false && points[i].x != -1 &&
			points[i].x >= -(float)xSize * 0.5f && points[i].x < (float)xSize * 1.5f &&
			points[i].y >= -(float)ySize * 0.5f && points[i].y < (float)ySize * 1.5f)
		{
			A.at<double>(i, 0) = points[i].x;
			A.at<double>(i, 1) = points[i].y;
			b.at<double>(i) = -1;
			count++;
		}
	}

	if (count < MINPTNUM) {
		return -1;
	}

	cv::Mat L = (A.t()*A).inv()*A.t()*b;

	cv::Point3f l, p;
	l.x = L.at<double>(0, 0);
	l.y = L.at<double>(1, 0);
	l.z = 1;

	count = 0;
	for (int i = 0; i < POINTNUM; i++)
	{
		if (outerior == true && points[i].x != -1)
		{
			error += lpDistance(l, points[i]);
			count++;
		}
		else if (outerior == false && points[i].x != -1 &&
			points[i].x >= -(float)xSize * 0.5f && points[i].x < (float)xSize * 1.5f &&
			points[i].y >= -(float)ySize * 0.5f && points[i].y < (float)ySize * 1.5f)
		{
			error += lpDistance(l, points[i]);
			count++;
		}

	}

	return error / (double)count;
}