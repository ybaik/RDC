#include "system.h"
#include <fstream>

System::System(int nLattice)
{
	_nLattice = nLattice;
}

System::~System()
{

}

void System::correct(const cv::Mat& img)
{
	int channel = img.channels();

	cv::Mat cimg, gimg;
	if (channel == 3)
	{
		cimg = img.clone();
		cv::cvtColor(cimg, gimg, CV_RGB2GRAY);
	}
	else
	{
		gimg = img.clone();
		cv::cvtColor(gimg, cimg, CV_GRAY2RGB);
	}

	// extract corners
	std::vector<cv::Point2f> corners;
	//extractCorners(gimg, cimg, corners, true, true);

	// auto labeling
	int nGrid = 24;
	AutoLabeling Labeling(_nLattice);
	std::vector<cv::Point2f> corners_labeled;
	//Labeling.autoLabeling(gimg, cimg, corners, corners_labeled, nGrid);


	vecPoint2f labeledPoints;
	cv::Mat canvas = cimg.clone();
	load_labeled_points("l1.txt", labeledPoints, canvas);


	std::vector<cv::Point2f> pts2d0;
	std::vector<cv::Point2f> pts2d1;
	std::vector<cv::Point2f> pts2d2;
	std::vector<cv::Point3f> pts3d0;
	std::vector<cv::Point3f> pts3d1;
	std::vector<cv::Point3f> pts3d2;

	int nGrid_sq = nGrid*nGrid;

	std::vector<std::vector<cv::Point2f>> imagePoints;
	std::vector<std::vector<cv::Point3f> > objectPoints;


	for (int i = 0; i < (int)labeledPoints.size(); i++)
	{
		const cv::Point2f& pt = labeledPoints[i];
		
		if (pt.x < 0 || pt.y < 0) {
			continue;
		}

		int plane = i / nGrid_sq;
		int index = i % nGrid_sq;

		if (plane == 0)
		{
			float x = index / nGrid;
			float y = index % nGrid;
			//pts3d0.push_back(cv::Point3f(x * 20, y * 20, 0));
			pts3d0.push_back(cv::Point3f(y * 20, x * 20, 0));
			pts2d0.push_back(pt);
		}
		if (plane == 1)
		{
			float y = index / nGrid;
			float z = index % nGrid;

			//if (y == 0) continue;

			pts3d1.push_back(cv::Point3f(z * 20, y * 20, 0));
			//pts3d1.push_back(cv::Point3f(0, y * 20, z * 20));
			pts2d1.push_back(pt);
		}
		if (plane == 2)
		{
			float z = index / nGrid;
			float x = index % nGrid;

			//if (z == 0) continue;
			//if (x == 0) continue;

			pts3d2.push_back(cv::Point3f(x * 20, z * 20, 0));
			//pts3d.push_back(cv::Point3f(x * 20, 0, z * 20));
			pts2d2.push_back(pt);
		}
	}

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;

	imagePoints.push_back(pts2d0);
	imagePoints.push_back(pts2d1);
	imagePoints.push_back(pts2d2);
	objectPoints.push_back(pts3d0);
	objectPoints.push_back(pts3d1);
	objectPoints.push_back(pts3d2);
	cv::calibrateCamera(objectPoints, imagePoints, cimg.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
	cv::Mat temp = cimg.clone();
	undistort(temp, cimg, cameraMatrix, distCoeffs);

	int a = 0;

	// correction
	//cv::Point2f cod;
	//double w;
	//std::vector<cv::Point2f> corners_corrected;
	//correction(cimg, nGrid, labeledPoints, corners_corrected, cod, w, true);
}

void System::extractCorners(
	const cv::Mat& gimg, 
#ifdef _DEBUG
	const cv::Mat& cimg,
#endif
	std::vector<cv::Point2f>& corners_out,
	bool bUseGaussian,
	bool bUseSharpen)
{
	cv::Mat gray = gimg.clone();
#ifdef _DEBUG
	cv::Mat canvas = cimg.clone();
#endif

	// do gaussian blurring
	if (bUseGaussian)
	{
		cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0);
	}

	// do sharpening
	if (bUseSharpen)
	{
		cv::addWeighted(gray, 1.5, gray, -0.5, 0, gray);
	}

	// extract corners
	std::vector<cv::Point2f> corners;

	int blockSize = 3;
	int maxCorners = 3000;
	bool useHarrisDetector = false;
	float qualityLevel = 0.05f;
	float minDistance = 8.0f;
	double k = 0.04;

	cv::goodFeaturesToTrack(gimg,
		corners,
		maxCorners,
		qualityLevel,
		minDistance,
		cv::Mat(),
		blockSize,
		useHarrisDetector,
		k);


	// check boundary condition
	int rows = gimg.rows;
	int cols = gimg.cols;

	for (int i = 0; i < corners.size(); i++)
	{
		cv::Point2f& pt = corners[i];

		if (pt.x < 2 || pt.x > cols - 3) {
			continue;
		}

		if (pt.y < 2 || pt.y > rows - 3) {
			continue;
		}

		corners_out.push_back(pt);
#ifdef _DEBUG
		cv::drawMarker(canvas, pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 5, 1);
#endif
	}

#ifdef _DEBUG
	cv::imshow("corners", canvas);
#endif
}

void System::load_labeled_points(char* fn, vecPoint2f& labeledPoints, cv::Mat& canvas)
{
	std::ifstream infile;
	infile.open(fn);

	vecPoint2f arrFilePoint;
	float x, y;

	int nGrid;
	infile >> nGrid;

	while (!infile.eof())
	{
		infile >> x >> y;
		arrFilePoint.push_back(cv::Point2f((float)(x), (float)(y)));
	}

	//arrFilePoint.RemoveAt(arrFilePoint.GetSize() - 1);

	infile.close();

	// view
	for (int i = 0; i < arrFilePoint.size(); i++)
	{
		CvPoint pt;
		pt.x = int(arrFilePoint[i].x + 0.5);
		pt.y = int(arrFilePoint[i].y + 0.5);

		if (pt.x < 0.0f || pt.y < 0.0f) continue;

		int r, g, b;
		r = (i / POINTNUM_SQ == 0) ? 255 : 0;
		g = (i / POINTNUM_SQ == 1) ? 255 : 0;
		b = (i / POINTNUM_SQ == 2) ? 255 : 0;

		cv::circle(canvas, pt, 2, CV_RGB(r, g, b), 2);
	}

	cv::imshow("image", canvas);

	labeledPoints.clear();
	labeledPoints = arrFilePoint;
}