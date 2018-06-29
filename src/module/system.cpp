#include "system.h"

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
	extractCorners(gimg, cimg, corners, true, true);


	// auto labeling
	int nGrid = 24;
	AutoLabeling Labeling(_nLattice);
	std::vector<cv::Point2f> corners_labeled;
	Labeling.autoLabeling(gimg, cimg, corners, corners_labeled, nGrid);


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

