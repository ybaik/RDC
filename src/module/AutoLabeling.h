#ifndef AUTO_LABELING_H
#define AUTO_LABELING_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


struct MarkerInfo
{
	MarkerInfo() {
		vertices.resize(4);
	}

	std::vector<cv::Point2f> vertices; // boundary 4 vertices
	cv::Point2f center; // a marker center point
	int area; // marker area
	float weight;
};

class AutoLabeling  
{
public:
	AutoLabeling(int nLattice);
	virtual ~AutoLabeling();

	bool autoLabeling(
		const cv::Mat& gimg, 
		const cv::Mat& cimg,
		const std::vector<cv::Point2f>& corners_in,
		std::vector<cv::Point2f>&corners_out,
		int nGrid);

	int _nLattice;

protected:

	// #1.Find marker
	void findMarkers(const cv::Mat& gimg, std::vector<MarkerInfo>& markerInfo); // main

	// #2 Align marker
	void alignMarkers(std::vector<MarkerInfo>& markerInfo);

	// #3 Find Mark Corners
	void findMarkCorners(
		const cv::Mat& gimg, 
		MarkerInfo& markerInfo, 
		std::vector<cv::Point2f>& corners, 
		int nIdx);

	// #4 Find center of rig and main lines
	void findRigCenterAndLines(
		const std::vector<MarkerInfo>& markerInfo,
		const std::vector<cv::Point2f>& corners,
		std::vector<cv::Point2f>& basis);
	
	// sub-functions of #4
	cv::Point3f computeLine(const cv::Point2f& p1, const cv::Point2f& p2);
	cv::Point2f computePoint(const cv::Point3f& l1, const cv::Point3f& l2);
	cv::Point2f findClosestPoint(const cv::Point2f& pt, const std::vector<cv::Point2f>& corners, float thres = FLT_MAX);
	cv::Point2f findPointusingLattice(cv::Point2f pt_curr, cv::Point2f pt_next, const std::vector<cv::Point2f>& corners, int nLattice);


	// #5 Input virtual corner in the markers
	void inputVirtualCorner(
		const std::vector<MarkerInfo>& markerInfo, 
		std::vector<cv::Point2f>& corners);


	// #6 Do auto-labeling
	void doAutoLabeling(
		const cv::Mat& cimg,
		const std::vector<cv::Point2f>& basis,
		const std::vector<cv::Point2f>& corners,
		std::vector<cv::Point2f>& arrLabeledPoint);

	// sub-functions of #6
	void findMainLineCorner(
		const std::vector<cv::Point2f>& arrCorner,
		std::vector<cv::Point2f>& arrLabeledPoint,
		const cv::Point2f& center,
		const cv::Point2f& pt,
		int size1,
		int size2,
		int nGrid);
	
	void findAllCorners(
		const std::vector<cv::Point2f>& arrCorner,
		std::vector<cv::Point2f>& arrLabeledPoint,
		int size, 
		int nGrid);


	// #7 ~~~
	void transposeAndInput(
		const std::vector<cv::Point2f>& arrSrc, 
		std::vector<cv::Point2f>& arrDst, 
		int idx1, 
		int idx2, 
		int nGrid);

};

#endif // AUTO_LABELING_H
