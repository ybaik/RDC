#include "AutoLabeling.h"

#include <map>

AutoLabeling::AutoLabeling(int nLattice)
{
	_nLattice = nLattice;
}

AutoLabeling::~AutoLabeling()
{

}

void AutoLabeling::findMarkers(const cv::Mat& gimg, std::vector<MarkerInfo>& markerInfo)
{
	// equalization
	cv::Mat img;
	cv::equalizeHist(gimg, img);

	// binarization
	cv::Mat bin;
	cv::threshold(img, bin, 120, 255, cv::THRESH_BINARY);

	// closing and dilate
	cv::Mat morph;
	cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::morphologyEx(bin, morph, cv::MORPH_CLOSE, kernal);
	cv::dilate(morph, morph, kernal);

	// invert value
	cv::bitwise_not(morph, morph);

	// labeling
	cv::Mat stats, centroids, labelImage;
	int nLabels = cv::connectedComponentsWithStats(morph, labelImage, stats, centroids, 8, CV_32S);

	//find labels reach to boundaries
	std::map<int, int> boundary_labels;
	int cols = gimg.cols;
	int rows = gimg.rows;
	for (int x = 0; x < cols; x++) {
		int label;
		label = labelImage.at<int>(0, x);
		boundary_labels[label] = 1;

		label = labelImage.at<int>(rows - 1, x);
		boundary_labels[label] = 1;
	}

	for (int y = 0; y < rows; y++) {
		int label;
		label = labelImage.at<int>(y, 0);
		boundary_labels[label] = 1;

		label = labelImage.at<int>(y, cols - 1);
		boundary_labels[label] = 1;
	}

	// find largest 3 blobs
	// label zero is background

	cv::Point2f img_center = cv::Point2f((float)cols/2, (float)rows/2);
	for (int label_id = 0; label_id < nLabels; label_id++)
	{
		if (boundary_labels.count(label_id)) 
		{
			continue;
		}

		int area = stats.at<int>(label_id, cv::CC_STAT_AREA);
		float x = (float)centroids.at<double>(label_id, 0);
		float y = (float)centroids.at<double>(label_id, 1);
		cv::Point2f center = cv::Point2f(x, y);

		float dist = cv::norm(center - img_center);
		float weight = area * (rows - dist) / rows;

		if (weight >= markerInfo[0].weight)
		{
			markerInfo[2] = markerInfo[1];
			markerInfo[1] = markerInfo[0];

			markerInfo[0].area = area;
			markerInfo[0].weight = weight;
			markerInfo[0].center = center;
		}
		else if (weight >= markerInfo[1].weight)
		{
			markerInfo[2] = markerInfo[1];

			markerInfo[1].area = area;
			markerInfo[1].weight = weight;
			markerInfo[1].center = center;
		}
		else if (weight >= markerInfo[2].weight)
		{
			markerInfo[2].area = area;
			markerInfo[2].weight = weight;
			markerInfo[2].center = center;
		}
	}
}

void AutoLabeling::alignMarkers(std::vector<MarkerInfo>& markerInfo)
{
 	// upper left marker  = plane 3,        * (3)  * (1)
 	// upper right marker = plane 1,
 	// bottom marker      = plane 2             *(2) 
 		
 	// find a marker on plane 2
	int nIdx[3];
	nIdx[1] = -1;
	float max_val = 0;

	for (int i=0 ; i < 3 ; i++)
	{
		float val = markerInfo[i].center.y;
		if (val > max_val)
		{
			max_val = val;
			nIdx[1] = i;
		}
	}
	
	// find a marker on plane 3
	nIdx[0] = -1;
	int min_val = INT_MAX;

	for (int i=0 ; i < 3 ; i++)
	{
		if (i== nIdx[1]) { continue; }

		float val = markerInfo[i].center.x;
		
		if (val < min_val)
		{
			min_val = val;
			nIdx[2] = i;
		}
	}

	// find the mark of plane 1
	for (int i=0 ; i < 3; i++)
	{
		if (i==nIdx[1]) { continue; }
		if (i==nIdx[2]) { continue; }
		nIdx[0] = i;
	}

	// marker alignment
	MarkerInfo tmmarkerInfo[3];

	for (int i=0 ; i < 3 ; i++)
	{
		tmmarkerInfo[i] = markerInfo[i];
	}

	for (int i=0 ; i < 3 ; i++)
	{
		markerInfo[i] = tmmarkerInfo[nIdx[i]];
	}
}

void AutoLabeling::findMarkCorners(
	const cv::Mat& gimg,
	MarkerInfo& markerInfo,
	std::vector<cv::Point2f>& corners,
	int nIdx)
{
	cv::Point2f center = markerInfo.center;

	float minDis1 = FLT_MAX;
	float minDis2 = FLT_MAX;
	float minDis3 = FLT_MAX;
	float minDis4 = FLT_MAX;

	for (size_t i=0 ; i < corners.size(); i++)
	{
		// distance check
		cv::Point2f valPt = corners[i]-center;
		float dist = valPt.x*valPt.x + valPt.y*valPt.y;

		if (dist > 2500) { continue; } // larger than 50 pixels

		// check if corner is in the marker
		int cnt = 0;
		for (int j=0 ; j < 10 ; j++)
		{
			int x = (int) floor ( (j*0.1f)*corners[i].x + (1.0f-j*0.1f)*center.x );
			int y = (int) floor ( (j*0.1f)*corners[i].y + (1.0f-j*0.1f)*center.y );
			int val = gimg.at<uchar>(y, x);

			if (128 > val) { cnt++; }
		}

		if ( (float)cnt/10.0f < 0.8f) continue;

		// sorting
		if     (dist <= minDis1)
		{
			markerInfo.vertices.at(3) = markerInfo.vertices[2];
			markerInfo.vertices.at(2) = markerInfo.vertices[1];
			markerInfo.vertices.at(1) = markerInfo.vertices[0];
			markerInfo.vertices.at(0) = corners[i];
			minDis4 = minDis3;
			minDis3 = minDis2;
			minDis2 = minDis1;
			minDis1 = dist;

		}
		else if (dist <= minDis2)
		{
			markerInfo.vertices.at(3) = markerInfo.vertices[2];
			markerInfo.vertices.at(2) = markerInfo.vertices[1];
			markerInfo.vertices.at(1) = corners[i];
			minDis4 = minDis3;
			minDis3 = minDis2;
			minDis2 = dist;

		}
		else if (dist <= minDis3)
		{
			markerInfo.vertices.at(3) = markerInfo.vertices[2];
			markerInfo.vertices.at(2) = corners[i];
			minDis4 = minDis3;
			minDis3 = dist;
		}
		else if (dist <= minDis4)
		{
			markerInfo.vertices.at(3) = corners[i];
			minDis4 = dist;
		}
	}

	// align marker corners
	std::vector<cv::Point2f> arrPt;
	arrPt = markerInfo.vertices;

	if (nIdx == 0) 	// find the mark of plane 1
	{
		for (int i=0 ; i < 4 ; i++)
		{
			cv::Point2f pt = arrPt[i];

			if (pt.x < center.x)
			{
				if (pt.y > center.y) { markerInfo.vertices.at(0) = pt; }
				else				 { markerInfo.vertices.at(1) = pt; }
			}
			else
			{
				if (pt.y > center.y) { markerInfo.vertices.at(3) = pt; }
				else				 { markerInfo.vertices.at(2) = pt; }
			}
		}
	}

	if (nIdx == 1) 	// find the mark of plane 2
	{
		for (int i=0 ; i < 4 ; i++)
		{
			cv::Point2f pt = arrPt[i];

			float dx = fabs(pt.x - center.x);
			float dy = fabs(pt.y - center.y);

			if (dy < dx)
			{
				if (pt.x > center.x) { markerInfo.vertices.at(1) = pt; }
				else				 { markerInfo.vertices.at(3) = pt; }
			}
			else
			{
				if (pt.y > center.y) { markerInfo.vertices.at(2) = pt; }
				else                 { markerInfo.vertices.at(0) = pt; }
			}
		}
	}

	if (nIdx == 2) 	// find the mark of plane 3
	{
		for (int i=0 ; i < 4 ; i++)
		{
			cv::Point2f pt = arrPt[i];

			if (pt.x < center.x)
			{
				if (pt.y > center.y) { markerInfo.vertices.at(1) = pt; }
				else                 { markerInfo.vertices.at(2) = pt; }
			}
			else
			{
				if (pt.y > center.y) { markerInfo.vertices.at(0) = pt; }
				else                 { markerInfo.vertices.at(3) = pt; }
			}
		}
	}
}

cv::Point3f AutoLabeling::computeLine(const cv::Point2f& p1, const cv::Point2f& p2)
{
	cv::Point3f x1 = cv::Point3f(p1.x, p1.y, 1.0f);
	cv::Point3f x2 = cv::Point3f(p2.x, p2.y, 1.0f);

	x1 /= cv::norm(x1);
	x2 /= cv::norm(x2);

	cv::Point3f l = x1.cross(x2);
	return l / cv::norm(l);
}

cv::Point2f AutoLabeling::computePoint(const cv::Point3f& l1, const cv::Point3f& l2)
{
	cv::Point3f x = l1.cross(l2);
	if (x.z == 0) { return cv::Point2f (-1, -1); };
	x /= x.z;
	return cv::Point2f(x.x, x.y);
}

cv::Point2f AutoLabeling::findClosestPoint(const cv::Point2f& pt, const std::vector<cv::Point2f>& corners, float thres)
{
	float opt_dist = FLT_MAX;
	int opt_idx = -1;
	for (size_t i = 0; i < corners.size(); i++)
	{
		float dist = cv::norm(corners[i] - pt);
		if (thres < dist)
		{
			continue;
		}

		if (dist < opt_dist)
		{
			opt_dist = dist;
			opt_idx = (int)i;
		}
	}

	if (opt_idx == -1)
	{
		return cv::Point2f(-1, -1);
	}

	return corners[opt_idx];
}

cv::Point2f AutoLabeling::findPointusingLattice(cv::Point2f pt_curr, cv::Point2f pt_next, const std::vector<cv::Point2f>& corners, int nLattice)
{
	cv::Point pt = pt_next + (pt_next - pt_curr) / 2;
	pt_curr = pt_next;
	pt_next = findClosestPoint(pt, corners);

	for (int i = 0; i < _nLattice - 3; i++)
	{
		pt = pt_next + (pt_next - pt_curr);
		pt_curr = pt_next;
		pt_next = findClosestPoint(pt, corners);
	}

	return pt_next;
}

void AutoLabeling::findRigCenterAndLines(
	const std::vector<MarkerInfo>& markerInfo,
	const std::vector<cv::Point2f>& corners,
	std::vector<cv::Point2f>& basis)
{
	// find line between plane 1 and 2
	// find line 1
	basis.at(1) = findPointusingLattice(markerInfo[0].vertices[1], markerInfo[0].vertices[0], corners, _nLattice);
	basis.at(0) = findPointusingLattice(markerInfo[0].vertices[2], markerInfo[0].vertices[3], corners, _nLattice);


	// find line between plane 2 and 3
	// find line 2
	basis.at(3) = findPointusingLattice(markerInfo[2].vertices[3], markerInfo[2].vertices[0], corners, _nLattice);
	basis.at(2) = findPointusingLattice(markerInfo[2].vertices[2], markerInfo[2].vertices[1], corners, _nLattice);

	cv::Point3f mainL1 = computeLine(basis[0], basis[1]);
	cv::Point3f mainL2 = computeLine(basis[2], basis[3]);
	basis.at(5) = findClosestPoint(computePoint(mainL1, mainL2), corners);

	// find line between plane 3 and 1
	// find line 3
	basis.at(4) = findPointusingLattice(markerInfo[0].vertices[2], markerInfo[0].vertices[1], corners, _nLattice);
}

void AutoLabeling::inputVirtualCorner(
	const std::vector<MarkerInfo>& markerInfo, 
	std::vector<cv::Point2f>& corners)
{
	for (int i=0 ; i < 3 ; i++)
	{
		cv::Point2f pt1 = (markerInfo[i].vertices[0]+markerInfo[i].vertices[1])/2.0f;
		cv::Point2f pt2 = (markerInfo[i].vertices[2]+markerInfo[i].vertices[3])/2.0f;

		corners.push_back(pt1);
		corners.push_back(pt2);

		cv::Point3f l1 = computeLine(pt1, pt2);

		pt1 = (markerInfo[i].vertices[1]+markerInfo[i].vertices[2])/2.0f;
		pt2 = (markerInfo[i].vertices[0]+markerInfo[i].vertices[3])/2.0f;

		corners.push_back(pt1);
		corners.push_back(pt2);

		cv::Point3f l2 = computeLine(pt1, pt2);

		corners.push_back(computePoint(l1, l2));
	}
}

void AutoLabeling::findMainLineCorner(
	const std::vector<cv::Point2f>& arrCorner,
	std::vector<cv::Point2f>& arrLabeledPoint,
	const cv::Point2f& center,
	const cv::Point2f& pt,
	int size1,
	int size2,
	int nGrid)
{
	for (int i=1 ; i < _nLattice; i++)
	{
		int x = (int) floor ( i*pt.x + (_nLattice-i)*center.x )/(float)_nLattice;
		int y = (int) floor ( i*pt.y + (_nLattice -i)*center.y )/(float)_nLattice;

		cv::Point2f refPt = findClosestPoint( cv::Point2f(x, y), arrCorner, 10);

		if (cv::norm(refPt - center) > 2.0)
		{
			arrLabeledPoint.at(size1 +       1) = refPt;
		 	arrLabeledPoint.at(size2 + i*nGrid) = refPt;
			break;
		}
	}
	
	for (int i=2 ; i < nGrid ; i++)
	{
		cv::Point2f refPt1 = arrLabeledPoint[size1 + i-2];
		cv::Point2f refPt2 = arrLabeledPoint[size1 + i-1];

		if (refPt1.x < 0 || refPt2.x < 0) 
		{
			continue; 
		}

		cv::Point2f targetPt = 2.0f*refPt2-refPt1;
		float dist = cv::norm(refPt2 - refPt1);
		cv::Point2f refPt = findClosestPoint(targetPt, arrCorner, cv::min(10.0f, dist));
	 	arrLabeledPoint.at(size1 +       i) = refPt;
	 	arrLabeledPoint.at(size2 + i*nGrid) = refPt; 
	}
}

void AutoLabeling::findAllCorners(
	const std::vector<cv::Point2f>& arrCorner,
	std::vector<cv::Point2f>& arrLabeledPoint,
	int size,
	int nGrid)
{
	cv::Point2f refPt = arrLabeledPoint[size+1] + (arrLabeledPoint[size+nGrid] - arrLabeledPoint[size]);
	arrLabeledPoint.at(size+nGrid+1) = findClosestPoint(refPt, arrCorner, 15);

	// search 2nd axis..
	for (int i=2 ; i < nGrid ; i++)
	{
		cv::Point2f refPt1 = arrLabeledPoint[size+nGrid*(i-2)+1];
		cv::Point2f refPt2 = arrLabeledPoint[size+nGrid*(i-1)+1];

		if (refPt1.x < 0 || refPt2.x < 0) { continue; }

		cv::Point2f targetPt = 2.0f*refPt2-refPt1;
		refPt = findClosestPoint(targetPt, arrCorner, 15);
	 	arrLabeledPoint.at(size+nGrid*i+1) = refPt;
	}

	for (int j=1 ; j < nGrid ; j++)
	{
		int idx = nGrid*j;

		for (int i=2 ; i < nGrid ; i++)
		{
			cv::Point2f refPt1 = arrLabeledPoint[size+idx+i-2];
			cv::Point2f refPt2 = arrLabeledPoint[size+idx+i-1];

			if (refPt1.x < 0 || refPt2.x < 0) { continue; }

			cv::Point2f targetPt = 2.0f*refPt2-refPt1;
			refPt = findClosestPoint(targetPt, arrCorner, 15);
	 		arrLabeledPoint.at(size+idx+i) = refPt;
		}
	}
}

void AutoLabeling::doAutoLabeling(
		const cv::Mat& cimg,
		const std::vector<cv::Point2f>& basis,
		const std::vector<cv::Point2f>& corners,
		std::vector<cv::Point2f>& arrLabeledPoint)
{

	// pt[0~5]
	// pt[5] == center point of rig (0, 0, 0)

	// arrLabeledPoint[              0 ~ nGridX*nGridY*1-1 ] plane 1 (x,y)
	// arrLabeledPoint[nGridX*nGridY*1 ~ nGridX*nGridY*2-1 ] plane 2 (z,x)
	// arrLabeledPoint[nGridX*nGridY*2 ~ nGridX*nGridY*3-1 ] plane 3 (y,z)


	// ------------------------------------------------------------------------------------------------
	// initialization
	// ------------------------------------------------------------------------------------------------
	int nTotalSize = (int)arrLabeledPoint.size();
	int nSize  = nTotalSize / 3; // per plane 
	int nSize2 = nSize+nSize;
	int nGrid  = sqrt(nSize);

	// ------------------------------------------------------------------------------------------------
	// save rig center point
	// ------------------------------------------------------------------------------------------------
	arrLabeledPoint.at(0) = basis[5];
	arrLabeledPoint.at(nSize) = basis[5];
	arrLabeledPoint.at(nSize2) = basis[5];


	// ------------------------------------------------------------------------------------------------
	// find main line corner
	// ------------------------------------------------------------------------------------------------

	// points on the line between plane 1 and 2
	findMainLineCorner(corners, arrLabeledPoint, basis[5], basis[0], 0     , nSize , nGrid);

	// points on the line between plane 2 and 3
	findMainLineCorner(corners, arrLabeledPoint, basis[5], basis[2], nSize , nSize2, nGrid);

	// points on the line between plane 3 and 1
	findMainLineCorner(corners, arrLabeledPoint, basis[5], basis[4], nSize2, 0     , nGrid);


	// ------------------------------------------------------------------------------------------------
	// find corners
	// ------------------------------------------------------------------------------------------------

	// points on the plane 1
	findAllCorners(corners, arrLabeledPoint, 0     , nGrid);
	
	// points on the plane 2
	findAllCorners(corners, arrLabeledPoint, nSize , nGrid);

	// points on the plane 3
	findAllCorners(corners, arrLabeledPoint, nSize2, nGrid);
}

void AutoLabeling::transposeAndInput(
	const std::vector<cv::Point2f>& arrSrc,
	std::vector<cv::Point2f>& arrDst,
	int idx1,
	int idx2,
	int nGrid)
{
	for (int j=0 ; j < nGrid ; j++)
	{
		int val1 = nGrid*j;
		int val2 = j;

		for (int i=0 ; i < nGrid ; i++)
		{
			cv::Point2f pt = arrSrc[idx1+val1+i];

			if (pt.x < 0)
			{
				arrDst.at(val2+idx2) = cv::Point2f(-1, -1);
			}
			else
			{
				arrDst.at(val2+idx2) = pt;
			}

			val2 += nGrid;
		}
	}
}

bool AutoLabeling::autoLabeling(
	const cv::Mat& gimg,
	const cv::Mat& cimg,
	const std::vector<cv::Point2f>& corners_in,
	std::vector<cv::Point2f>&corners_out,
	int nGrid)
{
 	int nSize  = nGrid*nGrid; // nGridX * nGridY (we assume that a checker board is square).
 	int nSize2 = nSize+nSize;
 	int nTotalSize = nSize*3; // nGridX * nGridY * Number of Plane
 
 	// Find big 3 markers
	std::vector<MarkerInfo> markerInfo(3);
 	findMarkers(gimg, markerInfo);

#ifdef _DEBUG
	cv::Mat canvas = cimg.clone();
	for (const auto& pt: corners_in)
	{
		cv::drawMarker(canvas, pt, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 5, 1);
	}

	for (size_t i = 0; i < markerInfo.size(); i++)
	{
		cv::drawMarker(canvas, markerInfo[i].center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 5, 1);
	}
#endif

 	// Align big 3 markers
 	alignMarkers(markerInfo);


#ifdef _DEBUG
	int fontFace = cv::FONT_HERSHEY_PLAIN;
	for (size_t i=0 ; i < markerInfo.size() ; i++)
	{
		cv::Point2f t = cv::Point2f(4, 4);
		cv::Point2f pt1 = markerInfo[i].center - t;
		cv::Point2f pt2 = markerInfo[i].center - t;

		char text[255];
		sprintf(text, "%d", i + 1);
		cv::putText(canvas, text, cv::Point(pt1.x+10, pt1.y+10), fontFace, 1.0, cv::Scalar(255,255,255));
	}
#endif

	// Find corners of markers
	std::vector<cv::Point2f> corners = corners_in;

	int cnt = 1;
	for (size_t i=0 ; i < 3 ; i++)
	{
		findMarkCorners(gimg, markerInfo[i], corners, i);

#ifdef _DEBUG		
		char text[255];
		for (size_t j=0 ; j < 4 ; j++)
		{
			cv::Point2f pt = markerInfo[i].vertices[j];

			cv::circle(canvas, pt, 3, cv::Scalar(0, 0, 255), 2);

			sprintf(text, "%d",cnt);
			cv::putText(canvas, text, cv::Point(pt.x+10, pt.y+10), fontFace, 0.8, cv::Scalar(0,255,255));
			cnt++;
		}
#endif
	}

	// Find center of rig and each axis

	std::vector<cv::Point2f> basis(6);
	findRigCenterAndLines(markerInfo, corners, basis);


#ifdef _DEBUG
	cv::line(canvas, basis[5], basis[0], cv::Scalar(0, 0, 255), 2); // line between plane 1 and 2
	cv::line(canvas, basis[5], basis[2], cv::Scalar(0, 0, 255), 2); // line between plane 2 and 3
	cv::line(canvas, basis[5], basis[4], cv::Scalar(0, 0, 255), 2); // line between plane 3 and 1
	cv::circle(canvas, basis[0], 3, cv::Scalar(255, 0, 0), 2);
	cv::circle(canvas, basis[2], 3, cv::Scalar(255, 0, 0), 2);
	cv::circle(canvas, basis[4], 3, cv::Scalar(255, 0, 0), 2);

	cv::circle(canvas, basis[1], 3, cv::Scalar(255, 0, 0), 2);
	cv::circle(canvas, basis[3], 3, cv::Scalar(255, 0, 0), 2);
	cv::circle(canvas, basis[5], 3, cv::Scalar(255, 0, 0), 2);

	cv::putText(canvas, "X", cv::Point(basis[0].x + 10, basis[0].y + 10), fontFace, 0.8, cv::Scalar(0, 255, 0));
	cv::putText(canvas, "Y", cv::Point(basis[2].x + 10, basis[2].y + 10), fontFace, 0.8, cv::Scalar(0, 255, 0));
	cv::putText(canvas, "Z", cv::Point(basis[4].x + 10, basis[4].y + 10), fontFace, 0.8, cv::Scalar(0, 255, 0));
#endif


	// Input virtual corner at the center of markers
	// (To make easy auto-labeling)
	inputVirtualCorner(markerInfo, corners);


	// do auto-labeling
	std::vector<cv::Point2f> arrLabeledPoint(nTotalSize);

	// initialize memory set to (-1, -1)
	for (int i=0 ; i < nTotalSize ; i++)
	{
		arrLabeledPoint.at(i) = cv::Point2f(-1,-1);
	}

	doAutoLabeling(cimg, basis, corners, arrLabeledPoint);

	
#ifdef _DEBUG
	// draw some features to make debug view
	CvScalar color = CV_RGB(255, 0, 255);

	for (size_t i = 0; i < arrLabeledPoint.size()-1; i++)
	{
		const cv::Point2f& pt_prev = arrLabeledPoint[i];
		const cv::Point2f& pt_next = arrLabeledPoint[i+1];

		if ((i + 1) % nGrid == 0) continue;

		if (pt_prev.x == -1 || pt_next.x == -1) continue;

		cv::line(canvas, pt_prev, pt_next, CV_RGB(255, 255, 255), 1);
	}

	for (size_t i=0 ; i < arrLabeledPoint.size() ; i++)
	{
		const cv::Point2f& pt = arrLabeledPoint[i];
		cv::drawMarker(canvas, pt, CV_RGB(255, 0, 0), cv::MARKER_CROSS, 5, 1);


		if (i >= nSize && i < nSize2)
		{
			color = CV_RGB(0, 0, 255);
		}
		else if (i > nSize2)
		{
			color = CV_RGB(255, 255, 0);
		}

		cv::drawMarker(canvas, pt, color, cv::MARKER_SQUARE, 5, 1);
	}

	color = CV_RGB(0, 0, 255);
	cv::line(canvas, arrLabeledPoint[0], arrLabeledPoint[_nLattice       ], color, 2); // line between plane 1 and 2
	cv::line(canvas, arrLabeledPoint[0], arrLabeledPoint[_nLattice+nSize ], color, 2); // line between plane 2 and 3
	cv::line(canvas, arrLabeledPoint[0], arrLabeledPoint[_nLattice+nSize2], color, 2); // line between plane 3 and 1

	color = CV_RGB(255, 0, 0);
	cv::circle(canvas, arrLabeledPoint[_nLattice       ], 3, color, 2);
	cv::circle(canvas, arrLabeledPoint[_nLattice+nSize ], 3, color, 2);
	cv::circle(canvas, arrLabeledPoint[_nLattice+nSize2], 3, color, 2);

	color = CV_RGB(0, 255, 0);
	for (int i=0 ; i < _nLattice ; i++)
	{
		cv::Point2f pt;
		pt = arrLabeledPoint[i];
		cv::drawMarker(canvas, pt, color, cv::MARKER_SQUARE, 5, 1);
		cv::drawMarker(canvas, pt, color, cv::MARKER_CROSS, 5, 1);

		pt = arrLabeledPoint[i+nSize];
		cv::drawMarker(canvas, pt, color, cv::MARKER_SQUARE, 5, 1);
		cv::drawMarker(canvas, pt, color, cv::MARKER_CROSS, 5, 1);

		pt = arrLabeledPoint[i+nSize2];
		cv::drawMarker(canvas, pt, color, cv::MARKER_SQUARE, 5, 1);
		cv::drawMarker(canvas, pt, color, cv::MARKER_CROSS, 5, 1);
	}

	cv::imshow("labeling", canvas);
#endif _DEBUG

	// Aligned save for RDC and camera calibration
	corners_out.clear();
	corners_out.resize(nSize2+nSize);

	// plane 1
	transposeAndInput(arrLabeledPoint, corners_out,     0,      0, nGrid);

	// plane 2
	transposeAndInput(arrLabeledPoint, corners_out, nSize, nSize2, nGrid);

	// plane 3
	transposeAndInput(arrLabeledPoint, corners_out, nSize2, nSize, nGrid);

 	return true;
}
