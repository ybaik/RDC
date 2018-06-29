#ifndef SYSTEM_H
#define SYSTEM_H

#include "AutoLabeling.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


#define DEBUG_DRAW

class System  
{
public:
	System(int nLattice);
	virtual ~System();

	void correct(const cv::Mat& img);

	// Extract Corners
 	void extractCorners(
		const cv::Mat& gimg,
#ifdef DEBUG_DRAW
		const cv::Mat& cimg, 
#endif
		std::vector<cv::Point2f>& corners_out,
		bool bUseGaussian,
		bool bUseSharpen);

protected:
	int _nLattice;
};

#endif // SYSTEM_H
