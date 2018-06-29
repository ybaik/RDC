#include <stdio.h>
#include <conio.h>
#include "module/system.h"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world341d.lib")
#else
#pragma comment(lib, "opencv_world341.lib")
#endif

void main()
{
	// load image
	cv::Mat img = cv::imread("l.jpg", 1);
	int nLattice = 7;
	System sys(nLattice);
	sys.correct(img);

	cv::waitKey(0);
	printf("test ended\n");
}