#include "rdclass.h"
#include "math.h"
#include "lineError.h"

#define HOLE 0
#define OUTSIDE 1

#define PI 3.1415926535

int sideCheck(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& center)
{
	cv::Point3f p1 = cv::Point3f(pt1.x, pt1.y, 1);
	cv::Point3f p2 = cv::Point3f(pt2.x, pt2.y, 1);
	cv::Point3f c = cv::Point3f(center.x, center.y, 1);
	cv::Point3f l = p1.cross(p2);

	float check = l.dot(c);

	if (check < 0) { return 1; }
	else		   { return 2; }
}

int CenterLocation(const cv::Point2f& center, const vecPoint2f& arrDisPoint)
{
	int firstCheck, secondCheck, thirdCheck;
	firstCheck	= sideCheck(arrDisPoint[0], arrDisPoint[1]                           , center);
	secondCheck	= sideCheck(arrDisPoint[0], arrDisPoint[POINTNUM*POINTNUM*2+POINTNUM], center);
	thirdCheck	= sideCheck(arrDisPoint[0], arrDisPoint[POINTNUM*POINTNUM*2+1]       , center);

	if (firstCheck == 2 && thirdCheck == 1)
	{
		return 1;
	}
	else if(firstCheck == 1 && secondCheck == 2)
	{
		return 2;
	}
	else
	{
		return 3;
	}
}

void gridDeform2(const cv::Mat& img, vecPoint2f& arrDisPoint, cv::Point2f& center)
{
	int cols = img.cols;
	int rows = img.rows;


	vecPoint2f ptArray = arrDisPoint;
	arrDisPoint.clear();

	int start = POINTNUM_SQ;
	int end = MIN(2 * POINTNUM_SQ, ptArray.size());

	for (int i = start; i < end; i++)
	{
		int temp = i - start;
		int newIdx = (temp%POINTNUM) * POINTNUM + temp / POINTNUM;
		newIdx += start;

		if (ptArray[newIdx].x == -1)
		{
			arrDisPoint.push_back(cv::Point2f(-1, -1));
		}
		else
		{
			arrDisPoint.push_back(cv::Point2f(cols - ptArray[newIdx].x, ptArray[newIdx].y));
		}
	}

	end = MIN(POINTNUM_SQ, ptArray.size());

	for (int i = 0; i < end; i++)
	{
		int newIdx = (i%POINTNUM) * POINTNUM + i / POINTNUM;

		if (ptArray[newIdx].x == -1)
		{
			arrDisPoint.push_back(cv::Point2f(-1, -1));
		}
		else
		{
			arrDisPoint.push_back(cv::Point2f(cols - ptArray[newIdx].x, ptArray[newIdx].y));
		}
	}

	start = 2 * POINTNUM_SQ;
	end = MIN(3 * POINTNUM_SQ, ptArray.size());

	for (int i = start; i < end; i++)
	{
		int temp = i - start;
		int newIdx = (temp%POINTNUM) * POINTNUM + temp / POINTNUM;
		newIdx += start;

		if (ptArray[newIdx].x == -1)
		{
			arrDisPoint.push_back(cv::Point2f(-1, -1));
		}
		else
		{
			arrDisPoint.push_back(cv::Point2f(cols - ptArray[newIdx].x, ptArray[newIdx].y));
		}
	}

	// change the center
	center.x = cols - center.x;
}

void gridDeform3(const cv::Mat& img, vecPoint2f& arrDisPoint, cv::Point2f& center)
{
	int cols = img.cols;
	int rows = img.rows;

	std::ifstream infile;

	vecPoint2f ptArray = arrDisPoint;
	cv::Point2f temp;

	arrDisPoint.clear();

	int start = 2 * POINTNUM_SQ;
	int end = MIN(3 * POINTNUM_SQ, ptArray.size());

	for (int i = start; i < end; i++)
	{
		int temp, newIdx;
		temp = i - start;
		newIdx = (temp%POINTNUM) * POINTNUM + temp / POINTNUM;
		newIdx += start;

		if (ptArray[newIdx].x == -1)
		{
			arrDisPoint.push_back(cv::Point2f(-1, -1));
		}
		else
		{
			arrDisPoint.push_back(cv::Point2f(cols - ptArray[newIdx].x, ptArray[newIdx].y));
		}
	}

	end = MIN(POINTNUM_SQ, ptArray.size());

	for (int i = 0; i < end; i++)
	{
		int newIdx;
		newIdx = i;


		if (ptArray[newIdx].x == -1)
		{
			arrDisPoint.push_back(cv::Point2f(-1, -1));
		}
		else
		{
			arrDisPoint.push_back(cv::Point2f(cols - ptArray[newIdx].x, ptArray[newIdx].y));
		}
	}

	start = POINTNUM_SQ;
	end = MIN(2 * POINTNUM_SQ, ptArray.size());

	for (int i = POINTNUM_SQ; i < end; i++)
	{
		int temp, newIdx;
		temp = i - start;
		newIdx = (temp%POINTNUM) * POINTNUM + temp / POINTNUM;
		newIdx += start;

		if (ptArray[newIdx].x == -1)
		{
			arrDisPoint.push_back(cv::Point2f(-1, -1));
		}
		else
		{
			arrDisPoint.push_back(cv::Point2f(cols - ptArray[newIdx].x, ptArray[newIdx].y));
		}
	}

	// change the center
	cv::Point2f center_temp;
	center_temp = center;
	center.x = center_temp.y;
	center.y = cols - center_temp.x;

}

void parameterTransform2(const cv::Mat& img, cv::Point2f& center)
{
	center.x	= img.cols - center.x;
}

void parameterTransform3(const cv::Mat& img, cv::Point2f& center)
{
	cv::Point2f center_temp;
	center_temp	= center;
	center.x	= img.cols - center_temp.y;
	center.y	= center_temp.x;
}

void correction(const cv::Mat& img, 
	const int nPointNum,
	const vecPoint2f& arrDisPoint_ori,
	vecPoint2f& arrCorPoint,
	cv::Point2f& ptCOD,
	double& w,
	bool COD_pp)
{
	vecPoint2f arrDisPoint;

	int rows = img.rows;
	int cols = img.cols;

	arrDisPoint = arrDisPoint_ori;

	// grid deformation
	SetofPoints setTemp(LINENUM1, nPointNum, arrDisPoint);
	
	ptCOD = findCenter(img, setTemp);

	int centerLoc = CenterLocation(ptCOD, arrDisPoint);
	if (centerLoc == 2)
		gridDeform2(img, arrDisPoint, ptCOD);
	if (centerLoc == 3)
		gridDeform3(img, arrDisPoint, ptCOD);


	SetofPoints set(LINENUM1, nPointNum, arrDisPoint);

	// find COD (center of distortion)
	ptCOD = findCenter(img, set);

	lineForAngle(img, ptCOD, set);

	int angleIdx[3];

	if(COD_pp == true) // COD == p.p 
	{
		findVanishPoint(img, ptCOD, set);
				
		calcAngl(img, ptCOD, set, angleIdx, COD_pp);
	}
	else // COD != p.p
	{
		calcAngl(img, ptCOD, set, angleIdx, COD_pp);

		findVanishPoint2(img, ptCOD, angleIdx, set);
		
		calcAngl(img, ptCOD, set, angleIdx, true);
	}

	reconstructLines(img, ptCOD, arrDisPoint, arrCorPoint, set);

	FOV(ptCOD, arrDisPoint, arrCorPoint, w, cols, rows);

	if (centerLoc == 2)
	{
		parameterTransform2(img, ptCOD);
	}
	if (centerLoc == 3)
	{
		parameterTransform3(img, ptCOD);
	}
}

cv::Point2f findCenter(const cv::Mat& img, SetofPoints& set)
{
	cv::Point2f ptCenter;
	int rows = img.rows;
	int cols = img.cols;
	int newRows = 3 * rows;
	int newCols = 3 * cols;

	int lineType;
	int cLine[3];

	cLine[0] = 2;	// starting line
	cLine[1] = 2;
	cLine[2] = 2;

	int lineNum = LINENUM1;

	int xend = 2 * cols;
	int yend = 2 * rows;
	

	int maxR, smaxR;
	int removR[LINENUM1];

	float error = FLT_MAX;
	
	int count= 0 ;

	while (error > 50)
	{
		for (lineType=0 ; lineType <= 2 ; lineType++)
		{
			// comupte line equation
			set.setTypeLine(lineType);
//			set.saveLines("Lines.txt");

			//draw line
//			setLinesFromFile(l[lineType], "lines.txt");

			for (int i=0 ; i < count ; i++)
			{
				maxRline(set.m_secondLines[lineType], &maxR, &smaxR, i, removR);
				removR[i] = maxR;
			}

			maxRline(set.m_secondLines[lineType], &maxR, &smaxR, count, removR);

			set.m_msLine[lineType].set( set.m_secondLines[lineType][maxR] , set.m_secondLines[lineType][smaxR] ) ;
		}

		error = calCenter(set.m_msLine, ptCenter);

		count++;
	}


	// drawing result
	if (SHOW_PROC == true)
	{
		int i, x1, x2, y1, y2;

		cv::Mat canvas = cv::Mat(newRows, newCols, CV_8UC3, cv::Scalar(0));
		img.copyTo(canvas(cv::Rect(cols, rows, cols, rows)));

		for (lineType = 0; lineType <= 2; lineType++)
		{
			int lineSize = 5, kkk, color;

			if (lineType != 0)
			{
				for (i = 0; i < LINENUM1 * 2; i++)
				{
					for (x1 = -cols; x1 < xend; x1++)
					{
						set.m_secondLines[lineType][i].getY(x1, &y1, &y2);

						y1 -= lineSize / 2;
						y2 -= lineSize / 2;

						for (kkk = 0; kkk < lineSize; kkk++)
						{
							if (kkk < 2 || kkk > 6) { color = 255; }
							else { color = 255; }

							if ((y1 > -rows) && (y1 < yend))
							{
								if (i == maxR)
								{
									canvas.at<cv::Scalar>(y1 + rows, x1 + cols) = CV_RGB(255, 0, 0);
								}
								else
								{
									canvas.at<cv::Scalar>(y1 + rows, x1 + cols) = CV_RGB(255, 0, 0);
								}
							}

							if ((y2 > -rows) && (y2 < yend))
							{
								if (i == maxR)
								{
									canvas.at<cv::Scalar>(y1 + rows, x1 + cols) = CV_RGB(255, 0, 0);
								}
								else
								{
									canvas.at<cv::Scalar>(y1 + rows, x1 + cols) = CV_RGB(255, 0, 0);
								}
							}

							y1++;
							y2++;
						}
					}
				}
			}
			else
			{
				for (i = 0; i < LINENUM1 * 2; i++)
				{
					for (y1 = -rows; y1 < yend; y1++)
					{
						set.m_secondLines[lineType][i].getX(y1, &x1, &x2);
						x1 -= lineSize / 2; x2 -= lineSize / 2;
						for (kkk = 0; kkk < lineSize; kkk++)
						{
							if (kkk < 2 || kkk > 6) { color = 255; }
							else { color = 255; }

							if ((x1 > -cols) && (x1 < xend))
							{
								if (i == maxR)
								{
									canvas.at<cv::Scalar>(y1 + rows, x1 + cols) = CV_RGB(255, 0, 0);
								}
								else
								{
									canvas.at<cv::Scalar>(y1 + rows, x1 + cols) = CV_RGB(255, 0, 0);
								}
							}

							if ((x2 > -cols) && (x2 < yend))
							{
								if (i == maxR)
								{
									canvas.at<cv::Scalar>(y1 + rows, x2 + cols) = CV_RGB(255, 0, 0);
								}
								else
								{
									canvas.at<cv::Scalar>(y1 + rows, x2 + cols) = CV_RGB(255, 0, 0);
								}
							}
							x1++;
							x2++;
						}
					}
				}
			}
			lineSize = 2;//9;
			if (lineType != 0)
			{
				for (x1 = -cols; x1 < xend; x1++)
				{
					y1 = set.m_msLine[lineType].getY(x1);
					y1 -= lineSize / 2;
					for (kkk = 0; kkk < lineSize; kkk++)
					{
						if (kkk < 2 || kkk >6) { color = 0; }
						else { color = 255; }

						if ((y1 > -rows) && (y1 < yend))
						{
							canvas.at<cv::Scalar>(y1 + rows, x1 + cols) = CV_RGB(color, color, color);
						}
						y1++;
						y2++;
					}
				}
			}
			else
			{
				for (y1 = -rows; y1 < yend; y1++)
				{
					x1 = set.m_msLine[lineType].getX(y1);

					x1 -= lineSize / 2;
					x2 -= lineSize / 2;

					for (kkk = 0; kkk < lineSize; kkk++)
					{
						if (kkk < 2 || kkk > 6) { color = 0; }
						else { color = 255; }

						if ((x1 > -cols) && (x1 < xend))
						{
							bigger(x1 + nXSize, y1 + nYSize, 1) = color;
							bigger(x1 + nXSize, y1 + nYSize, 0) = color;
							bigger(x1 + nXSize, y1 + nYSize, 2) = color;
						}
						x1++;
						x2++;
					}
				}
			}
		}

		cv::imshow("finding center", bigger);
	}

	return ptCenter;
}

void findVanishPoint(const cv::Point2f& center, SetofPoints& set) 
{
	// TODO: Add your control notification handler code here
	float angle_yz, angle_xz;

	for (int i=0 ; i < 3 ; i++)
	{
		set.m_msLine[i].modify(center);
	}

	findRAngle(&angle_yz, &angle_xz, set.m_msLine);
	findVP(angle_yz, angle_xz, set.m_vp, center, set.m_msLine);
}

void findVanishPoint2(const cv::Mat& img, const cv::Point2f& center, int * angleIdx, SetofPoints& set)
{
	// TODO: Add your control notification handler code here
	cv::Mat canvas;

	double tan_v[2];
	double tan_bar1[LINENUM1], tan_bar2[LINENUM1];
	
	//���� �ҽǼ��� ã�´�.
	//���� �����ʸ鿡 ���ؼ� ���

	//T'n loading
	for (int i = 0; i < angleIdx[0]; i++)
	{
		tan_bar1[i] = set.m_tangents[0][i];
	}//tan�� vanishing line�� ����� ������ ����Ǿ� �ִ�.

	for (i = 0; i < angleIdx[2]; i++)
	{
		tan_bar1[i] = set.m_tangents[2][i];
	}//tan�� vanishing line�� ����� ������ ����Ǿ� �ִ�.

//	tan_v[0] = findTanofV2(tan_bar1,tan_bar2, atan(-1/tan_bar1[LINENUM-1]), atan(tan_bar1[0]) );
	tan_v[0] = findTanofV(tan_bar1, atan(-1 / tan_bar1[angleIdx[0] - 1]), atan(tan_bar1[0]), angleIdx[0]);
	//	tan_v[1] = findTanofV(tan_bar2, atan(-1/tan_bar2[LINENUM1-1]), atan(tan_bar2[0]) );
	tan_v[1] = findTanofV(tan_bar2, atan(tan_bar2[0]), atan(-1 / tan_bar2[angleIdx[2] - 1]), angleIdx[2]);

	//���� ���ʸ鿡 ���ؼ� ���

	//T'n loading
	//�ҽǼ��� ��� ã������ �ҽ����� ã�´�.

	for (i = 0; i < 3; i++)
	{
		set.m_msLine[i].modify(center);
	}

	//�Ʒ� �ٿ� �ִ� vp���� ã�´�.
	cv::Point3f l1 = cv::Point3f(set.m_msLine[0].coef_x, set.m_msLine[0].coef_y, set.m_msLine[0].cons);
	//�������� ������ ����
	cv::Point3f l2 = cv::Point3f(0, -1, center.y + 500);
	l2.z = 943.282;//����� ��

	cv::Point3f x = l1.cross(l2);

	set.m_vp[0].x = x.x / x.z;
	set.m_vp[0].y = x.y / x.z;


	//���� �ٿ� �ִ� vp�� ã�´�.
	l1 = cv::Point3f(set.m_msLine[1].coef_x, set.m_msLine[1].coef_y, set.m_msLine[1].cons);

	//vanishing line
	StraightLine vLine;
	vLine.coef_x = -tan_v[0];
	vLine.coef_y = 1;
	vLine.cons = 0;
	cv::Point2f vpdouble;
	vpdouble.x = set.m_vp[0].x;
	vpdouble.y = set.m_vp[0].y;
	vLine.modify(vpdouble);

	l2 = cv::Point3f(vLine.coef_x, vLine.coef_y, vLine.cons);
	x = l1.cross(l2);

	set.m_vp[1].x = x.x / x.z;
	set.m_vp[1].y = x.y / x.z;

	//������ �ٿ� �ִ� vp�� ã�´�.
	l1 = cv::Point3f(set.m_msLine[2].coef_x, set.m_msLine[2].coef_y, set.m_msLine[2].cons);

	//vanishing line
	vLine.coef_x = -tan_v[1];
	vLine.coef_y = 1;
	vLine.cons = 0;
	vLine.modify(vpdouble);

	l2 = cv::Point3f(vLine.coef_x, vLine.coef_y, vLine.cons);
	x = l1.cross(l2);

	set.m_vp[2].x = x.x / x.z;
	set.m_vp[2].y = x.y / x.z;

	// p.p. ����
	std::ofstream outfile("pp.txt");

	cv::Point2f ppoint;
	ppoint = orthocenter(set.m_vp);

	outfile << ppoint.x << "\t" << ppoint.y;
	outfile.close();

	int XSize = img.cols;
	int YSize = img.rows;
	if (SHOW_PROC == true)
	{
		drawLnVP(center, set.m_msLine, set.m_vp, img, canvas);
		cv::circle(canvas, ppoint + cv::Point2f(XSize, YSize), 5, CV_RGB(255, 0, 0));
		cv::imshow("image", canvas);
	}
}

void lineForAngle(ByteImage inputImage, cv::Point2f center, SetofPoints& set) 
{
	int lineNum = LINENUM1;
	int m_lineType2;
	

	int i, x1, y1;

	int xsize=inputImage.GetXSize();
	int ysize=inputImage.GetYSize();
	int csize=inputImage.GetCSize();


	//���������� ���� ������� ���
	for (m_lineType2=0 ; m_lineType2 <= 2 ; m_lineType2++)
	{
		//line�� �� ã��
		set.setTypeLine2(m_lineType2);

		if (SHOW_PROC == true)
		{	
			//line �׸���
			
			if (m_lineType2 == 1)
			{
				for (i=0 ; i < LINENUM1 ; i++)
				{
					for (x1=0 ; x1 < xsize ; x1++)
					{
						set.s[m_lineType2][i].getY( x1 , &y1 );

						if ( (y1 > 0) && (y1 < ysize) )
						{
							inputImage(x1,y1,1)= 255;
							inputImage(x1,y1,0)= 0;
							inputImage(x1,y1,2)= 0;
						}
					} 
				}
			}
			else
			{
				for (i=0 ; i < LINENUM1 ; i++)
				{
					for (y1=0 ; y1 < ysize ; y1++)
					{
						set.s[m_lineType2][i].getX( y1 , &x1 );

						if( (x1 > 0) && (x1 < xsize) )
						{
							inputImage(x1,y1,0)= 255;
							inputImage(x1,y1,1)= 0;
							inputImage(x1,y1,2)= 0;
						}
					}
				}
			}
		}

	}

	if (SHOW_PROC == true)
	{
		cvCircle(inputImage, center, 2, CV_RGB(255,0,0));
		ImShow(inputImage, "lines");
	}
}

void calcAngl(ByteImage inputImage, cv::Point2f center, SetofPoints& set, int * angleIdx, bool COD_pp) 
{

	int lineNum = LINENUM1;
	int m_lineType2 ; 


	if (SHOW_PROC == true)
	{
		//���͸� ȭ�鿡 ǥ��
		cvCircle(inputImage, center, 2, CV_RGB(255,0,0));
	}



	cv::Point2f nPoint[3][LINENUM1];


	for (m_lineType2=0 ; m_lineType2 <= 2 ; m_lineType2++)
	{
		set.findAngle(center,m_lineType2, nPoint[m_lineType2], &angleIdx[m_lineType2], COD_pp);
	}

	if (SHOW_PROC == true)
	{
		// ���� ����� ���� ȭ�鿡 ǥ��
		for (int j=0 ; j < angleIdx[0] ; j++)
		{
			cvCircle(inputImage, nPoint[0][j],1, CV_RGB(255,255,255));
		}

		for (j=0 ; j < angleIdx[1] ; j++)
		{
			cvCircle(inputImage, nPoint[1][j],1, CV_RGB(255,255,255));
		}
		
		for (j=0 ; j < angleIdx[2] ; j++)
		{
			cvCircle(inputImage, nPoint[2][j],1, CV_RGB(255,255,255));
		}
		
		ImShow(inputImage, "angle");
	}
}



void reconstructLines(ByteImage inputImage, const cv::Point2f& center, const vecPoint2f& arrDisPoint, vecPoint2f& arrCorPoint, SetofPoints& set)
{
	int XSize = inputImage.GetXSize();
	int YSize = inputImage.GetYSize();

	ByteImage bigger;

	int m_centerV = 2;
	int m_centerH = 4;


	int i;

	double tanA,tanT;


	if (SHOW_PROC == true)
	{
		//���͸� ȭ�鿡 ǥ��
		cvCircle(inputImage, center, 2, CV_RGB(255,0,0));
	}


	for(i =0 ; i <3 ; i++)
	{
		set.m_msLine[i].modify(center);
	}

	//���� ������ �ҽ��� �����ִ� �κ���

	

	//tan a ���
	tanA =  (double)(set.m_vp[0].y - set.m_vp[1].y) / (double)( set.m_vp[1].x - set.m_vp[0].x);


//	double rect[4][2];
	double drect[4][2];

	//�����ϸ� ����
	float minDist[4];
	cv::Point2f  minPt[4];
	
	for (i=0 ; i < 4 ; i++)
	{
		minDist[i] = FLT_MAX;
	}

	for (i=0 ; i < 3*POINTNUM_SQ ; i++)
	{
		fPoint temp = arrDisPoint[i];
		float dist;
		dist = fpDistance(temp, center);

		if ( dist < minDist[0] )
		{
			for (int j=3 ; j > 0 ; j--)
			{
				minDist[j] = minDist[j-1];
				minPt  [j] = minPt  [j-1];
			}
			minDist[0] = dist;
			minPt[0] = temp;
		}
		else if ( dist < minDist[1] )
		{
			for (int j=3 ; j > 1 ; j--)
			{
				minDist[j] = minDist[j-1];
				minPt  [j] = minPt  [j-1];
			}
			minDist[1] = dist;
			minPt  [1] = temp;
		}
		else if ( dist < minDist[2] )
		{
			minDist[3] = minDist[2] ;
			minPt  [3] = minPt[2];

			minDist[2] = dist;
			minPt  [2] = temp;
		}
		else if ( dist < minDist[3] )
		{
			minDist[3] = dist;
			minPt  [3] = temp;
		}
	}
	

	float areaOriginal = GetArea(minPt[0], minPt[1], minPt[2], minPt[3]);


	if (SHOW_PROC == true)
	{
		drawLnVP(center, set.m_msLine, set.m_vp, inputImage, bigger);
		//�����
		for (i=0 ; i < 4 ; i++)
		{
			cvCircle(bigger, minPt[i] + cv::Point2f(XSize, YSize) ,2, CV_RGB(200,200,0));
		}
	}



	//�ʹ��� ideal ���� ���Ͽ� ���� 
	for (i=0 ; i < 3 ; i++)
	{
		set.m_vp[i].x+=XSize;
		set.m_vp[i].y+=YSize;
	}
	
	arrCorPoint.RemoveAll();

	StraightLine lines[2];
	
	for (i=0 ; i < POINTNUM ; i++)
	{
		//������ ���ϱ�
		tanT = 1/(set.m_factor[0]+i*set.m_offset[0]);
		tanT = ( tanT+tanA )/ (1 - tanT*tanA);
		lines[0].set(-tanT,set.m_vp[0]);
		
		for (int j=0 ; j < POINTNUM ; j++)
		{
			//������ ���ϱ�
			tanT = 1/(set.m_factor[1]+j*set.m_offset[1]);
			tanT = ( tanA - tanT )/ (1 + tanT*tanA);
			lines[1].set(-tanT,set.m_vp[1]);
			
			intersection(lines[0], lines[1], drect[0]);

			cv::Point2f pt;
			pt.x = drect[0][0] - XSize;
			pt.y = drect[0][1] - YSize;

			arrCorPoint.Add(pt);
		}
	}



	// ideal���� �� �� ã�� 
	for (i=0 ; i < 3 ; i++)
	{
		set.m_vp[i].x-=XSize;
		set.m_vp[i].y-=YSize;
	}

	cv::Point2f minPtIdl[4];

	for (i=0 ; i < POINTNUM_SQ ; i++ )
	{
		cv::Point2f temp = arrDisPoint[i];
		cv::Point2f tempIdl;
		
		tempIdl = arrCorPoint[i];

		if ( temp == minPt[0] )
		{
			minPtIdl[0] = tempIdl ;
		}
		else if ( temp == minPt[1] )
		{
			minPtIdl[1] = tempIdl ;
		}
		else if ( temp == minPt[2] )
		{
			minPtIdl[2] = tempIdl ;
		}
		else if ( temp == minPt[3] )
		{
			minPtIdl[3] = tempIdl ;
		}
	}
	
	double areaNew = GetArea(minPtIdl[0], minPtIdl[1], minPtIdl[2], minPtIdl[3]);
	double ratio;

	ratio = sqrt(areaOriginal/areaNew);
	
	for (i=0 ; i < 3 ; i ++)
	{
		set.m_vp[i].x = (set.m_vp[i].x-center.x)*ratio+center.x ;
		set.m_vp[i].y = (set.m_vp[i].y-center.y)*ratio+center.y ;
	}


	//�׸��� �׸��� �κ�

	for (i=0 ; i < 3 ; i++)
	{
		set.m_vp[i].x += XSize;
		set.m_vp[i].y += YSize;
	}


	//�׸��� �׸��鼭 ���ÿ� ���� ���Ͽ� ���� 
	//r�� ����
//	ofstream file_r("e_corr.txt");
	arrCorPoint.RemoveAll();

	
	for (i=0 ; i < POINTNUM ; i++)
	{
		//������ ���ϱ�
		tanT = 1/(set.m_factor[0]+i*set.m_offset[0]);
		tanT = ( tanT+tanA )/ (1 - tanT*tanA);
		lines[0].set(-tanT,set.m_vp[0]);
		drawLine(bigger , lines[0], 0);

		for (int j = 0 ; j < POINTNUM ; j++)
		{
			//������ ���ϱ�
			tanT = 1/(set.m_factor[1]+j*set.m_offset[1]);
			tanT = ( tanA - tanT )/ (1 + tanT*tanA);
			lines[1].set(-tanT,set.m_vp[1]);
			drawLine(bigger , lines[1], 1);
			
			intersection( lines[0], lines[1], drect[0]);
			
			cv::Point2f pt(drect[0][0]-XSize, drect[0][1]-YSize);

			arrCorPoint.Add(pt);
			
			//r ���
			pt -= center;
			
			double corr = sqrt(pt.x*pt.x + pt.y*pt.y);
//			file_r << corr << endl;
		}
	}

//	file_r.close();
	
// �ӽ�

	if (SHOW_PROC == true)
	{
		ImShow(bigger, "reconstructed lines");
	}
	
}




void FOV(const cv::Point2f& center, const vecPoint2f& arrDisPoint, vecPoint2f& arrCorPoint, double& w, int xSize, int ySize) 
{
	w = findWofFOV3(arrDisPoint, arrCorPoint, center, 0.0001,0.004, xSize, ySize);
 	ComputeInitArrCorPointWithFOV(arrDisPoint, arrCorPoint, w, center);
}

void ComputeInitArrCorPointWithFOV(const vecPoint2f& arrDisPoint, vecPoint2f& arrCorPoint, double w, cv::Point2f center)
{
	//Computing corrected points

	double xd, yd, xc, yc;
	
	arrCorPoint.RemoveAll();

	int size = POINTNUM*POINTNUM*3 ;

	for (int i=0 ; i < size ; i++)
	{
		xd = arrDisPoint[i].x;
		yd = arrDisPoint[i].y;

		cv::Point2f pt;

		if (xd == -1)
		{
			pt.x = -1;
			pt.y = -1;
		}
		else
		{
			double r = sqrt(pow(xd-center.x,2)+pow(yd-center.y,2));

			{
				float ratio = tan(r * w) /(r*2*tan(w/2)) ;
				//float ratio=(1/(w*r))*atan(2*r*tan(w/2));

				xc= center.x +((float)xd-center.x)*ratio ;
				yc= center.y +((float)yd-center.y)*ratio ;

				pt.x = xc;
				pt.y = yc;
			}
		}

		arrCorPoint.Add(pt);
	}
}

float lineErrorMeasure(const vecPoint2f& arrCorPoint, const int xSize, const int ySize)
{

	cv::Point2f pts[POINTNUM];
	LineError measure;
	float totalError = 0, error;
	int count= 0;

	for (int i = 0 ; i < 3*POINTNUM ; i++)
	{
		for (int j = 0 ; j < POINTNUM ; j++)
		{
			pts[j] = arrCorPoint[i*POINTNUM + j];
		}

		measure.inputPt(pts);
		error = measure.measureError(xSize, ySize);


		if (error != -1)
		{
			totalError += error;
			count ++;
		}
	}


	float meanError = totalError / (float)count;

	CString line;
	line.Format("error = %f", meanError ) ;
	//AfxMessageBox(line);

	return meanError;
}


void drawArc(ByteImage &canvas, const vecPoint2f &arr, SecondOrderLine arc, CvScalar color, int thickness)
{
	int x1,x2,y1,y2;
	int XSize = canvas.GetXSize();
	int YSize = canvas.GetYSize();
	
	// ���η� ����� ���η� ����� �Ǵ�
	cv::Point2f temp = arr[arr.GetSize()-1] - arr[0] ;

	int step;

	if(abs(temp.x) - abs(temp.y) > 0)
	{

		int minY = min( min(arr[0].y, arr[1].y), min(arr[2].y, arr[3].y) ) - YSize/10;
		int maxY = max( max(arr[0].y, arr[1].y), max(arr[2].y, arr[3].y) ) + YSize/10;
		if (temp.x > 0) step = 1;
		else step = -1;
		for(x1 = arr[0].x ; x1 * step <= arr[arr.GetSize()-1].x * step ; x1+= step )
		{
			arc.getY( x1 , &y1 , &y2 );
			y1 -= thickness/2 ; y2 -= thickness/2 ; 
			for(int kkk = 0; kkk< thickness ; kkk++)
			{
				if( (y1 >= max(0, minY)) && (y1 < min(YSize, maxY)))
				{
					canvas(x1,y1,0)= color.val[0];
					canvas(x1,y1,1)= color.val[1];
					canvas(x1,y1,2)= color.val[2];
				}
				if( (y2 >= max(0, minY)) && (y2 < min(YSize, maxY)))
				{
					canvas(x1,y2,0)= color.val[0];
					canvas(x1,y2,1)= color.val[1];
					canvas(x1,y2,2)= color.val[2];
				}
				y1 ++; y2++;
			}
		}
	}
	else
	{
		int minX = min( min(arr[0].x, arr[1].x), min(arr[2].x, arr[3].x) ) - XSize/10;
		int maxX = max( max(arr[0].x, arr[1].x), max(arr[2].x, arr[3].x) ) + XSize/10;
		if (temp.y > 0) step = 1;
		else step = -1;
		for(y1 = arr[0].y ; y1 * step <= arr[arr.GetSize()-1].y * step ; y1+= step )
		{
			arc.getX( y1 , &x1 , &x2 );
			x1 -= thickness/2 ; x2 -= thickness/2 ; 
			for(int kkk = 0; kkk< thickness ; kkk++)
			{
				if( (x1 >= max(0, minX)) && (x1 < min(XSize, maxX)))
				{
					canvas(x1,y1,0)= color.val[0];
					canvas(x1,y1,1)= color.val[1];
					canvas(x1,y1,2)= color.val[2];
				}
				if( (x2 >= max(0, minX)) && (x2 < min(XSize, maxX)))
				{
					canvas(x2,y1,0)= color.val[0];
					canvas(x2,y1,1)= color.val[1];
					canvas(x2,y1,2)= color.val[2];
				}
				x1 ++; x2++;
			}
		}
	}
}



float distanceFromConic(cv::Point2f pt, SecondOrderLine arc)
{
	DoubleMatrix matX(3), matC(3,3);
	matX(0) = pt.x;
	matX(1) = pt.y;
	matX(2) = 1;

	matC(0,0) = arc.coef_xx;
	matC(1,1) = arc.coef_yy;
	matC(2,2) = arc.cons;

	matC(0,2) = matC(2,0) = arc.coef_x / 2 ;
	matC(1,2) = matC(2,1) = arc.coef_y / 2 ;
	matC(0,1) = matC(1,0) = arc.coef_xy / 2 ;


	return (float)( matX.Transpose() * matC * matX )(0,0) ;
}



float fpDistance(cv::Point2f p1, cv::Point2f p2)
{
	return sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
}

float lpDistance(const cv::Point3f& l, const cv::Point2f& p)
{
	line2(0,0) = -line(1,0);
	line2(1,0) = line(0,0);

	cv::Point3f l_;
	l_.x = -l.y;
	l_.y = l.x;
	l_.z = -(l_.x * p.x + l_.y * p.y);

	cv::Point3f p_ = l.cross(l_);
	cv::Point2f pd;
	pd.x = p_.x / p_.z;
	pd.y = p_.y / p_.z;

	return cv::norm(p - pd);
}

void TakeRowsFrom(DoubleMatrix*mat,DoubleMatrix temp)
{
	int row = (temp).GetRowSize();
	int col = (temp).GetColSize();
	for(int i=0 ;i<row ; i++)
		for(int j = 0  ; j<col ; j++)
		{
			(*mat)(i,j)=temp(i,j);
		}
}
void TakeColsFrom(DoubleMatrix*mat,DoubleMatrix temp, int start)
{
	int row = (temp).GetRowSize();
	int col = (temp).GetColSize();
	for(int i=0 ;i<row ; i++)
		for(int j = 0  ; j<col ; j++)
		{
			(*mat)(i,start+j)=temp(i,j);
		}
}


DoubleMatrix nullVecter(DoubleMatrix mat)
{
	DoubleMatrix vecter, U, W, V;
	
	int row = mat.GetRowSize();
	int col = mat.GetColSize();
	vecter.Allocate(row);
	if(row < col)
	{
		DoubleMatrix temp = mat;
		(mat).Allocate(col,col);
		TakeRowsFrom(&mat, temp);
		row = col;
	}
	
	SVD(mat , U , W , V);

	int bestcolumn;
	double minvalue = DBL_MAX ;

	for (int k=0 ; k < col ; k++)
	{
		double value = W(k ,k);

		if (value < minvalue)
		{
			bestcolumn =   k ;
			minvalue = value ;
		}
	}
	if (minvalue == 0)
		vecter = V.GetColMatrix(bestcolumn ,1);
	
	return vecter;
}

cv::Point2f orthocenter(cv::Point2f* vertex)
{
	DoubleMatrix matA(3,2), matH(2), matb(3);
	DoubleMatrix vecA(2), vecB(2), vecC(2);
	
	vecA(0) = vertex[0].x;
	vecA(1) = vertex[0].y;
	vecB(0) = vertex[1].x;
	vecB(1) = vertex[1].y;
	vecC(0) = vertex[2].x;
	vecC(1) = vertex[2].y;


	matA.SetRowMatrix((vecB - vecC).Transpose(),0);
	matA.SetRowMatrix((vecA - vecC).Transpose(),1);
	matA.SetRowMatrix((vecA - vecB).Transpose(),2);
	
	matb.SetRowMatrix( (vecB - vecC).Transpose() * vecA , 0);
	matb.SetRowMatrix( (vecA - vecC).Transpose() * vecB , 1);
	matb.SetRowMatrix( (vecA - vecB).Transpose() * vecC , 2);

	matH = ( matA.Transpose() * matA ).Inverse() * matA.Transpose() * matb;
	
	cv::Point2f ocenter;
	ocenter.x = matH(0);
	ocenter.y = matH(1);

	return ocenter;

}

bool isDiagonal(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& otherPt1, const cv::Point2f& otherPt2)
{
	cv::Point3f p1 = cv::Point3f(pt1.x, pt1.y, 1);
	cv::Point3f p2 = cv::Point3f(pt2.x, pt2.y, 1);

	cv::Point3f p1_other = cv::Point3f(otherPt1.x, otherPt1.y, 1);
	cv::Point3f p2_other = cv::Point3f(otherPt2.x, otherPt2.y, 1);

	cv::Point3f l = p1.cross(p2);

	float sideness1, sideness2;
	sideness1 = l.dot(p1_other);
	sideness2 = l.dot(p2_other);

	if (sideness1*sideness2 < 0)
		return true;
	else if (sideness1*sideness2 > 0)
		return false;
	else if ((otherPt1.x - pt2.x) * (pt2.x - pt1.x) < 0) // collinear
		return true;
	else
		return false;
}

//Get Area
float GetArea(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d)
{
	cv::Point2f temp;
	if (isDiagonal(a, b, c, d) == true)
	{
		temp = b;
		b = c;
		c = temp;
	}
	else if (isDiagonal(b, c, d, a) == true)
	{
		temp = c;
		c = d;
		d = temp;
	}
	return (float)(abs(a.x * b.y + b.x * c.y
		+ c.x * d.y + d.x * a.y
		- b.x * a.y - c.x * b.y
		- d.x * c.y - a.x * d.y)) / 2.0;
}

//Get Area
double GetArea(double* point1, double* point2, double* point3, double* point4)
{
	return (abs(point1[0] * point2[1] + point2[0] * point4[1]
		+ point4[0] * point3[1] + point3[0] * point1[1]
		- point2[0] * point1[1] - point4[0] * point2[1]
		- point3[0] * point4[1] - point1[0] * point3[1])) / 2.0;
}

//get4area
void Get4Area(float x, float y, float*a, float*b, float*c, float*d)
{
	*a = (x - (float)((int)x)) * (y - (float)((int)y));
	*b = (1 + (float)((int)x) - x)*(y - (float)((int)y));
	*c = (x - (float)((int)x))*(1 + (float)((int)y) - y);
	*d = (1 + (float)((int)x) - x)*(1 + (float)((int)y) - y);

}

//set lines from a file.
void setLinesFromFile(SecondOrderLine*l , const char *filename)
{
	double xx , yy , xy , x , y , con;
	std::ifstream inputfile(filename);

	for(int i=0 ; i<LINENUM1*2 ; i++)
	{
		inputfile >> xx >> yy >> xy >> x >> y >> con;
		l[i].set(xx, yy, xy, x, y, con);
	}
	
	inputfile.close();

}

//set lines from a file.
void setLinesFromFile(ThirdOrderLine*s , const char *filename)
{
	/*
	double x1, x2, x3, con;
		
	ifstream inputfile(filename);
	
	

	for(int i=0 ; i<LINENUM1 ; i++)
	{
		inputfile >> x1 >> x2 >> x3 >>  con;
		s[i].set(x1, x2, x3, con);
	}
	
	inputfile.close();
//*/
}

void maxRline(SecondOrderLine*l, int *first, int *second, int count, int*removR)
{
	float max, temp, temp2;
	max = 0;
	*first = -1;
	*second = -1;
	bool skipFlag;
	for (int i = 0; i < LINENUM1; i++)
	{
		skipFlag = false;
		for (int j = 0; j < count; j++)
		{
			if (i == removR[j])
				skipFlag = true;
		}
		if (skipFlag == false)
		{
			temp = ((pow(l[i].coef_x, 2) + pow(l[i].coef_y, 2))
				/ (2 * pow(l[i].coef_xx, 2))
				- l[i].cons / l[i].coef_xx);
			if (temp > max)
			{
				temp2 = max;
				max = temp;
				*second = *first;
				*first = i;
			}
			else if (temp > temp2)
			{
				temp2 = temp;
				*second = i;
			}
		}
	}
}

void findVanishingPoint(SecondOrderLine*l,cv::Point2f*van)
{

}



float calCenter(StraightLine *three, cv::Point2f& ptCenter)
{
	DoubleMatrix matA(3,2);
	DoubleMatrix matx(2);
	DoubleMatrix matb(3);

	for (int i=0 ; i < 3 ; i++)
	{
		matA(i, 0) =  three[i].coef_x;
		matA(i, 1) =  three[i].coef_y;
		matb(i, 0) = -three[i].cons;
	}


	matx=(matA.Transpose()*matA).Inverse()*matA.Transpose()*matb;

	//matA.SaveToText("matA.txt");
	//matb.SaveToText("matb.txt");
	//matx.SaveToText("matx.txt");

	ptCenter.x = matx(0);
	ptCenter.y = matx(1);

	
	float error = 0;

	DoubleMatrix point(3);
	point(0) = matx(0);
	point(1) = matx(1);
	point(2) = 1;

	for(i = 0 ; i < 3 ; i++)
	{
		DoubleMatrix line(3);
		// line = (matA.GetRowMatrix(i,1)).Transpose();
		line(0) = three[i].coef_x;
		line(1) = three[i].coef_y;
		line(2) = three[i].cons;
		error += lpDistance(line,point);
	}

	return error;
}

float findTan(StraightLine line1, StraightLine line2)
{
	float angle1, angle2;
	
	angle1 = atan(-line1.coef_x/line1.coef_y);
	angle2 = atan(-line2.coef_x/line2.coef_y);
	
	return tan(angle2-angle1);

}

void findRAngle(float *angle_yz, float *angle_xz, StraightLine*msLine)
{
	float tanA, tanB;
	float x;

	tanA = findTan(msLine[0],msLine[1]);
	tanB = findTan(msLine[0],msLine[2]);
	
	x = sqrt( 1/(1-tanA/tanB) );
	*angle_yz = asin(x);

	x=1/(sin(*angle_yz)*cos(*angle_yz)*(tanB-tanA));
	*angle_xz = asin(x);
}

void findVP(float angle1, float angle2, cv::Point2f* vp, cv::Point2f center, StraightLine*msLine)
{
	float ratio[3];
	float tempx, tempy, tempm;

	ratio[0] = fabs( cos(angle2)/sin(angle2) );

	tempx = - sin(angle2)/cos(angle2);
	tempy = cos(angle1)/(cos(angle2)*sin(angle1));
	ratio[1] = sqrt( pow(tempx,2) + pow(tempy,2) );

	tempx = - sin(angle2)/cos(angle2);
	tempy = - sin(angle1)/(cos(angle1)*cos(angle2));
	ratio[2] = sqrt( pow(tempx,2) + pow(tempy,2) );

	for(int i = 0 ; i <3 ; i++ )
	{
		tempm= sqrt( pow(msLine[i].coef_y,2) + pow(msLine[i].coef_x,2) );
		if(i==2)
		{
			vp[i].x= center.x - (  msLine[i].coef_y * ratio[i] / tempm )*300;
			vp[i].y= center.y - ( -msLine[i].coef_x * ratio[i] / tempm )*300;
		}
		else
		{
			vp[i].x= center.x + (  msLine[i].coef_y * ratio[i] / tempm )*300;
			vp[i].y= center.y + ( -msLine[i].coef_x * ratio[i] / tempm )*300;
		}
	}

}


void drawLnVP(const cv::Point2f& center, StraightLine *msLine, cv::Point2f* vp, const cv::Mat& img, cv::Mat& canvas)
{
	int cols = image.cols;
	int rows = image.rows;

	int newCols = 3 * cols;
	int newRows = 3 * rows;

	int xend = 2 * xsize;
	int yend = 2 * ysize;

	canvas = cv::Mat(newRows, newCols, CV_8UC3, cv::Scalar(0));
	img.copyTo(canvas(cv::Rect(cols, rows, cols, rows)));

	cv::Point2f temp1, temp2;
	temp1.y = center.y;
	temp2.y = yend;
	temp1.x = msLine[0].getX(temp1.y);
	temp2.x = msLine[0].getX(temp2.y);
	cv::line(canvas, temp1 + cv::Point2f(xsize, ysize), temp2 + cv::Point2f(xsize, ysize), CV_RGB(255, 255, 255), 5);

	temp1.x = center.x;
	temp2.x = -xsize;
	temp1.y = msLine[1].getY(temp1.x);
	temp2.y = msLine[1].getY(temp2.x);
	cv::line(canvas, temp1 + cv::Point2f(xsize, ysize), temp2 + cv::Point2f(xsize, ysize), CV_RGB(255, 255, 255), 5);

	temp1.x = center.x;
	temp2.x = xend;
	temp1.y = msLine[2].getY(temp1.x);
	temp2.y = msLine[2].getY(temp2.x);
	cv::line(canvas, temp1 + cv::Point2f(xsize, ysize), temp2 + cv::Point2f(xsize, ysize), CV_RGB(255, 255, 255), 5);

	int a = vp[0].x;
	a = vp[0].y;
	a = vp[1].x;
	a = vp[1].y;
	a = vp[2].x;
	a = vp[3].y;


	cv::line(canvas, vp[0] + cv::Point2f(xsize, ysize), vp[1] + cv::Point2f(xsize, ysize), CV_RGB(255, 0, 0), 5);
	cv::line(canvas, vp[1] + cv::Point2f(xsize, ysize), vp[2] + cv::Point2f(xsize, ysize), CV_RGB(255, 0, 0), 5);
	cv::line(canvas, vp[0] + cv::Point2f(xsize, ysize), vp[2] + cv::Point2f(xsize, ysize), CV_RGB(255, 0, 0), 5);
}
/*
int sort(CPoint* point, int num, int type)
{
	if(num == 0 )
		return 0;


	CPoint temp;
	temp=point[0];
	int min;
	min = 0 ;
	
	if(type == 0 )
	{
		for(int i = 1 ; i < num ; i++)
		{
			if( temp.y > point[i].y)
			{
				min = i;
				temp = point[i];
			}
		}
		temp = point[0];
		point[0] = point[min];
		point[min] = temp;
	}
	else
	{
		for(int i = 1 ; i < num ; i++)
		{
			if( temp.x > point[i].x)
			{
				min = i;
				temp = point[i];
			}
		}
		temp = point[0];
		point[0] = point[min];
		point[min] = temp;
	}

	sort( &point[1], num-1, type);

	return 0;
}
//*/
int sort(cv::Point2f* point, int num, int type)
{
	if(num == 0 )
		return 0;


	cv::Point2f temp;
	temp=point[0];
	int min;
	min = 0 ;
	
	if(type == 0 )
	{
		for(int i = 1 ; i < num ; i++)
		{
			if( temp.y > point[i].y)
			{
				min = i;
				temp = point[i];
			}
		}
		temp = point[0];
		point[0] = point[min];
		point[min] = temp;
	}
	else
	{
		for(int i = 1 ; i < num ; i++)
		{
			if( temp.x > point[i].x)
			{
				min = i;
				temp = point[i];
			}
		}
		temp = point[0];
		point[0] = point[min];
		point[min] = temp;
	}

	sort( &point[1], num-1, type);

	return 0;
}

int sort(double * val, int num)
{
	if (num == 0)
	{
		return 0;
	}

	double temp;
	temp=val[0];
	int min;
	min = 0 ;
	
	for (int i=1 ; i < num ; i++)
	{
		if (temp > val[i])
		{
			min = i;
			temp = val[i];
		}
	}

	temp = val[0];
	val[0] = val[min];
	val[min] = temp;

	sort( &val[1], num-1);

	return 0;
}

double distance(CPoint a, CPoint b)
{
	return sqrt( pow(a.x-b.x,2)+pow(a.y-b.y,2) );
}
double distance(double ax, double ay, double bx, double by)
{
	return sqrt( pow(ax-bx,2)+pow(ay-by,2) );
}



double findNearestPoint(ThirdOrderLine line ,double ax, double ay, double bx, double by, cv::Point2f center, int type)
{
	if( (type == 0 || type ==2) && (ay-by < 0.0001 ) && (ay-by > -0.0001 ) ) //  ���μ�
		return ( ay + by ) / 2;
	else if( type == 1 && (ax-bx < 0.0001 ) && (ax-bx > -0.0001 ) )
		return ( ax + bx ) / 2;

	double mx, my ;

	if( type == 0 || type ==2) //  ���μ�
	{
		my = ( ay + by ) / 2 ;

		line.getX( my, &mx ) ; 

		if ( distance(ax,ay, center.x, center.y) < distance(bx, by, center.x, center.y) )
		{
			return findNearestPoint(line, ax, ay, mx, my, center, type);
		}
		else
		{
			return findNearestPoint(line, mx, my, bx, by, center, type);
		}
	}
	else
	{
		mx = ( ax + bx ) / 2 ;
		line.getY( mx, &my ) ; 

		if ( distance(ax,ay, center.x, center.y) < distance(bx, by, center.x, center.y) )
		{
			return findNearestPoint(line, ax, ay, mx, my, center, type);
		}
		else
		{
			return findNearestPoint(line, mx, my, bx, by, center, type);
		}
	}




}

void drawLine(cv::Mat& canvas, StraightLine& line, int type)
{
	int x1, y1;
	int xsize = canvas.cols;
	int ysize = canvas.rows;

	if (type == 0)
	{
		for (y1 = 0; y1 < ysize; y1++)
		{
			x1 = line.getX(y1);

			if ((x1 >= 0) && (x1 < xsize))
			{
				canvas.at<cv::Scalar>(y1, x1) = CV_RGB(255, 0, 0);
			}
		}
	}
	else
	{
		for (x1 = 0; x1 < xsize; x1++)
		{
			y1 = line.getY(x1);

			if ((y1 >= 0) && (y1 < ysize))
			{
				canvas.at<cv::Scalar>(y1, x1) = CV_RGB(255, 0, 0);
			}
		}
	}
}





void intersection(StraightLine line1, StraightLine line2, double* point)
{

	double temp = ( line1.coef_x * line2.coef_y - line1.coef_y * line2.coef_x);
	point[0] = ( line1.coef_y * line2.cons - line1.cons * line2.coef_y ) / temp;
	point[1] = ( line1.coef_x * line2.cons - line1.cons * line2.coef_x ) / (-temp);

}




double findWofFOV3(const vecPoint2f& arrDisPoint, const vecPoint2f& arrCorPoint, const cv::Point2f center, double left, double right, int xSize, int ySize)
{
	
	if(right - left < 0.000001)
	{
		return (left+right)/2;
	}
	else
	{
		double first, second;
		double firstError, secondError;


		first = left + (right-left)*2/5;
		second = right - (right-left)*2/5;
		
		firstError  = errorW(arrDisPoint, arrCorPoint, center, first, xSize, ySize);
		secondError = errorW(arrDisPoint, arrCorPoint, center, second, xSize, ySize);

		if(firstError > secondError)
		{
			return findWofFOV3(arrDisPoint, arrCorPoint, center, first, right, xSize, ySize);
		}
		else
		{
			return findWofFOV3(arrDisPoint, arrCorPoint, center, left, second, xSize, ySize);
		}
	}
}


double errorW(const vecPoint2f& arrDisPoint, const vecPoint2f& arrCorPoint, const cv::Point2f center, double w, int xSize, int ySize)
{
	double oldX, oldY, oldR , ratio ;
	double newX, newY, disX, disY;

	int count = 0;
	double newerror = 0;

	for (int i=0 ; i < POINTNUM*POINTNUM ; i++)
	{
		float ptx = arrDisPoint[i].x;
		float pty = arrDisPoint[i].y;
		disX = (double)ptx;
		disY = (double)pty;
		
		ptx = arrCorPoint[i].x;
		pty = arrCorPoint[i].y;

		oldX = (double)ptx;
		oldY = (double)pty;

		if (oldX != -1)
		{
			if (oldX > 0 && oldX < xSize && oldY > 0 && oldY < ySize)
			{
				oldR = sqrt( pow(oldX-center.x,2) + pow(oldY-center.y,2) );

				ratio = ((1.0/w) * atan( 2.0* oldR * tan(w/2.0) ) )/oldR;
			
				newX = (oldX-center.x)*ratio+center.x;
				newY = (oldY-center.y)*ratio+center.y;

				newerror += distance(disX,disY,newX,newY);
				count++;

			}
		}
	}
	
	newerror /= count ;

	return newerror;
}

double findTanofV(double*tan_bar, double left, double right, int num)
{
	if(left< 0)	left += PI;
	if(right< 0)	right += PI;
	if(abs(right - left) < 0.000000000001)
	{
		return tan((left+right)/2);
	}
	else
	{
		double first, second;
		double firstError, secondError;


		first = left + (right-left)*2/5;
		second = right - (right-left)*2/5;
		
		firstError = errorTanofV(tan_bar, first, num);
		secondError = errorTanofV(tan_bar, second, num);

		if(firstError > secondError)
		{
			return findTanofV(tan_bar, first, right, num);
		}
		else
		{
			return findTanofV(tan_bar, left, second, num);
		}
	}
}

double errorTanofV(double*tan_bar, double theta_v, int num)
{
	double tan_v = tan(theta_v);
	double be[LINENUM1];
	//���� ������ ���� �� ���
	for(int i = 0 ; i < num ; i++)
	{
		be[i] = (1 + tan_v*tan_bar[i]) / ( tan_bar[i] - tan_v );
	}


	//����
//	sort(be,LINENUM);

	//���� ����
	DoubleMatrix matA(num ,2);
	DoubleMatrix matx(2 ,1);
	DoubleMatrix matb(num ,1);

	
	//a�� d�� ã�´�.
	for(i=0 ; i<num ; i++)
	{
		matA(i,0) = 1;
		matA(i,1) = i;
		matb(i,0) = be[i];
	}


	matx = (matA.Transpose()*matA).Inverse()*matA.Transpose()*matb;
//	DoubleMatrix errorM = (matA*matx - matb).Transpose() *(matA*matx - matb);
//	TRACE_MAT(errorM);





//	double error = ((matA*matx - matb).Transpose() *(matA*matx - matb))(0,0);


	double error;
	error = 0 ;
	DoubleMatrix matS = matA * matx ;


	for(i =0 ; i<num ; i++)
	{
//		error += pow((1/matS(i)) - (1/matb(i)),2);
		error += pow(atan(1/matS(i)) - atan(1/matb(i)),2);
	}


	return error;
}

//////////////////////////////
//	Class SetofPoints		//
//////////////////////////////

SetofPoints::SetofPoints()
{
}

SetofPoints::SetofPoints(const int lineNum, const int pointNum, const vecPoint2f& arrPoint)
{
	cv::Point2f tmpPoint;

	int num = 0;
	int count = 0;

	for (int i = 0; i < pointNum*pointNum; i++)
	{
		tmpPoint = arrPoint[i];

		int val1 = count / pointNum;
		int val2 = count%pointNum;

		if (tmpPoint.x == -1)
		{
			a[0][val1][val2] = a[0][val1][val2 - 1];
			a[1][val2][val1] = a[1][val2][val1 - 1];
		}
		else
		{
			a[0][val1][val2].x = tmpPoint.x;
			a[0][val1][val2].y = tmpPoint.y;
			a[1][val2][val1].x = tmpPoint.x;
			a[1][val2][val1].y = tmpPoint.y;
		}
		count++;
	}

	count = 0;

	for (; i < 2 * pointNum*pointNum; i++)
	{
		tmpPoint = arrPoint[i];

		int val1 = count / pointNum;
		int val2 = count%pointNum;

		if (tmpPoint.x == -1)
		{
			a[0][val2 + LINENUM1][val1] = a[0][val2 + LINENUM1][val1 - 1];
			a[2][val1][val2] = a[2][val1][val2 - 1];
		}
		else
		{
			a[0][val2 + LINENUM1][val1].x = tmpPoint.x;
			a[0][val2 + LINENUM1][val1].y = tmpPoint.y;
			a[2][val1][val2].x = tmpPoint.x;
			a[2][val1][val2].y = tmpPoint.y;
		}

		count++;
	}

	count = 0;

	for (; i < 3 * pointNum*pointNum; i++)
	{
		tmpPoint = arrPoint[i];

		int val1 = count / pointNum;
		int val2 = count%pointNum;

		if (tmpPoint.x == -1)
		{
			a[1][val2 + LINENUM1][val1] = a[1][val2 + LINENUM1][val1 - 1];
			a[2][val1 + LINENUM1][val2] = a[2][val1 + LINENUM1][val2 - 1];
		}
		else
		{
			a[1][val2 + LINENUM1][val1].x = tmpPoint.x;
			a[1][val2 + LINENUM1][val1].y = tmpPoint.y;
			a[2][val1 + LINENUM1][val2].x = tmpPoint.x;
			a[2][val1 + LINENUM1][val2].y = tmpPoint.y;
		}

		count++;
	}
}

void SetofPoints::setTypeLine(int type)
{
	int lineNumber[3];
	lineNumber[0] = LINE0NUM;
	lineNumber[1] = LINE1NUM;
	lineNumber[2] = LINE2NUM;

	cv::Mat matA = cv::Mat(lineNumber[type], 4, CV_64F, cv::Scalar(0));
	cv::Mat matv = cv::Mat(4, 1, CV_64F);
	cv::Mat matx = cv::Mat(6, 1, CV_64F);
	cv::Mat matT = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
	cv::Mat point = cv::Mat(3, 1, CV_64F);

	for (int i = 0; i < LINENUM1 * 2; i++)
	{
		// normalize
		int count = 0;
		cv::Point2f src_center = cv::Point2f(0, 0);

		for (int j = 0; j < lineNumber[type]; j++)
		{
			if (a[type][i][j].x != -1)
			{
				src_center += a[type][i][j];
				count++;
			}
		}

		src_center /= count;

		//point normalization
		double src_dist = 0;
		for (int j = 0; j < lineNumber[type]; j++)
		{
			if (a[type][i][j].x != -1)
			{
				src_dist += cv::norm(src_center - a[type][i][j]);
			}
		}
		src_dist /= count;

		double s = sqrt(2) / src_dist;

		matT.at<double>(0, 0) = s;
		matT.at<double>(1, 1) = s;
		matT.at<double>(2, 2) = 1;
		matT.at<double>(0, 2) = -s * src_center.x;
		matT.at<double>(1, 2) = -s * src_center.y;

		// a process of finding a line
		for (j = 0; j < lineNumber[type]; j++)
		{
			if (a[type][i][j].x != -1)
			{
				point.at<double>(0, 0) = a[type][i][j].x;
				point.at<double>(1, 0) = a[type][i][j].y;
				point.at<double>(2, 0) = 1;
				point = matT * point;

				matA.at<double>(j, 0) = pow(point.at<double>(0, 0), 2) + pow(point.at<double>(1, 0), 2);
				matA.at<double>(j, 1) = point.at<double>(0, 0);
				matA.at<double>(j, 2) = point.at<double>(1, 0);
				matA.at<double>(j, 3) = 1;
			}
		}

		cv::Mat e, U, V;
		cv::SVDecomp(matA, e, U, V, cv::SVD::FULL_UV);

		int bestcolumn;
		double minvalue = DBL_MAX;

		for (int k = 0; k < 4; k++)
		{
			double value = W.at<double>(k, k);

			if (value < minvalue)
			{
				bestcolumn = k;
				minvalue = value;
			}
		}

		cv::Mat matC = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
		matC.at<double>(0, 0) = V.at<double>(0, bestcolumn);
		matC.at<double>(1, 1) = V.at<double>(0, bestcolumn);
		matC.at<double>(2, 2) = V.at<double>(3, bestcolumn);
		matC.at<double>(0, 2) = V.at<double>(1, bestcolumn) / 2;
		matC.at<double>(2, 0) = V.at<double>(1, bestcolumn) / 2;
		matC.at<double>(1, 2) = V.at<double>(2, bestcolumn) / 2;
		matC.at<double>(2, 1) = V.at<double>(2, bestcolumn) / 2;

		matC = (matT.t().dot(matC)).dot(matT);
		//matC = matT.Transpose() * matC * matT;

		matx.at<double>(0, 0) = matC.at<double>(0, 0);
		matx.at<double>(3, 0) = matC.at<double>(0, 2) * 2;
		matx.at<double>(4, 0) = matC.at<double>(1, 2) * 2;
		matx.at<double>(5, 0) = matC.at<double>(2, 2);

		//		matx = (matA.Transpose()*matA).Inverse()*matA.Transpose()*matb;
		m_secondLines[type][i].set(matx);
	}
}

void SetofPoints::setTypeLine2(int type)
{
	cv::Mat matA = cv::Mat(POINTNUM, 4, CV_64F, cv::Scalar(0));
	cv::Mat matx = cv::Mat(4, 1, CV_64F);
	cv::Mat matb = cv::Mat(POINTNUM, 1, CV_64F);

	cv::Mat U, W, V;
	bool bSingular;

	for (int i = 0; i < LINENUM1; i++)
	{
		bSingular = true;
		for (int j = 0; j < POINTNUM; j++)
		{
			if (type == 0)
			{
				matA.at<double>(j, 0) = 1;
				matA.at<double>(j, 1) = a[type][i][j].y;
				matA.at<double>(j, 2) = pow(a[type][i][j].y, 2);
				matA.at<double>(j, 3) = pow(a[type][i][j].y, 3);
				matb.at<double>(j, 0) = a[type][i][j].x;
				if (a[type][i][0].y != a[type][i][j].y)
					bSingular = false;
			}
			else if (type == 1)
			{
				matA.at<double>(j, 0) = 1;
				matA.at<double>(j, 1) = a[type][i][j].x;
				matA.at<double>(j, 2) = pow(a[type][i][j].x, 2);
				matA.at<double>(j, 3) = pow(a[type][i][j].x, 3);
				matb.at<double>(j, 0) = a[type][i][j].y;
				if (a[type][i][0].x != a[type][i][j].x)
					bSingular = false;
			}
			else
			{
				matA.at<double>(j, 0) = 1;
				matA.at<double>(j, 1) = a[0][LINENUM1 + i][j].y;
				matA.at<double>(j, 2) = pow(a[0][LINENUM1 + i][j].y, 2);
				matA.at<double>(j, 3) = pow(a[0][LINENUM1 + i][j].y, 3);
				matb.at<double>(j, 0) = a[0][LINENUM1 + i][j].x;
				if (a[0][LINENUM1 + i][0].y != a[0][LINENUM1 + i][j].y)
					bSingular = false;
			}
		}

		if (bSingular == false)
		{
			matx = (matA.t().dot(matA)).inv().dot(matA.t()).dot(matb);
			//matx = (matA.Transpose()*matA).Inverse()*matA.Transpose()*matb;
		}

		s[type][i].set(matx);
	}
}

void SetofPoints::sortPoint(int type)
{
	for(int line = 0 ; line < LINENUM1 ; line++)
	{
		sort(a[type][line], 10, type);
	}
}

void SetofPoints::findAngle(const cv::Point2f& center, int originalType, cv::Point2f * nPoint, int* indexNum, bool COD_pp)
{
	int		i;
	double	nearest;
	float	tangent[LINENUM1];

	// find nearest 2 points
	float minDist;
	float temp;
	int min1, min2;
	double dtemp;

	int linestart = 0;
	*indexNum = 0;

	bool startFlag = false, endFlag = false;

	float th = 1;

	int type = originalType;

	if (originalType == 2)
		type = 0;

	for (int originalLine = 0; originalLine < LINENUM1; originalLine++)
	{
		int line = originalLine;
		if (originalType == 2)
			line = originalLine + LINENUM1;

		minDist = fpDistance(a[type][line][0], center);
		min1 = 0;

		// find the nearest point
		for (int i = 1; i < LINE0NUM; i++)
		{
			temp = fpDistance(a[type][line][i], center);
			if (minDist > temp)
			{
				min1 = i;
				minDist = temp;
			}
		}

		// find the 2nd nearest point
		if (min1 == 0) {
			min2 = 1;
		} 
		else if (min1 == LINE0NUM - 1) {
			min2 = LINE0NUM - 2;
		}
		else if (cv::norm(a[type][line][min1 - 1] - center) < cv::norm(a[type][line][min1 + 1] - center)) {
			min2 = min1 - 1;
		}
		else {
			min2 = min1 + 1;
		}

		// find the nearest point again
		nearest = findNearestPoint(s[originalType][originalLine], a[type][line][min1].x, a[type][line][min1].y, a[type][line][min2].x, a[type][line][min2].y, center, originalType);

		float temp = a[type][line][0].x;
		temp = a[type][line][9].y;

		if (originalType == 0 || originalType == 2)
		{
			if (abs(a[type][line][0].y - nearest) > th && abs(a[type][line][19].y - nearest) > th)
			{
				if (startFlag != true)
				{
					s[originalType][originalLine].getX(nearest, &dtemp);

					nPoint[*indexNum].x = (float)dtemp;
					nPoint[*indexNum].y = (float)nearest;
					tangent[*indexNum] = s[originalType][originalLine].slope(nearest, originalType);
					startFlag = true;
					linestart = line;
					(*indexNum)++;
				}
				else if (endFlag != true)
				{
					s[originalType][originalLine].getX(nearest, &dtemp);

					nPoint[*indexNum].x = (float)dtemp;
					nPoint[*indexNum].y = (float)nearest;
					tangent[*indexNum] = s[originalType][originalLine].slope(nearest, originalType);
					(*indexNum)++;
				}
				else if (startFlag == true && endFlag != true)
				{
					endFlag = true;
					*indexNum = line - linestart;
				}

			}
		}
		else
		{
			if (abs(a[type][line][0].x - nearest) > th && abs(a[type][line][19].x - nearest) > th)
			{
				if (startFlag != true)
				{
					s[originalType][originalLine].getY(nearest, &dtemp);

					nPoint[*indexNum].x = (float)nearest;
					nPoint[*indexNum].y = (float)dtemp;
					tangent[*indexNum] = s[originalType][originalLine].slope(nearest, originalType);
					startFlag = true;
					linestart = line;
					(*indexNum)++;
				}
				else if (endFlag != true)
				{
					s[originalType][originalLine].getY(nearest, &dtemp);

					nPoint[*indexNum].x = (float)nearest;
					nPoint[*indexNum].y = (float)dtemp;
					tangent[*indexNum] = s[originalType][originalLine].slope(nearest, originalType);
					(*indexNum)++;
				}
				else if (startFlag == true && endFlag != true)
				{
					endFlag = true;
					*indexNum = line - linestart;
				}
			}
		}
	}

	if (originalType == 0)
	{
		for (int i = 0; i < *indexNum; i++)
		{
			m_tangents[0][i] = tangent[i];
		}
	}
	else if (originalType == 1)
	{
		for (int i = 0; i < *indexNum; i++)
		{
			m_tangents[1][i] = tangent[i];
		}
	}
	else //if (originalType == 2)
	{
		for (int i = 0; i < *indexNum; i++)
		{
			m_tangents[2][i] = tangent[i];
		}
	}

	if (COD_pp == true)
	{
		//���� ���� ���� �� ����

		//�̶� ��� tan���� -�� �ٿ��־� ���� �츮�� ���� ��ǥ��� ���ߵ��� �Ѵ�.

		double be[LINENUM1];

		double tanA, tanX[LINENUM1];

		double a[3], d[3];


		//compute tan a
		tanA = (m_vp[0].y - m_vp[1].y) / (m_vp[1].x - m_vp[0].x);


		if (originalType == 0)
		{
			if (*indexNum < 2)
			{
				/*
				outfile.open("adad.txt");
				outfile << -2.4 << endl << 0.1 << endl ;
				outfile.close();
				//*/
				m_factor[0] = -2.4;
				m_offset[0] = 0.1;
			}
			else
			{
				//���� ������ tan �� �ε�
				for (i = 0; i < *indexNum; i++)
				{
					tanX[i] = -m_tangents[0][i];
				}

				//���� ������ ���� �� ���
				for (i = 0; i < *indexNum; i++)
				{
					be[i] = (1 + tanA*tanX[i]) / (tanX[i] - tanA);
				}

				//����
				sort(be, *indexNum);

				//���� ����
				DoubleMatrix matA(*indexNum, 2);
				DoubleMatrix matx(2, 1);
				DoubleMatrix matb(*indexNum, 1);


				//a�� d�� ã�´�.
				for (i = 0; i < *indexNum; i++)
				{
					matA(i, 0) = 1;
					matA(i, 1) = i;
					matb(i, 0) = be[i];
				}


				matx = (matA.Transpose()*matA).Inverse()*matA.Transpose()*matb;
				double aaa = matx(0, 0);
				double ddd = matx(1, 0);



				d[0] = ddd / LINE_INTERVAL;		//���� ��ĭ�� d
				a[0] = aaa - linestart*d[0];		//���� a
			//	a[0] = aaa-11*d[0];		//���� a


				/*
				outfile.open("adad.txt");
				outfile << a[0] << endl << d[0] << endl ;
				outfile.close();
				*/
				m_factor[0] = a[0];
				m_offset[0] = d[0];
			}
		}

		else if (originalType == 1)
		{

			if (*indexNum < 2)
			{
				/*
				outfile.open("adad.txt", ios::ate );
				outfile << -2.4 << endl << 0.1 << endl ;
				outfile.close();
				*/
				m_factor[1] = -2.4;
				m_offset[1] = 0.1;
			}
			else
			{
				//���� ���μ��� ���Ͽ� ���غ���.


				//���� ������ tan �� �ε�
				for (i = 0; i < *indexNum; i++)
				{
					tanX[i] = -m_tangents[1][i];
				}


				//��� ������ ���� �� ���
				for (i = 0; i < *indexNum; i++)
				{
					be[i] = (1 + tanA*tanX[i]) / (tanA - tanX[i]);
				}


				//����
				sort(be, *indexNum);

				//���� ����
				DoubleMatrix matA(*indexNum, 2);
				DoubleMatrix matx(2, 1);
				DoubleMatrix matb(*indexNum, 1);

				//a�� d�� ã�´�.
				for (i = 0; i < *indexNum; i++)
				{
					matA(i, 0) = 1;
					matA(i, 1) = i;
					matb(i, 0) = be[i];
				}

				matx = (matA.Transpose()*matA).Inverse()*matA.Transpose()*matb;
				double aaa = matx(0, 0);
				double ddd = matx(1, 0);



				d[1] = ddd / LINE_INTERVAL;		//���� ��ĭ�� d
				a[1] = aaa - linestart*d[1];		//���� a
			//	a[1] = aaa-11*d[1];		//���� a


				/*
				outfile.open("adad.txt", ios::ate );
				outfile << a[1] << endl << d[1] << endl ;
				outfile.close();
				*/
				m_factor[1] = a[1];
				m_offset[1] = d[1];

			}
		}
		else if (originalType == 2)
		{
			if (*indexNum < 2)
			{
				/*
				outfile.open("adad.txt", ios::ate );
				outfile << -2.4 << endl << 0.1 << endl ;
				outfile.close();
				*/
				m_factor[2] = -2.4;
				m_offset[2] = 0.1;

			}
			else
			{
				//���� ������ tan �� �ε�
				for (i = 0; i < *indexNum; i++)
				{
					tanX[i] = -m_tangents[2][i];
				}


				//���� ������ ���� �� ���
				for (i = 0; i < *indexNum; i++)
				{
					be[i] = (1 + tanA*tanX[i]) / (tanX[i] - tanA);
				}


				//����
				sort(be, *indexNum);

				//���� ����
				DoubleMatrix matA(*indexNum, 2);
				DoubleMatrix matx(2, 1);
				DoubleMatrix matb(*indexNum, 1);


				//a�� d�� ã�´�.
				for (i = 0; i < *indexNum; i++)
				{
					matA(i, 0) = 1;
					matA(i, 1) = i;
					matb(i, 0) = be[i];
				}


				TRACE_MAT(matA);

				matx = (matA.Transpose()*matA).Inverse()*matA.Transpose()*matb;
				double aaa = matx(0, 0);
				double ddd = matx(1, 0);



				d[2] = ddd / LINE_INTERVAL;		//���� ��ĭ�� d
				a[2] = aaa - linestart*d[2];		//���� a

				/*
				outfile.open("adad.txt", ios::ate);
				outfile << a[2] << endl << d[2] << endl ;
				outfile.close();
				*/
				m_factor[2] = a[2];
				m_offset[2] = d[2];

			}

		}
	}
}

//////////////////////////////
//	Class SecondOrderLine	//
//////////////////////////////

void SecondOrderLine::set(const cv::Mat& input)
{
	coef_xx = input.at<double>(0, 0);
	coef_yy = input.at<double>(1, 0);
	coef_xy = input.at<double>(2, 0);
	coef_x = input.at<double>(3, 0);
	coef_y = input.at<double>(4, 0);
	cons = input.at<double>(5, 0);
}

void SecondOrderLine::set(double xx, double yy, double xy, double x, double y, double con)
{
	coef_xx = xx;
	coef_yy = yy;
	coef_xy = xy;
	coef_x = x;
	coef_y = y;
	cons = con;
}

void SecondOrderLine::getX(int y, int *x1, int *x2)
{
	float part;
	part = sqrt(pow(coef_xy*y + coef_x, 2) - 4 * coef_xx*(coef_yy*pow(y, 2) + coef_y*y + cons));
	*x1 = (int)((-coef_xy*y - coef_x - part) / (2 * coef_xx));
	*x2 = (int)((-coef_xy*y - coef_x + part) / (2 * coef_xx));
}

void SecondOrderLine::getY(int x, int *y1, int *y2)
{
	float part;
	part = sqrt(pow(coef_xy*x + coef_y, 2) - 4 * coef_yy*(coef_xx*pow(x, 2) + coef_x*x + cons));
	if (part >= 0)
	{
		*y1 = (int)((-coef_xy*x - coef_y - part) / (2 * coef_yy));
		*y2 = (int)((-coef_xy*x - coef_y + part) / (2 * coef_yy));
	}
	else
	{
		*y1 = -50000;
		*y2 = -50000;
	}
}


//////////////////////////////
//	Class ThirdOrderLine	//
//////////////////////////////

void ThirdOrderLine::set(const cv::Mat& input)
{
	cons = input.at<double>(0, 0);
	coef_x1 = input.at<double>(1, 0);
	coef_x2 = input.at<double>(2, 0);
	coef_x3 = input.at<double>(3, 0);
}

void ThirdOrderLine::set(double x1, double x2, double x3, double con)
{
	coef_x1 = x1;
	coef_x2 = x2;
	coef_x3 = x3;
	cons = con;
}

void ThirdOrderLine::getX(int y, int *x1)
{
	*x1 = (int)(coef_x1 * y + coef_x2 * pow(y, 2) + coef_x3 * pow(y, 3) + cons);
}

void ThirdOrderLine::getY(int x, int *y1)
{
	*y1 = (int)(coef_x1 * x + coef_x2 * pow(x, 2) + coef_x3 * pow(x, 3) + cons);
}

void ThirdOrderLine::getX(double y, double *x1)
{
	*x1 = (coef_x1 * y + coef_x2 * pow(y, 2) + coef_x3 * pow(y, 3) + cons);
}

void ThirdOrderLine::getY(double x, double *y1)
{
	*y1 = (coef_x1 * x + coef_x2 * pow(x, 2) + coef_x3 * pow(x, 3) + cons);
}

float ThirdOrderLine::slope(float a, int type)
{
	float ret;

	ret = 3 * coef_x3 * pow(a, 2) + 2 * coef_x2 * a + coef_x1;

	if (type == 0 || type == 2)
	{
		return 1 / ret;
	}
	else
	{
		return ret;
	}
}

//////////////////////////////
//	Class StraightLine		//
//////////////////////////////

void StraightLine::set(const char *file, int line)
{
	std::ifstream infile( file );
	for(int i = 0 ; i <= line ; i++)
		infile >> coef_x >> coef_y >> cons;
}

void StraightLine::set(SecondOrderLine a, SecondOrderLine b)
{
	double x = -a.coef_x*b.coef_xx + b.coef_x*a.coef_xx;
	double y = -a.coef_y*b.coef_xx + b.coef_y*a.coef_xx;
	double c = -a.cons*b.coef_xx + b.cons*a.coef_xx;

	if (c != 0)
	{
		x /= c;
		y /= c;
		c = 1;
	}
	else
	{
		x /= y;
		y = 1;
	}

	double tmp = fabs(x) + fabs(y) + fabs(c);

	this->coef_x = (float)x / tmp;
	this->coef_y = (float)y / tmp;
	this->cons = (float)c / tmp;
}

void StraightLine::set(double tan, const cv::Point2f& point)
{
	coef_x = tan;
	coef_y = -1 ;
	cons = -( coef_x * point.x  + coef_y * point.y);
}

int StraightLine::getX(int y)
{
	return (-coef_y * y - cons) / coef_x;
}

int StraightLine::getY(int x)
{
	return (-coef_x * x - cons) / coef_y;
}

void StraightLine::modify(const cv::Point2f& point)
{
	cons = -(coef_x * point.x + coef_y * point.y);
}

// functions

void loadPointsFromFile(std::ifstream ptFile, vecPoint2f *arrPoint)
{
	int ptNum;
	ptFile >> ptNum;
	for(int i = 0; i< 3*ptNum*ptNum; i++)
	{
		cv::Point2f temp;
		ptFile >> temp.x >> temp.y;
		(*arrPoint).Add(temp);
	}
}


void findHoleWith3pt(cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt3, cv::Point2f* center, float* radius)
{
	DoubleMatrix x1(3), x2(3), x3(3), xCenter(3);
	x1(0) = pt1.x;
	x1(1) = pt1.y;
	x1(2) = 1;

	x2(0) = pt2.x;
	x2(1) = pt2.y;
	x2(2) = 1;

	x3(0) = pt3.x;
	x3(1) = pt3.y;
	x3(2) = 1;

	DoubleMatrix l1(3), l2(3);

	l1 = Cross(x1,x2);
	l2 = Cross(x2,x3);

	DoubleMatrix newl1(3), newl2(3);

	newl1(0) = -l1(1);
	newl1(1) = l1(0);
	newl1(2) = - ( newl1(0)*(x1(0)+x2(0))/2.0f + newl1(1)*(x1(1)+x2(1))/2.0f );
	newl2(0) = -l2(1);
	newl2(1) = l2(0);
	newl2(2) = - ( newl2(0)*(x2(0)+x3(0))/2.0f + newl2(1)*(x2(1)+x3(1))/2.0f );


	xCenter = Cross(newl1,newl2);
	
	(*center).x = xCenter(0)/ xCenter(2);
	(*center).y = xCenter(1)/ xCenter(2);

	*radius = fpDistance(*center, pt1);
}

void findHoleWith4pt(cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt3, cv::Point2f pt4, cv::Point2f* center, float* radius)
{
	findHoleWith3pt(pt1, pt2, pt3, center, radius);
	if( fpDistance(*center, pt4) > *radius)
		return;
	findHoleWith3pt(pt1, pt2, pt4, center, radius);
	if( fpDistance(*center, pt3) > *radius)
		return;
	findHoleWith3pt(pt1, pt3, pt4, center, radius);
	if( fpDistance(*center, pt2) > *radius)
		return;
	findHoleWith3pt(pt2, pt3, pt4, center, radius);
}

DoubleMatrix findHomography(vecPoint2f first, vecPoint2f second)
{
	int ptNum;
	ptNum = first.GetSize();

	DoubleMatrix matA, matX, matb, matH;

	matA.Allocate(ptNum*2, 8);
	matX.Allocate(8,1);
	matb.Allocate(ptNum*2, 1);

	matH.Allocate(3,3);

	for(int i = 0 ; i < ptNum ; i ++)
	{
		matA(2*i, 0) = first[i].x;
		matA(2*i, 1) = first[i].y;
		matA(2*i, 2) = 1;
		matA(2*i, 6) = - first[i].x * second[i].x;
		matA(2*i, 7) = - first[i].y * second[i].x;
		matA(2*i+1, 3) = first[i].x;
		matA(2*i+1, 4) = first[i].y;
		matA(2*i+1, 5) = 1;
		matA(2*i+1, 6) = - first[i].x * second[i].y;
		matA(2*i+1, 7) = - first[i].y * second[i].y;
		
		matb(2*i) = second[i].x;
		matb(2*i+1) = second[i].y;
	}
	matX = ( matA.Transpose() * matA ).Inverse() * matA.Transpose() * matb;

	matH(0,0) = matX(0);
	matH(0,1) = matX(1);
	matH(0,2) = matX(2);
	matH(1,0) = matX(3);
	matH(1,1) = matX(4);
	matH(1,2) = matX(5);
	matH(2,0) = matX(6);
	matH(2,1) = matX(7);
	matH(2,2) = 1;

	return matH;
}

cv::Point2f ptCorrection(cv::Point2f cod, float w, cv::Point2f inputpt)
{
	float disR = fpDistance(cod, inputpt);
	float ratio = tan(disR * w) /(disR*2.0*tan(w/2.0)) ;
	
	cv::Point2f ret;
	ret.x = cod.x + ratio * ( inputpt.x - cod.x );
	ret.y = cod.y + ratio * ( inputpt.y - cod.y );

	return ret;
}


float lineErrorCheck(const int XSize, const int YSize, const cv::Point2f center, const float w, const vecPoint2f& arrDisPoint)
{

	//corpoints ���
	vecPoint2f	arrCorPoint;

	float xd, yd, xc, yc, ratio;
	double r;

	for(int i =0 ; i < POINTNUM_SQ*3 ; i++)
	{
		xd	= arrDisPoint[i].x;
		yd	= arrDisPoint[i].y;

		if(xd == -1)
		{
			arrCorPoint.Add(cv::Point2f(-1,-1));
		}
		else
		{
			r=sqrt(pow(xd-center.x,2)+pow(yd-center.y,2));

			ratio = tan(r * w) /(r*2*tan(w/2)) ;

			xc= center.x +((float)xd-center.x)*ratio ;
			yc= center.y +((float)yd-center.y)*ratio ;

			arrCorPoint.Add(cv::Point2f(xc,yc));
		}
	}

	return lineErrorMeasure(arrCorPoint, XSize, YSize);	
}


bool findMinErrorW(int XSize, int YSize, cv::Point2f center, const vecPoint2f& arrDisPoint, float* w, bool& bLocalMin)
{

	if (w[2] - w[0] < 0.000001)
	{
		return TRUE;
	}

	float lineError[3];

	lineError[0] = lineErrorCheck(XSize, YSize, center, w[0], arrDisPoint);	
	lineError[1] = lineErrorCheck(XSize, YSize, center, w[1], arrDisPoint);	
	lineError[2] = lineErrorCheck(XSize, YSize, center, w[2], arrDisPoint);


	if (lineError[0] < lineError[1] && lineError[0] < lineError[2])
	{
		if (bLocalMin == FALSE)
		{
			w[2] = w[1] ; 
			w[1] = w[0] ; 
			w[0] = w[1] - (w[2] - w[1]);
		}
		else
		{
			w[2] = (w[0]+w[1])/2 ; 
			w[1] = w[0] ; 
			w[0] = w[1] - (w[2] - w[1]);
		}
		return FALSE;
	}
	else if (lineError[1] < lineError[0] && lineError[1] < lineError[2])
	{
		bLocalMin = TRUE;

		w[0] = (w[0]+w[1])/2; 
		w[2] = (w[1]+w[2])/2;

		return FALSE;
	}
	else if (lineError[2] < lineError[0] && lineError[2] < lineError[1])
	{
		if (bLocalMin == FALSE)
		{
			w[0] = w[1] ; 
			w[1] = w[2] ; 
			w[2] = w[1] + (w[1] - w[0]);
		}
		else
		{
			w[0] = (w[1] + w[2])/2 ; 
			w[1] = w[2] ; 
			w[2] = w[1] + (w[1] - w[0]);
		}

		return FALSE;
	}
	else
	{
		return TRUE;
	}
}


float postProcessing(ByteImage &img, vecPoint2f& arrDisPoint, cv::Point2f center, float wInit )
{
	int XSize = img.GetXSize();
	int YSize = img.GetYSize();

	float w[3];

	w[0] = wInit - 0.002;
	w[1] = wInit;
	w[2] = wInit + 0.002;


	bool bLocalMin = FALSE;
	int cnt=0;

	while (1)
	{
		if (w[0] < 0) { w[0] = 0; }
	
		bool bSucess = findMinErrorW(XSize, YSize, center, arrDisPoint, w, bLocalMin);

		if(bSucess)
		{
			cnt++;
		}

		if (cnt > 1)
		{
			break;
		}
	}

	if (w[1] > 0.01)
	{
		return wInit;
	}
	else
	{
		return w[1];
	}
}
