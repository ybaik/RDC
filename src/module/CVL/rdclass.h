#include <fstream>
#include "parameters.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

typedef std::vector<cv::Point2f> vecPoint2f;

class SecondOrderLine
{
public:
	double coef_xx, coef_yy, coef_xy, coef_x, coef_y, cons;
	void set(const cv::Mat& input);
	void set(double xx, double yy, double xy, double x, double y, double con);
	void getX(int y, int *x1, int *x2);
	void getY(int x, int *y1, int *y2);
};

class ThirdOrderLine
{
public:
	double coef_x1, coef_x2, coef_x3, cons;
	void set(const cv::Mat& input);
	void set(double x1, double x2, double x3, double con);
	void getX(int y, int *x1);
	void getY(int x, int *y1);
	void getX(double y, double *x1);
	void getY(double x, double *y1);

	float slope(float a, int type);
};

class StraightLine
{
public:
	void set(const char *file, int line);
	void set(SecondOrderLine a, SecondOrderLine b);
	void set(double tan, const cv::Point2f& point);
	int getX(int y);
	int getY(int x);
	void modify(const cv::Point2f& point);

	double coef_x, coef_y, cons;
};

class SetofPoints
{

public:
	double			m_factor[3];
	double			m_offset[3];
	float			m_tangents[3][POINTNUM];
	cv::Point2f		m_vp[3];
	StraightLine	m_msLine[3];
	SecondOrderLine m_secondLines[3][LINENUM1 * 2];

	cv::Point2f		a[3][POINTNUM + LINENUM1][POINTNUM + LINENUM1];
	ThirdOrderLine	s[3][LINENUM1];

	SetofPoints();
	SetofPoints(const int lineNum, const int pointNum, const vecPoint2f& arrPoint);

	void setTypeLine(int type);
	void setTypeLine2(int type);

	void sortPoint(int type);
	void findAngle(const cv::Point2f& center, int originalType, cv::Point2f * nPoint, int* indexNum, bool COD_pp = true);
};


//function
void setLinesFromFile(SecondOrderLine*l, const char *filename);
void setLinesFromFile(ThirdOrderLine*s, const char *filename);
void findVanishingPoint(SecondOrderLine*l, cv::Point2f* van);
void maxRline(SecondOrderLine*l, int *first, int *second, int count, int*removR);
float calCenter(StraightLine *three, cv::Point2f& ptCenter);

float findTan(StraightLine line1, StraightLine line2);
void findRAngle(float *angle_yz, float *angle_xz, StraightLine*msLine);
//int sort(CPoint* point, int num, int type);
int sort(double* point, int num);
double distance(double ax, double ay, double bx, double by);


float GetArea(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d);
double GetArea(double* point1, double* point2, double* point3, double* point4);

void intersection(StraightLine line1, StraightLine line2, double* point);


double findTanofV(double*tan_bar, double left, double right, int num);
double errorTanofV(double*tan_bar, double tan_v, int num);

void Get4Area(float x, float y, float*a, float*b, float*c, float*d);



// new
void drawLnVP(const cv::Point2f& center, StraightLine *msLine, cv::Point2f* vp, const cv::Mat& img, cv::Mat& canvas);
void drawLine(cv::Mat& canvas, StraightLine& line, int type);
void findVP(float angle1, float angle2, cv::Point2f* vp, cv::Point2f center, StraightLine*msLine);

double findWofFOV3(const vecPoint2f& arrDisPoint, const vecPoint2f& arrCorPoint, const cv::Point2f center, double left, double right, int xSize, int ySize);

double errorW(const vecPoint2f& arrDisPoint, const vecPoint2f& arrCorPoint, const cv::Point2f center, double w, int xSize, int ySize);

bool isDiagonal(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& otherPt1, const cv::Point2f& otherPt2);

//functions.h
float fpDistance(cv::Point2f p1, cv::Point2f p2);
float ppDistance(cv::Mat p1, cv::Mat p2);
float lpDistance(const cv::Point3f& l, const cv::Point2f& p);
void TakeRowsFrom(cv::Mat*mat, cv::Mat temp);
void TakeColsFrom(cv::Mat*mat, cv::Mat temp, int start);
cv::Mat nullVecter(cv::Mat mat);

cv::Point2f orthocenter(cv::Point2f* vertex);



void correction(const cv::Mat& cimg,
	const int nPointNum,
	const vecPoint2f& arrDisPoint_ori,
	vecPoint2f& arrCorPoint,
	cv::Point2f& ptCOD,
	double& w,
	bool COD_pp = true);

cv::Point2f findCenter(const cv::Mat& img, SetofPoints& set);
void findVanishPoint(const cv::Point2f& center, SetofPoints& set);
void findVanishPoint2(const cv::Mat& img, const cv::Point2f& center, int * angleIdx, SetofPoints& set);
void lineForAngle(const cv::Mat img, cv::Point2f center, SetofPoints& set);
void calcAngl(const cv::Mat img, cv::Point2f center, SetofPoints& set, int* angleIdx, bool COD_pp = true);

void reconstructLines(const cv::Mat inputImage, const cv::Point2f& center, const vecPoint2f& arrDisPoint, vecPoint2f& arrCorPoint, SetofPoints& set);


void FOV(const cv::Point2f& center, const vecPoint2f& arrDisPoint, vecPoint2f& arrCorPoint, double& w, int xSize, int ySize);

void ComputeInitArrCorPointWithFOV(const vecPoint2f& arrDisPoint, vecPoint2f& arrCorPoint, double w, cv::Point2f center);

void drawArc(const cv::Mat &canvas, const vecPoint2f &arr, SecondOrderLine arc, CvScalar color, int thickness);
float distanceFromConic(cv::Point2f pt, SecondOrderLine arc);


void loadPointsFromFile(std::ifstream ptFile, vecPoint2f *arrPoint);

float lineErrorMeasure(const vecPoint2f& arrCorPoint, int xSize, int ySize);

void findHoleWith4pt(cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt3, cv::Point2f pt4, cv::Point2f* center, float* radius);

cv::Mat findHomography(vecPoint2f first, vecPoint2f second);

cv::Point2f ptCorrection(cv::Point2f cod, float w, cv::Point2f inputpt);

float postProcessing(const cv::Mat &img, vecPoint2f& arrDisPoint, cv::Point2f center, float wInit);

