#include<opencv2\opencv.hpp>
#include<iostream>
#include<vector>
#define WINDOW_NAME  "Regulating parameter"
#define PI 3.1415926
#include "time.h"

using namespace std;
using namespace cv;
//-----------------------------------【全局变量声明部分】--------------------------------------
//		描述：全局变量声明
//-----------------------------------------------------------------------------------------------
int blockSize = 5;
CvRect RectLine;
int constValue;
Mat gaussianFilImg, LaplacianImg, gaussianFilImg1;
Mat threshImg, threshImg1,closeImg;
////////////////霍夫变换相关类//////////
class LineFinder
{
private:
	std::vector<cv::Vec4i> lines;  // 直线对应的点参数向量
	double deltaRho;  //步长
	double deltaTheta;
	int minVote;  // 判断是直线的最小投票数
	double minLength;  // 判断是直线的最小长度
	double maxGap;  // 同一条直线上点之间的距离容忍度
public:
	LineFinder() : deltaRho(1), deltaTheta(PI / 180),
		minVote(10), minLength(0.), maxGap(0.) {}  //初始化

	void setAccResolution(double dRho, double dTheta)   // 设置步长
	{
		deltaRho = dRho;
		deltaTheta = dTheta;
	}

	void setMinVote(int minv)  // 设置最小投票数
	{
		minVote = minv;
	}

	void setLineLengthAndGap(double length, double gap)  // 设置最小线段长度和线段间距容忍度 
	{
		minLength = length;
		maxGap = gap;
	}


	std::vector<cv::Vec4i> findLines(cv::Mat& binary, cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255))  //寻找线段
	{
		lines.clear();
		cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
		std::cout << "HoughLinesP" << lines.size() << std::endl;//输出段数
		std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
		while (it2 != lines.end())
		{
			cv::Point pt1((*it2)[0], (*it2)[1]);
			cv::Point pt2((*it2)[2], (*it2)[3]);
			cv::line(image, pt1, pt2, cv::Scalar(255), 1);
			++it2;
		}
		imshow("bianhuan", image);
		return lines;
	}

	/*void drawDetectedLines(cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255))
	{
	std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
	while (it2 != lines.end())
	{
	cv::Point pt1((*it2)[0], (*it2)[1]);
	cv::Point pt2((*it2)[2], (*it2)[3]);
	cv::line(image, pt1, pt2, cv::Scalar(255), 1);
	++it2;
	}
	}*/
	//找到最长直线段同时返回最长直线参数,同时返回竖直矫正后的图片
	cv::Vec4i findlonggestLine(cv::Mat& binary, cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255))
	{
		lines.clear();
		cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
		std::cout << "HoughLinesP  " << lines.size() << std::endl;//输出段数
		int n = lines.size();
		vector<double> a(n, 0);
		int k = 0;

		//计算长度
		if (n != 0)
		{
			for (int i = 0; i < n; i++)
			{
				Vec4i L = lines[i];
				a[i] = sqrtf((L[0] - L[2])*(L[0] - L[2]) + (L[1] - L[3])*(L[1] - L[3]));
			}
		}
		//////////////////寻找最大值
		double temp = a[0];
		for (int i = 0; i < n; i++)
		{
			if (a[i] > temp)
			{
				temp = a[i];
				k = i;
			}
		}
		Vec4i maxline = lines[k];
		cout << maxline[0] << "    " << maxline[2]<<endl;
		Point p1(maxline[0], maxline[1]);
		Point p2(maxline[2], maxline[3]);
		Mat houghline;;
		image.copyTo(houghline);
		line(houghline, p1, p2, cv::Scalar(255), 1);
		imshow("最长线段", houghline);
		return maxline;

	}
	cv::Vec4i findLinesandRotate(cv::Mat& binary, cv::Mat &image, cv::Mat& houghimg1, cv::Scalar color = cv::Scalar(255, 255, 255))
	{
		lines.clear();
		cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
		std::cout << "HoughLinesP " << lines.size() << std::endl;//输出段数
		int n = lines.size();
		vector<double> a(n, 0);
		int k = 0;
		houghimg1.setTo(0);
		//计算长度
		if (n != 0)
		{
			for (int i = 0; i < n; i++)
			{
				Vec4i L = lines[i];
				a[i] = sqrtf((L[0] - L[2])*(L[0] - L[2]) + (L[1] - L[3])*(L[1] - L[3]));
			}
		}
		//////////////////寻找最大值
		double temp = a[0];
		for (int i = 0; i < n; i++)
		{
			if (a[i] > temp)
			{
				temp = a[i];
				k = i;
			}
		}
		Vec4i maxline = lines[k];
		cout << maxline[0] << "    " << maxline[2]<<endl;
		//画出此最大线段
		Point p1(maxline[0], maxline[1]);
		Point p2(maxline[2], maxline[3]);
		Mat houghline;;
		image.copyTo(houghline);
		line(houghline, p1, p2, cv::Scalar(255), 1);
		//imshow("最大直线", houghline);
		////////////进行水平矫正
		double angle = fastAtan2(maxline[3] - maxline[1], maxline[2] - maxline[0]) - 270;
		cout << angle;
		Point2f centerpoint = (image.cols / 2, image.rows / 2);
		Mat rotateMat = getRotationMatrix2D(centerpoint, angle, 1.0);

		warpAffine(image, houghimg1, rotateMat, image.size(), 1, 0, 0);
		//namedWindow("矫正", 0);
		//imshow("矫正", houghimg1);
		return maxline;
	}
};
int g_nElementShape = MORPH_RECT;//元素结构的形状
//变量接收的TrackBar位置参数
int g_nMaxIterationNum = 10;
int g_nOpenCloseNum = 0;
//-----------------------------------【全局函数声明部分】--------------------------------------
Mat sobeproc(const Mat &src);
void on_Trackbar(int, void*)
{
	adaptiveThreshold(LaplacianImg, threshImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	imshow(WINDOW_NAME, threshImg);
}
//-----------------------------------【on_OpenClose( )函数】----------------------------------
//		描述：【开运算/闭运算】窗口的回调函数
//-----------------------------------------------------------------------------------------------

static void on_OpenClose(int, void*)
{
	//偏移量的定义
	int offset = g_nOpenCloseNum - g_nMaxIterationNum;//偏移量
	int Absolute_offset = offset > 0 ? offset : -offset;//偏移量绝对值
	//自定义核
	Mat element = getStructuringElement(g_nElementShape, Size(Absolute_offset * 2 + 1,
		Absolute_offset * 2 + 1), Point(Absolute_offset, Absolute_offset));
	if (offset < 0)
		morphologyEx(threshImg, closeImg, MORPH_OPEN, element);
	else
		morphologyEx(threshImg, closeImg, MORPH_CLOSE, element);
	imshow(WINDOW_NAME, closeImg);
}
/********************像素取反*********************/
//函数名称：int CutRow（IplImage *BinaryImage）
//功能：行分割
//入口参数：二值化图像BianryImage,空白矩形Rect_aim
//效果：分割水尺区域，并返回水尺区域矩形
/***************************************************/
void qufan(IplImage *BinaryImage)
{
	int height, width, step;
	uchar * data;
	int i, j;
	height = BinaryImage->height;
	width = BinaryImage->width;
	data = (uchar *)BinaryImage->imageData;

	// invert image
	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			data[i*width + j] = 255 - data[i*width + j];
		}
	}

}

/********************水尺分割子程序*********************/
//函数名称：int CutRow（IplImage *BinaryImage）
//功能：行分割
//入口参数：二值化图像BianryImage,空白矩形Rect_aim
//效果：分割水尺区域，并返回水尺区域矩形
/***************************************************/
void CutRow(IplImage *BinaryImage, CvRect &Rect_aim)
{
	CvSize pSize;
	pSize.width = BinaryImage->width;
	pSize.height = BinaryImage->height;
	cout << pSize.width << "   " << pSize.height << endl;
	int *rowwidth = new int[pSize.width];
	int *rowheight = new int[pSize.height];
	//建立数组rowwidth用来存储每列白色像素数,同时求出白色像素最多的列数
	memset(rowwidth, 0, pSize.width*sizeof(int));
	memset(rowheight, 0, pSize.height*sizeof(int));
	int max_col = 0, max_pix = 0;
	for (int i = 0; i<pSize.width; i++)
	{
		for (int j = 0; j<pSize.height; j++)
		{
			if (cvGetReal2D(BinaryImage, j, i)>0)
				//统计白色的像素
				rowwidth[i] = rowwidth[i] + 1;
		}

		if (rowwidth[i]>max_pix)
		{
			max_pix = rowwidth[i];
			max_col = i;
		}
	}
	cout << "最大列白色像素  " << max_pix << "  列号  " << max_col << endl;

	//建立数组rowwidth用来存储每行白色像素数
	for (int i = 0; i<pSize.height; i++)
	{
		for (int j = 0; j<pSize.width; j++)
		{
			if (cvGetReal2D(BinaryImage, i, j)>0)
				//统计白色的像素
				rowheight[i] = rowheight[i] + 1;
		}
	}
	//查找水尺边界
	int width_left = 0, width_right = 0, height_top = 0, height_botton = 0;
	for (int i = max_col; i < pSize.width; i++)
	{
		if (rowwidth[i] > 13 && rowwidth[i + 1] <= 13)
		{
			width_right = i;
			cout << "width_right" << width_right << endl;
			break;
		}
	}
	for (int i = max_col; i >1; i--)
	{
		if (rowwidth[i]>4 && rowwidth[i - 1] <= 4)
		{
			width_left = i;
			cout << "width_left" << width_left << endl;
			break;
		}
	}
	for (int i = pSize.height / 3; i < pSize.height; i++)
	{
		if (rowheight[i - 1] >= 3 && rowheight[i] <3)
		{
			height_botton = i;
			cout << "height_botton" << height_botton << endl;
			break;
		}
	}
	for (int i = 0; i<pSize.height; i++)
	{
		if (rowheight[i - 1] <= 10 && rowheight[i]>10)
		{
			height_top = i;
			cout << "height_top" << height_top << endl;
			break;
		}
	}

	char *LinePath = (char *)malloc(30 * sizeof(char));
	/*if (LinePath == NULL)
	{
	printf("行图片路径内存分配失败");
	exit(1);
	}*/
	//确定矩形（留5像素边）
	RectLine = cvRect(width_left - 5, height_top - 5, width_right - width_left + 5, height_botton - height_top + 5);
	Rect_aim = RectLine;
	IplImage *RowImage = cvCreateImage(cvSize(RectLine.width, RectLine.height), IPL_DEPTH_8U, 1);
	cvSet(RowImage, cvScalar(0), 0);
	cvSetImageROI(BinaryImage, RectLine);
	cvCopy(BinaryImage, RowImage);

	sprintf(LinePath, "Row%d.jpg", width_left);
	//cvSaveImage(LinePath, RowImage);
	//cvShowImage("example", RowImage);
	cvReleaseImage(&RowImage);
	free(LinePath);
	LinePath = NULL;

	delete rowwidth;
	delete rowheight;
}
int cmp(const void *a, const void*b) { return *(int *)a - *(int *)b; }
/********************覆盖时间水印*********************/
//函数名称：void fugaishijian(Mat &img)
//功能：将覆盖时间水印
//入口参数：图像img
//效果：覆盖时间水印，便于下一步处理
/***************************************************/
void fugaishijian(Mat &img)
{
	cv::Mat
		image = cv::Mat::zeros(100, 900, CV_8UC1);
	image.setTo(cv::Scalar(0));

	cv::Rect roi_rect = cv::Rect(20, 20, image.cols, image.rows);
	image.copyTo(img(roi_rect));
}
int CutNum(IplImage *RotateRow, int row)
{
	cvThreshold(RotateRow, RotateRow, 50, 255, CV_THRESH_BINARY);
	//cvShowImage("二值", RotateRow);
	cvErode(RotateRow, RotateRow, 0, 1);
	//cvShowImage("腐蚀", RotateRow);
	cvDilate(RotateRow, RotateRow, 0, 2);
	//cvShowImage("膨胀", RotateRow);
	//对每列图像查找边缘
	CvMemStorage *OutlineSto = cvCreateMemStorage();
	CvSeq *Outlineseq = NULL;
	IplImage *TmpImage = cvCloneImage(RotateRow);
	int Num = cvFindContours(TmpImage, OutlineSto, &Outlineseq, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
	//确定轮廓顺序
	int *Rect_x = new int[Num];
	int x = 0;

	for (CvSeq *c = Outlineseq; c != NULL; c = c->h_next)
	{
		CvRect RectBound = cvBoundingRect(c, 0);
		Rect_x[x] = RectBound.x;
		x++;
	}

	qsort(Rect_x, Num, sizeof(int), cmp);
	//分割数字并单独存储
	for (CvSeq *c = Outlineseq; c != NULL; c = c->h_next)
	{
		CvRect RectBound = cvBoundingRect(c, 0);
		CvRect CutRect = cvRect(RectBound.x - 4, RectBound.y - 4, RectBound.width + 8, RectBound.height + 8);
		IplImage *ImgNo = cvCreateImage(cvSize(CutRect.width, CutRect.height), IPL_DEPTH_8U, 1);
		cvSet(ImgNo, cvScalar(0), 0); //将图像填充为黑色
		cvSetImageROI(RotateRow, CutRect);
		cvCopy(RotateRow, ImgNo);
		cvResetImageROI(RotateRow);
		int col = 0;
		for (int i = 0; i<Num; i++)
		{
			if (Rect_x[i] == RectBound.x)
			{
				col = i;
				break;
			}
		}
		//为图像存储路径分配内存
		char *SavePath = (char *)malloc(30 * sizeof(char));
		if (SavePath == NULL)
		{
			printf("分配内存失败");
			exit(1);
		}
		sprintf(SavePath, "Num%d(%d).jpg", row, col + 1);
		//表示第row行，第col+1个
		cvSaveImage(SavePath, ImgNo);
		free(SavePath);
		SavePath = NULL;
		cvReleaseImage(&ImgNo);
	}
	delete Rect_x;
	cvReleaseMemStorage(&OutlineSto);
	cvReleaseImage(&TmpImage);
	return Num;
}
/****************倾斜校正子程序*****************/
//函数名称：IplImage *Rotate(IplImage *RowImage）
//功能：对每行数字进行倾斜校正
//入口参数：行图像RowImage
//出口参数：旋转后的图像RotateRow
/***********************************************/

IplImage* rotateImage(IplImage* src, int angle, bool clockwise)
{
	angle = abs(angle) % 180;
	if (angle > 90)
	{
		angle = 90 - (angle % 90);
	}
	IplImage* dst = NULL;
	int width =
		(double)(src->height * sin(angle * CV_PI / 180.0)) +
		(double)(src->width * cos(angle * CV_PI / 180.0)) + 1;
	int height =
		(double)(src->height * cos(angle * CV_PI / 180.0)) +
		(double)(src->width * sin(angle * CV_PI / 180.0)) + 1;
	int tempLength = sqrt((double)src->width * src->width + src->height * src->height) + 10;
	int tempX = (tempLength + 1) / 2 - src->width / 2;
	int tempY = (tempLength + 1) / 2 - src->height / 2;
	int flag = -1;

	dst = cvCreateImage(cvSize(width, height), src->depth, src->nChannels);
	cvZero(dst);
	IplImage* temp = cvCreateImage(cvSize(tempLength, tempLength), src->depth, src->nChannels);
	cvZero(temp);

	cvSetImageROI(temp, cvRect(tempX, tempY, src->width, src->height));
	cvCopy(src, temp, NULL);
	cvResetImageROI(temp);

	if (clockwise)
		flag = 1;

	float m[6];
	int w = temp->width;
	int h = temp->height;
	m[0] = (float)cos(flag * angle * CV_PI / 180.);
	m[1] = (float)sin(flag * angle * CV_PI / 180.);
	m[3] = -m[1];
	m[4] = m[0];
	// 将旋转中心移至图像中间
	m[2] = w * 0.5f;
	m[5] = h * 0.5f;
	//
	CvMat M = cvMat(2, 3, CV_32F, m);
	cvGetQuadrangleSubPix(temp, dst, &M);
	cvReleaseImage(&temp);
	return dst;
}

IplImage *Rotate(IplImage *RowImage)
{
	//建立储存边缘检测结果图像canImage
	IplImage *canImage = cvCreateImage(cvGetSize(RowImage), IPL_DEPTH_8U, 1);
	//进行边缘检测
	cvCanny(RowImage, canImage, 30, 200, 3);
	//进行hough变换
	CvMemStorage *storage = cvCreateMemStorage();
	CvSeq *lines = NULL;
	lines = cvHoughLines2(canImage, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, 30, 0, 0);
	//统计与竖直夹角<30度的直线个数以及其夹角和
	int numLine = 0;
	float sumAng = 0.0;
	for (int i = 0; i<lines->total; i++)
	{
		float *line = (float *)cvGetSeqElem(lines, i);
		float theta = line[1];  //获取角度 为弧度制                                   
		if (theta<120 * CV_PI / 180 && theta>60 * CV_PI / 180)
		{
			numLine++;
			sumAng = sumAng + theta;
		}
	}
	//计算出平均倾斜角，anAng为角度制
	float avAng = (sumAng / numLine) * 180 / CV_PI;

	//获取二维旋转的仿射变换矩阵
	CvPoint2D32f center;
	center.x = float(RowImage->width / 2.0);
	center.y = float(RowImage->height / 2.0);
	float m[6];
	CvMat M = cvMat(2, 3, CV_32F, m);
	cv2DRotationMatrix(center, avAng, 1, &M);
	//建立输出图像RotateRow
	double a = sin(sumAng / numLine);
	double b = cos(sumAng / numLine);
	int width_rotate = int(RowImage->height*fabs(a) + RowImage->width*fabs(b));
	int height_rotate = int(RowImage->width*fabs(a) + RowImage->height*fabs(b));
	IplImage *RotateRow = cvCreateImage(cvSize(width_rotate, height_rotate), IPL_DEPTH_8U, 1);
	//变换图像，并用黑色填充其余值
	m[2] += (width_rotate - RowImage->width) / 2;
	m[5] += (height_rotate - RowImage->height) / 2;
	cvWarpAffine(RowImage, RotateRow, &M, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
	///////////逆转

	cvReleaseImage(&canImage);
	cvReleaseMemStorage(&storage);
	return RotateRow;
}
///////////////去除高光
int highlight_remove_Chi(IplImage* src, IplImage* dst, double Re)
{
	int height = src->height;
	int width = src->width;
	int step = src->widthStep;
	int i = 0, j = 0;
	unsigned char R, G, B, MaxC;
	double alpha, beta, alpha_r, alpha_g, alpha_b, beta_r, beta_g, beta_b, temp = 0, realbeta = 0, minalpha = 0;
	double gama, gama_r, gama_g, gama_b;
	unsigned char* srcData;
	unsigned char* dstData;
	for (i = 0; i<height; i++)
	{
		srcData = (unsigned char*)src->imageData + i*step;
		dstData = (unsigned char*)dst->imageData + i*step;
		for (j = 0; j<width; j++)
		{
			R = srcData[j * 3];
			G = srcData[j * 3 + 1];
			B = srcData[j * 3 + 2];

			alpha_r = (double)R / (double)(R + G + B);
			alpha_g = (double)G / (double)(R + G + B);
			alpha_b = (double)B / (double)(R + G + B);
			alpha = max(max(alpha_r, alpha_g), alpha_b);
			MaxC = max(max(R, G), B);// compute the maximum of the rgb channels
			minalpha = min(min(alpha_r, alpha_g), alpha_b);                 beta_r = 1 - (alpha - alpha_r) / (3 * alpha - 1);
			beta_g = 1 - (alpha - alpha_g) / (3 * alpha - 1);
			beta_b = 1 - (alpha - alpha_b) / (3 * alpha - 1);
			beta = max(max(beta_r, beta_g), beta_b);//将beta当做漫反射系数，则有                 // gama is used to approximiate the beta
			gama_r = (alpha_r - minalpha) / (1 - 3 * minalpha);
			gama_g = (alpha_g - minalpha) / (1 - 3 * minalpha);
			gama_b = (alpha_b - minalpha) / (1 - 3 * minalpha);
			gama = max(max(gama_r, gama_g), gama_b);

			temp = (gama*(R + G + B) - MaxC) / (3 * gama - 1);
			//beta=(alpha-minalpha)/(1-3*minalpha)+0.08;
			//temp=(gama*(R+G+B)-MaxC)/(3*gama-1);
			dstData[j * 3] = R - (unsigned char)(temp + 0.5);
			dstData[j * 3 + 1] = G - (unsigned char)(temp + 0.5);
			dstData[j * 3 + 2] = B - (unsigned char)(temp + 0.5);
		}
	}
	return 1;
}
//////////////////提取骨架,效果暂时一般
void chao_thinimage(Mat &srcimage)//单通道、二值化后的图像
{
	vector<Point> deletelist1;
	int Zhangmude[9];
	int nl = srcimage.rows;
	int nc = srcimage.cols;
	while (true)
	{
		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[7] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[5] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();

		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[3] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[1] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();
	}
}

int main()
{

	Mat dist;
	Mat plate = imread("15.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!plate.data) { cout << "error in read image please check it\n"; return false; }
	GaussianBlur(plate, gaussianFilImg, Size(5, 5), 0, 0);
	Laplacian(gaussianFilImg, dist, CV_16S, 3);
	convertScaleAbs(gaussianFilImg, LaplacianImg);
	constValue = 5;//自适应阈值化参数
	g_nOpenCloseNum = 10;//闭运算参数
	namedWindow(WINDOW_NAME, 0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	createTrackbar("threshold", WINDOW_NAME, &constValue, 100, on_Trackbar);
	createTrackbar("迭代值", WINDOW_NAME, &g_nOpenCloseNum, g_nMaxIterationNum * 10 + 1, on_OpenClose);
	//执行回调函数
	on_Trackbar(constValue, 0);
	on_OpenClose(g_nOpenCloseNum, 0);

	while (true)
	{
		int c = waitKey(0);
		if ((char)c == ' ')
		{
			vector<vector<Point>>contours;
			findContours(threshImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			drawContours(threshImg, contours, -1, Scalar(26, 24, 46), 3);
		}
		break;
	}
	fugaishijian(closeImg);
	//imshow("预处理", closeImg);

	//////////////////Mat与IplImage *的转换
	IplImage imgTmp = closeImg;
	IplImage *pImg = cvCloneImage(&imgTmp);
	IplImage *pImg1 = cvLoadImage("15.jpg", CV_LOAD_IMAGE_UNCHANGED);

	IplImage *BinaryImage = NULL;
	BinaryImage = cvCreateImage(cvGetSize(pImg), IPL_DEPTH_8U, 1);
	cvThreshold(pImg, BinaryImage, 10, 255, CV_THRESH_BINARY);

	//cvShowImage("example", BinaryImage);
	CvRect rect_aim;
	CutRow(BinaryImage, rect_aim);
	IplImage *RowImage = cvCreateImage(cvSize(rect_aim.width, rect_aim.height), IPL_DEPTH_8U, 3);
	cvSet(RowImage, cvScalar(0), 0);
	cvSetImageROI(pImg1, rect_aim);
	cvCopy(pImg1, RowImage);
	//cvShowImage("分割", RowImage);
	//cvSaveImage("彩色17.jpg", RowImage);

    /////////////////////////////////////////////////////
	//以上是从照片中获取水尺特征区域
	//以下是对水尺区域操作分割，同时
	/////////////////////////////////////////////////////
	
	Mat plate1(RowImage);
	//imshow("132", plate1);
	cvtColor(plate1, plate1, CV_BGR2GRAY);
	GaussianBlur(plate1, gaussianFilImg1, Size(5, 5), 0, 0);//降噪
	equalizeHist(gaussianFilImg1, gaussianFilImg1);//均衡化
	//imshow("预处理", gaussianFilImg1);
	threshold(gaussianFilImg1, threshImg1, 55, 255, CV_THRESH_BINARY);//二值化
	imshow("二值化0", threshImg1);
	threshold(threshImg1, threshImg1, 205, 255, CV_THRESH_BINARY_INV);//再次二值化
	imshow("二值化", threshImg1);
	
	cv::Mat contours2;
	cv::Canny(threshImg1, contours2, 125, 350);
	LineFinder finder;
	finder.setMinVote(120);
	finder.setLineLengthAndGap(150, 18);
	Mat jiaozheng;
	finder.findLinesandRotate(contours2, threshImg1, jiaozheng);//矫正以后的图

	cv::Mat contours3;
	cv::Canny(jiaozheng, contours3, 125, 350);
	LineFinder finder1;
	finder1.setMinVote(120);
	finder1.setLineLengthAndGap(150, 16);
	Vec4i cutline = finder1.findlonggestLine(contours3, jiaozheng);
	imshow("jiaozheng",jiaozheng);
	
	///////////分割成左右两部分,然后再分割左图，
	/////////////取含数字部分水尺//////////////
	Mat roi_left, roi_right;

	//cv::Mat roi_left = Mat::zeros(threshImg.rows, cutline[0], CV_8UC1), roi_right;//指定大小,没什么用
	//imshow("yuantu1",roi_left);
	cv::Rect rect_left(0, 0, cutline[0], jiaozheng.rows);
	jiaozheng(rect_left).copyTo(roi_left);
	//threshold(roi_left, roi_left, 200, 255, CV_THRESH_BINARY_INV);
	cv::Mat contours1;
	cv::Canny(roi_left, contours1, 125, 350);
	LineFinder finder2;
	finder2.setLineLengthAndGap(100, 60);
	finder2.findlonggestLine(contours1, roi_left);
	IplImage imgTmp1 = roi_left;
	IplImage *pImg2 = cvCloneImage(&imgTmp1);
//	CutNum(pImg2, 18);
	cv::imshow("rect_left", roi_left);

	cv::Rect rect_right(cutline[0], 0, jiaozheng.cols - cutline[0], jiaozheng.rows);
	jiaozheng(rect_right).copyTo(roi_right);
	cv::imshow("roi", roi_right);

	Mat element = getStructuringElement(MORPH_RECT, Size(28, 1));
	morphologyEx(roi_left, roi_left, MORPH_OPEN, element);
	threshold(roi_left, roi_left, 200, 255, CV_THRESH_BINARY_INV);
	//chao_thinimage(roi_left);
	imshow("开运算后提取骨架", roi_left);

	morphologyEx(roi_right, roi_right, MORPH_OPEN, element);
	threshold(roi_right, roi_right, 200, 255, CV_THRESH_BINARY_INV);
	//chao_thinimage(roi_right);
	imshow("右图开运算后取反提取骨架", roi_right);

	/////////直接将图片拼接////////////
	///////得到重绘的刻度尺///////////
	Mat result(roi_left.rows, roi_left.cols + roi_right.cols, roi_left.type());
	roi_left.colRange(0, roi_left.cols).copyTo(result.colRange(0, roi_left.cols));
	roi_right.colRange(0, roi_right.cols).copyTo(result.colRange(roi_left.cols, result.cols));
	imshow("拼接",result);
	
	waitKey();
	return 0;
}
//sobel算子求得一阶水平方向导数，以此求垂直边缘
Mat sobeproc(const Mat &src)
{
	Mat dst;
	Mat grad_x, grad_y, abs_grad_x, abs_grad_y;
	Sobel(src, grad_x, CV_8U, 1, 0, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);
	Sobel(src, grad_y, CV_8U, 0, 1, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);
	return dst;
}
