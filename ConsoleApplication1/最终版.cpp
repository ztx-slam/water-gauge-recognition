#include<opencv2\opencv.hpp>
#include<iostream>
#include<vector>
#define WINDOW_NAME  "Regulating parameter"
#define PI 3.1415926
#include "time.h"

using namespace std;
using namespace cv;
//-----------------------------------��ȫ�ֱ����������֡�--------------------------------------
//		������ȫ�ֱ�������
//-----------------------------------------------------------------------------------------------
int blockSize = 5;
CvRect RectLine;
int constValue;
Mat gaussianFilImg, LaplacianImg, gaussianFilImg1;
Mat threshImg, threshImg1,closeImg;
////////////////����任�����//////////
class LineFinder
{
private:
	std::vector<cv::Vec4i> lines;  // ֱ�߶�Ӧ�ĵ��������
	double deltaRho;  //����
	double deltaTheta;
	int minVote;  // �ж���ֱ�ߵ���СͶƱ��
	double minLength;  // �ж���ֱ�ߵ���С����
	double maxGap;  // ͬһ��ֱ���ϵ�֮��ľ������̶�
public:
	LineFinder() : deltaRho(1), deltaTheta(PI / 180),
		minVote(10), minLength(0.), maxGap(0.) {}  //��ʼ��

	void setAccResolution(double dRho, double dTheta)   // ���ò���
	{
		deltaRho = dRho;
		deltaTheta = dTheta;
	}

	void setMinVote(int minv)  // ������СͶƱ��
	{
		minVote = minv;
	}

	void setLineLengthAndGap(double length, double gap)  // ������С�߶γ��Ⱥ��߶μ�����̶� 
	{
		minLength = length;
		maxGap = gap;
	}


	std::vector<cv::Vec4i> findLines(cv::Mat& binary, cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255))  //Ѱ���߶�
	{
		lines.clear();
		cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
		std::cout << "HoughLinesP" << lines.size() << std::endl;//�������
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
	//�ҵ��ֱ�߶�ͬʱ�����ֱ�߲���,ͬʱ������ֱ�������ͼƬ
	cv::Vec4i findlonggestLine(cv::Mat& binary, cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255))
	{
		lines.clear();
		cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
		std::cout << "HoughLinesP  " << lines.size() << std::endl;//�������
		int n = lines.size();
		vector<double> a(n, 0);
		int k = 0;

		//���㳤��
		if (n != 0)
		{
			for (int i = 0; i < n; i++)
			{
				Vec4i L = lines[i];
				a[i] = sqrtf((L[0] - L[2])*(L[0] - L[2]) + (L[1] - L[3])*(L[1] - L[3]));
			}
		}
		//////////////////Ѱ�����ֵ
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
		imshow("��߶�", houghline);
		return maxline;

	}
	cv::Vec4i findLinesandRotate(cv::Mat& binary, cv::Mat &image, cv::Mat& houghimg1, cv::Scalar color = cv::Scalar(255, 255, 255))
	{
		lines.clear();
		cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
		std::cout << "HoughLinesP " << lines.size() << std::endl;//�������
		int n = lines.size();
		vector<double> a(n, 0);
		int k = 0;
		houghimg1.setTo(0);
		//���㳤��
		if (n != 0)
		{
			for (int i = 0; i < n; i++)
			{
				Vec4i L = lines[i];
				a[i] = sqrtf((L[0] - L[2])*(L[0] - L[2]) + (L[1] - L[3])*(L[1] - L[3]));
			}
		}
		//////////////////Ѱ�����ֵ
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
		//����������߶�
		Point p1(maxline[0], maxline[1]);
		Point p2(maxline[2], maxline[3]);
		Mat houghline;;
		image.copyTo(houghline);
		line(houghline, p1, p2, cv::Scalar(255), 1);
		//imshow("���ֱ��", houghline);
		////////////����ˮƽ����
		double angle = fastAtan2(maxline[3] - maxline[1], maxline[2] - maxline[0]) - 270;
		cout << angle;
		Point2f centerpoint = (image.cols / 2, image.rows / 2);
		Mat rotateMat = getRotationMatrix2D(centerpoint, angle, 1.0);

		warpAffine(image, houghimg1, rotateMat, image.size(), 1, 0, 0);
		//namedWindow("����", 0);
		//imshow("����", houghimg1);
		return maxline;
	}
};
int g_nElementShape = MORPH_RECT;//Ԫ�ؽṹ����״
//�������յ�TrackBarλ�ò���
int g_nMaxIterationNum = 10;
int g_nOpenCloseNum = 0;
//-----------------------------------��ȫ�ֺ����������֡�--------------------------------------
Mat sobeproc(const Mat &src);
void on_Trackbar(int, void*)
{
	adaptiveThreshold(LaplacianImg, threshImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	imshow(WINDOW_NAME, threshImg);
}
//-----------------------------------��on_OpenClose( )������----------------------------------
//		��������������/�����㡿���ڵĻص�����
//-----------------------------------------------------------------------------------------------

static void on_OpenClose(int, void*)
{
	//ƫ�����Ķ���
	int offset = g_nOpenCloseNum - g_nMaxIterationNum;//ƫ����
	int Absolute_offset = offset > 0 ? offset : -offset;//ƫ��������ֵ
	//�Զ����
	Mat element = getStructuringElement(g_nElementShape, Size(Absolute_offset * 2 + 1,
		Absolute_offset * 2 + 1), Point(Absolute_offset, Absolute_offset));
	if (offset < 0)
		morphologyEx(threshImg, closeImg, MORPH_OPEN, element);
	else
		morphologyEx(threshImg, closeImg, MORPH_CLOSE, element);
	imshow(WINDOW_NAME, closeImg);
}
/********************����ȡ��*********************/
//�������ƣ�int CutRow��IplImage *BinaryImage��
//���ܣ��зָ�
//��ڲ�������ֵ��ͼ��BianryImage,�հ׾���Rect_aim
//Ч�����ָ�ˮ�����򣬲�����ˮ���������
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

/********************ˮ�߷ָ��ӳ���*********************/
//�������ƣ�int CutRow��IplImage *BinaryImage��
//���ܣ��зָ�
//��ڲ�������ֵ��ͼ��BianryImage,�հ׾���Rect_aim
//Ч�����ָ�ˮ�����򣬲�����ˮ���������
/***************************************************/
void CutRow(IplImage *BinaryImage, CvRect &Rect_aim)
{
	CvSize pSize;
	pSize.width = BinaryImage->width;
	pSize.height = BinaryImage->height;
	cout << pSize.width << "   " << pSize.height << endl;
	int *rowwidth = new int[pSize.width];
	int *rowheight = new int[pSize.height];
	//��������rowwidth�����洢ÿ�а�ɫ������,ͬʱ�����ɫ������������
	memset(rowwidth, 0, pSize.width*sizeof(int));
	memset(rowheight, 0, pSize.height*sizeof(int));
	int max_col = 0, max_pix = 0;
	for (int i = 0; i<pSize.width; i++)
	{
		for (int j = 0; j<pSize.height; j++)
		{
			if (cvGetReal2D(BinaryImage, j, i)>0)
				//ͳ�ư�ɫ������
				rowwidth[i] = rowwidth[i] + 1;
		}

		if (rowwidth[i]>max_pix)
		{
			max_pix = rowwidth[i];
			max_col = i;
		}
	}
	cout << "����а�ɫ����  " << max_pix << "  �к�  " << max_col << endl;

	//��������rowwidth�����洢ÿ�а�ɫ������
	for (int i = 0; i<pSize.height; i++)
	{
		for (int j = 0; j<pSize.width; j++)
		{
			if (cvGetReal2D(BinaryImage, i, j)>0)
				//ͳ�ư�ɫ������
				rowheight[i] = rowheight[i] + 1;
		}
	}
	//����ˮ�߽߱�
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
	printf("��ͼƬ·���ڴ����ʧ��");
	exit(1);
	}*/
	//ȷ�����Σ���5���رߣ�
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
/********************����ʱ��ˮӡ*********************/
//�������ƣ�void fugaishijian(Mat &img)
//���ܣ�������ʱ��ˮӡ
//��ڲ�����ͼ��img
//Ч��������ʱ��ˮӡ��������һ������
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
	//cvShowImage("��ֵ", RotateRow);
	cvErode(RotateRow, RotateRow, 0, 1);
	//cvShowImage("��ʴ", RotateRow);
	cvDilate(RotateRow, RotateRow, 0, 2);
	//cvShowImage("����", RotateRow);
	//��ÿ��ͼ����ұ�Ե
	CvMemStorage *OutlineSto = cvCreateMemStorage();
	CvSeq *Outlineseq = NULL;
	IplImage *TmpImage = cvCloneImage(RotateRow);
	int Num = cvFindContours(TmpImage, OutlineSto, &Outlineseq, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
	//ȷ������˳��
	int *Rect_x = new int[Num];
	int x = 0;

	for (CvSeq *c = Outlineseq; c != NULL; c = c->h_next)
	{
		CvRect RectBound = cvBoundingRect(c, 0);
		Rect_x[x] = RectBound.x;
		x++;
	}

	qsort(Rect_x, Num, sizeof(int), cmp);
	//�ָ����ֲ������洢
	for (CvSeq *c = Outlineseq; c != NULL; c = c->h_next)
	{
		CvRect RectBound = cvBoundingRect(c, 0);
		CvRect CutRect = cvRect(RectBound.x - 4, RectBound.y - 4, RectBound.width + 8, RectBound.height + 8);
		IplImage *ImgNo = cvCreateImage(cvSize(CutRect.width, CutRect.height), IPL_DEPTH_8U, 1);
		cvSet(ImgNo, cvScalar(0), 0); //��ͼ�����Ϊ��ɫ
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
		//Ϊͼ��洢·�������ڴ�
		char *SavePath = (char *)malloc(30 * sizeof(char));
		if (SavePath == NULL)
		{
			printf("�����ڴ�ʧ��");
			exit(1);
		}
		sprintf(SavePath, "Num%d(%d).jpg", row, col + 1);
		//��ʾ��row�У���col+1��
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
/****************��бУ���ӳ���*****************/
//�������ƣ�IplImage *Rotate(IplImage *RowImage��
//���ܣ���ÿ�����ֽ�����бУ��
//��ڲ�������ͼ��RowImage
//���ڲ�������ת���ͼ��RotateRow
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
	// ����ת��������ͼ���м�
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
	//���������Ե�����ͼ��canImage
	IplImage *canImage = cvCreateImage(cvGetSize(RowImage), IPL_DEPTH_8U, 1);
	//���б�Ե���
	cvCanny(RowImage, canImage, 30, 200, 3);
	//����hough�任
	CvMemStorage *storage = cvCreateMemStorage();
	CvSeq *lines = NULL;
	lines = cvHoughLines2(canImage, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, 30, 0, 0);
	//ͳ������ֱ�н�<30�ȵ�ֱ�߸����Լ���нǺ�
	int numLine = 0;
	float sumAng = 0.0;
	for (int i = 0; i<lines->total; i++)
	{
		float *line = (float *)cvGetSeqElem(lines, i);
		float theta = line[1];  //��ȡ�Ƕ� Ϊ������                                   
		if (theta<120 * CV_PI / 180 && theta>60 * CV_PI / 180)
		{
			numLine++;
			sumAng = sumAng + theta;
		}
	}
	//�����ƽ����б�ǣ�anAngΪ�Ƕ���
	float avAng = (sumAng / numLine) * 180 / CV_PI;

	//��ȡ��ά��ת�ķ���任����
	CvPoint2D32f center;
	center.x = float(RowImage->width / 2.0);
	center.y = float(RowImage->height / 2.0);
	float m[6];
	CvMat M = cvMat(2, 3, CV_32F, m);
	cv2DRotationMatrix(center, avAng, 1, &M);
	//�������ͼ��RotateRow
	double a = sin(sumAng / numLine);
	double b = cos(sumAng / numLine);
	int width_rotate = int(RowImage->height*fabs(a) + RowImage->width*fabs(b));
	int height_rotate = int(RowImage->width*fabs(a) + RowImage->height*fabs(b));
	IplImage *RotateRow = cvCreateImage(cvSize(width_rotate, height_rotate), IPL_DEPTH_8U, 1);
	//�任ͼ�񣬲��ú�ɫ�������ֵ
	m[2] += (width_rotate - RowImage->width) / 2;
	m[5] += (height_rotate - RowImage->height) / 2;
	cvWarpAffine(RowImage, RotateRow, &M, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
	///////////��ת

	cvReleaseImage(&canImage);
	cvReleaseMemStorage(&storage);
	return RotateRow;
}
///////////////ȥ���߹�
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
			beta = max(max(beta_r, beta_g), beta_b);//��beta����������ϵ��������                 // gama is used to approximiate the beta
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
//////////////////��ȡ�Ǽ�,Ч����ʱһ��
void chao_thinimage(Mat &srcimage)//��ͨ������ֵ�����ͼ��
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
	constValue = 5;//����Ӧ��ֵ������
	g_nOpenCloseNum = 10;//���������
	namedWindow(WINDOW_NAME, 0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	createTrackbar("threshold", WINDOW_NAME, &constValue, 100, on_Trackbar);
	createTrackbar("����ֵ", WINDOW_NAME, &g_nOpenCloseNum, g_nMaxIterationNum * 10 + 1, on_OpenClose);
	//ִ�лص�����
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
	//imshow("Ԥ����", closeImg);

	//////////////////Mat��IplImage *��ת��
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
	//cvShowImage("�ָ�", RowImage);
	//cvSaveImage("��ɫ17.jpg", RowImage);

    /////////////////////////////////////////////////////
	//�����Ǵ���Ƭ�л�ȡˮ����������
	//�����Ƕ�ˮ����������ָͬʱ
	/////////////////////////////////////////////////////
	
	Mat plate1(RowImage);
	//imshow("132", plate1);
	cvtColor(plate1, plate1, CV_BGR2GRAY);
	GaussianBlur(plate1, gaussianFilImg1, Size(5, 5), 0, 0);//����
	equalizeHist(gaussianFilImg1, gaussianFilImg1);//���⻯
	//imshow("Ԥ����", gaussianFilImg1);
	threshold(gaussianFilImg1, threshImg1, 55, 255, CV_THRESH_BINARY);//��ֵ��
	imshow("��ֵ��0", threshImg1);
	threshold(threshImg1, threshImg1, 205, 255, CV_THRESH_BINARY_INV);//�ٴζ�ֵ��
	imshow("��ֵ��", threshImg1);
	
	cv::Mat contours2;
	cv::Canny(threshImg1, contours2, 125, 350);
	LineFinder finder;
	finder.setMinVote(120);
	finder.setLineLengthAndGap(150, 18);
	Mat jiaozheng;
	finder.findLinesandRotate(contours2, threshImg1, jiaozheng);//�����Ժ��ͼ

	cv::Mat contours3;
	cv::Canny(jiaozheng, contours3, 125, 350);
	LineFinder finder1;
	finder1.setMinVote(120);
	finder1.setLineLengthAndGap(150, 16);
	Vec4i cutline = finder1.findlonggestLine(contours3, jiaozheng);
	imshow("jiaozheng",jiaozheng);
	
	///////////�ָ������������,Ȼ���ٷָ���ͼ��
	/////////////ȡ�����ֲ���ˮ��//////////////
	Mat roi_left, roi_right;

	//cv::Mat roi_left = Mat::zeros(threshImg.rows, cutline[0], CV_8UC1), roi_right;//ָ����С,ûʲô��
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
	imshow("���������ȡ�Ǽ�", roi_left);

	morphologyEx(roi_right, roi_right, MORPH_OPEN, element);
	threshold(roi_right, roi_right, 200, 255, CV_THRESH_BINARY_INV);
	//chao_thinimage(roi_right);
	imshow("��ͼ�������ȡ����ȡ�Ǽ�", roi_right);

	/////////ֱ�ӽ�ͼƬƴ��////////////
	///////�õ��ػ�Ŀ̶ȳ�///////////
	Mat result(roi_left.rows, roi_left.cols + roi_right.cols, roi_left.type());
	roi_left.colRange(0, roi_left.cols).copyTo(result.colRange(0, roi_left.cols));
	roi_right.colRange(0, roi_right.cols).copyTo(result.colRange(roi_left.cols, result.cols));
	imshow("ƴ��",result);
	
	waitKey();
	return 0;
}
//sobel�������һ��ˮƽ���������Դ���ֱ��Ե
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
