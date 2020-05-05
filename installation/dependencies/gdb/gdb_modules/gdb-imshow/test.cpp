/* Copyright (c) 2012, Renato Florentino Garcia <fgarcia.renato@gmail.com>
 *                     Stefano Pellegrini <stefpell@ee.ethz.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the authors nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    Mat mat8UC1 (250, 250, CV_8UC1,  Scalar(0));
    Mat mat8SC1 (250, 250, CV_8SC1,  Scalar(0));
    Mat mat16UC1(250, 250, CV_16UC1, Scalar(0));
    Mat mat16SC1(250, 250, CV_16SC1, Scalar(0));
    Mat mat32SC1(250, 250, CV_32SC1, Scalar(0));
    Mat mat32FC1(250, 250, CV_32FC1, Scalar(0.0));
    Mat mat64FC1(250, 250, CV_64FC1, Scalar(0.0));
    rectangle(mat8UC1,  Point(50,50), Point(200,150), Scalar(150));
    rectangle(mat8SC1,  Point(50,50), Point(200,150), Scalar(-70));
    rectangle(mat16UC1, Point(50,50), Point(200,150), Scalar(50000));
    rectangle(mat16SC1, Point(50,50), Point(200,150), Scalar(-20000));
    rectangle(mat32SC1, Point(50,50), Point(200,150), Scalar(-1e9));
    rectangle(mat32FC1, Point(50,50), Point(200,150), Scalar(-1e9));
    rectangle(mat32FC1, Point(50,50), Point(200,150), Scalar(0.7));
    rectangle(mat32FC1, Point(40,40), Point(210,160), Scalar(-0.7));
    rectangle(mat64FC1, Point(50,50), Point(200,150), Scalar(0.07));
    rectangle(mat64FC1, Point(40,40), Point(210,160), Scalar(-0.07));

    Mat mat8UC3 (250, 250, CV_8UC3,  Scalar(0, 0, 0));
    Mat mat8SC3 (250, 250, CV_8SC3,  Scalar(0, 0, 0));
    Mat mat16UC3(250, 250, CV_16UC3, Scalar(0, 0, 0));
    Mat mat16SC3(250, 250, CV_16SC3, Scalar(0, 0, 0));
    Mat mat32SC3(250, 250, CV_32SC3, Scalar(0, 0, 0));
    Mat mat32FC3(250, 250, CV_32FC3, Scalar(0.0, 0.0, 0.0));
    Mat mat64FC3(250, 250, CV_64FC3, Scalar(0.0, 0.0, 0.0));
    rectangle(mat8UC3,  Point(50,50), Point(200,150), Scalar(150,0,0));
    rectangle(mat8SC3,  Point(50,50), Point(200,150), Scalar(-70,0,0));
    rectangle(mat16UC3, Point(50,50), Point(200,150), Scalar(50000,0,0));
    rectangle(mat16SC3, Point(50,50), Point(200,150), Scalar(-20000,0,0));
    rectangle(mat32SC3, Point(50,50), Point(200,150), Scalar(-1e9,0,0));
    rectangle(mat32FC3, Point(50,50), Point(200,150), Scalar(0.7,0,0));
    rectangle(mat32FC3, Point(40,40), Point(210,160), Scalar(-0.7,0,0));
    rectangle(mat64FC3, Point(50,50), Point(200,150), Scalar(0.07,0,0));
    rectangle(mat64FC3, Point(40,40), Point(210,160), Scalar(-0.07,0,0));

    Mat matImg = imread("gogh.jpg");
    Mat matRoi(matImg, Rect(50, 50, 100, 100));

    // Will fail to show
    Mat mat8UC2(13, 13, CV_8UC2);

    //--------------------------------------------------------------------------
    IplImage *ipl8UC1  = cvCreateImage(cvSize(250, 250), IPL_DEPTH_8U,  1); cvSet(ipl8UC1,  cvScalar(0));
    IplImage *ipl8SC1  = cvCreateImage(cvSize(250, 250), IPL_DEPTH_8S,  1); cvSet(ipl8SC1,  cvScalar(0));
    IplImage *ipl16UC1 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_16U, 1); cvSet(ipl16UC1, cvScalar(0));
    IplImage *ipl16SC1 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_16S, 1); cvSet(ipl16SC1, cvScalar(0));
    IplImage *ipl32SC1 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_32S, 1); cvSet(ipl32SC1, cvScalar(0));
    IplImage *ipl32FC1 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_32F, 1); cvSet(ipl32FC1, cvScalar(0.0));
    IplImage *ipl64FC1 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_64F, 1); cvSet(ipl64FC1, cvScalar(0.0));
    cvRectangle(ipl8UC1,  cvPoint(50,50), cvPoint(200,150), cvScalar(150));
    cvRectangle(ipl8SC1,  cvPoint(50,50), cvPoint(200,150), cvScalar(-70));
    cvRectangle(ipl16UC1, cvPoint(50,50), cvPoint(200,150), cvScalar(50000));
    cvRectangle(ipl16SC1, cvPoint(50,50), cvPoint(200,150), cvScalar(-20000));
    cvRectangle(ipl32SC1, cvPoint(50,50), cvPoint(200,150), cvScalar(-1e9));
    cvRectangle(ipl32FC1, cvPoint(50,50), cvPoint(200,150), cvScalar(-1e9));
    cvRectangle(ipl32FC1, cvPoint(50,50), cvPoint(200,150), cvScalar(0.7));
    cvRectangle(ipl32FC1, cvPoint(40,40), cvPoint(210,160), cvScalar(-0.7));
    cvRectangle(ipl64FC1, cvPoint(50,50), cvPoint(200,150), cvScalar(0.07));
    cvRectangle(ipl64FC1, cvPoint(40,40), cvPoint(210,160), cvScalar(-0.07));

    IplImage *ipl8UC3  = cvCreateImage(cvSize(250, 250), IPL_DEPTH_8U,  3); cvSet(ipl8UC3,  cvScalar(0, 0, 0));
    IplImage *ipl8SC3  = cvCreateImage(cvSize(250, 250), IPL_DEPTH_8S,  3); cvSet(ipl8SC3,  cvScalar(0, 0, 0));
    IplImage *ipl16UC3 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_16U, 3); cvSet(ipl16UC3, cvScalar(0, 0, 0));
    IplImage *ipl16SC3 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_16S, 3); cvSet(ipl16SC3, cvScalar(0, 0, 0));
    IplImage *ipl32SC3 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_32S, 3); cvSet(ipl32SC3, cvScalar(0, 0, 0));
    IplImage *ipl32FC3 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_32F, 3); cvSet(ipl32FC3, cvScalar(0.0, 0.0, 0.0));
    IplImage *ipl64FC3 = cvCreateImage(cvSize(250, 250), IPL_DEPTH_64F, 3); cvSet(ipl64FC3, cvScalar(0.0, 0.0, 0.0));
    cvRectangle(ipl8UC3,  cvPoint(50,50), cvPoint(200,150), cvScalar(150,0,0));
    cvRectangle(ipl8SC3,  cvPoint(50,50), cvPoint(200,150), cvScalar(-70,0,0));
    cvRectangle(ipl16UC3, cvPoint(50,50), cvPoint(200,150), cvScalar(50000,0,0));
    cvRectangle(ipl16SC3, cvPoint(50,50), cvPoint(200,150), cvScalar(-20000,0,0));
    cvRectangle(ipl32SC3, cvPoint(50,50), cvPoint(200,150), cvScalar(-1e9,0,0));
    cvRectangle(ipl32FC3, cvPoint(50,50), cvPoint(200,150), cvScalar(0.7,0,0));
    cvRectangle(ipl32FC3, cvPoint(40,40), cvPoint(210,160), cvScalar(-0.7,0,0));
    cvRectangle(ipl64FC3, cvPoint(50,50), cvPoint(200,150), cvScalar(0.07,0,0));
    cvRectangle(ipl64FC3, cvPoint(40,40), cvPoint(210,160), cvScalar(-0.07,0,0));

    IplImage *iplImg = cvLoadImage("gogh.jpg");
    IplImage *iplRoi = cvLoadImage("gogh.jpg");
    cvSetImageROI(iplRoi, cvRect(50, 50, 100, 100));

    // Will fail to show
    IplImage *ipl8UC2 = cvCreateImage(cvSize(13, 13), IPL_DEPTH_8U, 2);

    cvReleaseImage(&ipl8UC1);
    cvReleaseImage(&ipl8SC1);
    cvReleaseImage(&ipl16UC1);
    cvReleaseImage(&ipl16SC1);
    cvReleaseImage(&ipl32SC1);
    cvReleaseImage(&ipl32FC1);
    cvReleaseImage(&ipl64FC1);
    cvReleaseImage(&ipl8UC3);
    cvReleaseImage(&ipl8SC3);
    cvReleaseImage(&ipl16UC3);
    cvReleaseImage(&ipl16SC3);
    cvReleaseImage(&ipl32SC3);
    cvReleaseImage(&ipl32FC3);
    cvReleaseImage(&ipl64FC3);
    cvReleaseImage(&iplImg);
    cvReleaseImage(&iplRoi);
    cvReleaseImage(&ipl8UC2);

    return 0;
}
