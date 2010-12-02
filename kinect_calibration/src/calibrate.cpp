/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**

\author Kurt Konolige, Patrick Mihelich

**/

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h> // cv::undistort isn't in the above!?

#include <Eigen/Core>
#include <Eigen/LU>

using namespace cv;
using namespace Eigen;
using namespace std;

// image size
#define ROWS 480
#define COLS 640

// checkerboard pattern
int ccols = 7;
int crows = 6;

// cell size
double csize = .108;

// Pixel offset from IR image to depth image
//cv::Point2f ir_depth_offset = cv::Point2f(-4.835, -3.888);
//cv::Point2f ir_depth_offset = cv::Point2f(-4.5, -3.5);
cv::Point2f ir_depth_offset = cv::Point2f(-4, -3);
//cv::Point2f ir_depth_offset = cv::Point2f(0, 0);

// changes shifts to disparities
// 1084 is shift offset (from CCNY data); assume 1/8 pixel
//#define SHIFT_OFFSET 1084

#define SHIFT_SCALE 0.125
//#define SHIFT_SCALE 0.125

double shift2disp(int shift, double shift_offset)
{
  return SHIFT_SCALE*(double)(shift_offset - shift);
}

// colorizes a depth pixel
uint16_t t_gamma[2048];         // gamma conversion for depth

void setDepthColor(uint8_t *cptr, int d)
{
  int pval = t_gamma[d];
  int lb = pval & 0xff;
  switch (pval >> 8)
    {
    case 0:
      cptr[2] = 255;
      cptr[1] = 255 - lb;
      cptr[0] = 255 - lb;
      break;
    case 1:
      cptr[2] = 255;
      cptr[1] = lb;
      cptr[0] = 0;
      break;
    case 2:
      cptr[2] = 255 - lb;
      cptr[1] = 255;
      cptr[0] = 0;
      break;
    case 3:
      cptr[2] = 0;
      cptr[1] = 255;
      cptr[0] = lb;
      break;
    case 4:
      cptr[2] = 0;
      cptr[1] = 255 - lb;
      cptr[0] = 255;
      break;
    case 5:
      cptr[2] = 0;
      cptr[1] = 0;
      cptr[0] = 255 - lb;
      break;
    default:
      cptr[2] = 0;
      cptr[1] = 0;
      cptr[0] = 0;
      break;
    }
}


// 
// read in IR images, perform monocular calibration, check distortion
// arguments:
//   [dir]          data directory (without trailing slash); default cwd
//   [cell size, m] size of edge of each square in chessboard
//   [#rows #cols]  number of rows and cols of interior chessboard;
//                  default 6x7
//

int
main(int argc, char **argv)
{
  printf("Optional arguments: [data dir] [#rows #cols]\n");
  
  char fdir[1024];
  sprintf(fdir, ".");
  if (argc > 1)
    {
      sprintf(fdir,"%s",argv[1]);
    }

  if (argc > 2)
    csize = atof(argv[2]);

  if (argc > 4)
    {
      crows = atoi(argv[3]);
      ccols = atoi(argv[4]);
    }

  
  // construct the planar pattern
  vector<Point3f> pat;
  for (int i=0; i<ccols; i++)
    for (int j=0; j<crows; j++)
      pat.push_back(Point3f(i*csize,j*csize,0));

  // read in images, set up feature points and patterns
  vector<vector<Point3f> > pats;
  vector<vector<Point2f> > points;

  int fnum = 0;

  while (1)
    {
      char fname[1024];
      sprintf(fname,"%s/img_ir_%02d.png",fdir,fnum++);
      Mat img = imread(fname,-1);
      if (img.data == NULL) break; // no data, not read, break out

      vector<cv::Point2f> corners;
      bool ret = cv::findChessboardCorners(img,Size(crows,ccols),corners);

      if (ret)
        printf("Found corners in image %s\n",fname);
      else
        printf("*** Didn't find corners in image %s\n",fname);

      cv::cornerSubPix(img, corners, Size(5,5), Size(-1,-1),
                       TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 30, 0.1));

      // Adjust corners detected in IR image to where they would appear in the depth image
      /// @todo Currently assuming a constant offset for all depths
      for (int i = 0; i < corners.size(); ++i)
        corners[i] += ir_depth_offset;

      pats.push_back(pat);
      points.push_back(corners);
    }


  // calibrate monocular camera
  Mat camMatrix;
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;
  double rp_err;
  rp_err = calibrateCamera(pats, points, Size(COLS,ROWS), camMatrix, distCoeffs,
                           rvecs, tvecs,
                           CV_CALIB_FIX_K3 | 
                           CV_CALIB_FIX_K2 | 
                           CV_CALIB_FIX_K1 | 
                           CV_CALIB_ZERO_TANGENT_DIST |
                           //CV_CALIB_FIX_PRINCIPAL_POINT |
                           CV_CALIB_FIX_ASPECT_RATIO
                          );

  printf("\nCalibration results:\n");

  double *dptr = distCoeffs.ptr<double>(0);
  printf("\nDistortion coefficients \nFirst two (k1, k2) should be close to zero\n");
  printf("k1: %f \nk2: %f\n", dptr[0], dptr[1]);
  printf("\nLast three (t1,t2,k3) are zero by specification\n");
  printf("t1: %f \nt2: %f \nk3: %f\n", dptr[2], dptr[3], dptr[4]);

  std::cout << distCoeffs.rows << " " << distCoeffs.cols << std::endl;

  // print camera matrix
  printf("\nCamera matrix\n");
  dptr = camMatrix.ptr<double>(0);
  double f = *dptr;             // focal length
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
        printf("%f ",*dptr++);
      printf("\n");
    }
  printf("\nReprojection error = %f\n\n", rp_err);

  
  // Read in depth images, fit readings to computed depths

  fnum = 0;
  std::vector<cv::Vec2d> ls_src1;
  std::vector<double> ls_src2;
  //printf("Z\tr\n");
  while (1)
    {
      // Load raw depth readings
      char fname[1024];
      sprintf(fname,"%s/img_depth_%02d.png",fdir,fnum);
      Mat img_depth = imread(fname,-1);
      if (img_depth.data == NULL) break; // no data, not read, break out

      // Get corner points and extrinsic parameters
      const cv::Mat pattern(pats[fnum]); // 3-channel matrix view of vector<Point3f>
      vector<Point2f> &corners = points[fnum];
      cv::Mat rvec = rvecs[fnum];
      cv::Mat tvec = tvecs[fnum];
      cv::Mat rot3x3;
      cv::Rodrigues(rvec, rot3x3);

      // Transform object points into camera coordinates using (rvec, tvec)
      cv::Mat world_points;
      cv::Mat xfm(3, 4, cv::DataType<double>::type);
      cv::Mat xfm_rot = xfm.colRange(0,3);
      cv::Mat xfm_trans = xfm.col(3);
      rot3x3.copyTo(xfm_rot);
      tvec.reshape(1,3).copyTo(xfm_trans);
      cv::transform(pattern, world_points, xfm);
 
      for (int j = 0; j < corners.size(); ++j) {
        double Z = world_points.at<cv::Vec3f>(j)[2];   // actual depth
        double r = img_depth.at<uint16_t>(corners[j]); // sensor reading
        ls_src1.push_back(cv::Vec2d(-1.0, Z));
        ls_src2.push_back(Z*r);
        //printf("%.4f\t%.0f\n", Z, r);
      }

      fnum++;
    }

  cv::Mat depth_params;
  double A, B;
  double b; // baseline
  if (cv::solve(cv::Mat(ls_src1).reshape(1), cv::Mat(ls_src2), depth_params,
                DECOMP_LU | DECOMP_NORMAL)) {
    A = depth_params.at<double>(0);
    B = depth_params.at<double>(1);
    double f = camMatrix.ptr<double>()[0];
    b = SHIFT_SCALE * A / f;
    printf("A = %f\nB = %f\nBaseline between projector and depth camera = %f\n", A, B, b);
  }
  else {
    printf("**** Failed to solve least-squared problem ****\n");
  }

  // 
  // calibrate IR to RGB images
  //

  // read in rgb files
  fnum = 0;
  vector<vector<Point2f> > pointsRGB; // RGB corners
  printf("\n");
  while (1)
    {
      char fname[1024];
      sprintf(fname,"%s/img_rgb_%02d.png",fdir,fnum);
      Mat img = imread(fname,1);
      if (img.data == NULL) break; // no data, not read, break out

      vector<cv::Point2f> corners;
      bool ret = cv::findChessboardCorners(img,Size(crows,ccols),corners);

      if (ret)
        printf("Found corners in image %s\n",fname);
      else
        printf("*** Didn't find corners in image %s\n",fname);

      Mat gray;
      cv::cvtColor(img, gray, CV_RGB2GRAY);
      cv::cornerSubPix(gray, corners, Size(5,5), Size(-1,-1),
                       TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 30, 0.1));

      pointsRGB.push_back(corners);

      fnum++;
    }

  // calibrate monocular camera
  Mat camMatrixRGB;
  Mat distCoeffsRGB = Mat::zeros(4,1,CV_64F);
  // initialize camera matrix
  camMatrixRGB = (Mat_<double>(3,3) << 1, 0, 320, 0, 1, 240, 0, 0, 1);

  rp_err = calibrateCamera(pats, pointsRGB, Size(COLS,ROWS), camMatrixRGB, distCoeffsRGB,
                           rvecs, tvecs,
                           //CV_CALIB_FIX_K1 |
                           //CV_CALIB_FIX_K2 |
                           //CV_CALIB_FIX_K3 |
                           //CV_CALIB_ZERO_TANGENT_DIST |
                           //CV_CALIB_FIX_PRINCIPAL_POINT |
                           CV_CALIB_FIX_ASPECT_RATIO
                          );

  // distortion results
  printf("\nCalibration results:\n");

  dptr = distCoeffsRGB.ptr<double>(0);
  printf("\nDistortion coefficients \nFirst two (k1, k2) should be close to zero\n");
  printf("k1: %f \nk2: %f\n", dptr[0], dptr[1]);
  printf("\nLast three (t1,t2,k3) are zero by specification\n");
  printf("t1: %f \nt2: %f \nk3: %f\n", dptr[2], dptr[3], dptr[4]);

  // print camera matrix
  printf("\nCamera matrix\n");
  dptr = camMatrixRGB.ptr<double>(0);
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
        printf("%f ",*dptr++);
      printf("\n");
    }
  printf("\nReprojection error = %f\n\n", rp_err);

  // stereo calibration between IR and RGB
  Mat R,T,E,F;
  rp_err = stereoCalibrate(pats,points,pointsRGB,camMatrix,distCoeffs,
                           camMatrixRGB,distCoeffsRGB,Size(crows,ccols),
                           R,T,E,F);
  
  dptr = T.ptr<double>(0);
  printf("\nTranslation between depth and RGB sensors (m):\n");
  for (int i=0; i<3; i++)
    printf("%f ",dptr[i]);
  printf("\n");

  printf("\nRotation matrix:\n");
  dptr = R.ptr<double>(0);
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
        printf("%f ",*dptr++);
      printf("\n");
    }
  printf("\nReprojection error = %f\n\n", rp_err);


  Matrix4d Q,S;                 // transformations
  Matrix<double,3,4> P;         // projection

  // from u,v,d of depth camera to XYZ
  double *cptr = camMatrix.ptr<double>(0);
  Q << 1, 0, 0,    -cptr[2],  // -cx
       0, 1, 0,    -cptr[5],  // -cy
       0, 0, 0,     cptr[0],  // focal length
       0, 0, 1.0/b, 0;        // baseline

  // from XYZ of depth camera to XYZ of RGB camera
  dptr = R.ptr<double>(0);
  double *tptr = T.ptr<double>(0);
  S << dptr[0], dptr[1], dptr[2], tptr[0],
       dptr[3], dptr[4], dptr[5], tptr[1],
       dptr[6], dptr[7], dptr[8], tptr[2],
       0,       0,       0,       1;

  // from XYZ to u,v in RGB camera
  cptr = camMatrixRGB.ptr<double>(0);
  P << cptr[0], 0,       cptr[2], 0,
       0,       cptr[4], cptr[5], 0,
       0,       0,       1,       0;

  Matrix<double,3,4> D = P*S*Q;
  std::cout << "Transform matrix:" << std::endl << D << std::endl << std::endl;

#if 0
  // a little test, should give a result that are close
  Vector4d p;
  Vector3d q;
  p << uu, vv, disp, 1;
  q = D*p;
  
  cout << "Test the transform, input:" << endl;
  cout << p.transpose() << endl << "Output (should be ~455,380):" << endl;
  cout << q.transpose()/q[2] << endl << endl;
#endif

  //
  // create rectified disparity images and save
  //

  // set up gamma for depth colorizer
  for (int i = 0; i < 2048; i++)
  {
    float v = i / 2048.0;
    v = powf (v, 3) * 6;
    t_gamma[i] = v * 6 * 256;
  }


  fnum = 0;
  cout << "Rectifying disparity images" << endl;
  while (1)
    {
      char fname[1024];
      sprintf(fname,"%s/img_depth_%02d.png",fdir,fnum);
      Mat img = imread(fname,-1);
      if (img.data == NULL) break; // no data, not read, break out

      sprintf(fname,"%s/img_rgb_%02d.png",fdir,fnum);
      Mat imgRGB = imread(fname,1);
      if (imgRGB.data == NULL) break; // no data, not read, break out

      // Rectify RGB image
      cv::Mat imgRgbRect;
      cv::undistort(imgRGB, imgRgbRect, camMatrixRGB, distCoeffsRGB);

      uint16_t *dptr = img.ptr<uint16_t>(0);

      Mat imgr  = Mat::zeros(ROWS,COLS,CV_16UC1); // depth image mapped to RGB image
      Mat imgrc = Mat::zeros(ROWS,COLS,CV_8UC3); // depth image mapped to RGB image, colorized
      Mat imgc  = Mat::zeros(ROWS,COLS,CV_8UC3); // original depth image colorized
      Mat imgdc = Mat::zeros(ROWS,COLS,CV_8UC3); // RGB mapped to depth image

      uint16_t *rptr = imgr.ptr<uint16_t>(0);
      uint8_t *rcptr = imgrc.ptr<uint8_t>(0);
      uint8_t *cptr  = imgc.ptr<uint8_t>(0);
      uint8_t *dcptr = imgdc.ptr<uint8_t>(0);
      //uint8_t *rgbptr = imgRGB.ptr<uint8_t>(0);
      uint8_t *rgbptr = imgRgbRect.ptr<uint8_t>(0);

      int k=0;
      for (int i=0; i<ROWS; i++)
        for (int j=0; j<COLS; j++,k++) // k is depth image index
          {
            double d = shift2disp(dptr[k], B);
            if (d <= 0)
              d = 0.0;          // not valid
            Vector4d p;
            p << j,i,d,1;
            Vector3d q;
            q = D*p;
            int u = (int)(q[0]/q[2]+0.5);
            int v = (int)(q[1]/q[2]+0.5);
            setDepthColor(&cptr[3*(i*COLS+j)],dptr[k]);
            if (u < 0 || v < 0 || u >= COLS || v >= ROWS)
              continue;
            int disp = (int)(d*16+0.499);
            int kk = v*COLS+u;  // kk is corresponding RGB image index
            if (rptr[kk] < disp) // z-buffer check
              {
                rptr[kk] = disp;
                setDepthColor(&rcptr[3*kk],dptr[k]);
              }
            if (d != 0.0)
              memcpy(&dcptr[3*k],&rgbptr[3*kk],3); // RGB mapped to depth image
          }

      sprintf(fname,"%s/img_depth_rect_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgr);
      sprintf(fname,"%s/img_depth_rect_color_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgrc);
      sprintf(fname,"%s/img_depth_color_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgc);
      sprintf(fname,"%s/img_rgb_mapped_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgdc);
      sprintf(fname,"%s/img_rgb_rect_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgRgbRect);

      fnum++;
    }
  

  return 0;
}
