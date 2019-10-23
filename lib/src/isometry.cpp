// MODIFIED TO ALLOW ESTIMATION OF ISOMETRIES
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#include "habitrack/isometry.h"

#include <cmath>
/* #include <iostream> */

namespace cv
{
Mat estimateIsometry2D(InputArray _from, InputArray _to, OutputArray _inliers,
        double ransacReprojThreshold, size_t maxIters, double confidence,
        size_t refineIters)
{
    // BOILERPLATE CODE
    Mat from = _from.getMat(), to = _to.getMat();
    const int count = from.checkVector(2);
    bool result = false;
    Mat H;

    CV_Assert( count >= 0 && to.checkVector(2) == count );

    if (from.type() != CV_32FC2 || to.type() != CV_32FC2)
    {
        Mat tmp1, tmp2;
        from.convertTo(tmp1, CV_32FC2);
        from = tmp1;
        to.convertTo(tmp2, CV_32FC2);
        to = tmp2;
    }
    // convert to N x 1 vectors
    from = from.reshape(2, count);
    to = to.reshape(2, count);

    Mat inliers;
    if(_inliers.needed())
    {
        _inliers.create(count, 1, CV_8U, -1, true);
        inliers = _inliers.getMat();
    }

    // run robust estimation
    Ptr<PointSetRegistrator::Callback> cb = makePtr<Isometry2DEstimatorCallback>();
    result = createRANSACPointSetRegistrator(cb, 2, ransacReprojThreshold,
        confidence, static_cast<int>(maxIters))->run(from, to, H, inliers);

    if(result && count > 2 && refineIters)
    {
        // reorder to start with inliers
        compressElems(from.ptr<Point2f>(), inliers.ptr<uchar>(), 1, count);
        int inliers_count = compressElems(to.ptr<Point2f>(), inliers.ptr<uchar>(), 1, count);
        if(inliers_count > 0)
        {
            Mat src = from.rowRange(0, inliers_count);
            Mat dst = to.rowRange(0, inliers_count);
            // H is
            //     a -b tx
            //     b  a ty
            // Hvec model for LevMarq is
            //     (alpha, tx, ty)
            //  where alpha is the angle for the rotation matrix
            double *Hptr = H.ptr<double>();
            double angle = std::atan2(Hptr[3], Hptr[0]);
            double Hvec_buf[3] = {angle, Hptr[2], Hptr[5]};
            Mat Hvec (3, 1, CV_64F, Hvec_buf);
            LMSolver::create(makePtr<Isometry2DRefineCallback>(src, dst),
                static_cast<int>(refineIters))->run(Hvec);

            // update H with refined parameters
            Hptr[0] = Hptr[4] = std::cos(Hvec_buf[0]);
            Hptr[1] = -std::sin(Hvec_buf[0]);
            Hptr[2] = Hvec_buf[1];
            Hptr[3] = std::sin(Hvec_buf[0]);
            Hptr[5] = Hvec_buf[2];
        }
    }

    if (!result)
    {
        H.release();
        if(_inliers.needed())
        {
            inliers = Mat::zeros(count, 1, CV_8U);
            inliers.copyTo(_inliers);
        }
    }

    return H;
}

int Isometry2DEstimatorCallback::runKernel( InputArray _m1, InputArray _m2,
        OutputArray _model ) const
{
    Mat m1 = _m1.getMat(), m2 = _m2.getMat();
    const Point2f* from = m1.ptr<Point2f>();
    const Point2f* to   = m2.ptr<Point2f>();
    _model.create(2, 3, CV_64F);
    Mat M_mat = _model.getMat();
    double *M = M_mat.ptr<double>();

    // we need only 2 points to estimate transform
    double x1 = from[0].x;
    double y1 = from[0].y;
    double x2 = from[1].x;
    double y2 = from[1].y;

    double X1 = to[0].x;
    double Y1 = to[0].y;
    double X2 = to[1].x;
    double Y2 = to[1].y;

    // calculate mean and shift to origin
    double meanFromx = (x1 + x2) / 2.0;
    double meanFromy = (y1 + y2) / 2.0;
    double meanFromX = (X1 + X2) / 2.0;
    double meanFromY = (Y1 + Y2) / 2.0;
    x1 -= meanFromx; x2 -= meanFromx;
    y1 -= meanFromy; y2 -= meanFromy;
    X1 -= meanFromX; X2 -= meanFromX;
    Y1 -= meanFromY; Y2 -= meanFromY;

    double normFrom1 = std::sqrt(x1*x1 + y1*y1);
    double normFrom2 = std::sqrt(x2*x2 + y2*y2);
    double normTo1 = std::sqrt(X1*X1 + Y1*Y1);
    double normTo2 = std::sqrt(X2*X2 + Y2*Y2);

    // calculate mean angle
    // mean value of cosinus between the vectors determined by the dot product
    double cosMean = ((x1 * X1 + y1 * Y1)/(normFrom1 * normTo1)
        + (x2 * X2 + y2 * Y2)/(normFrom2 * normTo2)) / 2.0;

    // mean value of sinus between the vectors determined by the cross product
    double sinMean = (std::abs(x1 * Y1 - y1 * X1)/(normFrom1 * normTo1)
        + std::abs(x2 * Y2 - y2 * X2)/(normFrom2 * normTo2)) / 2.0;

    // translation determined by
    // (midpoint of "TO" points) - Rotation * (midpoint of "FROM" points)
    double tx = meanFromX - (cosMean * meanFromx - sinMean * meanFromy);
    double ty = meanFromY - (sinMean * meanFromx + cosMean * meanFromy);

    // set model, rotation part is antisymmetric
    // | M0   M1   M2 |
    // | M3   M4   M5 |

    M[0] = M[4] = cosMean;
    M[1] = -sinMean;
    M[2] = tx;
    M[3] = sinMean;
    M[5] = ty;
    return 1;
}

void Isometry2DEstimatorCallback::computeError(InputArray _m1, InputArray _m2,
    InputArray _model, OutputArray _err) const
{
    Mat m1 = _m1.getMat(), m2 = _m2.getMat(), model = _model.getMat();
    const Point2f* from = m1.ptr<Point2f>();
    const Point2f* to   = m2.ptr<Point2f>();
    const double* F = model.ptr<double>();

    int count = m1.checkVector(2);
    CV_Assert( count > 0 );

    _err.create(count, 1, CV_32F);
    Mat err = _err.getMat();
    float* errptr = err.ptr<float>();
    // transform matrix to floats
    float F0 = (float)F[0], F1 = (float)F[1], F2 = (float)F[2];
    float F3 = (float)F[3], F4 = (float)F[4], F5 = (float)F[5];

    for(int i = 0; i < count; i++ )
    {
        const Point2f& f = from[i];
        const Point2f& t = to[i];

        float a = F0*f.x + F1*f.y + F2 - t.x;
        float b = F3*f.x + F4*f.y + F5 - t.y;

        errptr[i] = a*a + b*b;
    }
}

bool Isometry2DEstimatorCallback::checkSubset(InputArray _ms1, InputArray _ms2,
    int count) const
{
    Mat ms1 = _ms1.getMat();
    Mat ms2 = _ms2.getMat();
    // check collinearity and also check that points are too close
    return !haveCollinearPoints(ms1, count) && !haveCollinearPoints(ms2, count);
}

Isometry2DRefineCallback::Isometry2DRefineCallback(InputArray _src, InputArray _dst)
{
    src = _src.getMat();
    dst = _dst.getMat();
}

bool Isometry2DRefineCallback::compute(InputArray _param, OutputArray _err,
        OutputArray _Jac) const

{
    int i, count = src.checkVector(2);
    Mat param = _param.getMat();
    _err.create(count*2, 1, CV_64F);
    Mat err = _err.getMat(), J;
    if( _Jac.needed())
    {
        _Jac.create(count*2, param.rows, CV_64F);
        J = _Jac.getMat();
        CV_Assert(J.isContinuous() && J.cols == 3);
    }

    const Point2f* M = src.ptr<Point2f>();
    const Point2f* m = dst.ptr<Point2f>();
    const double* h = param.ptr<double>();
    double* errptr = err.ptr<double>();
    double* Jptr = J.data ? J.ptr<double>() : 0;

    /* double error = 0.0; */
    for( i = 0; i < count; i++ )
    {
        double Mx = M[i].x, My = M[i].y;
        double xi = std::cos(h[0])*Mx - std::sin(h[0])*My + h[1];
        double yi = std::sin(h[0])*Mx + std::cos(h[0])*My + h[2];
        errptr[i*2] = xi - m[i].x;
        errptr[i*2+1] = yi - m[i].y;
        /* error = errptr[i*2]*errptr[i*2] +  errptr[i*2+1]*errptr[i*2+1]; */

        /*
        Jacobian is:

            {-sin(a)*x - cos(a)*y, 1, 0}
            {cos(a)*x - sin(a)*y, 0, 1}
        */
        if(Jptr)
        {
            Jptr[0] = -std::sin(h[0])*Mx - std::cos(h[0])*My;
            Jptr[1] = 1.0; Jptr[2] = 0.0;

            Jptr[3] = std::cos(h[0])*Mx - std::sin(h[0])*My;
            Jptr[4] = 0.0; Jptr[5] = 1.0;

            // move pointer
            Jptr += 3*2;
        }
    }

    return true;
}

} // namespace cv

