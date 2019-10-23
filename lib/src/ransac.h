// WARNING: THIS IS HIGHLY REDUNDANT CODE DIRECTLY FROM THE THE OPENCV REPO
// BECAUSE OPENCV DOES NOT ALLOW INTERFACING ITS RANSAC MODULE
// COPIED COMMIT HASH: 8f15a609af
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
#ifndef CV_RANSAC_H
#define CV_RANSAC_H

#include <opencv2/core.hpp>

namespace cv
{
class PointSetRegistrator : public Algorithm
{
public:
    class Callback
    {
    public:
        virtual ~Callback() {}
        virtual int runKernel(InputArray m1, InputArray m2, OutputArray model) const = 0;
        virtual void computeError(InputArray m1, InputArray m2, InputArray model, OutputArray err) const = 0;
        virtual bool checkSubset(InputArray, InputArray, int) const { return true; }
    };

    virtual void setCallback(const Ptr<PointSetRegistrator::Callback>& cb) = 0;
    virtual bool run(InputArray m1, InputArray m2, OutputArray model, OutputArray mask) const = 0;
};

int RANSACUpdateNumIters(double p, double ep, int modelPoints, int maxIters);

class RANSACPointSetRegistrator : public PointSetRegistrator
{
public:
    RANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb=Ptr<PointSetRegistrator::Callback>(),
                              int _modelPoints=0, double _threshold=0, double _confidence=0.99, int _maxIters=1000);

    int findInliers( const Mat& m1, const Mat& m2, const Mat& model, Mat& err, Mat& mask, double thresh ) const;

    bool getSubset( const Mat& m1, const Mat& m2,
                    Mat& ms1, Mat& ms2, RNG& rng,
                    int maxAttempts=1000 ) const;

    bool run(InputArray _m1, InputArray _m2, OutputArray _model, OutputArray _mask) const CV_OVERRIDE;
    void setCallback(const Ptr<PointSetRegistrator::Callback>& _cb) CV_OVERRIDE;

    Ptr<PointSetRegistrator::Callback> cb;
    int modelPoints;
    double threshold;
    double confidence;
    int maxIters;
};

inline bool haveCollinearPoints(const Mat& m, int count)
{
    int j, k, i = count-1;
    const Point2f* ptr = m.ptr<Point2f>();

    // check that the i-th selected point does not belong
    // to a line connecting some previously selected points
    // also checks that points are not too close to each other
    for( j = 0; j < i; j++ )
    {
        double dx1 = ptr[j].x - ptr[i].x;
        double dy1 = ptr[j].y - ptr[i].y;
        for( k = 0; k < j; k++ )
        {
            double dx2 = ptr[k].x - ptr[i].x;
            double dy2 = ptr[k].y - ptr[i].y;
            if( fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
                return true;
        }
    }
    return false;
}

Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(
    const Ptr<PointSetRegistrator::Callback>& _cb, int _modelPoints, double _threshold,
    double _confidence, int _maxIters);

template<typename T> inline int compressElems(T* ptr, const uchar* mask, int mstep,
    int count)
{
    int i, j;
    for(i = j = 0; i < count; i++)
        if(mask[i*mstep])
        {
            if(i > j)
                ptr[j] = ptr[i];
            j++;
        }
    return j;
}

}

#endif // CV_RANSAC_H
