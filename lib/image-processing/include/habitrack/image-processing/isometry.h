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
#ifndef CV_ISOMETRY_H
#define CV_ISOMETRY_H

#include <habitrack/image-processing/ransac.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace cv
{
Mat estimateIsometry2D(InputArray _from, InputArray _to, OutputArray inliers = noArray(),
    double ransacReprojThreshold = 3, size_t maxIters = 2000, double confidence = 0.99,
    size_t refineIters = 10);

class Isometry2DEstimatorCallback : public PointSetRegistrator::Callback
{
public:
    int runKernel(InputArray _m1, InputArray _m2, OutputArray _model) const override;
    void computeError(
        InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err) const override;
    bool checkSubset(InputArray _ms1, InputArray _ms2, int count) const override;
};

class Isometry2DRefineCallback : public LMSolver::Callback
{
private:
    Mat src;
    Mat dst;

public:
    Isometry2DRefineCallback(InputArray _src, InputArray _dst);
    bool compute(InputArray _param, OutputArray _err, OutputArray _Jac) const override;
};
} // namespace cv

#endif // CV_ISOMETRY_H
