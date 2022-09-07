/*****************************************************************************
 * Copyright (c) 2011-2014 The FIMTrack Team as listed in CREDITS.txt        *
 * http://fim.uni-muenster.de                                             	 *
 *                                                                           *
 * This file is part of FIMTrack.                                            *
 * FIMTrack is available under multiple licenses.                            *
 * The different licenses are subject to terms and condition as provided     *
 * in the files specifying the license. See "LICENSE.txt" for details        *
 *                                                                           *
 *****************************************************************************
 *                                                                           *
 * FIMTrack is free software: you can redistribute it and/or modify          *
 * it under the terms of the GNU General Public License as published by      *
 * the Free Software Foundation, either version 3 of the License, or         *
 * (at your option) any later version. See "LICENSE-gpl.txt" for details.    *
 *                                                                           *
 * FIMTrack is distributed in the hope that it will be useful,               *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
 * GNU General Public License for more details.                              *
 *                                                                           *
 *****************************************************************************
 *                                                                           *
 * For non-commercial academic use see the license specified in the file     *
 * "LICENSE-academic.txt".                                                   *
 *                                                                           *
 *****************************************************************************
 *                                                                           *
 * If you are interested in other licensing models, including a commercial-  *
 * license, please contact the author at fim@uni-muenster.de      			 *
 *                                                                           *
 *****************************************************************************/

#ifndef QTOPENCVCORE_H
#define QTOPENCVCORE_H

#include <QImage>
#include <QSize>
#include <QString>
#include <QStringList>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <string>

#include <opencv2/opencv.hpp>

namespace QtOpencvCore
{
QImage img2qimg(cv::Mat const& img);
QImage img2qimgRaw(cv::Mat const& img);
cv::Mat qimg2img(QImage const& qimg);

std::vector<std::string>& qstrList2strList(
    QStringList const& qStringList, std::vector<std::string>& strList);

std::string qstr2str(QString const& qstr);
QString str2qstr(std::string const& str);
cv::Rect qRect2Rect(QRectF const& r);
cv::RotatedRect qRect2RotatedRect(QRectF const& r);
QPointF point2qpoint(cv::Point const& p);
cv::Point qpoint2point(QPointF const& p);
}

#endif // QTOPENCVCORE_H
