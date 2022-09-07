#include "qtOpencvCore.h"

namespace QtOpencvCore
{
QImage img2qimg(cv::Mat const& img)
{
    cv::Mat tmp_img;
    // convert the color to RGB (OpenCV uses BGR)
    switch (img.type())
    {
    case CV_8UC1:
        cv::cvtColor(img, tmp_img, cv::COLOR_GRAY2BGR);
        break;
    case CV_8UC3:
        cv::cvtColor(img, tmp_img, cv::COLOR_BGR2RGB);
        break;
    default:
        return QImage();
    }

    // return the QImage
    return QImage(static_cast<uchar*>(tmp_img.data), tmp_img.cols, tmp_img.rows, tmp_img.step,
        QImage::Format_RGB888);
}

QImage img2qimgRaw(cv::Mat const& img)
{
    // create a temporary image (parameter img is const!)
    cv::Mat dst;

    // convert the color to RGB (OpenCV uses BGR)
    switch (img.type())
    {
    case CV_8UC1:
        cv::cvtColor(img, dst, cv::COLOR_GRAY2BGR);
        break;
    default:
        dst = img.clone();
        break;
    }

    int h = dst.rows;
    int w = dst.cols;
    int channels = dst.channels();
    QImage qimg = QImage(w, h, QImage::Format_RGB32);
    uchar* data = dst.data;

    for (int y = 0; y < h; y++, data += dst.step)
    {
        for (int x = 0; x < w; x++)
        {
            uchar r, g, b, a;
            r = g = b = a = 0;
            if (channels == 1)
            {
                r = data[x * channels];
                g = data[x * channels];
                b = data[x * channels];
            }
            else if (channels == 3 || channels == 4)
            {
                r = data[x * channels + 2];
                g = data[x * channels + 1];
                b = data[x * channels];
            }

            if (channels == 4)
            {
                a = data[x * channels + 3];
                qimg.setPixel(x, y, qRgba(r, g, b, a));
            }
            else
            {
                qimg.setPixel(x, y, qRgb(r, g, b));
            }
        }
    }
    return qimg;
}

std::vector<std::string>& qstrList2strList(
    QStringList const& qStringList, std::vector<std::string>& strList)
{
    for (QStringList::const_iterator it = qStringList.begin(); it != qStringList.end(); it++)
    {
        strList.push_back(qstr2str((*it)));
    }
    return strList;
}

std::string qstr2str(QString const& qstr)
{
    // return the converted QString (now std string)
    return qstr.toStdString();
}

QString str2qstr(std::string const& str)
{
    // return the converted std string (now QString)
    return QString::fromStdString(str);
}

cv::Rect qRect2Rect(QRectF const& r) { return cv::Rect(r.x(), r.y(), r.width(), r.height()); }

cv::RotatedRect qRect2RotatedRect(QRectF const& r)
{
    cv::Rect cvRect = qRect2Rect(r);
    return cv::RotatedRect(cv::Point2f(cvRect.x + cvRect.width / 2, cvRect.y + cvRect.height / 2),
        cv::Size2f(cvRect.width, cvRect.height), 180.0);
}

QPointF point2qpoint(cv::Point const& p) { return QPointF(p.x, p.y); }

cv::Point qpoint2point(QPointF const& p) { return cv::Point(p.x(), p.y()); }

cv::Mat qimg2img(const QImage& qimg)
{
    cv::Mat img;
    img = cv::Mat(
        qimg.height(), qimg.width(), CV_8UC4, const_cast<uchar*>(qimg.bits()), qimg.bytesPerLine());
    return img;
}

} // namespace QtOpencvCore
