#pragma once
#include <iostream>
#include <zbar.h>
#include <opencv2/opencv.hpp>

struct decodedObject
{
    std::string type;
    std::string data;
    std::vector<cv::Point> location;
};

class QRSCanner
{
public:
    QRSCanner()
    {
        oimageScaner.reset(new zbar::ImageScanner());
        oimageScaner->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    }

    std::vector<decodedObject> QRCodeDecoder(cv::Mat RawDataInputBGR)
    {
        decodedObjects.clear();
        cv::cvtColor(RawDataInputBGR, TmpGrayMat, cv::COLOR_BGR2GRAY);
        zbar::Image image(RawDataInputBGR.cols, RawDataInputBGR.rows, "Y800", (uchar *)TmpGrayMat.data, RawDataInputBGR.cols * RawDataInputBGR.rows);
        int n = oimageScaner->scan(image);
        for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {
            decodedObject obj;
            obj.type = symbol->get_type_name();
            obj.data = symbol->get_data();
            for (int i = 0; i < symbol->get_location_size(); i++)
            {
                obj.location.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
            }
            decodedObjects.push_back(obj);
        }
        return decodedObjects;
    }

    static cv::Mat QRCodeDrawer(std::vector<decodedObject> decodedObject2, cv::Mat RawDataInputBGR)
    {
        for (int i = 0; i < decodedObject2.size(); i++)
        {
            std::vector<cv::Point> points = decodedObject2[i].location;
            std::vector<cv::Point> hull;
            if (points.size() > 4)
                cv::convexHull(points, hull);
            else
                hull = points;
            int n = hull.size();

            for (int j = 0; j < n; j++)
            {
                cv::line(RawDataInputBGR, hull[j], hull[(j + 1) % n], cv::Scalar(255, 0, 0), 1);
            }
        }
        return RawDataInputBGR;
    }

    ~QRSCanner()
    {
        oimageScaner.reset();
        oimageScaner.release();
    }

private:
    cv::Mat TmpGrayMat;
    std::vector<decodedObject> decodedObjects;
    std::unique_ptr<zbar::ImageScanner> oimageScaner;
};