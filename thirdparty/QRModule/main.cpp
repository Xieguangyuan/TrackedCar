#include <opencv2/opencv.hpp>
#include "src/qrscanner.hpp"

int main(int argc, char *argv[])
{
    QRSCanner myScanner;
    std::vector<decodedObject> decodeOB;

    cv::VideoCapture cap(0);

    while (true)
    {
        cv::Mat tmpMat;
        cap >> tmpMat;

        decodeOB = myScanner.QRCodeDecoder(tmpMat);
        tmpMat = myScanner.QRCodeDrawer(decodeOB, tmpMat);
        for (size_t i = 0; i < decodeOB.size(); i++)
        {
            std::cout << decodeOB[i].data << "\n";
        }

        imshow("test", tmpMat);
        cv::waitKey(10);
    }
}
