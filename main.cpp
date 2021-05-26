#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <fstream>
#include <wiringPi.h>
#include <queue>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "HMC5883l.h"
#include "YJSP_Schedule.hpp"
#include "saftyMat.hpp"
#include "pca9685.h"
#include "lcd1602.h"
#include "filter.h"
#include "VL53L1XDev.hpp"
#include "thirdparty/RuModule/SRC/_Excutable/Drive_Json.hpp"
#include "thirdparty/QRModule/src/qrscanner.hpp"
#include "thirdparty/RuModule/SRC/_Excutable/Drive_Json.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/CameraDrive/Drive_V4L2Reader.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/VisionAIDrive/Drive_OpenCVDN.hpp"
#include "thirdparty/RuModule/SRC/MessageController/MessageController.hpp"
using namespace cv;
using json = nlohmann::json;
std::queue<std::string> dataQueue;
float MAX_Area = 0;
float AreaInput = 0;
float AreaOutput = 0;
float YawOutput = 0;
float CXOutput = 0;
float CXOutputILast = 0;
float CXOutputDLast = 0;
double CXInput = 280;
int cx = 0;
int cy = 0;
float compass_x_horizontal = 0;
float compass_y_horizontal = 0;
void configWrite(const char *configDir, const char *Target, double obj);
double configSettle(const char *configDir, const char *Target);
void PIDCaclOut(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                float &last_I_Data, float &last_D_Data,
                float P_Gain, float I_Gain, float D_Gain, float I_Max);

int main(int argc, char *argv[])
{

    CVInferConfig InferConfigs;
    InferConfigs.Confidence_Threshold = 0.8;
    InferConfigs.File_args1 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.xml";
    InferConfigs.File_args2 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.bin";
    CVInferEngine MyEngine(InferConfigs);

    int rc = lcd1602Init(1, 0x27);
    int argvs = 0;

    while ((argvs = getopt(argc, argv, "RTMFYK")) != -1)
    {
        switch (argvs)
        {
        case 'R':
        {
            cv::VideoCapture cap(0);
            cv::Mat src;
            cv::Mat tmp;
            while (true)
            {
                cap >> src;
                resize(src, src, cv::Size(640, 480));
                rotate(src, src, cv::ROTATE_180);
                cvtColor(src, tmp, COLOR_BGR2HSV);
                //inRange(tmp, Scalar(90, 160, 0), Scalar(100, 255, 255), tmp); //blue               Scalar(LOW_H,LOW_S,LOW_V),Scalar(HIGH_H,HIGH_S,HIGH_V)
                //inRange(tmp, Scalar(60, 80, 70), Scalar(90, 140, 255), tmp); //green
                inRange(tmp, Scalar(150, 160, 0), Scalar(179, 255, 255), tmp); //red

                erode(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));    //腐蚀
                dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(20, 20))); //膨胀

                std::vector<std::vector<cv::Point>> contours;
                findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                std::cout << contours.size() << "\n";
                MAX_Area = 0;
                int cx = 0;
                int cy = 0;
                int upY = INT_MAX, lowY = 0, upX, lowX;
                for (int i = 0; i < contours.size(); i++)
                {
                    if (contourArea(contours[i]) > 100)
                    {
                        if (contourArea(contours[i]) > MAX_Area)
                        {
                            MAX_Area = contourArea(contours[i]);
                            for (int j = 0; j < contours[i].size(); j++)
                            {
                                if (contours[i][j].y > lowY)
                                {
                                    lowY = contours[i][j].y;
                                    lowX = contours[i][j].x;
                                }
                                if (contours[i][j].y < upY)
                                {
                                    upY = contours[i][j].y;
                                    upX = contours[i][j].x;
                                }
                            }
                            std::vector<cv::Moments> mu(contours.size());
                            mu[i] = moments(contours[i], false);
                            cx = mu[i].m10 / mu[i].m00;
                            cy = mu[i].m01 / mu[i].m00;
                        }
                    }
                }
                CXInput = cx - 280;
                AreaInput = 50000.f - MAX_Area;
                PIDCaclOut(CXInput, CXInput, CXInput, CXOutput, CXOutputILast, CXOutputDLast, 0.5, 0, 0, 100.f);
                PIDCaclOut(AreaInput, AreaInput, AreaInput, AreaOutput, CXOutputILast, CXOutputILast, 0.5, 0, 0, 100.f);
                std::cout
                    << "AreaOutput =" << AreaOutput << "\n";
                std::cout
                    << "CXOutput =" << CXOutput << "\n";

                // std::cout << "low = (" << lowX << ", " << lowY << ")" << std::endl
                //           << "up  = (" << upX << ", " << upY << ")" << std::endl;
                // std::cout << "cx:" << cx << ","
                //           << "cy" << cy << std::endl;
                circle(src, Point(cx, cy), 10, Scalar(255, 0, 255));

                imshow("Window", src);
                imshow("test", tmp);
                waitKey(10);
            }
        }
        break;
        case 'M':
        {
            YJSP_AP::YJSP SPGO;
            SPGO.YJSP_Init();
            SPGO.MPUThreadREG();
            SPGO.RCThreadREG();
            SPGO.GPSThreadRDG();
            SPGO.OnRCDataCome([](int *I) {

            });
            SPGO.ESCThreadREG();
            std::thread QRcamer = std::thread([&] {
                cv::TickMeter timeDec;
                timeDec.reset();
                double fps;
                double count = 0;
                cv::VideoCapture cap(0);
                cap.set(cv::CAP_PROP_FPS, 30);
                cv::namedWindow("Window", cv::WINDOW_NORMAL);
                cv::setWindowProperty("Window", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
                cv::Mat src;
                while (true)
                {
                    timeDec.start();
                    count++;
                    cap >> src;
                    timeDec.stop();
                    fps = count / timeDec.getTimeSec();
                    // std::cout << "FPS: " << fps << "\n";
                    if (count > 30)
                    {
                        count = 0;
                        timeDec.reset();
                    }

                    cv::putText(src, cv::format(" FPS %.2f", fps), cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2, 8);

                    imshow("Window", src);
                    waitKey(1);
                }
            });
            SPGO.DEBUGThreadREG();
            usleep(-1);
        }
        break;
        case 'F':
        {
            V4L2Tools::V4L2Drive cap("/dev/video0", {.ImgWidth = 256,
                                                     .ImgHeight = 192,
                                                     .FrameBuffer = 1,
                                                     .Is_fastMode = true,
                                                     .PixFormat = V4L2_PIX_FMT_YUYV});
            cap.V4L2Control(V4L2_CID_ZOOM_ABSOLUTE, 0x8005);
            usleep(50000);
            cap.V4L2Control(V4L2_CID_ZOOM_ABSOLUTE, 0x8801);
            cv::namedWindow("test", cv::WINDOW_NORMAL);
            cv::setWindowProperty("test", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            unsigned char *tmpDataBGR = cap.RGB24DataInit();
            while (true)
            {
                unsigned char *tmpData = cap.V4L2Read();
                cap.yuyv2rgb24(tmpData, tmpDataBGR, 256, 192);
                cv::Mat tmpMat = cv::Mat(192, 256, CV_8UC3, tmpDataBGR);

                cv::resize(tmpMat, tmpMat, cv::Size(640, 480));
                //cv::rotate(tmpMat, tmpMat, cv::ROTATE_180);

                imshow("test", tmpMat);
                cv::waitKey(10);
            }
        }
        break;
        case 'T':
        {
            cv::TickMeter timeDec;
            timeDec.reset();
            double fps;
            double count = 0;
            cv::VideoCapture cap(0);
            cap.set(cv::CAP_PROP_FPS, 30);
            cv::namedWindow("Window", cv::WINDOW_NORMAL);
            cv::setWindowProperty("Window", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            cv::Mat src;
            while (true)
            {
                timeDec.start();
                count++;
                cap >> src;
                timeDec.stop();
                fps = count / timeDec.getTimeSec();
                // std::cout << "FPS: " << fps << "\n";
                if (count > 30)
                {
                    count = 0;
                    timeDec.reset();
                }

                cv::putText(src, cv::format(" FPS %.2f", fps), cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2, 8);
                imshow("Window", src);
                waitKey(1);
            }
        }
        break;
        case 'Y':
        {
            int Last = 0;
            int now = 0;
            int nowL = 0;
            double speed = 0.f;
            int stime = 0;
            int etime = 0;
            VL53L1XDevice Test;
            Test.begin("/dev/i2c-1", 0x29);
            Test.startMeasurement(0);
            Test.writeRegister(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, 0x33);
            Test.~VL53L1XDevice();
            Test.begin("/dev/i2c-1", 0x33);
            sleep(2);

            while (true)
            {
                stime = micros();
                if (Test.newDataReady())
                {
                    now = (int)(Test.getDistance() / 10.f);

                    nowL = nowL * 0.7 + now * 0.3;
                    speed = (nowL - Last) / (100000.f / 1000000.f);
                    std::cout << nowL << " ";
                    std::cout << speed << " ";
                    std::cout << etime - stime << " \n";
                    Last = nowL;
                }

                etime = micros();
                if (100000.f < (etime - stime))
                    usleep(50);
                else
                    usleep(100000.f - (etime - stime));
            }
            break;
        }

        case 'K':
        {
            //=======================================================//

            MessageController::WebSocketServer myserver;
            myserver.WebSocketServerInit().OnConnection([&](auto *req, auto *data) {
                                              std::string output = "4000";
                                              MessageController::WebSocketServer::dataSender(req, output, WCT_TXTDATA);
                                          })
                .OnMessage([&](auto *req, auto *data, auto opcode, auto len) {
                    std::string output;
                    if (strncmp(data, "4010", 4) == 0)
                    {
                        output = "4110";
                        MessageController::WebSocketServer::dataSender(req, output, WCT_TXTDATA);
                    }
                    if (strncmp(data, "4110", 4) == 0)
                    {
                        output = "4112";
                        MessageController::WebSocketServer::dataSender(req, output, WCT_TXTDATA);
                    }
                    if (strncmp(data, "4350", 4) == 0)
                    {
                        output = dataQueue.front();
                        MessageController::WebSocketServer::dataSender(req, output, WCT_TXTDATA);
                    }
                })
                .OnDisConnection([&](auto *req) {
                    return;
                })
                .OnError([&](auto *req, auto *data, auto opcode, auto len) {
                    std::string output = "4300/ERROR!";
                    MessageController::WebSocketServer::dataSender(req, output, WCT_TXTDATA);
                })
                .Run();
            //=======================================================//
            HMC5883L hmc5883l;
            YJSP_AP::YJSP MPU;
            // Initialize
            if (hmc5883l.hmc5883l_init(&hmc5883l) != HMC5883L_OKAY)
            {
                fprintf(stderr, "Error: %d\n", hmc5883l.SF._error);
                exit(1);
            }
            while (true)
            {
                // Read
                hmc5883l.hmc5883l_read(&hmc5883l);
                hmc5883l.SF.ClearCount++;
                if (hmc5883l.SF.ClearCount == 100)
                {
                    system("clear");
                    hmc5883l.SF.ClearCount = 0;
                }
                compass_x_horizontal = hmc5883l.SF.x * cos(MPU.SF.myData._uORB_Real__Roll * -0.0174533) + hmc5883l.SF.y * sin(MPU.SF.myData._uORB_Real_Pitch * 0.0174533) * sin(MPU.SF.myData._uORB_Real__Roll * -0.0174533) - hmc5883l.SF.z * cos(MPU.SF.myData._uORB_Real_Pitch * 0.0174533) * sin(MPU.SF.myData._uORB_Real__Roll * -0.0174533);
                compass_y_horizontal = hmc5883l.SF.y * cos(MPU.SF.myData._uORB_Real_Pitch * 0.0174533) + hmc5883l.SF.z * sin(MPU.SF.myData._uORB_Real_Pitch * 0.0174533);

                std::cout << "\033[20A";
                std::cout << "\033[K";
                std::cout << "X: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.x << " |"
                          << "Y: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.y << " |"
                          << "Z: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.z << " |\n";
                std::cout << "Scaled_X: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.x_scaled << " |"
                          << "Scaled_Y: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.y_scaled << " |"
                          << "Scaled_Z: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.z_scaled << " |\n";
                std::cout << "Orientation_Deg: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.orientation_deg << " |"
                          << "Orientation_Rad: " << std::setw(7) << std::setfill(' ') << hmc5883l.SF.orientation_rad << " |\n";
                std::cout << "Compass_X_Horizontal: " << std::setw(7) << std::setfill(' ') << compass_x_horizontal << " |"
                          << "Compass_Y_Horizontal: " << std::setw(7) << std::setfill(' ') << compass_y_horizontal << " |\n";

                std::string dataBuffer[20];
                if (dataQueue.size() > 2)
                    dataQueue.pop();
                json OutputJSON = {
                    {"type", 4200},
                    {"MAGX", hmc5883l.SF.x},
                    {"MAGY", hmc5883l.SF.y},
                    {"MAGZ", hmc5883l.SF.z},
                    {"DEG", hmc5883l.SF.orientation_deg},
                };
                dataBuffer[0] = OutputJSON.dump();
                dataQueue.push(MessageController::dataCreator(255, dataBuffer, 2));
                usleep(20000);
            }
            break;
        }
        }
    }
}
void PIDCaclOut(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                float &last_I_Data, float &last_D_Data,
                float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
    //P caculate
    outputData = P_Gain * inputDataP;
    //D caculate
    outputData += D_Gain * (inputDataD - last_D_Data);
    last_D_Data = inputDataD;
    //I caculate
    last_I_Data += inputDataD * I_Gain;
    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;
    //P_I_D Mix OUTPUT
    outputData += last_I_Data;
}

double configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}

void configWrite(const char *configDir, const char *Target, double obj)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    Configdata[Target] = obj;
    std::ofstream configs;
    configs.open(configDir);
    configs << std::setw(4) << Configdata << std::endl;
    configs.close();
}
