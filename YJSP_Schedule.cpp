#include "YJSP_Schedule.hpp"
int YJSP_AP::YJSP::YJSP_Init()
{
    wiringPiSetup();
    pinMode(29, OUTPUT);
    DF.fd = pca9685Setup(65, 0x40, 50);
    DF.myIbusDevice = new Ibus(DF.RCDeviceInfo);
    DF.MPUDevice = new RPiMPU9250(1, false, 1, 0x68, TF.TimeMax, 0);
    DF.GPSDevice = new GPSUart(DF.GPSDeviceInfo);
    SF.AccelCaliData[MPUAccelCaliX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Cali");
    SF.AccelCaliData[MPUAccelCaliY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Cali");
    SF.AccelCaliData[MPUAccelCaliZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Cali");
    SF.AccelCaliData[MPUAccelScalX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Scal");
    SF.AccelCaliData[MPUAccelScalY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Scal");
    SF.AccelCaliData[MPUAccelScalZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Scal");
    PF.PIDYawPGain = configSettle("../MPU9250Car.json", "_Car_PIDYawPGain");
    PF.PIDYawIGain = configSettle("../MPU9250Car.json", "_Car_PIDYawIGain");
    PF.PIDYawDGain = configSettle("../MPU9250Car.json", "_Car_PIDYawDGain");
    DF.MPUDevice->MPUCalibration(SF.AccelCaliData);
    DF.MPUDevice->MPUSensorsDataGet();
    DF.MPUDevice->ResetMPUMixAngle();
    RF.RCForwardMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIDDLE");
    RF.RCForwardMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIN");
    RF.RCForwardMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MAX");
    RF.RCHorizontalMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIDDLE");
    RF.RCHorizontalMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIN");
    RF.RCHorizontalMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MAX");
    RF.RCYawMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIDDLE");
    RF.RCYawMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIN");
    RF.RCYawMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MAX");
}

void YJSP_AP::YJSP::MPUThreadREG()
{
    TF.MPUThreading = std::thread([&] {
        while (true)
        {
            TF.TimeStart = micros();
            TF.TimeNext = TF.TimeStart - TF.TimeEnd;
            SF.myData = DF.MPUDevice->MPUSensorsDataGet();
            SF.Real_Yaw += ((float)SF.myData._uORB_MPU9250_G_Z / 65.5) / (float)TF.TimeMax;
            EF.SPEED_X = EF.SPEED_X + (int)SF.myData._uORB_Acceleration_X * (TF.TimeMax / 1000000.f);
            RF.RCYaw != 0 ? SF.Real_Yaw = 0 : SF.Real_Yaw = SF.Real_Yaw;
            if (RF.RC_Auto)
            {
                PF.ForInput = EF.SPEED_X - RF.Auto_Forward;
                PF.YawInput = (RF.Auto_Yaw + SF.myData._uORB_Gryo___Yaw) + SF.Real_Yaw * 8.f - RF.InputYaw * 15.f;
                PIDCacl(PF.ForInput, PF.ForInput, PF.ForInput, RF.TotalForward, PF.PIDForwardLastIData,
                        PF.PIDForwardLastDData, PF.PIDForwardPGain, PF.PIDForwardIGain, PF.PIDForwardDGain, 300.f);
            }
            else
            {
                PF.YawInput = (RF.RCYaw + SF.myData._uORB_Gryo___Yaw + SF.Real_Yaw * 8.f);
                RF.TotalForward = RF.RCForward;
            }

            PIDCacl(PF.YawInput, PF.YawInput, PF.YawInput, RF.TotalYaw, PF.PIDYawLastIData,
                    PF.PIDYawLastDData, PF.PIDYawPGain, PF.PIDYawIGain, PF.PIDYawDGain, 200.f);

            EF.SpeedA1 = RF.TotalForward + RF.TotalYaw;
            EF.SpeedA2 = RF.TotalForward - RF.TotalYaw;

            if (EF.SpeedA1 > 0 || EF.SpeedA2 > 0)
            {
                EF.SpeedA1TO = 100.f + (EF.SpeedA1 / 500.f) * 3900.f;
                EF.SpeedA2TO = 100.f + (EF.SpeedA2 / 500.f) * 3900.f;
                EF.SpeedA1TO = EF.SpeedA1TO > 4000 ? 4000 : EF.SpeedA1TO;
                EF.SpeedA2TO = EF.SpeedA2TO > 4000 ? 4000 : EF.SpeedA2TO;
                EF.SpeedA1TO = EF.SpeedA1TO < -4000 ? -4000 : EF.SpeedA1TO;
                EF.SpeedA2TO = EF.SpeedA2TO < -4000 ? -4000 : EF.SpeedA2TO;
            }
            else if (EF.SpeedA1 < 0 || EF.SpeedA2 < 0)
            {

                EF.SpeedA1TO = (EF.SpeedA1 / 500.f) * 4000.f;
                EF.SpeedA2TO = (EF.SpeedA2 / 500.f) * 4000.f;
                EF.SpeedA1TO = EF.SpeedA1TO > 4000 ? 4000 : EF.SpeedA1TO;
                EF.SpeedA2TO = EF.SpeedA2TO > 4000 ? 4000 : EF.SpeedA2TO;
                EF.SpeedA1TO = EF.SpeedA1TO < -4000 ? -4000 : EF.SpeedA1TO;
                EF.SpeedA2TO = EF.SpeedA2TO < -4000 ? -4000 : EF.SpeedA2TO;
            }

            TF.TimeEnd = micros();
            if (TF.TimeMax < ((TF.TimeEnd - TF.TimeStart) + TF.TimeNext) || (TF.TimeNext) < 0)
                usleep(50);
            else
                usleep(TF.TimeMax - (TF.TimeEnd - TF.TimeStart) - TF.TimeNext);
            TF.TimeEnd = micros();
        }
    });
}

void YJSP_AP::YJSP::GPSThreadRDG()
{

    TF.GPSThreading = std::thread([&] {
        DF.GPSDevice->GPSReOpen();
        while (true)
        {
            SF.myGPSData = DF.GPSDevice->GPSParse();
            if (!SF.myGPSData.DataUnCorrect && SF.myGPSData.lat != 0 && SF.myGPSData.lng != 0)
            {
                SF.GPSLatTrue = SF.myGPSData.lat;
                SF.GPSLngTrue = SF.myGPSData.lng;
            }
            usleep(150000);
        }
    });
}

void YJSP_AP::YJSP::PositionThreadREG()
{
    TF.NewPosThreading = std::thread([&] {
        int Last = 0;
        int now = 0;
        int nowL = 0;
        SF.speed_y = 0.f;
        int stime = 0;
        int etime = 0;
        VL53L1XDevice Testy;
        Testy.begin("/dev/i2c-1", 0x29);
        Testy.Setmode('L');
        Testy.startMeasurement(0);
        while (true)
        {
            stime = micros();
            if (Testy.newDataReady())
            {
                now = (int)(Testy.getDistance() / 10.f);
                SF.distance_Y = SF.distance_Y * 0.1 + now * 0.9;
                SF.speed_y = (now - Last) / (100000.f / 1000000.f);
                Last = now;
            }

            etime = micros();
            if (100000.f < (etime - stime))
                usleep(50);
            else
                usleep(100000.f - (etime - stime));
        }
    });
}

void YJSP_AP::YJSP::RCThreadREG()
{
    TF.RCThreading = std::thread([&] {
        while (true)
        {
            int lose = DF.myIbusDevice->IbusRead(RF.IbusData, 4000, 2);
            // for (size_t i = 0; i < 4; i++)
            // {
            //     RF.IbusData[i] = pt1FilterApply4(&DF.RCLPF[i], RF.IbusData[i], 5, (4000.f / 1000000.f));
            // }

            if (lose > 2)
            {
                SF.Real_Yaw = 0;
                RF.RCARM = true;
            }
            if (RF.IbusData[1] < RF.RCForwardMiddle + 10 && RF.IbusData[1] > RF.RCForwardMiddle - 10)
                RF.RCForward = 0;
            else
                RF.RCForward = RF.IbusData[1] - RF.RCForwardMiddle;

            if (RF.IbusData[3] < RF.RCYawMiddle + 10 && RF.IbusData[3] > RF.RCYawMiddle - 10)
                RF.RCYaw = 0;
            else
                RF.RCYaw = RF.IbusData[3] - RF.RCYawMiddle;
            if (RF.IbusData[9] < 1400 && RF.IbusData[9] > 900)
                RF.RCARM = true;
            else
                RF.RCARM = false;
            if (RF.IbusData[6] < 1400 && RF.IbusData[6] > 900)
                RF.RCLED = true;
            else
                RF.RCLED = false;
        }
    });
}

void YJSP_AP::YJSP::RCReset()
{
    Ibus myRC;
    int a;
    int lose;
    int Middle0, Middle1, Middle2, Middle3;
    double Max[10];
    double Min[10];
    double IbusDataTotal_0 = 0, IbusDataTotal_1 = 0, IbusDataTotal_2 = 0, IbusDataTotal_3 = 0;

    for (int i = 1; i < 500; i++)
    {
        lose = myRC.IbusRead(RF.IbusData, 4000, 2);
        if (lose != -1)
        {
            IbusDataTotal_0 += RF.IbusData[0];
            IbusDataTotal_1 += RF.IbusData[1];
            IbusDataTotal_2 += RF.IbusData[2];
            IbusDataTotal_3 += RF.IbusData[3];
        }
        else
            i--;
    }
    Middle0 = IbusDataTotal_0 / 500.f;
    Middle1 = IbusDataTotal_1 / 500.f;
    Middle2 = IbusDataTotal_2 / 500.f;
    Middle3 = IbusDataTotal_3 / 500.f;
    std::cout << "通道1-4的中间值校准完毕\n";
    std::cout << Middle0 << "   " << Middle1 << "   " << Middle2 << "   " << Middle3 << "\n";
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIDDLE", Middle0);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIDDLE", Middle1);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MIDDLE", Middle2);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIDDLE", Middle3);
    //======================================//
    for (size_t i = 0; i < 10; i++)
    {
        Max[i] = Middle0;
    }
    for (size_t i = 0; i < 10; i++)
    {
        Min[i] = Middle0;
    }

    int RCTuneFlag = -1;
    bool RCTuning = true;
    std::thread Tuning = std::thread([&] {
        while (RCTuning)
        {
            lose = myRC.IbusRead(RF.IbusData, 4000, 2);
            if (lose != -1)
            {
                Max[0] = Max[0] < RF.IbusData[0] ? RF.IbusData[0] : Max[0];
                Max[1] = Max[1] < RF.IbusData[1] ? RF.IbusData[1] : Max[1];
                Max[2] = Max[2] < RF.IbusData[2] ? RF.IbusData[2] : Max[2];
                Max[3] = Max[3] < RF.IbusData[3] ? RF.IbusData[3] : Max[3];

                Min[0] = Min[0] > RF.IbusData[0] ? RF.IbusData[0] : Min[0];
                Min[1] = Min[1] > RF.IbusData[1] ? RF.IbusData[1] : Min[1];
                Min[2] = Min[2] > RF.IbusData[2] ? RF.IbusData[2] : Min[2];
                Min[3] = Min[3] > RF.IbusData[3] ? RF.IbusData[3] : Min[3];
            }
        }
    });

    std::cin >> RCTuneFlag;
    RCTuning = false;
    Tuning.join();

    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MAX", Max[0]);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MAX", Max[1]);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MAX", Max[2]);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MAX", Max[3]);

    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIN", Min[0]);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIN", Min[1]);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MIN", Min[2]);
    configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIN", Min[3]);
}

void YJSP_AP::YJSP::ESCThreadREG()
{
    TF.ESCThreading = std::thread([&] {
        while (true)
        {
            if (!RF.RCARM)
            {
                if (EF.SpeedA1 > 0)
                {
                    pca9685PWMWrite(DF.fd, 9, 0, (int)abs(EF.SpeedA1TO));
                    pca9685PWMWrite(DF.fd, 8, 0, 100);
                }
                else
                {
                    pca9685PWMWrite(DF.fd, 8, 0, (int)abs(EF.SpeedA1TO));
                    pca9685PWMWrite(DF.fd, 9, 0, 100);
                }

                if (EF.SpeedA2 > 0)
                {
                    pca9685PWMWrite(DF.fd, 11, 0, (int)abs(EF.SpeedA2TO));
                    pca9685PWMWrite(DF.fd, 10, 0, 100);
                }
                else
                {
                    pca9685PWMWrite(DF.fd, 10, 0, (int)abs(EF.SpeedA2TO));
                    pca9685PWMWrite(DF.fd, 11, 0, 100);
                }
            }
            else
            {
                SF.Real_Yaw = 0;
                pca9685PWMWrite(DF.fd, 8, 0, 100);
                pca9685PWMWrite(DF.fd, 9, 0, 100);
                pca9685PWMWrite(DF.fd, 10, 0, 100);
                pca9685PWMWrite(DF.fd, 11, 0, 100);
            }
            if (!RF.RCLED)
                digitalWrite(29, HIGH);
            else
                digitalWrite(29, LOW);

            usleep(6000);
        }
    });
}

void YJSP_AP::YJSP::Servo_Raw(int pin, int angle)
{
    int Initial_angle = 180;
    double Magnification = 1.33;
    double Finall_angle = Initial_angle + angle * Magnification;
    pca9685PWMWrite(DF.fd, pin, 0, (int)Finall_angle);
    usleep(5);
    pca9685PWMReset(DF.fd);
}

void YJSP_AP::YJSP::UserInput(int Forward, int Yaw, int Input_Yaw)
{
    RF.Auto_Forward = Forward;
    RF.Auto_Yaw = Yaw;
    RF.InputYaw = Input_Yaw;
}

void YJSP_AP::YJSP::DEBUGThreadREG()
{
    while (true)
    {
        OnRCDataInComing(RF.IbusData);
        TF.ClearCount++;
        if (TF.ClearCount == 100)
        {
            system("clear");
            TF.ClearCount = 0;
        }
        std::cout << "\033[20A";
        std::cout << "\033[K";
        std::cout << "Accel Roll: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Accel__Roll << "|"
                  << "AccelPitch: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Accel_Pitch << "| \n";
        std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Gryo__Roll << "|"
                  << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Gryo_Pitch << "|"
                  << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Gryo___Yaw << "| \n";
        std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Real__Roll << "|"
                  << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Real_Pitch << "| \n";
        std::cout << "AccelX    : " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Acceleration_X << "cm/s2|"
                  << "AccelY    : " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Acceleration_Y << "cm/s2|"
                  << "AccelZ    : " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Acceleration_Z << "cm/s2| \n";
        std::cout << "Real_Yaw: " << std::setw(7) << std::setfill(' ') << (int)SF.Real_Yaw << "\n";
        for (int i = 0; i < 10; i++)
        {
            std::cout << RF.IbusData[i] << " ";
        }
        std::cout << "\n";
        std::cout << "RCLED      " << RF.RCLED << "           \n";
        std::cout << "RCARM      " << RF.RCARM << "           \n";
        std::cout << "RCForward: " << RF.RCForward << "           \n";
        std::cout << "RCYaw:     " << RF.RCYaw << "                \n";
        std::cout << "TotalYaw:  " << RF.TotalYaw << "                \n";
        std::cout << "speed A1   " << EF.SpeedA1TO << "               \n";
        std::cout << "speed A2   " << EF.SpeedA2TO << "               \n";

        std::cout << "satillites: " << SF.myGPSData.satillitesCount << "\n";
        std::cout << "DataError: " << SF.myGPSData.DataUnCorrect << "\n";
        std::cout << "lat: " << std::setprecision(9) << SF.GPSLatTrue << " \n";
        std::cout << "lng: " << std::setprecision(10) << SF.GPSLngTrue << " \n";

        usleep(20000);
    }
}

void YJSP_AP::YJSP::PIDCacl(float inputDataP, float inputDataI, float inputDataD, float &outputData,
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

double YJSP_AP::YJSP::configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}

double YJSP_AP::YJSP::configWrite(const char *configDir, const char *Target, double obj)
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
