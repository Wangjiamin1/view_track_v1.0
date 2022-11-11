#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <dirent.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "kcftracker.hpp"
#include "ViewLink.h"
#include "cmdline.h"
#include "pid.h"

using namespace std;
using namespace cv;

cv::Mat rgbFrame;
cv::Mat infraredFrame;
bool manualFlag = true;
double x = 600;
double y = 600;
int nFrames = 0;

cv::VideoCapture visibleCamera("rtspsrc location=rtsp://192.168.2.119:554/stream0 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink max-buffers=1 drop=true sync=false", cv::CAP_GSTREAMER);
cv::VideoCapture IRCamera("rtspsrc location=rtsp://192.168.2.119:554/stream1 latency=0 ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink max-buffers=1 drop=true sync=false", cv::CAP_GSTREAMER);
cv::VideoCapture showCamera = visibleCamera;
std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    // std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}
std::tm *gettm(std::time_t timestamp)
{
    std::time_t milli = timestamp /*+ (std::time_t)8*60*60*1000*/; //此处转化为东八区北京时间，如果是其它时区需要按需求修改
    auto mTime = std::chrono::milliseconds(milli);
    auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm *now = std::gmtime(&tt);
    printf("%4d年%02d月%02d日 %02d:%02d:%02d.%d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec, milli % 1000);
    return now;
}

//-------------------------------------------PID-----------------------------------------------------------

float refX = 0;
float mesX = 0;
float errX = refX - mesX;
float preErrX = errX;
float Kp = 6;
float Kd = 0.2;
float Ki = 0.0;
float dt = 0.03; // 30ms
float inte = 1;
float outX = 0;

float pidProc(float errX)
{
    inte += errX;
    float out = Kp * errX + Ki * inte + Kd * (errX - preErrX);

    preErrX = errX;
    // printf("pid:errx:%f pixel, out:%f pixel\n", errX, out);
    return out;
}

//------------------------------------------------------------------------------------------------

//------------------------------------------viewLink-------------------------------------------------------
bool g_bConnected = false;
int VLK_ConnStatusCallback(int iConnStatus, const char *szMessage, int iMsgLen, void *pUserParam)
{
    if (VLK_CONN_STATUS_TCP_CONNECTED == iConnStatus)
    {
        cout << "TCP Gimbal connected !!!" << endl;
        g_bConnected = true;
    }
    else if (VLK_CONN_STATUS_TCP_DISCONNECTED == iConnStatus)
    {
        cout << "TCP Gimbal disconnected !!!" << endl;
        g_bConnected = false;
    }
    else if (VLK_CONN_STATUS_SERIAL_PORT_CONNECTED == iConnStatus)
    {
        cout << "serial port connected !!!" << endl;
        g_bConnected = true;
    }
    else if (VLK_CONN_STATUS_SERIAL_PORT_DISCONNECTED == iConnStatus)
    {
        cout << "serial port disconnected !!!" << endl;
        g_bConnected = false;
    }
    else
    {
        cout << "unknown connection stauts: " << iConnStatus << endl;
        g_bConnected = false;
    }

    return 0;
}

int VLK_DevStatusCallback(int iType, const char *szBuffer, int iBufLen, void *pUserParam)
{
    if (VLK_DEV_STATUS_TYPE_MODEL == iType)
    {
        VLK_DEV_MODEL *pModel = (VLK_DEV_MODEL *)szBuffer;
        cout << "model code: " << pModel->cModelCode << ", model name: " << pModel->szModelName << endl;
    }
    else if (VLK_DEV_STATUS_TYPE_CONFIG == iType)
    {
        VLK_DEV_CONFIG *pDevConfig = (VLK_DEV_CONFIG *)szBuffer;
        cout << "VersionNO: " << pDevConfig->cVersionNO << ", DeviceID: " << pDevConfig->cDeviceID << ", SerialNO: " << pDevConfig->cSerialNO << endl;
    }
    else if (VLK_DEV_STATUS_TYPE_TELEMETRY == iType)
    {
        /*
         * once device is connected, telemetry information will keep updating,
         * in order to avoid disturbing user input, comment out printing telemetry information
         */
        // VLK_DEV_TELEMETRY *pTelemetry = (VLK_DEV_TELEMETRY *)szBuffer;
        //  cout << "Yaw: " << pTelemetry->dYaw << ", Pitch: " << pTelemetry->dPitch << ", sensor type: " << pTelemetry->emSensorType << ", Zoom mag times: " << pTelemetry->sZoomMagTimes << endl;
        //   cout << "Yaw: " << pTelemetry->dYaw << ", Pitch: " << pTelemetry->dPitch << ", sensor type: " << pTelemetry->emSensorType << endl;
        // gettm(getTimeStamp());
        // printf("Yaw:%lf, Pitch:%lf\n", pTelemetry->dYaw, pTelemetry->dPitch);

        // cout<<"emTrackerStatus"<<pTelemetry->emTrackerStatus<<endl;
    }
    else
    {
        cout << "error: unknown status type: " << iType << endl;
    }

    return 0;
}
//---------------------------------------------------------------------------------------------------------

//-----------------------------------------键盘按键检测--------------------------------------------------
static int g_keyboardinput = 0;
static bool pipFlag = false;

void scanKeyboard()
{
    //    struct termios
    //      {
    //        tcflag_t c_iflag;		/* input mode flags */
    //        tcflag_t c_oflag;		/* output mode flags */
    //        tcflag_t c_cflag;		/* control mode flags */
    //        tcflag_t c_lflag;		/* local mode flags */
    //        cc_t c_line;			/* line discipline */
    //        cc_t c_cc[NCCS];		/* control characters */
    //        speed_t c_ispeed;		/* input speed */
    //        speed_t c_ospeed;		/* output speed */
    //    #define _HAVE_STRUCT_TERMIOS_C_ISPEED 1
    //    #define _HAVE_STRUCT_TERMIOS_C_OSPEED 1
    //      };
    while (1)
    {

        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
        new_settings = stored_settings;            //
        new_settings.c_lflag &= (~ICANON);         //
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings); //

        g_keyboardinput = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    }
}

//------------------------------------------------------------------------------------------------------

void zoomInOut()
{
    while (1)
    {
        if (g_keyboardinput == 'q')
        {
            VLK_ZoomIn(4);
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            VLK_StopZoom();
            g_keyboardinput = 0;
        }
        if (g_keyboardinput == 'e')
        {
            VLK_ZoomOut(4);
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            VLK_StopZoom();
            g_keyboardinput = 0;
        }
    }
}

float pixel2anglerate(int diff)
{
    if (abs(diff) < 10)
        return 0;
    printf("pixel2anglerate ret:%f\n", (float)diff / 20 / 0.05);
    return (float)diff / 20 / 0.05;
}

const float anglerate2MovePara = 120;
int setSpeed(float angleratex, float angleratey)
{
    // max speed 270 degree/sec,0.27 deg/ms,
    // 1 deg = 20pixel, 5.4 pixel/ms,162 pixel/frame
    if (abs(angleratex) > 270)
        angleratex = angleratex > 0 ? 270 : -270;
    if (abs(angleratey) > 270)
        angleratey = angleratey > 0 ? 270 : -270;
    printf("VLK_Move set:%f, %f\n", angleratex * anglerate2MovePara, angleratey * anglerate2MovePara);
    VLK_Move(angleratex * anglerate2MovePara, angleratey * anglerate2MovePara);
}
//----------------------------------------------手动操作云台------------------------------------------------
void manual_gimbal()
{
    while (1)
    {
        switch (g_keyboardinput)
        {
        case 'w':
        case 'W':
            VLK_Move(0, 4000);
            break;
        case 's':
        case 'S':
            VLK_Move(0, -4000);
            break;
        case 'a':
        case 'A':
            VLK_Move(4000, 0);
            break;
        case 'd':
        case 'D':
            VLK_Move(-4000, 0);
            break;
        case 'p':
            VLK_Stop();
            break;
        case 'q':
            VLK_ZoomIn(4);
            break;
        case 'e':
            VLK_ZoomOut(4);
            // std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            break;
        case 'h':
        case 'H':
            VLK_Home();
            break;
        case 27:
            VLK_Home();
            VLK_UnInit();
            exit(0);
            break;
        case 'o':
            setSpeed(pixel2anglerate(10), 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            VLK_Move(0, 0);
        case '0':
            if (pipFlag)
            {
                VLK_SetImageColor(VLK_IMAGE_TYPE_VISIBLE1, 1, VLK_IR_COLOR_RUSTY);
                pipFlag = false;
            }
            else
            {
                VLK_SetImageColor(VLK_IMAGE_TYPE_VISIBLE1, 0, VLK_IR_COLOR_RUSTY);
                pipFlag = true;
            }
            break;
        case 'i':
            showCamera = IRCamera;
            break;
        case 'v':
            showCamera = visibleCamera;
            break;
        case '1':
            manualFlag = false;
            
            VLK_Stop();
            nFrames = 0;
            break;
        case '2':
            cout << "press \'w\' move up \n";
            cout << "press \'s\' move down \n";
            cout << "press \'a\' move left \n";
            cout << "press \'d\' move right \n";
            cout << "press \'p\' stop move\n";
            cout << "press \'h\' move to home posiion \n";
            cout << "press \'q\' zoom in, \'e\' zoom out\n";
            cout << "press \'1\' begin track, \'2\' stop track\n";
            cout << "press \'0\' picture in picture on or off\n";
            cout << "press \'i\' iR camera\n";
            cout << "press \'v\' visible camera\n";
            cout << "press \'esc\' exit" << endl;
            
            manualFlag = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            VLK_Stop();
            break;
        }
        g_keyboardinput = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main(int argc, char *argv[])
{

    if (!showCamera.isOpened() || !IRCamera.isOpened())
    {
        std::cout << "Connection failed \n";
        return -1;
    }

    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = true;
    bool LAB = false;

    // Create KCFTracker object
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    float xMin;
    float yMin;
    float width;
    float height;
    int center_x;
    int center_y;

    // xMin = 608;
    // yMin =328;
    width = 100;
    height = 100;
    center_x = 640;
    center_y = 360;
    xMin = center_x - width / 2;
    yMin = center_y - height / 2;

    int cnt = 0;

    Rect result;

    std::thread t1(&scanKeyboard);
    std::thread t2(&zoomInOut);
    std::thread t3(&manual_gimbal);

    /*
    -------------------------------------------ViewLink init代码----------------------------------------------
    */
    // parse cmd line
    cmdline::parser a;
    a.add<string>("type", 't', "connection type", true, "tcp", cmdline::oneof<string>("serial", "tcp"));
    a.add<string>("ip", 'i', "gimbal tcp ip", false, "192.168.2.119");
    a.add<int>("port", 'p', "gimbal tcp port", false, 2000);
    a.add<string>("serial", 's', "serial port name", false, "/dev/ttyS0");
    a.add<int>("baudrate", 'b', "serial port baudrate", false, 115200);
    a.parse_check(argc, argv);

    // print sdk version
    cout << "ViewLink SDK version: " << GetSDKVersion() << endl;

    // initialize sdk
    int iRet = VLK_Init();
    if (VLK_ERROR_NO_ERROR != iRet)
    {
        cout << "VLK_Init failed, error: " << iRet << endl;
        return -1;
    }

    // register device status callback
    VLK_RegisterDevStatusCB(VLK_DevStatusCallback, NULL);

    // connect device
    if (0 == a.get<string>("type").compare("tcp"))
    {
        VLK_CONN_PARAM param;
        memset(&param, 0, sizeof(param));
        param.emType = VLK_CONN_TYPE_TCP;
        strncpy(param.ConnParam.IPAddr.szIPV4, a.get<string>("ip").c_str(), sizeof(param.ConnParam.IPAddr.szIPV4) - 1);
        param.ConnParam.IPAddr.iPort = a.get<int>("port");

        cout << "connecting gimbal ip: " << a.get<string>("ip") << ", port: " << a.get<int>("port") << "..." << endl;
        iRet = VLK_Connect(&param, VLK_ConnStatusCallback, NULL);
        if (VLK_ERROR_NO_ERROR != iRet)
        {
            cout << "VLK_Connect failed, error: " << iRet << endl;
            goto quit;
        }
    }
    else if (0 == a.get<string>("type").compare("serial"))
    {
        VLK_CONN_PARAM param;
        memset(&param, 0, sizeof(param));
        param.emType = VLK_CONN_TYPE_SERIAL_PORT;
        strncpy(param.ConnParam.SerialPort.szSerialPortName, a.get<string>("serial").c_str(), sizeof(param.ConnParam.SerialPort.szSerialPortName) - 1);
        param.ConnParam.SerialPort.iBaudRate = a.get<int>("baudrate");

        cout << "connecting gimbal serial: " << a.get<string>("serial") << ", baudrate: " << a.get<int>("baudrate") << "..." << endl;
        iRet = VLK_Connect(&param, VLK_ConnStatusCallback, NULL);
        if (VLK_ERROR_NO_ERROR != iRet)
        {
            cout << "VLK_Connect failed, error: " << iRet << endl;
            goto quit;
        }
    }
    else
    {
        cout << "unknown conntion type !!!" << endl;
        goto quit;
    }

    cout << "wait device connected..." << endl;
    while (1)
    {
        if (g_bConnected)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    //---------------------------------------------------------------------------------------------------------
    // VLK_TRACK_MODE_PARAM param;
    // param.emTrackTempSize = VLK_TRACK_TEMPLATE_SIZE_64;

    t1.detach();
    t2.detach();
    t3.detach();

    static int lastx, lasty;
    int diffx, diffy;
    lastx = xMin;
    lasty = yMin;
    while (1)
    {
        // manualFlag = false;
        while (!manualFlag)
        {
            showCamera >> rgbFrame;
            // gettm(getTimeStamp());
            if (!rgbFrame.empty())
            {
                if (nFrames == 0)
                    tracker.init(Rect(xMin, yMin, width, height), rgbFrame);
                else
                {
                    result = tracker.update(rgbFrame);
                    rectangle(rgbFrame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(48, 48, 255), 2, 8);

                    // VLK_Move(10*(result.x+result.width/2-center_x),(center_y-result.y-result.height/2)*10);

                    diffx = result.x + result.width / 2 - center_x;
                    diffy = center_y - result.y - result.height / 2;
                    // setSpeed(pixel2anglerate(diffx),pixel2anglerate(diffy));
                    // if(diffx > 0)
                    //     setSpeed(pixel2anglerate(diffx),0);
                    // else
                    // setSpeed(0,0);

                    VLK_Move(pidProc(diffx), pidProc(diffy));
                }
                // printf("ref:x%d,ref y:%d, result:x%d,y:%d, diff x:%d, diff y:%d\n",
                //  center_x, center_y, result.x+result.width/2, result.y+result.height/2, diffx, diffy);
                // lastx = result.x;
                // lasty = result.y;
                nFrames++;
            }
            cv::imshow("rgbFrame", rgbFrame);
            int key = cv::waitKey(1);
            if (key == 27)
            {
                VLK_Home();
                VLK_UnInit();
                break;
            }

            // scout<<g_keyboardinput<<endl;
        }
        showCamera >> rgbFrame;
        if (!rgbFrame.empty())
        {
            cv::imshow("rgbFrame", rgbFrame);
            int key = cv::waitKey(1);
            if (key == 27)
            {
                VLK_Home();
                VLK_UnInit();
                break;
            }
        }
    }

quit:
    // uninitial sdk
    VLK_UnInit();
    return 0;
    // std::cout << cv::getBuildInformation() << std::endl;
}
// gst-launch-1.0 rtspsrc location=rtsp://192.168.2.119:554/stream0 latency=0 ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! video/x-raw,width=1280,height=720 ! autovideosink
// gst-launch-1.0 -v rtspsrc location=rtsp://192.168.2.119:554/stream0 ! rtph264depay !  h264parse ! omxh264dec ! nvvidconv ! video/x-raw, width=1280, height=720 ! videoconvert !  ximagesink sync=false

// gst-launch-1.0 -v rtspsrc location=rtsp://192.168.2.119:554/stream0 ! rtph264depay !  h264parse ! nvv4l2decoder enable-max-performance=1 drop-frame-interval=1 ! nvvidconv ! xvimagesink sync=false -e -v
// https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/Multimedia/AcceleratedGstreamer.html#video-streaming-with-gstreamer-1-0
