#pragma once
#include <pylon/PylonIncludes.h>
#include <pylon/TlFactory.h>
#include <pylon/ThreadPriority.h>
#include <pylon/_InstantCameraParams.h>
//#include <pylon/usb/_UsbChunkData.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <chrono>
#include <iostream>
#include <QtCore/QObject>
#include <QtGui/QImage>
#include <fstream>
#include <QtWidgets/QLabel>
#include <vector>
#include <QtCore/QString>
#include <QtGui/QPainter>

using namespace Pylon;
using namespace std;
using namespace std::chrono;
using namespace Basler_UsbCameraParams;

typedef CBaslerUsbInstantCamera Camera_t;
typedef CBaslerUsbGrabResultPtr GrabResultPtr_t;
typedef CBaslerUsbImageEventHandler ImageEventHandler_t;


#define PIXEL_MONO  0x01000000
    // Bitmask value of the color pixel type. Internal use only.
#define PIXEL_COLOR 0x02000000
    // Sets the bit count of pixel type. Internal use only.
#define PIXEL_BIT_COUNT(n) ((n) << 16)
//  
class Camera : public QObject
{
    Q_OBJECT
    public:
        GrabResultPtr_t ptrGrabResult;
        Camera_t camera;
        CPylonImage dst_img;
        CImageFormatConverter converter;
        int TimeStamp = 0, TimeStampPrev = 0;
        double Duration;
        int i = 0;
        QLabel mylabel;
        vector<QImage> imageArray;
        Camera();
        ~Camera();
    public slots:
        void Run();
};

