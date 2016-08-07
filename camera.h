#pragma once
#include <chrono>
#include <iostream>
#include <QtWidgets/QLabel>
#include <QtGui/QPainter>
#include <pylon/PylonIncludes.h>
#include <pylon/TlFactory.h>
#include <pylon/ThreadPriority.h>
#include <pylon/_InstantCameraParams.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <opencv2/opencv.hpp>

using namespace Pylon;
using namespace std;
using namespace Basler_UsbCameraParams;

typedef CBaslerUsbInstantCamera Camera_t;
typedef CBaslerUsbGrabResultPtr GrabResultPtr_t;
typedef CBaslerUsbImageEventHandler ImageEventHandler_t;

// Example of a device specific handler for image events.
class CSampleImageEventHandler : public ImageEventHandler_t
{
public:
    virtual void OnImageGrabbed( Camera_t& camera, const GrabResultPtr_t& ptrGrabResult)
    {
        // The chunk data is attached to the grab result and can be accessed anywhere.

        // Generic parameter access:
        // This shows the access via the chunk data node map. This method is available for all grab result types.
        GenApi::CIntegerPtr chunkTimestamp( ptrGrabResult->GetChunkDataNodeMap().GetNode( "ChunkTimestamp"));

        // Access the chunk data attached to the result.
        // Before accessing the chunk data, you should check to see
        // if the chunk is readable. When it is readable, the buffer
        // contains the requested chunk data.
        if ( IsReadable( chunkTimestamp))
            cout << "OnImageGrabbed: TimeStamp (Result) accessed via node map: " << chunkTimestamp->GetValue() << endl;

        // Native parameter access:
        // When using the device specific grab results the chunk data can be accessed
        // via the members of the grab result data.
        if ( IsReadable(ptrGrabResult->ChunkTimestamp))
            cout << "OnImageGrabbed: TimeStamp (Result) accessed via result member: " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;
    }
};
  
class Camera
{
    public:
        GrabResultPtr_t ptrGrabResult;
        Camera_t camera;
        CPylonImage pylon_img;
        CImageFormatConverter converter;
        int TimeStamp = 0, TimeStampPrev = 0;
        Camera();
        uint8_t* grab_image();
        ~Camera();
};

