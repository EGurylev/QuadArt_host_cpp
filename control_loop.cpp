#include "control_loop.h"

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

Camera::Camera()
{
    PylonInitialize();
    // Only look for cameras supported by Camera_t
    CDeviceInfo info;
    info.SetDeviceClass(Camera_t::DeviceClass());
    // Attach object with the first found camera device that matches the specified device class.
    camera.Attach(CTlFactory::GetInstance().CreateFirstDevice(info));
    // Print the model name of the camera.
    cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

    // Register an image event handler that accesses the chunk data.
    camera.RegisterImageEventHandler( new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
    
    converter.OutputPixelFormat = PixelType_RGB8packed;// PixelType_Mono8
    // Open the camera.
    camera.Open();
    camera.ChunkModeActive.SetValue(true);
    camera.ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
	camera.ChunkEnable.SetValue(true);
    //N = 300;
    camera.StartGrabbing(200, GrabStrategy_LatestImageOnly);
	mylabel.show();
    //camera.GrabLoopThreadPriorityOverride = true;
    //camera.GrabLoopThreadPriority = 0;
    SetRTThreadPriority(GetCurrentThreadHandle(), 99);
    //camera.GrabLoopThreadPriority = 25;
    //cout << camera.InternalGrabEngineThreadPriorityOverride() << endl;
}

void Camera::Run()
{
        if (camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            high_resolution_clock::time_point t1 = high_resolution_clock::now();

            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            //cout << GetRTThreadPriority(GetCurrentThreadHandle()) << endl;
            converter.Convert( dst_img, ptrGrabResult);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
				if (IsReadable(ptrGrabResult->ChunkTimestamp))
					TimeStamp = ptrGrabResult->ChunkTimestamp.GetValue();
				cout << "Difference in grabbing time " << (TimeStamp - TimeStampPrev) / 1000 << endl;
                
                uint8_t* p = static_cast<uint8_t*>(dst_img.GetBuffer());
                QImage image( p, 1280, 1024, 1280 * 3, QImage::Format_RGB888);
                QPixmap pix = QPixmap::fromImage(image);
                QPainter painter( &pix );
                QPen mypen(QColor(255,255,255));
                painter.setPen(mypen);
                painter.setFont( QFont("Arial", 28) );
                auto now = std::chrono::system_clock::now();
                auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
                auto epoch = now_ms.time_since_epoch();
                auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
                long duration = value.count();
                QString labelText(QString::number(duration));
                painter.drawText( QPoint(500, 500), labelText);

                QImage img = pix.toImage();

                if (i < 50)
                {
                    imageArray.push_back(img.copy());
                }
				i += 1;
                //mylabel.setPixmap(pix);

                //mylabel.repaint();
                //mylabel.show();
                //cout << camera.OutputQueueSize() << " ";

            }
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            Duration = duration_cast<microseconds>( t2 - t1 ).count();
            cout << Duration << endl;
            TimeStampPrev = TimeStamp;
         }
}

Camera::~Camera()
{
    PylonTerminate();

    cout << imageArray.size() << endl;
    for (int j = 0; j < imageArray.size(); j++)
    {
        QString filename = "img_";
        filename.append(QString::number(j));
        filename.append(".jpg");
        QImage im = imageArray[j];
        im.save(filename);
    }
}
