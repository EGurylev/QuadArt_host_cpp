#include "camera.h"

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
    
    converter.OutputPixelFormat = PixelType_BGR8packed;// PixelType_Mono8
    // Open the camera.
    camera.Open();
    camera.ChunkModeActive.SetValue(true);
    camera.ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
	camera.ChunkEnable.SetValue(true);
    camera.StartGrabbing(200, GrabStrategy_LatestImageOnly);
    //camera.GrabLoopThreadPriorityOverride = true;
    //camera.GrabLoopThreadPriority = 0;
    SetRTThreadPriority(GetCurrentThreadHandle(), 99);
    //camera.GrabLoopThreadPriority = 25;
    //cout << camera.InternalGrabEngineThreadPriorityOverride() << endl;
}

Mat Camera::grab_image()
{
        if (camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            high_resolution_clock::time_point t1 = high_resolution_clock::now();

            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            //cout << GetRTThreadPriority(GetCurrentThreadHandle()) << endl;
            converter.Convert( pylon_img, ptrGrabResult);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
				if (IsReadable(ptrGrabResult->ChunkTimestamp))
					TimeStamp = ptrGrabResult->ChunkTimestamp.GetValue();
				cout << "Difference in grabbing time " << (TimeStamp - TimeStampPrev) / 1000 << endl;

            }
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            Duration = duration_cast<microseconds>( t2 - t1 ).count();
            //cout << Duration << endl;
            TimeStampPrev = TimeStamp;
            //Convert to OpenCV format
            opencv_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3,
            	static_cast<uint8_t*>(pylon_img.GetBuffer()));
         }
         return opencv_img;
}

Camera::~Camera()
{
    PylonTerminate();
}
