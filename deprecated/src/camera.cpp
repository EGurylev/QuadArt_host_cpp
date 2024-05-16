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
    std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;

    // Register an image event handler that accesses the chunk data.
    camera.RegisterImageEventHandler(new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
    
    converter.OutputPixelFormat = PixelType_BGR8packed;// PixelType_Mono8
    // Open the camera.
    camera.Open();
    camera.ChunkModeActive.SetValue(true);
    camera.ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
	camera.ChunkEnable.SetValue(true);
    camera.StartGrabbing(GrabStrategy_LatestImageOnly);
    //camera.GrabLoopThreadPriorityOverride = true;
    //camera.GrabLoopThreadPriority = 0;
    //SetRTThreadPriority(GetCurrentThreadHandle(), 99);
    //camera.GrabLoopThreadPriority = 25;
    //cout << camera.InternalGrabEngineThreadPriorityOverride() << endl;
}

uint8_t* Camera::grab_image()
{
        if (camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.

            camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

            //cout << GetRTThreadPriority(GetCurrentThreadHandle()) << endl;
            converter.Convert(pylon_img, ptrGrabResult);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
				if (IsReadable(ptrGrabResult->ChunkTimestamp))
					TimeStamp = ptrGrabResult->ChunkTimestamp.GetValue();
				//std::cout << "Difference in grabbing time " << 
					//(TimeStamp - TimeStampPrev) / 1000 << std::endl;
            }
            
            TimeStampPrev = TimeStamp;
         }
         return static_cast<uint8_t*>(pylon_img.GetBuffer());
}

Camera::~Camera()
{
    PylonTerminate();
}
