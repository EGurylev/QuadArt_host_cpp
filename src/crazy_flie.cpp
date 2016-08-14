#include "crazy_flie.h"

crazy_flie::crazy_flie()
{
	libusb_device **list;
	libusb_device *found = NULL;
	libusb_init(NULL);
	ssize_t cnt = libusb_get_device_list(NULL, &list);
	cout << "Number of usb devices is " << cnt << endl;
	
	for (int i = 0; i < cnt; i++)
	{
		libusb_device *device = list[i];
		libusb_device_descriptor desc = {0};
		libusb_get_device_descriptor(device, &desc);
		cout << "USB device # " << i << endl;
		cout << desc.idVendor << endl;
		cout << desc.idProduct << endl;
    }
    
    cf_dh = libusb_open_device_with_vid_pid(NULL, 
			cradio_vid, cradio_pid);
			
}

crazy_flie::~crazy_flie()
{
	libusb_close(cf_dh);
	libusb_exit(NULL);	
}
