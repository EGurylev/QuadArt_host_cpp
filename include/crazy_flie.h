/*
Class which provides access to Crazyflie 2.0.
*/

#include "common.h"
#include <libusb-1.0/libusb.h>

class crazy_flie
{
	private:
		//CrazyRadio parameters
		const int cradio_vid = 0x1915;
		const int cradio_pid = 0x7777;
		libusb_device_handle* cf_dh;
	public:
		crazy_flie();
		~crazy_flie();
};
