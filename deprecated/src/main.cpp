#include "control_loop.h"
#include <unistd.h>

int main(int argc, char *argv[])
{
    
    Loop loop_obj;
    
    loop_obj.run();

    //Do nothing until specified time passed
    while(loop_obj.timer._execute)
    {
		usleep(10000);
    }

    return 0;
}

