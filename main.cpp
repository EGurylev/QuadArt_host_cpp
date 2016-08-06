#include "control_loop.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    Loop loop_obj;
    loop_obj.run();

    return a.exec();
}

