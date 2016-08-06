#include "control_loop.h"
#include <QtWidgets/QApplication>
#include <QtCore/QTimer>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Camera mycam;
    QTimer mytimer;
    QObject::connect(&mytimer, SIGNAL(timeout()),
                         &mycam, SLOT(Run()));

    mytimer.start(10);

    return a.exec();
}

