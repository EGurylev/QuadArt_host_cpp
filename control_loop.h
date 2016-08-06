#pragma once
#include <fstream>
#include <vector>
#include <chrono>
#include <iostream>
#include <QtCore/QObject>
#include <QtGui/QImage>
#include <QtCore/QString>
#include <QtGui/QPainter>
#include <QtWidgets/QLabel>
#include <QtCore/QTimer>
#include "camera.h"

class Loop : public QObject
{
	Q_OBJECT
	private:
		QTimer loop_timer;
		Camera cam_obj;
		Mat img;
		int timer_period = 10;
		int cnt = 0;
		int num_debug_img = 50;
		vector<Mat> img_array;
		QLabel img_label;
	public:
		Loop();
		void run();
		void save_img_debug();
		~Loop();
	public slots:
        void update();
};
