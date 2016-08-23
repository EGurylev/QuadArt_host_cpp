/*
Control loop interface
*/

#pragma once

#include <QtCore/QObject>
#include <QtGui/QImage>
#include <QtCore/QString>
#include <QtGui/QPainter>
#include <QtWidgets/QLabel>
#include <QtCore/QTimer>
#include "common.h"
#include "camera.h"
#include "img_proc.h"
#include "pose_estim.h"
#include "Crazyradio.h"
#include "Crazyflie.h"

struct log_block
{
	float roll;
	float pitch;
	float yaw;
} __attribute__((packed));

class Loop : public QObject
{
	Q_OBJECT
	private:
		QTimer loop_timer;
		Camera cam_obj;
		uint8_t* img_p;
		int timer_period = 10;
		QLabel img_label;
		cv::Mat rvec, tvec;
		img_proc img_proc_obj;
		pose_estimator pe_obj;
		pose6D pose_est, pose_meas;
		std::pair<std::vector<std::string>,
			std::vector<std::vector<float>>> logger;
		Crazyflie cf_obj;
		int thrust = 20000;
	public:
		Loop();
		void run();
		void logging();
		void log_callback(uint32_t time_in_ms,
			log_block* data);
		void log2file();
		~Loop();
	public slots:
        void update();
};
