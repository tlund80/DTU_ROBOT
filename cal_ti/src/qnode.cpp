/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/cal_ti/qnode.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/cal_ti/rot.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace CalTI {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
{
	init();
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"CalTI");
	qRegisterMetaType<tiv::pose>("tiv::pose");
	if ( ! ros::master::check() ) {
		return false;
	}

	_leftCamName = "no_left_cam_name_detected";
	_rightCamName = "no_right_cam_name_detected";
	start();
	return true;
}

void QNode::startStream(const std::string& leftTopic)
{
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	//mImageSubscriberLeft = it.subscribe("/usb_cam/image_raw", 1, &QNode::imageLeftCallback, this);
	mImageSubscriberLeft = it.subscribe(leftTopic.c_str(), 1, &QNode::imageLeftCallback, this);
}

void QNode::startStream(const std::string& leftTopic, const std::string& rightTopic)
{
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);

	//mImageSubscriberLeft = it.subscribe("/usb_cam/image_raw", 1, &QNode::imageLeftCallback, this);
	mImageSubscriberLeft = it.subscribe(leftTopic.c_str(), 1, &QNode::imageLeftCallback, this);
	mImageSubscriberRight = it.subscribe(rightTopic.c_str(), 1, &QNode::imageRightCallback, this);


	unsigned foundL = leftTopic.find_last_of("/");
	unsigned foundR = rightTopic.find_last_of("/");
	std::string left_path;
	left_path.append(leftTopic.substr(0,foundL));
	left_path.append("/camera_info");

	std::string right_path;
	right_path.append(rightTopic.substr(0,foundR));
	right_path.append("/camera_info");

	std::cout << "Left:" << left_path << std::endl;
	std::cout << "Right:" << right_path << std::endl;

	mCamInfoSubscriberLeft = n.subscribe(left_path.c_str(), 1, &QNode::leftCamInfoCallback, this);
	mCamInfoSubscriberRight = n.subscribe(right_path.c_str(), 1, &QNode::rightCamInfoCallback, this);

}

void QNode::startStreamWithTCP(const std::string& HandEyeTopic, const std::string& robotTCP)
{
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);

	//mImageSubscriberLeft = it.subscribe("/usb_cam/image_raw", 1, &QNode::imageLeftCallback, this);
	mImageSubscriberHandEye = it.subscribe(HandEyeTopic.c_str(), 1, &QNode::imageHandEyeCallback, this);
	TCP_sub = n.subscribe(robotTCP.c_str(), 1, &QNode::TCPCallback,this);

/*	unsigned foundL = HandEyeTopic.find_last_of("/");

	std::string left_path;
	left_path.append(HandEyeTopic.substr(0,foundL));
	left_path.append("/camera_info");

	std::cout << "Left:" << left_path << std::endl;

	mCamInfoSubscriberLeft = n.subscribe(left_path.c_str(), 1, &QNode::leftCamInfoCallback, this);
*/
}

void QNode::stopStream()
{
	mImageSubscriberLeft.shutdown();
	mImageSubscriberRight.shutdown();
	mImageSubscriberHandEye.shutdown();
	TCP_sub.shutdown();

	mCamInfoSubscriberLeft.shutdown();
	mCamInfoSubscriberRight.shutdown();
}

void QNode::run() {
	ros::spin();
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::leftCamInfoCallback(const sensor_msgs::CameraInfo& msg)
{
	_leftCamName = msg.header.frame_id;
	_sharedData->setCameraNameLeft(QString::fromStdString(_leftCamName));
}

void QNode::rightCamInfoCallback(const sensor_msgs::CameraInfo& msg)
{
	_rightCamName = msg.header.frame_id;
	_sharedData->setCameraNameRight(QString::fromStdString(_rightCamName));

}

void QNode::imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//std::cout << "left name: " << msg->header.frame_id << std::endl;
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat cvImg = cv_ptr->image;
		QImage qImg = cvImg2QImg(&cvImg);
		//emit updateImage(qImg.rgbSwapped(), (int) dti::LEFT);
		Q_EMIT updateImage(qImg.rgbSwapped(), (int) dti::LEFT);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void QNode::imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat cvImg = cv_ptr->image;
		QImage qImg = cvImg2QImg(&cvImg);
		//emit updateImageRight(qImg.rgbSwapped());
		//emit updateImage(qImg.rgbSwapped(), (int) dti::RIGHT);
		Q_EMIT updateImage(qImg.rgbSwapped(), (int) dti::RIGHT);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void QNode::imageHandEyeCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//std::cout << "left name: " << msg->header.frame_id << std::endl;
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat cvImg = cv_ptr->image;
		QImage qImg = cvImg2QImg(&cvImg);
		//emit updateImage(qImg.rgbSwapped(), (int) dti::HAND);
		Q_EMIT updateImage(qImg.rgbSwapped(), (int) dti::HAND);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void QNode::TCPCallback(const robot_msgs::Pose::ConstPtr& msg)
{
	//Convert from meters to milimeters
	tiv::pose p;
	p.t.x =  msg->x *1000;
	p.t.y =  msg->y  *1000;
	p.t.z =  msg->z *1000;

	Eigen::Vector3d rpy;
	rpy(0) = msg->roll;
	rpy(1) = msg->pitch;
	rpy(2) = msg->yaw;

	Eigen::Matrix3d R;
	tiv::ti_rpy2mat(rpy,R);

	p.q.fromMatrix(R);

	//emit updateTCP(p,(int) dti::HAND);
	Q_EMIT updateTCP(p,(int) dti::HAND);
}


QImage QNode::cvImg2QImg(cv::Mat *imgOCV) {
	QImage *imgQ;
	if (imgOCV->type() == CV_8UC1 )
		imgQ = new QImage(imgOCV->data, imgOCV->cols, imgOCV->rows,
				imgOCV->step, QImage::Format_Indexed8);
	else if (imgOCV->type() == CV_8UC3)
		imgQ = new QImage(imgOCV->data, imgOCV->cols, imgOCV->rows,
				imgOCV->step, QImage::Format_RGB888);
	else if (imgOCV->type() == CV_16UC3 )
		imgQ = new QImage(imgOCV->data, imgOCV->cols, imgOCV->rows,
				imgOCV->step, QImage::Format_RGB16);

	return imgQ->copy(0,0,imgQ->width(), imgQ->height());

}

}  // namespace CalTI
