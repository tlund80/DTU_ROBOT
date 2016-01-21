/**
 * @file /include/CalTI/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef CalTI_QNODE_HPP_
#define CalTI_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifdef USE_ROS
    #include <ros/ros.h>
    #include <std_msgs/String.h>
    #include <image_transport/image_transport.h>
    #include <camera_info_manager/camera_info_manager.h>
    #include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/image_encodings.h>
    #include <robot_msgs/Pose.h>
#endif


#include <sstream>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "config.hpp"
#include "SharedData.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace CalTI {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	void startStream(const std::string& leftTopic, const std::string& rightTopic);
	void startStream(const std::string& leftTopic);
	void startStreamWithTCP(const std::string& HandEyeTopic, const std::string& robotTCP);
	void stopStream();

Q_SIGNALS:
	void updateImage(const QImage&, int);
	void updateTCP(const tiv::pose&, int);
	void updateImageLeft(const QImage&);
	void updateImageRight(const QImage&);
	void consoleSignal(QString msg);
    	void rosShutdown();

private:
    static QImage cvImg2QImg(cv::Mat *imgOCV);
    void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageRightCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageHandEyeCallback(const sensor_msgs::ImageConstPtr& msg);
    void leftCamInfoCallback(const sensor_msgs::CameraInfo& msg);
    void rightCamInfoCallback(const sensor_msgs::CameraInfo& msg);
    void TCPCallback(const robot_msgs::Pose::ConstPtr& msg);

    int init_argc;
    char** init_argv;
    image_transport::Subscriber mImageSubscriberLeft;
    image_transport::Subscriber mImageSubscriberRight;
    image_transport::Subscriber mImageSubscriberHandEye;
    ros::Subscriber mCamInfoSubscriberLeft;
    ros::Subscriber mCamInfoSubscriberRight;
    ros::Subscriber TCP_sub;

    std::string _leftCamName;
    std::string _rightCamName;

    dti::SharedData *_sharedData;
};

}  // namespace CalTI

#endif /* CalTI_QNODE_HPP_ */
