/*
 * HandEyeCalibration.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: thomas
 */

#include "../../include/cal_ti/HandEyeCalibration.hpp"

namespace dti {

HandEyeCalibration::HandEyeCalibration(SharedData* sharedData) {
	// TODO Auto-generated constructor stub
	setSharedData(sharedData);
	clearAllData();
	_handEyeTransform = cv::Mat(4,4,CV_64F);
	_calibrationTargetTransform = cv::Mat(4,4,CV_64F);
}

HandEyeCalibration::~HandEyeCalibration() {
	// TODO Auto-generated destructor stub
}

void HandEyeCalibration::printResults(){
	if(_camPoses.size() > 0 && _robPoses.size() > 0)
	{
		std::string str = calibResultsAsString();
        Q_EMIT consoleSignal(QString(str.c_str()), dti::VerboseLevel::INFO);
	}
	
}

void HandEyeCalibration::storeResults(std::string path){

	if (_rms < 0) {
        Q_EMIT consoleSignal("Hand/Eye calibration must be performed before storing the results!!\n", dti::VerboseLevel::WARN);
		return;
	}

	std::cout << "Store path: " <<  path << std::endl;

	cv::FileStorage fs(path, CV_STORAGE_WRITE);

	if (!fs.isOpened()){
		fs.open(path, CV_STORAGE_WRITE);
	}
	if (fs.isOpened()) {
	//	fs << "%--> HAND EYE CALIBRATION RESULTS <-- \n";
	//	fs << " --> Image acqusition parameters <-- \n";

		fs << "image_count" 		<< _validImgs;
		fs << "image_size"			<< _imageSize;
		fs << "square_size"			<< _squareSize;
		fs << "pattern_size" 		<< _boardSize;
		fs << "robot_pose_count" 	<< (int)_robPoses.size();

	//	fs << " --> Robot parameters <-- \n";
		fs << "hand_eye_transform"   << _handEyeTransform;
		fs << "calibration_Target_Transform"   << _calibrationTargetTransform;

		fs.release();
	}else
        Q_EMIT consoleSignal("Error: can not save the Hand/Eye parameters\n", dti::VerboseLevel::ERROR);

}

void HandEyeCalibration::loadRobotPoses()
{

    QString path = _sharedData->getRobotPosePath();
  //  Q_EMIT consoleSignal(QString("Loading robot poses from: ").append(path).append("\n"), dti::VerboseLevel::INFO);
    if (_robPoses.size() != 0) {
        Q_EMIT consoleSignal("Robot poses already loaded into the system!!\n", dti::VerboseLevel::INFO);
    return;
    }

	cv::FileStorage fs(path.toStdString(), CV_STORAGE_READ);

	int count;
	std::string rep;
	cv::Mat m61(6,1,CV_64F);
	cv::Mat m71(7,1,CV_64F);

	if (!fs.isOpened()){
        fs.open(path.toStdString(), cv::FileStorage::READ);
	}
	if (fs.isOpened()) {

		fs["pose_count"]  >> count;
		fs["rotation_representation"] >> rep;
		fs["is_inverted"] >> _isInverted;

		if(rep.compare("Axis_Angle") == 0){
			for(int i = 0; i<= count-1; i++){
				std::stringstream ss;
				ss << "pose_";
				ss << i;
				std::string st =  ss.str();
				fs[st] >> m61;

				tiv::pose p;

				Eigen::Vector3d eaa;
				Eigen::Matrix3d R;

				p.t.x = m61.at<double>(0,0);
				p.t.y = m61.at<double>(1,0);
				p.t.z = m61.at<double>(2,0);
				eaa(0,0) = m61.at<double>(3,0);
				eaa(1,0) = m61.at<double>(4,0);
				eaa(2,0) = m61.at<double>(5,0);

				tiv::eaa2mat(eaa,R);
				p.q.fromMatrix(R);

				_robPoses.push_back(p);

				//std::cout << m61 << std::endl;

			}

		}else if(rep.compare("Euler_RPY") == 0){
			for(int i = 0; i<= count-1; i++){
				std::stringstream ss;
				ss << "pose_";
				ss << i;
				std::string st =  ss.str();
				fs[st] >> m61;

				tiv::pose p;
				Eigen::Vector3d rpy;
				Eigen::Matrix3d R;

				p.t.x = m61.at<double>(0,0);
				p.t.y = m61.at<double>(1,0);
				p.t.z = m61.at<double>(2,0);
				rpy(0,0) = m61.at<double>(3,0);
				rpy(1,0) = m61.at<double>(4,0);
				rpy(2,0) = m61.at<double>(5,0);

				tiv::ti_rpy2mat(rpy, R);

				p.q.fromMatrix(R);
				_robPoses.push_back(p);

				//std::cout << m61 << std::endl;

			}

		}else if(rep.compare("Euler_YPR") == 0){

		}else if(rep.compare("Quaternion") == 0){
			for(int i = 0; i<= count-1; i++){
				std::stringstream ss;
				ss << "pose_";
				ss << i;
				std::string st =  ss.str();

				fs[st] >> m71;
				tiv::pose p;
				p.t.x = m71.at<double>(0,0);
				p.t.y = m71.at<double>(1,0);
				p.t.z = m71.at<double>(2,0);
				p.q.w = m71.at<double>(3,0);
				p.q.x = m71.at<double>(4,0);
				p.q.y = m71.at<double>(5,0);
				p.q.z = m71.at<double>(6,0);

				_robPoses.push_back(p);

         //       std::cout << m71 << std::endl;
			}
		}

		fs.release();

        _sharedData->clearRobotPoseArray();
        for(int i = 0; i<_robPoses.size();i++){
            _sharedData->addRobotPose2Array(_robPoses[i]);
        }
	}

}

void HandEyeCalibration::saveRobotPoses(std::string path)
{
	if (_robPoses.size() <= 0) {
            Q_EMIT consoleSignal(QString("No robot poses to save into file!!\n"), dti::VerboseLevel::WARN);
			return;
	}

		RobotFlags robFlags = _sharedData->getRobotFlags();
		_isInverted = robFlags._isInverted;

            cv::FileStorage fs(path, cv::FileStorage::WRITE);
			cv::Mat m61(6,1,CV_64F);
			cv::Mat m71(7,1,CV_64F);

			if (!fs.isOpened()){
				fs.open(path, CV_STORAGE_WRITE);
			}
			if (fs.isOpened()) {
				//	fs << "%--> HAND EYE CALIBRATION RESULTS <-- \n";
				//	fs << " --> Image acqusition parameters <-- \n";
				fs << "pose_count" << (int)_robPoses.size();
				fs << "is_inverted" << robFlags._isInverted;

				switch (robFlags._type) {
				case dti::Axis_Angle:
					std::cout << "Axis_Angle" << std::endl;
					fs << "rotation_representation" << "Axis_Angle";

					for(int i = 0; i<= (int)_robPoses.size()-1; i++){
						tiv::pose rp = _robPoses[i];

						std::vector<double> v = rp.q.toVector();
						Eigen::Matrix3d R;
						R << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
						Eigen::Vector3d eaa;
						tiv::mat2eaa(R,eaa);

						m61.at<double>(0,0) = rp.t.x;
						m61.at<double>(1,0) = rp.t.y;
						m61.at<double>(2,0) = rp.t.z;
						m61.at<double>(3,0) = eaa(0,0);
						m61.at<double>(4,0) = eaa(1,0);
						m61.at<double>(5,0) = eaa(2,0);

						std::stringstream ss;
						ss << "pose_";
						ss << i;
						std::string st =  ss.str();
						fs << st << m61;
					}

					break;

				case dti::Euler_RPY:
					std::cout << "Euler_RPY" << std::endl;
					fs << "rotation_representation" << "Euler_RPY";

					for(int i = 0; i<= (int)_robPoses.size()-1; i++){
						tiv::pose rp = _robPoses[i];

						std::vector<double> v = rp.q.toVector();
						Eigen::Matrix3d R;
						R << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
						Eigen::Vector3d rpy;
						tiv::ti_mat2rpy(R,rpy);

						m61.at<double>(0,0) = rp.t.x;
						m61.at<double>(1,0) = rp.t.y;
						m61.at<double>(2,0) = rp.t.z;
						m61.at<double>(3,0) = rpy(0,0);
						m61.at<double>(4,0) = rpy(1,0);
						m61.at<double>(5,0) = rpy(2,0);

						std::stringstream ss;
						ss << "pose_";
						ss << i;
						std::string st =  ss.str();
						fs << st << m61;

						}
					break;

				case dti::Euler_YPR:
					std::cout << "Euler_YRP is Not implemented yet!!!!!!!!!!!!!!" << std::endl;
					//fs << "rotation_representation" << "Euler_YPR";

					break;
				case dti::Quaternion:
					std::cout << "Quaternion" << std::endl;
					fs << "rotation_representation" << "Quaternion";

					for(int i = 0; i<= (int)_robPoses.size()-1; i++){
						tiv::pose rp = _robPoses[i];
						m71.at<double>(0,0) = rp.t.x;
						m71.at<double>(1,0) = rp.t.y;
						m71.at<double>(2,0) = rp.t.z;
						m71.at<double>(3,0) = rp.q.w;
						m71.at<double>(4,0) = rp.q.x;
						m71.at<double>(5,0) = rp.q.y;
						m71.at<double>(6,0) = rp.q.z;

						std::stringstream ss;
						ss << "pose_";
						ss << i;
						std::string st =  ss.str();
						fs << st << m71;
					}
					break;
				default:
					break;
				}

			}

			if (fs.isOpened()) {
				fs.release();
			} else
                Q_EMIT consoleSignal("Error: can not save the Hand/Eye parameters\n", dti::VerboseLevel::ERROR);


}

void HandEyeCalibration::init(){

	updateCalibrationFlags();
	clearAllData();
}

void HandEyeCalibration::updateCalibrationFlags()
{
	_cornerFlags 	= _sharedData->getCornerFlagsHandEye();
}

void HandEyeCalibration::clearAllData()
{
	// clear points
	_imagePoints.clear();
	_objectPoints.clear();
	_goodImages.clear();
	_badImages.clear();
	_validImgs = 0;

	_robPoses.clear();
	_camPoses.clear();
}

void HandEyeCalibration::detectARMarker(cv::Mat& img, CameraID CAM, bool found[3], float MarkerSize)
{
	cv::Mat rotMatrix(3,3,cv::DataType<double>::type);

	//Detect marker in the image
	 CameraParameters CamParam;
	 MarkerDetector MDetector;
	 std::vector<Marker> Markers;
	 CamParam.readFromXMLFile(_sharedData->getIntrinsicHandEyePath().toStdString());

	 //Ok, let's detect
	 MDetector.detect(img,Markers,CamParam,MarkerSize,false);

	 if(Markers.size() > 0) found[CAM] = true;
	 else found[CAM] = false;

	 //for each marker, draw info and its boundaries in the image
	 for (unsigned int i=0;i<Markers.size();i++) {
	        //std::cout<<Markers[i]<<endl;
	        Markers[i].draw(img,cv::Scalar(0,0,255),2);
	 }

	 //draw a 3d cube in each marker if there is 3d info
	 if (  CamParam.isValid() && MarkerSize!=-1)
	   for (unsigned int i=0;i<Markers.size();i++) {
	        CvDrawingUtils::draw3dCube(img,Markers[i],CamParam);
	       // CvDrawingUtils::draw3dAxis(img,Markers[i],CamParam);
	   }

	 cv::Mat tvec = Markers[0].Tvec;
	 cv::Mat rvec = Markers[0].Rvec;
	 cv::Rodrigues(rvec, rotMatrix);

	 std::cout << "\n====================TRANSLATION VECTOR:===================== " << std::endl;
	 std::cout << tvec << std::endl;

	 std::cout << "====================ROTATION MATRIX:===================== " << std::endl;


	 Eigen::Matrix3d r;
	 r(0,0) = rotMatrix.at<float>(0); r(0,1) = rotMatrix.at<float>(1); r(0,2) = rotMatrix.at<float>(2);
	 r(1,0) = rotMatrix.at<float>(3); r(1,1) = rotMatrix.at<float>(4); r(1,2) = rotMatrix.at<float>(5);
	 r(2,0) = rotMatrix.at<float>(6); r(2,1) = rotMatrix.at<float>(7); r(2,2) = rotMatrix.at<float>(8);

	 std::cout << r << std::endl;

	 tiv::pose p;
	 p.q.fromMatrix(r);
	 p.t.x = tvec.at<float>(0);
	 p.t.y = tvec.at<float>(1);
	 p.t.z = tvec.at<float>(2);

	 //Store camera poses from camera pose estimation
	 _camPoses.push_back(p);

	  //show input with augmented information
	  //cv::imshow("in",img);
	//show also the internal image resulting from the threshold operation
	//  cv::imshow("thes", MDetector.getThresholdedImage() );
//	  cv::waitKey(0);//wait for key to be pressed



}

int HandEyeCalibration::addImagePoints(std::vector<cv::Point2f> corners, int imgId)
{

	_imagePoints[_validImgs]  = corners;

	switch(_sharedData->getBoardTypeHandEye())
	{
	case dti::CHECKERBOARD:
        //We want the frame of the calibration plate to be located in the middle and not in the corner!!
		int midx, midy;
        if(_boardSize.height > _boardSize.width){
            midx=  floor((double)_boardSize.width/2);
            midy = ceil((double)(_boardSize.height/2)-1);
        }else if(_boardSize.height < _boardSize.width){
            midx=  floor((double)(_boardSize.width/2)-1);
            midy = ceil((double)(_boardSize.height/2));

        }else{
            std::cerr << "REALLY!! ARE YOU SURE YOU WANt TO USE A SYMMETRIC CALIBRATION PLATE? BOARD SIZE (" << _boardSize.width << "," << _boardSize.height << ")" << std::endl;
        }

		for(int i = midy; i >= -midy-1;i--){
			for(int j = midx; j >= -midx;j--){
	//	for(int j = _boardSize.height; j > 0 ; j-- )
	//	{
	//		for( int k = _boardSize.width; k > 0 ; k-- )
	//		{
				_objectPoints[_validImgs].push_back(cv::Point3f((getSquareSize()*j),-(getSquareSize()*i),0.00f));
				//std::cout << (int)_objectPoints[_validImgs].size() << std::endl;
				//std::cout << "world_points: \n" << "X: " << getSquareSize()*j << " Y: " << -(getSquareSize()*i) << " Z: " << 0.00f << std::endl;
			}
		}
		break;
	case dti::SYMMETRIC_CIRCLES_GRID:
		for(int j = 0; j < _boardSize.height; j++ )
		{
			for( int k = 0; k < _boardSize.width; k++ )
			{
				_objectPoints[_validImgs].push_back(cv::Point3f(k * getSquareSize(), -(j * getSquareSize()), 0)); //todo needs to be tested.
			}
		}
		break;

	case dti::ASYMMETRIC_CIRCLES_GRID:
		for(int j = 0; j < _boardSize.height; j++ )
		{
			for( int k = 0; k < _boardSize.width; k++ )
			{
				_objectPoints[_validImgs].push_back(cv::Point3f((2*k + j % 2) * getSquareSize(), -(j * getSquareSize()), 0));
			}
		}
		break;

	case dti::AR_MARKER:
			//NOP
		break;
	}

	_goodImages.push_back(imgId);

	_validImgs++;
	return _validImgs;
}

double HandEyeCalibration::computeReprojectionErrors(const std::vector<cv::Point3f>& objectPoints,
                                                    const std::vector<cv::Point2f>& imagePoints,
                                                    const cv::Mat& rvecs, const cv::Mat& tvecs,
                                                    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
		//reprojected points
	    std::vector<cv::Point2f> imagePoints2;
	    double err = 0;
	    projectPoints(cv::Mat(objectPoints), rvecs, tvecs,
	                  cameraMatrix, distCoeffs, imagePoints2);

	    //Compute L1 Norm
	    err = norm(cv::Mat(imagePoints), cv::Mat(imagePoints2), CV_L1 );

	    double err_sum = 0.0;
	    for (int i = 0 ; i < (int)imagePoints.size(); i++) {
	    	double dx = imagePoints[i].x - imagePoints2[i].x;
	    	double dy = imagePoints[i].y - imagePoints2[i].y;
	    	err_sum += sqrt(dx * dx + dy * dy);
	    }

	    err = err_sum / imagePoints.size();

	    return err;
}

void HandEyeCalibration::computeCameraPose(){


    Q_EMIT consoleSignal("Computing camera poses from all views!!\n", dti::VerboseLevel::INFO);


	if(_validImgs <= 0){
        Q_EMIT consoleSignal("Corner detection must be performed before computing camera poses!!\n", dti::VerboseLevel::WARN);
		return;
	}

    if(_imagePoints.size() != _objectPoints.size()){
        Q_EMIT consoleSignal("Number of object points & image points is not equal. Cannot compute camera poses!!\n", dti::VerboseLevel::WARN);
        return;
    }

    if(_objectPoints.size() == 0){
        Q_EMIT consoleSignal("No object world coordinates. Cannot compute camera poses!!\n", dti::VerboseLevel::WARN);
        return;
    }
    if(_imagePoints.size() == 0){
        Q_EMIT consoleSignal("No image points. Cannot compute camera poses!!\n", dti::VerboseLevel::WARN);
        return;
    }

/*	std::cout << "==============OBJ POINTS ==============" << std::endl;
	std::cout << "OBJ POINTS: " << (int)_objectPoints[0].size()<<  std::endl;
	for(int i = 0; i<= (int)_objectPoints[0].size()-1; i++){
		std::cout << "x:" << _objectPoints[0][i].x << " y: " << _objectPoints[0][i].y << " z:" << _objectPoints[0][i].z << std::endl;

	}
	std::cout << "==============IMAGE POINTS ==============" << std::endl;
	std::cout << "IMAGE POINTS: " << (int)_imagePoints[0].size()<<  std::endl;
	for(int i = 0; i<= (int)_imagePoints[0].size()-1; i++){
			std::cout << "x:" << _imagePoints[0][i].x << " y: " << _imagePoints[0][i].y <<  std::endl;

		}
*/
	/* Read intrinsic parameters from files */
	QString intrinsicLeftPath = _sharedData->getIntrinsicHandEyePath();

	cv::Mat intrinsic, distortion;
	readIntrinsic(intrinsicLeftPath.toStdString(),
				  &intrinsic, &distortion,
				  CAMERA_MATRIX_NAME,
				  DISTORTION_COEFF_NAME );


	cv::Mat rvec;
	cv::Mat tvec;
	cv::Matx44d res;

	cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);

	//Clear previous camera poses
	_camPoses.clear();
     double mean_reproj = 0; int count = 0;
/*
	std::vector<tiv::pose> A; // wrl_frame = A[i] * cam_frame
	tiv::pose cam;
	std::vector<tiv::pose> _camtest;

	cam.q = tiv::quat(0.023632, 0.966909, -0.009164, -0.253858);
	cam.t = tiv::pt3d(-15.150674, -6.368933, 401.177889);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.056871, 0.934484, -0.021497, -0.350776);
	cam.t = tiv::pt3d(-17.84849, -5.0085, 381.268701);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.049817, -0.939364, -0.011665, -0.339082);
	cam.t = tiv::pt3d(-1.667783, -8.996447, 383.522726);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.014428, 0.97458, -0.008093, 0.22343);
	cam.t = tiv::pt3d(1.014743, -9.686424, 386.576722);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.247894, 0.937881, -0.001932, -0.242742);
	cam.t = tiv::pt3d(-14.217108, -11.13798, 402.882019);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.332928,0.922519,0.028841,-0.193094);
	cam.t = tiv::pt3d(-2.877023,-17.389621,401.10735);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.195227,-0.952634,-0.059617,-0.225436);
	cam.t = tiv::pt3d(5.764908,-9.271883,397.437079);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.257967,-0.949168,-0.029072,-0.178012);
	cam.t = tiv::pt3d(2.803173,-0.989031,403.479942);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.148484,0.969982,-0.062019,0.182321);
	cam.t = tiv::pt3d(-1.27019,-7.380023,463.426145);
	_camtest.push_back(cam);

	cam.q = tiv::quat(0.268753,-0.929181,0.02231,0.252779);
	cam.t = tiv::pt3d(-12.043013,-11.371784,377.971354);
	_camtest.push_back(cam);
*/
	try{

            for(int i = 0; i< (int)_objectPoints.size(); i++){
                std::vector<cv::Point3f> objPoints = _objectPoints[i];
				std::vector<cv::Point2f> imgPoints = _imagePoints[i];

                std::cout << "objPoints " << objPoints.size() << std::endl;
                std::cout << "imgPoints " << imgPoints.size() << std::endl;

                assert(objPoints.size() == imgPoints.size());

                try{

                    //	cv::solvePnPRansac(objPoints,imgPoints,intrinsic,distortion,rvec,tvec,true,200,6.0);
                        cv::solvePnP(objPoints,imgPoints,intrinsic,distortion,rvec,tvec,false,CV_ITERATIVE);
                        cv::Rodrigues(rvec, rotationMatrix);
                }catch(cv::Exception &e){
                    std::cerr << "OpenCV Exception in HandEyeCalibration::computeCameraPose(): " << e.what() << std::endl;
                    Q_EMIT consoleSignal("OpenCV Exception in HandEyeCalibration::computeCameraPose(): " + QString::fromStdString(e.what()) +"\n", dti::VerboseLevel::ERROR);
                }

                Eigen::MatrixXd Proj(3, 4);
                Eigen::Matrix3d A;
                Eigen::MatrixXd Wrl(4, 3);
                for (int _i = 0; _i < 3;_i++){
                    for (int _j = 0; _j < 3;_j++)
                    {
                        A(_i, _j) =  intrinsic.at<double>(_i,_j);
                        Proj(_i, _j) = rotationMatrix.at<double>(_i,_j);
                    }
                    Proj(_i, 3) = tvec.at<double>(_i);
                    Wrl(0,_i) = objPoints[_i].x;
                    Wrl(1,_i) = objPoints[_i].y;
                    Wrl(2,_i) = objPoints[_i].z;
                    Wrl(3,_i) = 1;
                }

                Eigen::MatrixXd Img = A * Proj* Wrl;
                Eigen::MatrixXd ImgMeasured(2, 3);
				for (int _i = 0; _i < 3;_i++){
					Img(0, _i) /= Img(2, _i);
					Img(1, _i) /= Img(2, _i);
					Img(2, _i) /= Img(2, _i);

					ImgMeasured(0,_i) = imgPoints[_i].x;
					ImgMeasured(1,_i) = imgPoints[_i].y;
				}

				//std::cout << "Image points projected: " << std::endl;
				//std::cout << Img << std::endl;
				//std::cout << "Image points measured: " << std::endl;
				//std::cout << ImgMeasured << std::endl;


                std::vector<float> reprojErrs;
				double reprojerr = computeReprojectionErrors(objPoints,imgPoints,rvec,tvec,intrinsic,distortion);

                mean_reproj+=reprojerr;

				Eigen::Matrix3d R;
				R(0,0) = rotationMatrix.at<double>(0); R(0,1) = rotationMatrix.at<double>(1); R(0,2) = rotationMatrix.at<double>(2);
				R(1,0) = rotationMatrix.at<double>(3); R(1,1) = rotationMatrix.at<double>(4); R(1,2) = rotationMatrix.at<double>(5);
				R(2,0) = rotationMatrix.at<double>(6); R(2,1) = rotationMatrix.at<double>(7); R(2,2) = rotationMatrix.at<double>(8);

				std::cout << "====================REPROJECTION ERROR:===================== " << std::endl;
				std::cout << reprojerr << std::endl;

				std::cout << "====================TRANSLATION VECTOR:===================== " << std::endl;
				std::cout << tvec << std::endl;

				std::cout << "====================ROTATION MATRIX:===================== " << std::endl;
				std::cout << R << std::endl;

			//	Eigen::Vector3d rpy1;
			//	tiv::ti_mat2rpy(R,rpy1);
			//	std::cout << rpy1 << std::endl;
			/*	Eigen::Matrix3d RZ;
				RZ(0,0) = -1; RZ(0,1) = 0; RZ(0,2) = 0;
				RZ(1,0) = 0; RZ(1,1) = -1; RZ(1,2) = 0;
				RZ(2,0) = 0;        RZ(2,1) = 0;       RZ(2,2) = 1;

				Eigen::Matrix3d RES = RZ * R;
*/

			  //tiv::pose pose = _camtest[i];
			//	Eigen::Matrix3d te = cam.q.toMatrix();
			//	Eigen::Vector3d rpy;
			//	tiv::ti_mat2rpy(te,rpy);

			//	std::cout << rpy << std::endl;
			//	Eigen::Matrix4d test = pose.toMatrix();
			//	std::cout << test << std::endl;


				tiv::pose p;
				p.q.fromMatrix(R);
				p.t.x = tvec.at<double>(0);
				p.t.y = tvec.at<double>(1);
				p.t.z = tvec.at<double>(2);
/*
				std::cout << "====================REPROJECTION ERROR:===================== " << std::endl;
				std::cout << reprojerr << std::endl;
				std::cout << "====================TRANSLATION VECTOR:===================== " << std::endl;
				std::cout << "x: " << p.t.x << " y: " << p.t.y << " z: " << p.t.z  << std::endl;

				std::cout << "====================QUAT VECTOR:===================== " << std::endl;
				std::cout << "w: " << p.q.w << " x: " << p.q.x << " y: " << p.q.y << " z: " << p.q.z << std::endl;

*/
				//Store camera poses from camera pose estimation
				_camPoses.push_back(p);
                count = i;
			}
		}catch(cv::Exception &h)
		{
			std::cout << h.what() << std::endl;
		}

     Q_EMIT consoleSignal(QString("Finish.. Mean reprojection error for all ").append(QString::number(count)).append(" views = ").append(QString::number(mean_reproj/count)), dti::VerboseLevel::INFO);

}

double HandEyeCalibration::calibrate()
{
	if(_camPoses.size() == 0){
            Q_EMIT consoleSignal("Cannot preform hand/eye calibration.\n  --> No Camera poses computed. Please detect corners and calculate camera poses!!\n", dti::VerboseLevel::WARN);
            return 0;
    }

	if(_robPoses.size() == 0){

        if(!_sharedData->getRobotPosePath().isEmpty()){
            loadRobotPoses();
        }else{
            Q_EMIT consoleSignal("Cannot preform hand/eye calibration.\n  --> No Robot poses not recorded or loaded.\n", dti::VerboseLevel::WARN);
            return 0;
        }
    }

	if((_robPoses.size() == _camPoses.size()) && _robPoses.size() != 0 )
	{
	/*	for(int i = 0; i<= (int)_robPoses.size()-1; i++){
				tiv::pose rp = _robPoses[i];
				Eigen::Matrix3d R = rp.q.toMatrix();
				Eigen::Vector3d rpy;
				tiv::ti_mat2rpy(R,rpy);
				std::cout << "x: " << rp.t.x << " y: " << rp.t.y << " z: " << rp.t.z << std::endl;
				std::cout << "r: " << rpy[0] << "p: " << rpy[1] << " y: " << rpy[2] << std::endl;
				std::cout << "w: " << rp.q.w << "x: " << rp.q.x << " y: " << rp.q.y << " z: " << rp.q.z << std::endl;

		}

		for(int i = 0; i<= (int)_camPoses.size()-1; i++){
						tiv::pose rp = _camPoses[i];
						Eigen::Matrix3d R = rp.q.toMatrix();
						std::cout << "x: " << rp.t.x << " y: " << rp.t.y << " z: " << rp.t.z << std::endl;
		}
*/
		//RobotFlags robFlags = _sharedData->getRobotFlags();
		if(!_isInverted){
			std::cout << "Inverting robot poses!!" << std::endl;
			//Invert robot poses. result in robot pose expressed base in tool coordinates
			for(int i = 0; i<= (int)_robPoses.size()-1; i++){
					tiv::pose rp = _robPoses[i];
					tiv::pose invp = rp.getInv();
					_robPoses[i] = invp;
		}
		}

		tiv::pose X, Z;
		tiv::dornaika(_robPoses, _camPoses, Z, X, true);//true

		Eigen::Matrix4d _heTransform = X.toMatrix();

		_handEyeTransform.at<double>(0,0) = _heTransform(0,0); 	_handEyeTransform.at<double>(0,1) = _heTransform(0,1); 	_handEyeTransform.at<double>(0,2) = _heTransform(0,2); 	_handEyeTransform.at<double>(0,3) = _heTransform(0,3);
		_handEyeTransform.at<double>(1,0) = _heTransform(1,0); 	_handEyeTransform.at<double>(1,1) = _heTransform(1,1); 	_handEyeTransform.at<double>(1,2) = _heTransform(1,2); 	_handEyeTransform.at<double>(1,3) = _heTransform(1,3);
		_handEyeTransform.at<double>(2,0) = _heTransform(2,0); 	_handEyeTransform.at<double>(2,1) = _heTransform(2,1); 	_handEyeTransform.at<double>(2,2) = _heTransform(2,2); 	_handEyeTransform.at<double>(2,3) = _heTransform(2,3);
		_handEyeTransform.at<double>(3,0) = _heTransform(3,0); 	_handEyeTransform.at<double>(3,1) = _heTransform(3,1); 	_handEyeTransform.at<double>(3,2) = _heTransform(3,2); 	_handEyeTransform.at<double>(3,3) = _heTransform(3,3);

		//std::cout << "X estimated: " << std::endl;
		//std::cout << _handEyeTransform << std::endl;

		Eigen::Matrix4d _ctTransform = Z.toMatrix();

		_calibrationTargetTransform.at<double>(0,0) = _ctTransform(0,0); 	_calibrationTargetTransform.at<double>(0,1) = _ctTransform(0,1); 	_calibrationTargetTransform.at<double>(0,2) = _ctTransform(0,2); 	_calibrationTargetTransform.at<double>(0,3) = _ctTransform(0,3);
		_calibrationTargetTransform.at<double>(1,0) = _ctTransform(1,0); 	_calibrationTargetTransform.at<double>(1,1) = _ctTransform(1,1); 	_calibrationTargetTransform.at<double>(1,2) = _ctTransform(1,2); 	_calibrationTargetTransform.at<double>(1,3) = _ctTransform(1,3);
		_calibrationTargetTransform.at<double>(2,0) = _ctTransform(2,0); 	_calibrationTargetTransform.at<double>(2,1) = _ctTransform(2,1); 	_calibrationTargetTransform.at<double>(2,2) = _ctTransform(2,2); 	_calibrationTargetTransform.at<double>(2,3) = _ctTransform(2,3);
		_calibrationTargetTransform.at<double>(3,0) = _ctTransform(3,0); 	_calibrationTargetTransform.at<double>(3,1) = _ctTransform(3,1); 	_calibrationTargetTransform.at<double>(3,2) = _ctTransform(3,2); 	_calibrationTargetTransform.at<double>(3,3) = _ctTransform(3,3);

		Eigen::Matrix3d matA, matX, matZ, matB, XA, BZ;
		std::vector<double> v = Z.q.toVector();
		matZ << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
		v = X.q.toVector();
		matX << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];

		tiv::pose xa, bz;
		double meanTerror = 0;
		double meanRerror = 0;
		int count = 0;
		for (size_t i = 0; i < _camPoses.size(); i++) {
			v = _camPoses[i].q.toVector();
			matA << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
			v = _robPoses[i].q.toVector();
			matB << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];

			XA = matX * matA;
			BZ = matB * matZ;

			Eigen::Matrix3d N = XA - BZ;
			tiv::pt3d t = X.q.mul(_camPoses[i].t) + X.t - (_robPoses[i].q.mul(Z.t) + _robPoses[i].t);

			std::cout  << std::endl << "|| Rx * Ra[i] - Rb[i] * Rz ||_2 = " << N.norm() << std::endl;
			std::cout << N << std::endl;
			std::cout << N.norm() << std::endl;

			meanRerror += N.norm();

			std::cout << "|| Rx * Ta[i] + Tx - (Rb[i] * Tz + Tb[i]) ||_2 = " << t.norm() << std::endl;
			std::cout << t.norm() << std::endl;

			meanTerror += t.norm();

			xa = X * _camPoses[i];
			bz = _robPoses[i] * Z;
			count++;
		}

		double tErr =  meanTerror/count;
		std::cout << "Mean translation error = " << tErr << "\n";
		std::stringstream ss;
		ss << "Mean translation error = "; ss << tErr; ss << "\n";
        Q_EMIT consoleSignal(QString::fromStdString(ss.str()), dti::VerboseLevel::INFO);
        _rms = tErr;

		ss.str("");
		double rErr =  meanRerror/count;
		std::cout << "Mean rotation error = " << rErr << "\n";
		ss << "Mean rotation error = "; ss << rErr; ss << "\n";
        Q_EMIT consoleSignal(QString::fromStdString(ss.str()), dti::VerboseLevel::INFO);
		std::cout << ss << std::endl;

	}else
        Q_EMIT consoleSignal(QString("Number of camera poses N=").append(QString::number(_camPoses.size())).append(
                             " and robot poses N=").append(QString::number(_robPoses.size())).append(
                             " is not equal!!\n"), dti::VerboseLevel::WARN);

        _rms = 1;
	return 0;
}

void HandEyeCalibration::printCalibResults(){
	if(_camPoses.size() > 0 && _robPoses.size() > 0){
		printResults();
	}
}

std::string HandEyeCalibration::calibResultsAsString()
{
	std::stringstream ss;
    ss << " --> (HAND/EYE) CALIBRATION RESULTS <-- \n";

	ss << "image_count: " 			<< _validImgs			<< "\n\n";
	ss << "image_size: [" 			<< _imageSize.height 	<< ", " << _imageSize.width 	<< "]\n\n";
	ss << "square_size: "			<< _squareSize			<< "\n\n";
	ss << "Pattern size: [" 		<< _boardSize.height 	<< ", " << _boardSize.width	<< "]\n\n";

	ss << "HAND_EYE_TRANSFORM [X]:\n"  	 << _handEyeTransform			<< "\n\n";
	ss << "CALIBRATION_TARGET_TRANSFORM [Z]:\n" << _calibrationTargetTransform			<< "\n\n";

    std::cout << ss.str() << std::endl;
	 return ss.str();
}

void HandEyeCalibration::setFilesLists(QStringList files)
{
	_fileList.clear();

	if(_fileList.size() != _fileList.size())
            Q_EMIT consoleSignal("The amount of left and right files does not match!\n --> They MUST be the same <--\n", dti::VerboseLevel::INFO);

	_fileList.append(files);
}

void HandEyeCalibration::resizePoints(int newSize)
{
	_imagePoints.resize(newSize);
	_objectPoints.resize(newSize);
}

} /* namespace perception */
