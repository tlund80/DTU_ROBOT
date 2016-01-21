// tiv_test.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "../../include/cal_ti/tiv_types.hpp"
#include "../../include/cal_ti/dornaika.hpp"
#include "../../include/cal_ti/rot.hpp"
#include <iostream>
#include <eigen3/Eigen/Core>
/*
Eigen::Vector3d mat2eaa(const Eigen::Matrix3d& R) {
	const double eps1 = static_cast<double>(1e-6);
	const double eps2 = static_cast<double>(1e-12);
	double angle(0), x(0), y(0), z(0);
	Eigen::Vector3d eaa;
	if (fabs(R(0,1)-R(1,0)) < eps1 && fabs(R(0,2)-R(2,0)) < eps1 && fabs(R(1,2)-R(2,1)) < eps1) {
		if (fabs(R(0,1)+R(1,0)) < eps2 && fabs(R(0,2)+R(2,0)) < eps2 && fabs(R(1,2)+R(2,1)) < eps2 && fabs(R(0,0) + R(1,1) + R(2,2)) < eps2) {
			eaa << 0,0,0; return eaa;
		}
		
		angle = tiv::pi;
		double xx = (R(0,0) + 1) / 2;
		double yy = (R(1,1) + 1) / 2;
		double zz = (R(2,2) + 1) / 2;
		double xy = (R(0,1) + R(1,0)) / 4;
		double xz = (R(0,2) + R(2,0)) / 4;
		double yz = (R(1,2) + R(2,1)) / 4;
		if ((xx > yy) && (xx > zz)) {
			if (xx < eps1) {
				x = 0;
				y = z = sqrt(2.0) / 2.0;
			} else {
				x = sqrt(xx);
				y = xy / x;
				z = xz / x;
			}
		} else if (yy > zz) {
			if (yy < eps1) {
				x = z = sqrt(2.0) / 2.0;
				y = 0;
			} else {
				y = sqrt(yy);
				x = xy / y;
				z = yz / y;
			}
		} else {
			if (zz < eps1) {
				x = y = sqrt(2.0) / 2.0;
				z = 0;
			} else {
				z = sqrt(zz);
				x = xz / z;
				y = yz / z;
			}
		}

		eaa << angle * x, angle * y, angle * z;
		return eaa;
	}

	double s = sqrt(pow(R(2,1)-R(1,2), 2) + pow(R(0,2)-R(2,0), 2) + pow(R(1,0)-R(0,1),2));
	if (fabs(s) < eps1) {
		s = 1.0;
	}

	angle = acos((R(0,0) + R(1,1) + R(2,2) - 1.0) / 2);
	x = (R(2,1) - R(1,2)) / s;
	y = (R(0,2) - R(2,0)) / s;
	z = (R(1,0) - R(0,1)) / s;
	
	eaa << angle * x, angle * y, angle * z;
	return eaa;
}

Eigen::Matrix3d eaa2mat(const double _a, const double _b, const double _c) {
	Eigen::Vector3d axis; axis << _a, _b, _c;
	double angle = axis.norm();

	if (angle < DBL_EPSILON) {
		axis << 0, 0, 1;
	} else {
		axis.normalize();
	}

	Eigen::Matrix3d R;
	double c = cos(angle);
	double s = sin(angle);
	double t = 1 - c;
	R(0, 0) = c + axis(0) * axis(0) * t;
	R(1, 1) = c + axis(1) * axis(1) * t;
	R(2, 2) = c + axis(2) * axis(2) * t;
	double tmp1 = axis(0) * axis(1) * t;
	double tmp2 = axis(2) * s;
	R(1, 0) = tmp1 + tmp2;
	R(0, 1) = tmp1 - tmp2;
	tmp1 = axis(0) * axis(2) * t;
	tmp2 = axis(1) * s;
	R(2, 0) = tmp1 - tmp2;
	R(0, 2) = tmp1 + tmp2;
	tmp1 = axis(1) * axis(2) * t;
	tmp2 = axis(0) * s;
	R(2, 1) = tmp1 + tmp2;
	R(1, 2) = tmp1 - tmp2;
	return R;
}
/*
Eigen::Vector3d mat2rpy(const Eigen::Matrix3d& R) {
	const double Pi = tiv::pi;
	Eigen::Vector3d _rpy;
    const double r11 = R(0, 0);
    const double r12 = R(0, 1);
    const double r21 = R(1, 0);
    const double r22 = R(1, 1);
    const double r31 = R(2, 0);
    const double r32 = R(2, 1);
    const double r33 = R(2, 2);

    double sum = r11*r11 + r21*r21;

    // We check for rounding errors in input so that we don't take the sqrt of
    // some negative number.
    if (sum < 0) sum = 0;
    // TODO: Check that if sum < 0 then sum ~= 0 also.

    const double cos_beta = sqrt(sum);
    const double sin_beta = -r31;

    // If beta == 90 deg or beta == -90 deg:
    if (fabs(cos_beta) < DBL_EPSILON) {

        // If beta == -90 deg:
        if (sin_beta < 0) {
            _rpy(0) = 0;
            _rpy(1) = static_cast<double>(-Pi / 2);
            _rpy(2) = - atan2(r12, r22);
        }

        // If beta == 90 deg:
        else {
            _rpy(0) = 0;
            _rpy(1) = static_cast<double>(Pi / 2);
            _rpy(2) = atan2(r12, r22);
        }

    } else {
        _rpy(1) = static_cast<double>(atan2(sin_beta, cos_beta));
        //_rpy(0) = static_cast<double>(atan2(r21 / cos_beta, r11 / cos_beta));
		_rpy(0) = static_cast<double>(atan2(r21, r11));
		//_rpy(2) = static_cast<double>(atan2(r32 / cos_beta, r33 / cos_beta));
        _rpy(2) = static_cast<double>(atan2(r32, r33));
    }
	return _rpy;
}

Eigen::Matrix3d rpy2mat(const double roll, const double pitch, const double yaw) {

	Eigen::Matrix3d R;
	double a = roll, b = pitch, c = yaw;
    const double ca = cos(a);
    const double sa = sin(a);
    const double cb = cos(b);
    const double sb = sin(b);
    const double cc = cos(c);
    const double sc = sin(c);

    R(0, 0) =    ca * cb,
    R(0, 1) =    ca * sb * sc-sa * cc,
    R(0, 2) =    ca * sb * cc+sa * sc,

    R(1, 0) =    sa * cb,
    R(1, 1) =    sa * sb * sc+ca * cc,
    R(1, 2) =    sa * sb * cc-ca * sc,

    R(2, 0) =    -sb,
    R(2, 1) =    cb * sc,
    R(2, 2) =    cb * cc;
	return R;
}
*/

/*
tiv::quat mat2quat(const Eigen::Matrix3d& R) {
	tiv::quat q;
	q.fromMatrix(R);
	return q;
}

tiv::quat ypr2quat(const double yaw , const double pitch, const double roll) {
	// roll: rx, pitch: ry, yaw: rz, a common convention. supply input parameter of this function accordingly.
	double w, x, y, z;
	w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
	z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);

	double rx = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	double ry = asin(2 * (w * y - z * x));
	double rz = atan2(2 * (w * z + x * y), 1 - 2 * (y *y + z *z));

	std::cout << "\nyaw: " << yaw << std::endl;
	std::cout << "pitch: " << pitch << std::endl;
	std::cout << "roll: " << roll << std::endl;

	std::cout << "\nrx: " << rx << std::endl;
	std::cout << "ry: " << ry << std::endl;
	std::cout << "rz: " << rz << std::endl;

	return tiv::quat(w, x, y, z);
}
*/
//tiv::quat rpy2quat(const double roll, const double pitch , const double yaw) {

/*
tiv::quat rpy2quat(const double a , const double b, const double g, const int index) {
	double c1 = cos(a), s1 = sin(a), c2 = cos(b), s2 = sin(b), c3 = cos(g), s3 = sin(g);

	std::string seq;
	switch (index) {
	case 0:
		seq = "xzy"; break;
	case 1:
		seq = "xyz"; break;
	case 2:
		seq = "yxz"; break;
	case 3:
		seq = "yzx"; break;
	case 4:
		seq = "zyx"; break;
	case 5:
		seq = "zxy"; break;
	case 6:
		seq = "xzx"; break;
	case 7:
		seq = "xyx"; break;
	case 8:
		seq = "yxy"; break;
	case 9:
		seq = "yzy"; break;
	case 10:
		seq = "zyz"; break;
	case 11:
		seq = "zxz"; break;
	}

	std::vector<double> r(9, 0);
	if (seq.compare("xzy") == 0) {
		r[0] = c2*c3;	r[1] = -s2;	r[2] = c2*s3;
		r[3] = s1*s3+c1*c3*s2;	r[4] = c1*c2;	r[5] = c1*s2*s3 - c3*s1;
		r[6] = c3*s1*s2-c1*s3;	r[7] = c2*s1;	r[8] = c1*c3 + s1*s2*s3;
	} else if (seq.compare("xyz") == 0) {
		r[0] = c2*c3;	r[1] = -c2*s3;	r[2] = s2;
		r[3] = c1*s3+c3*s1*s2;	r[4] = c1*c3-s1*s2*s3;	r[5] = -c2*s1;
		r[6] = s1*s3-c1*c3*s2;	r[7] = c3*s1+c1*s2*s3;	r[8] = c1*c2;
	} else if (seq.compare("yxz") == 0) {
		r[0] = c1*c3+s1*s2*s3;	r[1] = c3*s1*s2-c1*s3;	r[2] = c2*s1;
		r[3] = c2*s3;	r[4] = c2*c3;	r[5] = -s2;
		r[6] = c1*s2*s3-c3*s1;	r[7] = s1*s3+c1*c2*s2;	r[8] = c1*c2;
	} else if (seq.compare("yzx") == 0) {
		r[0] = c1*c2;	r[1] = s1*s3-c1*c3*s2;	r[2] = c3*s1+c1*s2*s3;
		r[3] = s2;	r[4] = c2*c3;	r[5] = -c2*s3;
		r[6] = -c2*s1;	r[7] = c1*s3+c3*s1*s2;	r[8] = c1*c3-s1*s2*s3;
	} else if (seq.compare("zyx") == 0) {
		r[0] = c1*c2;	r[1] = c1*s2*s3-c3*s1;	r[2] = s1*s3+c1*c3*s2;
		r[3] = c2*s1;	r[4] = c1*c3+s1*s2*s3;	r[5] = c3*s1*s2-c1*s3;
		r[6] = -s2;	r[7] = c2*s3;	r[8] = c2*c3;
	} else if (seq.compare("zxy") == 0) {
		r[0] = c1*c3-s1*s2*s3;	r[1] = -c2*s1;	r[2] = c1*s3+c3*s1*s2;
		r[3] = c3*s1+c1*s2*s3;	r[4] = c1*c2;	r[5] = s1*s3-c1*c3*s2;
		r[6] = -c2*s3;	r[7] = s2;	r[8] = c2*c3;
	} else if (seq.compare("xzx") == 0) {
		r[0] = c2;	r[1] = -c3*s2;	r[2] = s2*s3;
		r[3] = c1*s2;	r[4] = c1*c2*c3-s1*s3;	r[5] = -c3*s1 - c1*c2*s3;
		r[6] = s1*s2;	r[7] = c1*s3+c2*c3*s1;	r[8] = c1*c3-c2*s1*s3;
	} else if (seq.compare("xyx") == 0) {
		r[0] = c2;	r[1] = s2*s3;	r[2] = c3*s2;
		r[3] = s1*s2;	r[4] = c1*c3-c2*s1*s3;	r[5] = -c1*s3-c2*c3*s1;
		r[6] = -c1*s2;	r[7] = c3*s1+c1*c2*s3;	r[8] = c1*c2*c3-s1*s3;
	} else if (seq.compare("yxy") == 0) {
		r[0] = c1*c3-c2*s1*s3;	r[1] = s1*s2;	r[2] = c1*s3+c2*c3*s1;
		r[3] = s2*s3;	r[4] = c2;	r[5] = -c3*s2;
		r[6] = -c3*s1-c1*c2*s3;	r[7] = c1*s2;	r[8] =c1*c2*c3-s1*s3 ;
	} else if (seq.compare("yzy") == 0) {
		r[0] = c1*c2*c3-s1*s3;	r[1] = -c1*s2;	r[2] = c3*s1+c1*c2*s3;
		r[3] = c3*s2;	r[4] = c2;	r[5] = s2*s3;
		r[6] = -c1*s3-c2*c3*s1;	r[7] = s1*s2;	r[8] = c1*c3-c2*s1*s3;
	} else if (seq.compare("zyz") == 0) {
		r[0] = c1*c2*c3-s1*s3;	r[1] = -c3*s1-c1*c2*s3;	r[2] = c1*s2;
		r[3] = c1*s3+c2*c3*s1;	r[4] = c1*c3-c2*s1*s3;	r[5] = s1*s2;
		r[6] = -c3*s2;	r[7] = s2*s3;	r[8] = c2;
	} else if (seq.compare("zxz") == 0) {
		r[0] = c1*c3-c2*s1*s3;	r[1] = -c1*s3-c2*c3*s1;	r[2] = s1*s2;
		r[3] = c3*s1+c1*c2*s3;	r[4] = c1*c2*c3-s1*s3;	r[5] = -c1*s2;
		r[6] = s2*s3;	r[7] = c3*s2;	r[8] = c2;
	}

	tiv::quat q;
	q.fromVector(r);
	return q;
}

*/


int test()
{
	const bool tool_is_gripper = false;

	std::vector<tiv::pose> A; // wrl_frame = A[i] * cam_frame
	tiv::pose cam;
	std::vector<tiv::pose> B;
	tiv::pose tool;

	cam.q = tiv::quat(0.023632, 0.966909, -0.009164, -0.253858);
	cam.t = tiv::pt3d(-15.150674, -6.368933, 401.177889);
	A.push_back(cam);

	cam.q = tiv::quat(0.056871, 0.934484, -0.021497, -0.350776);
	cam.t = tiv::pt3d(-17.84849, -5.0085, 381.268701);
	A.push_back(cam);

	cam.q = tiv::quat(0.049817, -0.939364, -0.011665, -0.339082);
	cam.t = tiv::pt3d(-1.667783, -8.996447, 383.522726);
	A.push_back(cam);

	cam.q = tiv::quat(0.014428, 0.97458, -0.008093, 0.22343);
	cam.t = tiv::pt3d(1.014743, -9.686424, 386.576722);
	A.push_back(cam);
	
	cam.q = tiv::quat(0.247894, 0.937881, -0.001932, -0.242742);
	cam.t = tiv::pt3d(-14.217108, -11.13798, 402.882019);
	A.push_back(cam);

	cam.q = tiv::quat(0.332928,0.922519,0.028841,-0.193094);
	cam.t = tiv::pt3d(-2.877023,-17.389621,401.10735);
	A.push_back(cam);

	cam.q = tiv::quat(0.195227,-0.952634,-0.059617,-0.225436);
	cam.t = tiv::pt3d(5.764908,-9.271883,397.437079);
	A.push_back(cam);

	cam.q = tiv::quat(0.257967,-0.949168,-0.029072,-0.178012);
	cam.t = tiv::pt3d(2.803173,-0.989031,403.479942);
	A.push_back(cam);

	cam.q = tiv::quat(0.148484,0.969982,-0.062019,0.182321);
	cam.t = tiv::pt3d(-1.27019,-7.380023,463.426145);
	A.push_back(cam);

	cam.q = tiv::quat(0.268753,-0.929181,0.02231,0.252779);
	cam.t = tiv::pt3d(-12.043013,-11.371784,377.971354);
	A.push_back(cam);

	Eigen::Matrix3d R;
	Eigen::Vector3d eaa, rpy;

	Eigen::MatrixXd qt(6, 10);

	if (tool_is_gripper) {
		qt << 0.09309,	0.101635,	-0.09235,	-0.06481,	0.07493,	0.05644,	-0.07295,	-0.06308,	-0.08807,	0.093817,
			0.30926,	0.30213,	0.33043,	0.3119,		0.25173,	0.23637,	0.38167,	0.42018,	0.26298,	0.39817,
			0.17435,	0.12907,	0.172,		0.1766,		0.10577,	0.07723,	0.22365,	0.23521,	0.21133,	0.20482,
			-0.03315,	-0.03087,	0.05013,	-0.04761,	0.10214,	0.16693,	0.20757,	0.14203,	-0.21202,	-0.2147,
			0.49601,	0.7026,		-0.71115,	-0.47091,	0.4666,		0.34027,	-0.443748,	-0.35504,	-0.35763,	0.4494,
			0.05175,	0.12012,	-0.10595,	0.05948,	0.55027,	0.73067,	-0.43395,	-0.53791,	0.36426,	-0.59793;

		for (int i = 0; i < qt.cols(); i++) {
			tool.t = tiv::pt3d(qt(0, i), qt(1, i), qt(2, i)); tool.t *= 1000;
			rpy << qt(3, i), qt(4, i), qt(5, i);
			tiv::ti_rpy2mat(rpy, R);
			tool.q.fromMatrix(R);
			B.push_back(tool.getInv());
		}
	} else {
		qt <<-234.02,	-290.79,	284.25,		198.59,		-204.57,	-162.92,	209.77,		170.33,		204.87,		-233.79,
			-289.88,	-261.08,	-353.69,	-302.69,	-110.16,	-54.75,		-480.65,	-558.32,	-182.62,	-537.8,	
			440.09,		359.17,		400.39,		445.35,		337.44,		291.62,		471.49,		478.77,		475.85,		430.51,	
			0.061,		0.0913,		0.0131,		-0.0445,	0.0408,		-0.0465,	0.141,		0.0578,		-0.1974,	0.0946, // 4-th
			2.6447,		2.4353,		-2.4267,	-2.6685,	2.5738,		2.5965,		-2.6127,	-2.6718,	-2.7101,	2.5508,	
			-0.0799,	-0.1603,	-0.1061,	0.0641,		-0.693,		-0.9506,	-0.5118,	-0.7,		0.4454,		0.7183; // 6-th

		for (int i = 0; i < qt.cols(); i++) {
			eaa << qt(3, i), qt(4, i), qt(5, i);
			tiv::eaa2mat(eaa, R);
			tool.q.fromMatrix(R);
			tool.t = tiv::pt3d(qt(0, i), qt(1, i), qt(2, i));
			B.push_back(tool.getInv());
		}
	}

	tiv::pose X, Z;
	tiv::dornaika(B, A, Z, X, true);

	Eigen::Matrix3d matA, matX, matZ, matB, XA, BZ;
	std::vector<double> v = Z.q.toVector();	
	matZ << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
	v = X.q.toVector();
	matX << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];

	tiv::pose xa, bz;
	for (size_t i = 0; i < A.size(); i++) {
		v = A[i].q.toVector();
		matA << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
		v = B[i].q.toVector();
		matB << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];

		XA = matX * matA;
		BZ = matB * matZ;

		Eigen::Matrix3d N = XA - BZ;
		tiv::pt3d t = X.q.mul(A[i].t) + X.t - (B[i].q.mul(Z.t) + B[i].t);

		std::cout  << std::endl << "|| Rx * Ra[i] - Rb[i] * Rz ||_2 = " << N.norm() << std::endl;
		std::cout << N << std::endl;
		std::cout << N.norm() << std::endl;

		std::cout << "|| Rx * Ta[i] + Tx - (Rb[i] * Tz + Tb[i]) ||_2 = " << t.norm() << std::endl;
		std::cout << t.norm() << std::endl;

		xa = X * A[i];
		bz = B[i] * Z;
	}

	// following camera and tool pose are not used in calibration.	
	// these are data set in order to see the calibration validity.	
	cam.q = tiv::quat(0.009165, -0.999924, -0.001532, 0.008063);
	cam.t = tiv::pt3d(-4.947342, -4.661999, 421.258604);

	if (tool_is_gripper) {
		// from rostopic /ur5/tcp
		tool.t = tiv::pt3d(0.01067,0.32297,0.2313899); tool.t *= 1000;
		rpy << -0.01953,-0.00514,-0.00243;
		tiv::ti_rpy2mat(rpy, R);
		tool.q.fromMatrix(R);
	} else {
		tool.t = tiv::pt3d(-10.09, -325.92, 532.67);
		eaa << -0.0305, -3.1356, -0.0056;
		tiv::eaa2mat(eaa, R);
		tool.q.fromMatrix(R);// = mat2quat(eaa2mat(-0.0305,-3.1356,-0.0056));
	}


	// from ur5 teaching pendent
	std::cout << "\n\n====== Estimating camera pose, A, from hand-eye calibration result ====== " << std::endl;
	std::cout << "A estimated = "<< std::endl;
	tiv::pose Aest = X.getInv() * tool.getInv() * Z;
	std::cout << Aest.toMatrix() << std::endl;
	std::cout << "A pre-calibrated = " << std::endl;
	std::cout << cam.toMatrix() << std::endl;

	std::cout << "Estimated camera pose rotation error = " << (Aest.toMatrix() - cam.toMatrix()).block<3, 3>(0, 0).norm() << std::endl;
	std::cout << "Estimated camera pose translation error = " << (Aest.toMatrix() - cam.toMatrix()).block<3, 1>(0, 3).norm() << std::endl;

	return 0;
}
