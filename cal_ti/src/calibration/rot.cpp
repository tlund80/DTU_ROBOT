#include "../../include/cal_ti/rot.hpp"
#include "../../include/cal_ti/tiv_types.hpp"
#include <cfloat>

// matrix equivalent to axis-angle: eaa = angle * axis (be careful when angle is close to 0).
extern "C" void tiv::mat2eaa(const Eigen::Matrix3d& R, Eigen::Vector3d& eaa) {
	const double eps1 = 1e-6;
	const double eps2 = 1e-12;
	double angle(0), x(0), y(0), z(0);
	//Eigen::Vector3d eaa;
	if (fabs(R(0,1) - R(1,0)) < eps1 && fabs(R(0,2) - R(2,0)) < eps1 && fabs(R(1,2) - R(2,1)) < eps1) {
		if (fabs(R(0,1) + R(1,0)) < eps2 && fabs(R(0,2) + R(2,0)) < eps2 && fabs(R(1,2) + R(2,1)) < eps2 && fabs(R(0,0) + R(1,1) + R(2,2)) < eps2) {
			eaa << 0,0,0; 
			return;
			//return eaa;
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
		return;// eaa;
	}

	double s = sqrt(pow(R(2,1) - R(1,2), 2) + pow(R(0,2) - R(2,0), 2) + pow(R(1,0) - R(0,1),2));
	if (fabs(s) < eps1) {
		s = 1.0;
	}

	angle = acos((R(0,0) + R(1,1) + R(2,2) - 1.0) / 2);
	x = (R(2,1) - R(1,2)) / s;
	y = (R(0,2) - R(2,0)) / s;
	z = (R(1,0) - R(0,1)) / s;
	
	eaa << angle * x, angle * y, angle * z;
	return;// eaa;
}


extern "C" void tiv::eaa2mat(const Eigen::Vector3d& eaa, Eigen::Matrix3d& R) {
	Eigen::Vector3d axis = eaa;
	double angle = axis.norm();

	if (angle < DBL_EPSILON) {
		axis << 0, 0, 1;
	} else {
		axis.normalize();
	}

	//Eigen::Matrix3d R;
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
	//return R;
}

// in this conversion, rpy = {roll, pitch, yaw} are representing rotation w.r.t. z-axis, y-axis, and x-axis respectively.
extern "C" void tiv::ti_mat2rpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy) {
	//Eigen::Vector3d _rpy;
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
            rpy(0) = 0;
			rpy(1) = static_cast<double>(-tiv::pi / 2);
            rpy(2) = - atan2(r12, r22);
        }

        // If beta == 90 deg:
        else {
            rpy(0) = 0;
			rpy(1) = static_cast<double>(tiv::pi / 2);
            rpy(2) = atan2(r12, r22);
        }

    } else {
        rpy(1) = static_cast<double>(atan2(sin_beta, cos_beta));
        //_rpy(0) = static_cast<double>(atan2(r21 / cos_beta, r11 / cos_beta));
		rpy(0) = static_cast<double>(atan2(r21, r11));
		//_rpy(2) = static_cast<double>(atan2(r32 / cos_beta, r33 / cos_beta));
        rpy(2) = static_cast<double>(atan2(r32, r33));
    }
	//return _rpy;
}

extern "C" void tiv::ti_rpy2mat(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R) {
	//Eigen::Matrix3d R;
	double a = rpy(0)/*roll*/, b = rpy(1)/*pitch*/, c = rpy(2)/*yaw*/;
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
	//return R;
}

extern "C" void tiv::mat2hab(const Eigen::Matrix3d& R, Eigen::Vector3d& hab) {
	if (R(1, 0) > 0.999999999) {
		hab(0) = atan2(R(0, 2), R(2, 2));		
		hab(1) = tiv::pi * 0.5;
		hab(2) = 0.0;
	} else if (R(1, 0) < -0.999999999) {
		hab(0) = atan2(R(0, 2), R(2, 2));		
		hab(1) = -tiv::pi * 0.5;
		hab(2) = 0.0;
	} else {
		hab(0) = atan2(-R(2, 0), R(0, 0));
		hab(1) = asin(R(1, 0));
		hab(2) = atan2(-R(1, 2), R(1, 1));
	}
}

extern "C" void tiv::hab2mat(const Eigen::Vector3d& hab, Eigen::Matrix3d& R) {
	double sa = std::sin(hab(1));
	double ca = std::cos(hab(1));
	double sb = std::sin(hab(2));
	double cb = std::cos(hab(2));
	double sh = std::sin(hab(0));
	double ch = std::cos(hab(0));
	
	R(0, 0) =  ch * ca;		R(0, 1) = -ch * sa * cb + sh * sb;			R(0, 2) =  ch * sa * sb + sh * cb;
	R(1, 0) =  sa;			R(1, 1) =  ca * cb;							R(1, 2) = -ca * sb ;
	R(2, 0) = -sh * ca;		R(2, 1) =  sh * sa * cb + ch * sb;			R(2, 2) = -sh * sa * sb + ch * cb;
}
