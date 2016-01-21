#ifndef TIV_TYPES_H
#define TIV_TYPES_H

#include <eigen3/Eigen/Core>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace tiv { // teknologisk institut vision library for robot vision application.
	static const double eps = 1e-7;
	static const double pi = 3.14159265358979323846;
	static const double rad2deg = 180 / pi;
	static const double deg2rad = pi / 180;

	typedef struct pt2d {
		double u, v;
		pt2d() : u(0), v(0) {}
		pt2d(const pt2d& other) : u(other.u), v(other.v) {}
		pt2d(const double _u, const double _v) { u = _u; v = _v; }
		pt2d& operator = (const pt2d& rhs) {
			u = rhs.u;
			v = rhs.v;
			return *this;
		}
		pt2d& operator += (const pt2d& rhs){
			u += rhs.u;
			v += rhs.v;
			return *this;
		}
		pt2d& operator -= (const pt2d& rhs){
			u -= rhs.u;
			v -= rhs.v;
			return *this;
		}
		pt2d operator + (const pt2d& rhs) const { return pt2d(u + rhs.u, v + rhs.v); }
		pt2d operator - (const pt2d& rhs) const { return pt2d(u - rhs.u, v - rhs.v); }
		pt2d operator - () const{ return pt2d(-u, -v); }
		void transform(const double theta, const pt2d& t){
			double x = cos(theta) * u - sin(theta) * v + t.u;
			double y = sin(theta) * u + cos(theta) * v + t.v;
			u = x;
			v = y;
		}
		pt2d getTransform(const double theta, const pt2d& t) {
			pt2d x;
			x.transform(theta, t);
			return x;
		}
	} pt2d;

	typedef struct pt3d {
		double x, y, z;
		pt3d() : x(0), y(0), z(0) {}
		pt3d(const pt3d& other) { x = other.x; y = other.y; z = other.z; }
		pt3d(const double _x, const double _y, const double _z) { x = _x; y = _y; z = _z; }
		pt3d& operator = (const pt3d& rhs) {
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			return *this;		
		}
		pt3d& operator *= (const double rhs){
			x *= rhs;
			y *= rhs;
			z *= rhs;
			return *this;
		}
		pt3d& operator /= (const double rhs){
			if (fabs(rhs) < eps) {
				throw std::overflow_error("Divide by zero exception");
			}
			x /= rhs;
			y /= rhs;
			z /= rhs;
			return *this;
		}
		pt3d operator * (const double rhs) const { return pt3d(x*rhs, y*rhs, z*rhs); }
		pt3d operator / (const double rhs) const { 
			if (fabs(rhs) < eps) {
				throw std::overflow_error("Divide by zero exception");
			}
			return pt3d(x/rhs, y/rhs, z/rhs); 
		}
		pt3d& operator += (const pt3d& rhs) {
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
			return *this;
		}
		pt3d& operator -= (const pt3d& rhs) {
			x -= rhs.x;
			y -= rhs.y;
			z -= rhs.z;
			return *this;
		}
		pt3d operator + (const pt3d& rhs) const  {	return pt3d(x + rhs.x, y + rhs.y, z + rhs.z);}
		pt3d operator - (const pt3d& rhs) const {	return pt3d(x - rhs.x, y - rhs.y, z - rhs.z);}
		pt3d operator - () const {	return pt3d(-x, -y, -z);}
		double norm() const { return sqrt(x * x + y * y + z * z); }
		double dot(const pt3d& b) const { return x * b.x + y * b.y + z * b.z; }
		pt3d cross(const pt3d& b) const { return pt3d(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);}
		double angle(const pt3d& b) const { 	
			pt3d a = *this;
			double num = a.dot(b);
			double den = a.norm() * b.norm();
			return acos(num / den);
		}
	} pt3d;

	typedef struct uv_wrl {
		std::vector<pt2d> uv;
		pt3d wrl;
		uv_wrl() {
			uv.clear();
			wrl = pt3d(0,0,0);
		}
		uv_wrl(const uv_wrl& other) { uv = other.uv; wrl = other.wrl; }
		uv_wrl(const pt2d& _uv, const pt3d& _wrl) {
			uv.clear();
			uv.push_back(_uv);
			wrl = _wrl;
		}
		uv_wrl(const std::vector<pt2d>& _uv, const pt3d& _wrl) {
			uv = _uv;
			wrl = _wrl;
		}
		uv_wrl& operator = (const uv_wrl& rhs) {
			uv = rhs.uv;
			wrl = rhs.wrl;
			return *this;
		}
	} uv_wrl;

	typedef struct circle {
		double x, y, radius;
		circle() : x(100), y(100), radius(20) {}
		circle(const double cx, const double cy, const double r) {
			x = cx; y = cy; radius = r;
		}
		circle(const circle& other) {
			x = other.x;
			y = other.y;
			radius = other.radius;
		}
		circle& operator = (const circle& rhs) {
			x = rhs.x;
			y = rhs.y;
			radius = rhs.radius;
			return *this;
		}
		bool isInside(const pt2d& uv) {
			return (uv.u - x) * (uv.u - x) + (uv.v - y) * (uv.v - y) < radius * radius ? true : false;
		}
	} circle;

	double dot(const tiv::pt3d& a, const tiv::pt3d& b);
	tiv::pt3d cross(const tiv::pt3d& a, const tiv::pt3d& b);

	typedef struct quat {
		double w, x, y, z;
		quat() :  w(0), x(0), y(0), z(0) {}
		quat(const quat& other) : w(other.w), x(other.x), y(other.y), z(other.z) {}
		quat(const double _w, const double _x, const double _y, const double _z) { w = _w; x = _x; y = _y; z = _z; }
		void inv() {
			double den = w * w + x * x + y * y + z * z;
			if (fabs(den) < eps) {
				throw std::overflow_error("Divide by zero exception");
			}

			w /= den;
			den = -den;
			x /= den;
			y /= den;
			z /= den;
		}
		quat getInv() const {
			quat q(w, x, y, z);
			q.inv();
			return q;
		}
		quat getConj() const { return quat(w, -x, -y, -z);	}
		void conj() {
			x = -x;
			y = -y;
			z = -z;
		}
		void normalize() {
			double den = w * w + x * x + y * y + z * z;
			den = sqrt(den);
			if (fabs(den) < eps) {
				throw std::overflow_error("Divide by zero exception");
			}

			w /= den;
			x /= den;
			y /= den;
			z /= den;
		}
		double length() const {	return sqrt(w * w + x * x + y * y + z * z); }
		quat& operator = (const quat& rhs) {
			w = rhs.w;
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			return *this;
		}
		quat& operator *= (const quat& rhs)  {
			w = rhs.w * w - rhs.x * x - rhs.y * y - rhs.z * z;
			x = rhs.w * x + rhs.x * w - rhs.y * z + rhs.z * y;
			y = rhs.w * y + rhs.x * z + rhs.y * w - rhs.z * x;
			z = rhs.w * z - rhs.x * y + rhs.y * x + rhs.z * w;
			return *this;
		}
		quat operator * (const quat& rhs) const {
			quat q;
			q.w = rhs.w * w - rhs.x * x - rhs.y * y - rhs.z * z;
			q.x = rhs.w * x + rhs.x * w - rhs.y * z + rhs.z * y;
			q.y = rhs.w * y + rhs.x * z + rhs.y * w - rhs.z * x;
			q.z = rhs.w * z - rhs.x * y + rhs.y * x + rhs.z * w;
			return q;
		}
		quat& operator /= (const quat& rhs) {
			double den = rhs.w * rhs.w + rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z;
			if (fabs(den) < eps) {
				throw std::overflow_error("Divide by zero exception");
			}
			double a = rhs.w * w + rhs.x * x + rhs.y * y + rhs.z * z;
			double b = rhs.w * x - rhs.x * w - rhs.y * z + rhs.z * y;
			double c = rhs.w * y + rhs.x * z - rhs.y * w - rhs.z * x;
			double d = rhs.w * z - rhs.x * y + rhs.y * x - rhs.z * w;

			w = a / den;
			x = b / den;
			y = c / den;
			z = d / den;
			return *this;
		}
		quat operator / (const quat& rhs) const {
			double den = rhs.w * rhs.w + rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z;
			if (fabs(den) < eps) { return quat(1, 0, 0, 0); } 
			return quat(
				(rhs.w * w + rhs.x * x + rhs.y * y + rhs.z * z) / den,
				(rhs.w * x - rhs.x * w - rhs.y * z + rhs.z * y) / den,
				(rhs.w * y + rhs.x * z - rhs.y * w - rhs.z * x) / den,
				(rhs.w * z - rhs.x * y + rhs.y * x - rhs.z * w) / den);
		}
		std::vector<double> toVector() const {
			std::vector<double> r(9, 0.0);
			//r[0] = 1 - 2*y*y - 2*z*z;	r[1] = 2*x*y - 2*z*w;		r[2] = 2*x*z + 2*y*w;
			//r[3] = 2*x*y + 2*z*w;		r[4] = 1 - 2*x*x - 2*z*z;	r[5] = 2*y*z - 2*x*w;
			//r[6] = 2*x*z - 2*y*w;		r[7] = 2*y*z + 2*x*w;		r[8] = 1 - 2*x*x - 2*y*y;

			double sqw = w*w;    double sqx = x*x;    double sqy = y*y;    double sqz = z*z;
			// invs (inverse square length) is only required if quaternion is not already normalised  
			double invs = 1 / (sqx + sqy + sqz + sqw);
			double m00 = ( sqx - sqy - sqz + sqw)*invs ; 
			// since sqw + sqx + sqy + sqz =1/invs*invs    
			double m11 = (-sqx + sqy - sqz + sqw)*invs ;    double m22 = (-sqx - sqy + sqz + sqw)*invs ;       
			double tmp1 = x*y;    double tmp2 = z*w;    double m10 = 2.0 * (tmp1 + tmp2)*invs ;   
			double m01 = 2.0 * (tmp1 - tmp2)*invs ;   
			tmp1 = x*z;    tmp2 = y*w;  
			double m20 = 2.0 * (tmp1 - tmp2)*invs ;   
			double m02 = 2.0 * (tmp1 + tmp2)*invs ;
			tmp1 = y*z;    tmp2 = x*w;  
			double m21 = 2.0 * (tmp1 + tmp2)*invs ;  
			double m12 = 2.0 * (tmp1 - tmp2)*invs ;      

			r[0] = m00;
			r[1] = m01;
			r[2] = m02;

			r[3] = m10;
			r[4] = m11;
			r[5] = m12;

			r[6] = m20;
			r[7] = m21;
			r[8] = m22;

			return r;
		}
		void fromVector(const std::vector<double>& R) {
			double tr = R[0] + R[4] + R[8];
			if (tr > 0) {
				double s = sqrt(tr + 1) * 2;
				w = s / 4;
				x = (R[7] - R[5]) / s;
				y = (R[2] - R[6]) / s;
				z = (R[3] - R[1]) / s;
			} else if (R[0] > R[4] && R[0] > R[8]) {
				double s = sqrt(1 + R[0] - R[4] - R[8]) * 2;
				w = (R[7] - R[5]) / s;
				x = s / 4;
				y = (R[1] + R[3]) / s;
				z = (R[2] + R[6]) / s;
			} else if (R[4] > R[8]) {
				double s = sqrt(1 + R[4] - R[0] - R[8]) * 2;
				w = (R[2] - R[6]) / s;
				x = (R[1] + R[3]) / s;
				y = s / 4;
				z = (R[5] + R[7]) / s;
			} else {
				double s = sqrt(1 + R[8] - R[0] - R[4]) * 2;
				w = (R[3] - R[1]) / s;
				x = (R[2] + R[6]) / s;
				y = (R[5] + R[7]) / s;
				z = s / 4;
			}
		}

		void fromMatrix(const Eigen::Matrix3d& R) {
			std::vector<double> r(9, 0);
			r[0] = R(0,0);			r[1] = R(0,1);			r[2] = R(0,2);
			r[3] = R(1,0);			r[4] = R(1,1);			r[5] = R(1,2);
			r[6] = R(2,0);			r[7] = R(2,1);			r[8] = R(2,2);
			return fromVector(r);
		}

		Eigen::Matrix3d toMatrix() const {
			std::vector<double> r = toVector();
			Eigen::Matrix3d R;
			R << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
			return R;
		}

		pt3d mul(const pt3d& t) const {
			tiv::pt3d v;
			v.x = (1 - 2*y*y - 2*z*z) * t.x + (2*x*y - 2*z*w) * t.y + (2*x*z + 2*y*w) *t.z;
			v.y = (2*x*y + 2*z*w) * t.x + (1 - 2*x*x - 2*z*z) * t.y + (2*y*z - 2*x*w) *t.z;
			v.z = (2*x*z - 2*y*w) * t.x + (2*y*z + 2*x*w) * t.y + (1 - 2*x*x - 2*y*y) *t.z;
			return v;
		}
		
	} quat;
	
	
	typedef struct intrinsic {
		double fx, fy, cx, cy, skew;
		double focal_mm, sensor_width, sensor_height; // these 3 parameters are needed when working with Halcon library. also Halcon assumes no skewness in the parameter.
		intrinsic() : fx(1000), fy(1000), cx(320), cy(240), skew(0), focal_mm(12), sensor_width(640), sensor_height(480) {}
		intrinsic(const intrinsic& other) : fx(other.fx), fy(other.fy), cx(other.cx), cy(other.cy), skew(other.skew), focal_mm(other.focal_mm), sensor_width(other.sensor_width), sensor_height(other.sensor_height) {}
		intrinsic(const double _fx, const double _fy, const double _cx, const double _cy, const double _skew) {
			fx = _fx;	fy = _fy;	cx = _cx;	cy = _cy; skew = _skew;
			focal_mm = 12.0; sensor_width = 640; sensor_height = 480;
		}

		intrinsic& operator = (const intrinsic& rhs) {
			fx = rhs.fx; fy =rhs.fy; cx = rhs.cx; cy = rhs.cy; skew = rhs.skew;
			focal_mm = rhs.focal_mm; sensor_width = rhs.sensor_width; sensor_height = rhs.sensor_height;
			return *this;
		}
		void inv() {
			if (fx < tiv::eps || fy < tiv::eps) {
				throw std::overflow_error("Divide by zero exception");
			}

			double a = 1 / fx;
			double b = -skew / (fx * fy);
			double c = (skew * cy - cx * fy) / (fx * fy);
			double d = 1 / fy;
			double e = -cy / fy;

			fx = a;
			fy = d;
			cx = c;
			cy = e;
			skew = b;
		}
		intrinsic getInv() const {
			if (fx < tiv::eps || fy < tiv::eps) {
				throw std::overflow_error("Divide by zero exception");
			}

			intrinsic K(*this);
			K.fx = 1 / fx;
			K.fy = 1 / fy;
			K.cx = (skew * cy - cx * fy) / (fx * fy);
			K.cy = -cy / fy;
			K.skew = -skew / (fx * fy);
			return K;
		}
		std::vector<double> toVector() const {
			std::vector<double> a(9, 0.0);
			a[0] = fx;			a[1] = skew;		a[2] = cx;
			a[4] = fy;			a[5] = cy;
			a[8] = 1;
			return a;
		}

		Eigen::Matrix3d toMatrix() const {
			Eigen::Matrix3d A;
			A << fx, skew, cx, 0, fy, cy, 0, 0, 1;
			return A;
		}
	} intrinsic;

	// converting image to camera coordinate system after having intrinsic inverted.
	pt3d operator * (const intrinsic& K, const pt3d& t);

	typedef struct pose {
		quat q;
		pt3d t;
		pose() : q(1, 0, 0, 0), t(0, 0, 0) {}
		pose(const pose& other) : q(other.q), t(other.t) {}
		pose(const quat& _q, const pt3d& _t) {
			q = _q;
			t = _t;
		}
		pose(const Eigen::Matrix3d& _R, const pt3d& _t) {
			q.fromMatrix(_R);
			t = _t;
		}
		pose(const Eigen::Matrix4d& Rt) {
			q.fromMatrix(Rt.block<3, 3>(0, 0));
			t = pt3d(Rt(0, 3), Rt(1, 3), Rt(2, 3));
		}

		void inv() { q.inv(); t = q.mul(-t); }
		pose getInv() const {	
			pose p(q, t);
			p.inv();
			return p;
		}
		pose& operator = (const pose& rhs) {	
			q = rhs.q;
			t = rhs.t;
			return *this;
		}
		pose& operator *= (const pose& rhs) {
			t += q.mul(rhs.t);
			q *= rhs.q;
			return *this;		
		}
		pose operator * (const pose& rhs) const {	return pose(q * rhs.q, q.mul(rhs.t) + t);	}
		std::vector<double> toVector() const {
			std::vector<double> rt = q.toVector();
			rt.insert(rt.begin() + 3, t.x);
			rt.insert(rt.begin() + 7, t.y);
			rt.push_back(t.z);
			rt.push_back(0);
			rt.push_back(0);
			rt.push_back(0);
			rt.push_back(1);
			return rt;
		}
		Eigen::Matrix4d toMatrix() const {
			std::vector<double> r = q.toVector();
			Eigen::Matrix4d P; P.setZero();
			P.block<3, 3>(0, 0) << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
			P(0, 3) = t.x;
			P(1, 3) = t.y;
			P(2, 3) = t.z;
			P(3, 3) = 1.0;
			return P;
		}
		void fromVector(const std::vector<double>& rt) {
			std::vector<double> r(9, 0.0);
			r[0] = rt[0];	r[1] = rt[1];	r[2] = rt[2];
			r[3] = rt[4];	r[4] = rt[5];	r[5] = rt[6];
			r[6] = rt[8];	r[7] = rt[9];	r[8] = rt[10];
			q.fromVector(r);
			t = tiv::pt3d(rt[3], rt[7], rt[11]);
		}

		void fromMatrix(const Eigen::Matrix4d& Rt) {
			std::vector<double> r(9, 0.0);
			r[0] = Rt(0, 0);	r[1] = Rt(0, 1);	r[2] = Rt(0, 2);
			r[3] = Rt(1, 0);	r[4] = Rt(1, 1);	r[5] = Rt(1, 2);
			r[6] = Rt(2, 0);	r[7] = Rt(2, 1);	r[8] = Rt(2, 2);
			q.fromVector(r);
			t = tiv::pt3d(Rt(0, 3), Rt(1, 3), Rt(2, 3));
		}
	} pose;

	pt3d operator * (const pose& lhs, const pt3d& rhs);

	typedef struct distortion {
		double k1, k2, k3, t1, t2;
		distortion() : k1(0), k2(0), k3(0), t1(0), t2(0) {}
		distortion(const distortion& other) : k1(other.k1), k2(other.k2), k3(other.k3), t1(other.t1), t2(other.t2) {}
		distortion(const double _k1, const double _k2, const double _k3) {
			k1 = _k1;	k2 = _k2;	k3 = _k3; t1 = 0; t2 = 0;
		}
		distortion(const double _k1, const double _k2, const double _k3, const double _t1, const double _t2) {
			k1 = _k1;	k2 = _k2;	k3 = _k3; t1 = _t1; t2 = _t2;
		}
		distortion& operator = (const distortion& rhs) {
			k1 = rhs.k1;
			k2 = rhs.k2;
			k3 = rhs.k3;
			t1 = rhs.t1;
			t2 = rhs.t2;
			return *this;		
		}
	} distortion;

	typedef struct eig_corner_param {
		std::vector<std::vector<pt2d> > roi, mask;
		double min_distance, quality_th, contrast_th;
		unsigned int block_size;
		eig_corner_param() {
			roi.clear();
			mask.clear();
			min_distance = 20;
			quality_th = 0.1;
			contrast_th = 20;
			block_size = 3;
		}
		eig_corner_param(const eig_corner_param& other) {
			roi = other.roi;
			mask = other.mask;
			min_distance = other.min_distance;
			quality_th = other.quality_th;
			contrast_th = other.contrast_th;
			block_size = other.block_size;
		}
		eig_corner_param& operator = (const eig_corner_param& rhs) {
			roi = rhs.roi;
			mask = rhs.mask;
			min_distance = rhs.min_distance;
			quality_th = rhs.quality_th;
			contrast_th = rhs.contrast_th;
			block_size = rhs.block_size;
			return *this;
		}
	} eig_corner_param;

	typedef struct corner_sort_param {
		bool sort;
		unsigned int num_x, num_y;
		std::vector<pt2d> roi;
		double line_fit_th;
		corner_sort_param() : sort(true), num_x(16), num_y(12), line_fit_th(7.5) {
			roi.clear();
		}
		corner_sort_param(const corner_sort_param& other) {
			sort = other.sort;
			num_x = other.num_x;
			num_y = other.num_y;
			roi = other.roi;
			line_fit_th = other.line_fit_th;
		}
		corner_sort_param& operator = (const corner_sort_param& rhs) {
			sort = rhs.sort;
			num_x = rhs.num_x;
			num_y = rhs.num_y;
			roi = rhs.roi;
			line_fit_th = rhs.line_fit_th;
			return *this;
		}
	} corner_sort_param;

	typedef struct two_dim_param {
		intrinsic K;
		pose P;
		double z, Rz;
		bool remove_distortion;
		two_dim_param() : z(0.0), Rz(0), remove_distortion(true) {
			K = intrinsic(1000,1000,320,240,0);
			P = pose(quat(1,0,0,0), pt3d(0,0,300));
		}
		two_dim_param(const two_dim_param& other) {
			K = other.K;
			P = other.P;
			z = other.z;
			Rz = other.Rz;
			remove_distortion = other.remove_distortion;
		}
		two_dim_param& operator = (const two_dim_param& rhs) {
			K = rhs.K;
			P = rhs.P;
			z = rhs.z;
			Rz = rhs.Rz;
			remove_distortion = rhs.remove_distortion;
			return *this;
		}
	} two_dim_param;

	typedef struct two_half_dim_param {
		intrinsic K;
		pose P;
		double Rz;
		bool remove_distortion;
		two_half_dim_param() : Rz(0.0), remove_distortion(true) {
			K = intrinsic(1000,1000,320,240,0);
			P = pose(quat(1,0,0,0), pt3d(0,0,300));
		}
		two_half_dim_param(const two_half_dim_param& other) {
			K = other.K;
			P = other.P;
			Rz = other.Rz;
			remove_distortion = other.remove_distortion;
		}
		two_half_dim_param& operator = (const two_half_dim_param& rhs) {
			K = rhs.K;
			P = rhs.P;
			Rz = rhs.Rz;
			remove_distortion = rhs.remove_distortion;
			return *this;
		}
	} two_half_dim_param;

	// p: parameter vector to be optimized.
	// x: error vector to be minimized.
	// m: dim(p)
	// n: dim(x)
	// a: additional parameter vector. not to be optimized; it is common to minimize, f(p, a) = x.
	typedef void (*objFunc)(double *p, double *x, int m, int n, void* a);
}

#endif
