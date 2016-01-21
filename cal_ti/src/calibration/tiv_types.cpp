#include "../../include/cal_ti/tiv_types.hpp"

double tiv::dot(const tiv::pt3d& a, const tiv::pt3d& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

tiv::pt3d cross(const tiv::pt3d& a, const tiv::pt3d& b) { 
	return tiv::pt3d(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

/*
tiv::pt3d operator * (const tiv::quat& q, const tiv::pt3d& t) {
	tiv::pt3d v;
	v.x = (1 - 2*q.y*q.y - 2*q.z*q.z) * t.x + (2*q.x*q.y - 2*q.z*q.w) * t.y + (2*q.x*q.z + 2*q.y*q.w) *t.z;
	v.y = (2*q.x*q.y + 2*q.z*q.w) * t.x + (1 - 2*q.x*q.x - 2*q.z*q.z) * t.y + (2*q.y*q.z - 2*q.x*q.w) *t.z;
	v.z = (2*q.x*q.z - 2*q.y*q.w) * t.x + (2*q.y*q.z + 2*q.x*q.w) * t.y + (1 - 2*q.x*q.x - 2*q.y*q.y) *t.z;
	return v;
}
*/

tiv::pt3d operator * (const tiv::intrinsic& K, const tiv::pt3d& t) {	
	return tiv::pt3d(K.fx * t.x + K.skew * t.y + K.cx * t.z, K.fy * t.y + K.cy * t.z, t.z) / t.z;
}


tiv::pt3d operator * (const tiv::pose& lhs, const tiv::pt3d& rhs) {
	tiv::pt3d v;
	v.x = (1 - 2*lhs.q.y*lhs.q.y - 2*lhs.q.z*lhs.q.z) * rhs.x + (2*lhs.q.x*lhs.q.y - 2*lhs.q.z*lhs.q.w) * rhs.y + (2*lhs.q.x*lhs.q.z + 2*lhs.q.y*lhs.q.w) *rhs.z;
	v.y = (2*lhs.q.x*lhs.q.y + 2*lhs.q.z*lhs.q.w) * rhs.x + (1 - 2*lhs.q.x*lhs.q.x - 2*lhs.q.z*lhs.q.z) * rhs.y + (2*lhs.q.y*lhs.q.z - 2*lhs.q.x*lhs.q.w) *rhs.z;
	v.z = (2*lhs.q.x*lhs.q.z - 2*lhs.q.y*lhs.q.w) * rhs.x + (2*lhs.q.y*lhs.q.z + 2*lhs.q.x*lhs.q.w) * rhs.y + (1 - 2*lhs.q.x*lhs.q.x - 2*lhs.q.y*lhs.q.y) *rhs.z;
	v += lhs.t;
	return v;
}




