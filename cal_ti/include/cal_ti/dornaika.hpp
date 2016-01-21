#ifndef DORNAIKA_H_
#define DORNAIKA_H_

#include "tiv_types.hpp"

// papaer: simultaneous robot-world and hand-eye calibration, dornaika & horaud
// some modification is made:
// 1. transform order: AX=ZB ---> XA = BZ
// 2. finding qx and qz: evaluate all eigen vectors and select qz providing the minimum error. 
// 3. only 18 elements are used in optimization, but one can use all 24 parameters by altering error function.

namespace tiv {
	/*
	@param A: Array of tool transform w.r.t. robot base.
	@param B: Array of camera pose w.r.t. world frame.
	@return X: Transform of camera w.r.t. tool flange.
	@return Z: Transform of world frame w.r.t. robot base.
	@param with_lm: Enabling Levenberg-Marquardt parameter optimization; the minimum number of elements in A and B is 9 but provide 15 or more poses.
	Note: Camera pose orthogonal to world frame making numerically unstable results (for now). Exclude such configuration.
	*/
    extern "C" bool dornaika(const std::vector<tiv::pose>& A, const std::vector<tiv::pose>& B, tiv::pose& X, tiv::pose& Z, const bool with_lm);
	void dornaika_err_func(double *p, double *x, int m, int n, void* a);
	static int counter = 0;
}

#endif
