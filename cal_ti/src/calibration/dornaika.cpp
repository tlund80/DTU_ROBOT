#include <iterator>
#include <utility>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>
#include <iostream>
extern "C" {
#include "../cal_ti/ext/levmar-2.6/levmar.h"
}
#include <include/cal_ti/dornaika.hpp>


extern "C" bool tiv::dornaika(const std::vector<tiv::pose>& A, const std::vector<tiv::pose>& B, tiv::pose& X, tiv::pose& Z, const bool with_lm) {

	if (A.size() != B.size() || A.size() < 3) { return false; }
	Eigen::Matrix4d W, Q, C, CtC;
	C.setZero();

	const size_t n = A.size();
	for (size_t i = 0; i < n; i++) {
		tiv::quat qA = A[i].q;
		tiv::quat qB = B[i].q;

		Q(0, 0) = qA.w;	Q(0, 1) = -qA.x;	Q(0, 2) = -qA.y;	Q(0, 3) = -qA.z;
		Q(1, 0) = qA.x;	Q(1, 1) =  qA.w;	Q(1, 2) = -qA.z;	Q(1, 3) =  qA.y;
		Q(2, 0) = qA.y;	Q(2, 1) =  qA.z;	Q(2, 2) =  qA.w;	Q(2, 3) = -qA.x;
		Q(3, 0) = qA.z;	Q(3, 1) = -qA.y;	Q(3, 2) =  qA.x;	Q(3, 3) =  qA.w;

		W(0, 0) = qB.w;	W(0, 1) = -qB.x;	W(0, 2) = -qB.y;	W(0, 3) = -qB.z;
		W(1, 0) = qB.x;	W(1, 1) =  qB.w;	W(1, 2) =  qB.z;	W(1, 3) = -qB.y;
		W(2, 0) = qB.y;	W(2, 1) = -qB.z;	W(2, 2) =  qB.w;	W(2, 3) =  qB.x;
		W(3, 0) = qB.z;	W(3, 1) =  qB.y;	W(3, 2) = -qB.x;	W(3, 3) =  qB.w;

		C += -Q.transpose() * W;
	}

	CtC = C.transpose() * C;

	Eigen::MatrixXd S(8, 8); 
	Eigen::Matrix4d nI = n * Eigen::Matrix4d::Identity();
	S.block<4, 4>(0, 0) = nI;
	S.block<4, 4>(4, 4) = nI;
	S.block<4, 4>(0, 4) = C;
	S.block<4, 4>(4, 0) = C.transpose();

	Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(CtC, true);
	Eigen::Vector4d eig_vals = eigen_solver.eigenvalues().real();
	Eigen::Matrix4d eig_vecs = eigen_solver.eigenvectors().real();

	//Eigen::Vector4d func_qxqz;
	std::vector<double> func_axzb(4, 0.0);

	for (size_t i = 0; i < 4; i++) {
		Eigen::Vector4d vqz = eig_vecs.col(i);
		Eigen::Vector4d vqx = (1.0 / sqrt(eig_vals(i))) * C * vqz;
		vqx.normalize();

		tiv::quat z = tiv::quat(vqz(0), vqz(1), vqz(2), vqz(3));
		tiv::quat x = tiv::quat(vqx(0), vqx(1), vqx(2), vqx(3));

		Eigen::Matrix3d zero_mat;
		for (size_t j = 0; j < A.size(); j++) {
			zero_mat = A[j].q.toMatrix() * x.toMatrix() - z.toMatrix() * B[j].q.toMatrix();
			func_axzb[i] += zero_mat.norm();
		}
	}

	size_t min_index = std::distance(func_axzb.begin(), std::min_element(func_axzb.begin(), func_axzb.end()));
	Eigen::Vector4d qz = eig_vecs.col(min_index);
	Eigen::Vector4d qx = (1.0 / sqrt(eig_vals(min_index))) * C * qz;

	Z.q = tiv::quat(qz(0), qz(1), qz(2), qz(3));
	X.q = tiv::quat(qx(0), qx(1), qx(2), qx(3));

	// solving for tx and tz
	Eigen::MatrixXd Lhs(3 * n, 6); 
	Eigen::VectorXd Rhs(3 * n);

	Eigen::Matrix3d Rx, Rz;
	Eigen::Vector3d tB, tA, tRhs;
	std::vector<double> r = Z.q.toVector();
	Rz << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
	r = X.q.toVector();
	Rx << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
	
	for (size_t i = 0; i < n; i++) {	
		r = A[i].q.toVector();
		Lhs.block<3, 3>(3 * i, 0) << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
		Lhs.block<3, 3>(3 * i, 3) << -1, 0, 0, 0, -1, 0, 0, 0, -1;

		tB << B[i].t.x, B[i].t.y, B[i].t.z;
		tA << A[i].t.x, A[i].t.y, A[i].t.z;

		tRhs = Rz * tB - tA;

		Rhs(3 * i) = tRhs(0);
		Rhs(3 * i + 1) = tRhs(1);
		Rhs(3 * i + 2) = tRhs(2);
	}

	Eigen::VectorXd sol = Lhs.fullPivLu().solve(Rhs);

	X.t = tiv::pt3d(sol(0), sol(1), sol(2));
	Z.t = tiv::pt3d(sol(3), sol(4), sol(5));


	Eigen::Matrix4d RxTx = X.toMatrix();
	Eigen::Matrix4d RzTz = Z.toMatrix();

	if (!with_lm) { 
		return true;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Levenberg-Marquadt non-linear parameter optimization with Jacobina matrix approximation using finite difference method.	////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	const int dim_params = 18;
	const int dim_errors = 2 * n;

	if (dim_params > dim_errors) {
		return false;
	}

	double xz[dim_params];
	double *ab  = new double[16 * n * 2];
	double *all_zero = new double[dim_errors];
	for (int i = 0; i < dim_errors; i++) {
		all_zero[i] = 0;
	}

	Eigen::Matrix4d mat_x = X.toMatrix();
	Eigen::Matrix4d mat_z = Z.toMatrix();

	xz[0] = mat_x(0, 0);	xz[1] = mat_x(0, 1);	xz[6] = mat_x(0, 3);
	xz[2] = mat_x(1, 0);	xz[3] = mat_x(1, 1);	xz[7] = mat_x(1, 3);
	xz[4] = mat_x(2, 0);	xz[5] = mat_x(2, 1);	xz[8] = mat_x(2, 3);

	xz[9]  = mat_z(0, 0);	xz[10] = mat_z(0, 1);	xz[15] = mat_z(0, 3);
	xz[11] = mat_z(1, 0);	xz[12] = mat_z(1, 1);	xz[16] = mat_z(1, 3);
	xz[13] = mat_z(2, 0);	xz[14] = mat_z(2, 1);	xz[17] = mat_z(2, 3);

	for (size_t i = 0; i < n; i++) {
		std::vector<double> a = A[i].toVector();
		std::vector<double> b = B[i].toVector();
		for (int j = 0; j < 16; j++) {
			ab[16 * i + j] = a[j];
			ab[16 * (i + n) + j] = b[j];
		}
	}

	double lm_info[LM_INFO_SZ], lm_opts[LM_OPTS_SZ];
	lm_opts[0]= LM_INIT_MU; 
	lm_opts[1]=1E-15; 
	lm_opts[2]=1E-15; 
	lm_opts[3]=1E-20;
	lm_opts[4]= LM_DIFF_DELTA;

	if (dlevmar_dif(dornaika_err_func, xz, all_zero/*err*/, dim_params, dim_errors, 4999, lm_opts, lm_info, NULL, NULL, ab) >= 0) {
		// successful. update parameters.
		mat_x(0, 0) = xz[0];	mat_x(0, 1) = xz[1];			mat_x(0, 3) = xz[6];
		mat_x(1, 0) = xz[2];	mat_x(1, 1) = xz[3];			mat_x(1, 3) = xz[7];
		mat_x(2, 0) = xz[4];	mat_x(2, 1) = xz[5];			mat_x(2, 3) = xz[8];

		mat_z(0, 0) = xz[9];	mat_z(0, 1) = xz[10];			mat_z(0, 3) = xz[15];
		mat_z(1, 0) = xz[11];	mat_z(1, 1) = xz[12];			mat_z(1, 3) = xz[16];
		mat_z(2, 0) = xz[13];	mat_z(2, 1) = xz[14];			mat_z(2, 3) = xz[17];

		mat_x.block<3, 1>(0, 2) = mat_x.block<3, 1>(0, 0).cross(mat_x.block<3, 1>(0, 1));
		mat_z.block<3, 1>(0, 2) = mat_z.block<3, 1>(0, 0).cross(mat_z.block<3, 1>(0, 1));

		X = tiv::pose(mat_x);
		Z = tiv::pose(mat_z);
		X.q.normalize();
		Z.q.normalize();
	}

	std::cout << "====== LM info. ===================" << std::endl;
	std::cout << "||e||_2 with initial parameters: " << lm_info[0] << std::endl;
	std::cout << "||e||_2 with optimized parameters: " << lm_info[1] << std::endl;
	std::cout << "||J^T e||_inf: " << lm_info[2] << std::endl;
	std::cout << "||Dp||_2: " << lm_info[3] << std::endl;
	std::cout << "mu/max[J^T J]_ii: " << lm_info[4] << std::endl;
	std::cout << "# of iteration: " << lm_info[5] << std::endl;
	std::cout << "Reason of termination: " << lm_info[6] << std::endl;
	std::cout << "# of function evaluation: " << lm_info[7] << std::endl;
	std::cout << "# of Jacobian evaluation: " << lm_info[8] << std::endl;
	std::cout << "# attempts for reducing error: " << lm_info[9] << std::endl;

	delete[] all_zero;
	delete[] ab;
	return true;
}

void tiv::dornaika_err_func(double *p, double *x, int m, int n, void* a) {
	// p = [Rx(0,0), Rx(0,1), Rx(1,0), Rx(1,1), Rx(2,0), Rx(2,1), Tx.x, Tx.y, Tx.z, Rz(0,0), Rz(0,1), Rz(1,0), Rz(1,1), Rz(2,0), Rz(2,1), Tz.x, Tz.y, Tz.z]
	// x = [||Ra[i]*Rx - Rz*Rb[i]||, ||Ra[i] * Tx - Rz * Tb[i] - Tz||, ..., ||Rx' * Rx - I||, ||Rz' * Rz - I||]
	// a = [RaTa[i], ... , RbTb[i], ... ] 
	Eigen::Matrix3d Rx, Rz;//, Rx_zero, Rz_zero;
	Eigen::Vector3d Tx, Tz, Ta, Tb;

	Rx << p[0], p[1], 0.0, p[2], p[3], 0.0, p[4], p[5], 0.0;
	Tx << p[6],	 p[7],	p[8];

	Rz << p[9], p[10], 0.0, p[11], p[12], 0.0, p[13], p[14], 0.0;
	Tz << p[15], p[16], p[17];

	Rx.col(2) = Rx.col(0).cross(Rx.col(1));
	Rz.col(2) = Rz.col(0).cross(Rz.col(1));

	tiv::quat qx;	qx.fromMatrix(Rx);	qx.normalize();	Rx = qx.toMatrix();
	tiv::quat qz;	qz.fromMatrix(Rz);	qz.normalize();	Rz = qz.toMatrix();


	Eigen::Matrix4d RaTa, RbTb;
	//int k = (n - 2) / 2;
	int k = n / 2;

	double* ab = (double* )a;

	double sum (0);
	for (int j = 0, i = 0; i < k; i++) {
		j = i * 16;
		RaTa << ab[j], ab[j + 1], ab[j + 2], ab[j + 3], ab[j + 4], ab[j + 5], ab[j + 6], ab[j + 7],  ab[j + 8], ab[j + 9], ab[j + 10], ab[j + 11], 0, 0, 0, 1.0;

		j = (i + n / 2) * 16;
		RbTb << ab[j], ab[j + 1], ab[j + 2], ab[j + 3], ab[j + 4], ab[j + 5], ab[j + 6], ab[j + 7],  ab[j + 8], ab[j + 9], ab[j + 10], ab[j + 11], 0, 0, 0, 1.0;

		Eigen::Matrix3d R = RaTa.block<3, 3>(0, 0) * Rx - Rz * RbTb.block<3, 3>(0, 0);
		Eigen::Vector3d T = RaTa.block<3, 3>(0, 0) * Tx + RaTa.block<3, 1>(0, 3) - Rz * RbTb.block<3, 1>(0, 3) - Tz;

		x[2 * i] = R.norm() * 1E01;
		x[2 * i + 1] = T.norm();

		sum += x[2 * i] + x[2 * i + 1];
	}

	if (counter % 100 < 1) {
		std::cout << "Iter #: " << counter << ": " << sum << std::endl;
	}
	counter++;
}

