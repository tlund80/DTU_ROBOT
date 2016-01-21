#include "cvtools.h"

#ifdef _WIN32
#include <cstdint>
#endif

#include <stdio.h>

//#include <vtk_png.h>//png.h>
//#include <vtk_zlib.h>//zlib.h>

namespace cvtools{

// Convert an BGR 3-channel image back into a Bayered image
void cvtColorBGRToBayerBG(const cv::Mat &imBGR, cv::Mat &imBayerBG){

    imBayerBG.create(imBGR.size(), CV_8UC1);


    for(int r=0; r<imBGR.rows; r++){
        for(int c=0; c<imBGR.cols; c++){

            bool evenRow = r % 2;
            bool evenCol = c % 2;

            if(evenRow & evenCol)
                imBayerBG.at<uchar>(r,c) = imBGR.at<cv::Vec3b>(r,c)[0];
            else if(!evenRow & !evenCol)
                imBayerBG.at<uchar>(r,c) = imBGR.at<cv::Vec3b>(r,c)[2];
            else
                imBayerBG.at<uchar>(r,c) = imBGR.at<cv::Vec3b>(r,c)[1];

        }

    }


}


// Removes matches not satisfying the epipolar constraint.
// F is the fundamental matrix.
// Works like cv::correctMatches(), except it removes matches with an error distance greater than maxD.
void removeIncorrectMatches(const cv::Mat F, const std::vector<cv::Point2f> &q0, const std::vector<cv::Point2f> &q1, const float maxD,
                                std::vector<cv::Point2f> q0Correct, std::vector<cv::Point2f> q1Correct){

    int n0 = (int)q0.size(), n1 = (int)q1.size();
    q0Correct.reserve(n0);
    q1Correct.reserve(n1);

    // Point to line distance
//    for( int i = 0; i < n1; i++ ){
//        cv::Vec3f p0 = cv::Vec3f(q0[i].x, q0[i].y, 1.0);
//        // Epipolar line defined by p0
//        cv::Vec3f l = F*p0;
//        l /= sqrt(l(0)*l(0) + l(1)*l(1));
//        for( int j = 0; j < n2; j++ ){
//            cv::Vec3f p1 = cv::Vec3f(q1[i].x, q1[i].y, 1.0);
//            // Signed distance to line
//            float d = l.dot(p1);
//            if(d < maxD){
//                q0Correct.push_back(q0[i]);
//                q1Correct.push_back(q1[i]);
//            }
//        }
//    }

    // Symmetric epipolar distance
    std::vector<cv::Point3f> l0, l1;
    cv::computeCorrespondEpilines(q0, 1, F, l0);
    cv::computeCorrespondEpilines(q1, 2, F, l1);

    for(int i = 0; i < n0; i++){
        cv::Vec3f p0 = cv::Vec3f(q0[i].x, q0[i].y, 1.0);
        for(int j = 0; j < n1; j++){
            cv::Vec3f p1 = cv::Vec3f(q1[j].x, q1[j].y, 1.0);
            float d01 = l0[i].dot(p1);
            float d10 = l1[j].dot(p0);
            float d = d01*d01 + d10*d10;
            if(d < maxD){
                q0Correct.push_back(q0[i]);
                q1Correct.push_back(q1[i]);
            }
        }
    }

//    // Sampson Error (H&Z, p. 287) (expensive...)
//    std::vector<cv::Point3f> p0, p1;
//    cv::convertPointsToHomogeneous(q0, p0);
//    cv::convertPointsToHomogeneous(q1, p1);
//    cv::Mat Fp0Mat = cv::Mat(F)*cv::Mat(p0).reshape(1).t();
//    cv::Mat FTp1Mat = cv::Mat(F.t())*cv::Mat(p1).reshape(1).t();
//    for( int i = 0; i < n1; i++ ){
//        cv::Vec3f Fp0 = Fp0Mat.col(i);
//        for( int j = 0; j < n2; j++ ){
//            cv::Vec3f FTp1 = FTp1Mat.col(j);
//            cv::Matx<float,1,1> p1TFp0 = cv::Matx31f(p1[j]).t()*F*cv::Matx31f(p0[i]);
//            float d = p1TFp0(0)*p1TFp0(0) / (Fp0(0)*Fp0(0) + Fp0(1)*Fp0(1) + FTp1(0)*FTp1(0) + FTp1(1)*FTp1(1));
//            if(d < maxD){
//                q0Correct.push_back(q0[i]);
//                q1Correct.push_back(q1[i]);
//            }
//        }
//    }

    return;
}

// Performs bitwise right-shift on every value of matrix I. This is done e.g. to remove the least significant bits in a 16 to 8-bit image conversion.
void rshift(cv::Mat& I, unsigned int shift){

    int nRows = I.rows;
    int nCols = I.cols;

    if (I.isContinuous()){
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    unsigned short* p;
    for( i = 0; i < nRows; ++i){
        p = I.ptr<unsigned short>(i);
        for ( j = 0; j < nCols; ++j){
            p[j] = p[j]>>shift;
        }
    }
}


// Lightly modified OpenCV function which accepts a line width argument
void drawChessboardCorners(cv::InputOutputArray _image, cv::Size patternSize, cv::InputArray _corners, bool patternWasFound, int line_width){
    cv::Mat corners = _corners.getMat();
    if( corners.empty() )
        return;
    cv::Mat image = _image.getMat(); CvMat c_image = _image.getMat();
    int nelems = corners.checkVector(2, CV_32F, true);
    CV_Assert(nelems >= 0);
    cvtools::cvDrawChessboardCorners( &c_image, patternSize, (CvPoint2D32f*)corners.data,
                             nelems, patternWasFound, line_width);
}



void cvDrawChessboardCorners(CvArr* _image, CvSize pattern_size, CvPoint2D32f* corners, int count, int found, int line_width){
    const int shift = 0;
    const int radius = 12;
    const int r = radius*(1 << shift);
    int i;
    CvMat stub, *image;
    double scale = 1;
    int type, cn, line_type;

    image = cvGetMat( _image, &stub );

    type = CV_MAT_TYPE(image->type);
    cn = CV_MAT_CN(type);
    if( cn != 1 && cn != 3 && cn != 4 )
        CV_Error( CV_StsUnsupportedFormat, "Number of channels must be 1, 3 or 4" );

    switch( CV_MAT_DEPTH(image->type) )
    {
    case CV_8U:
        scale = 1;
        break;
    case CV_16U:
        scale = 256;
        break;
    case CV_32F:
        scale = 1./255;
        break;
    default:
        CV_Error( CV_StsUnsupportedFormat,
            "Only 8-bit, 16-bit or floating-point 32-bit images are supported" );
    }

    line_type = type == CV_8UC1 || type == CV_8UC3 ? CV_AA : 8;

    if( !found )
    {
        CvScalar color = {{0,0,255}};
        if( cn == 1 )
            color = cvScalarAll(200);
        color.val[0] *= scale;
        color.val[1] *= scale;
        color.val[2] *= scale;
        color.val[3] *= scale;

        for( i = 0; i < count; i++ )
        {
            CvPoint pt;
            pt.x = cvRound(corners[i].x*(1 << shift));
            pt.y = cvRound(corners[i].y*(1 << shift));
            cvLine( image, cvPoint( pt.x - r, pt.y - r ),
                    cvPoint( pt.x + r, pt.y + r ), color, line_width, line_type, shift );
            cvLine( image, cvPoint( pt.x - r, pt.y + r),
                    cvPoint( pt.x + r, pt.y - r), color, line_width, line_type, shift );
            cvCircle( image, pt, r+(1<<shift), color, line_width, line_type, shift );
        }
    }
    else
    {
        int x, y;
        CvPoint prev_pt = {0, 0};
        const int line_max = 7;
        static const CvScalar line_colors[line_max] =
        {
            {{0,0,255}},
            {{0,128,255}},
            {{0,200,200}},
            {{0,255,0}},
            {{200,200,0}},
            {{255,0,0}},
            {{255,0,255}}
        };

        for( y = 0, i = 0; y < pattern_size.height; y++ )
        {
            CvScalar color = line_colors[y % line_max];
            if( cn == 1 )
                color = cvScalarAll(200);
            color.val[0] *= scale;
            color.val[1] *= scale;
            color.val[2] *= scale;
            color.val[3] *= scale;

            for( x = 0; x < pattern_size.width; x++, i++ )
            {
                CvPoint pt;
                pt.x = cvRound(corners[i].x*(1 << shift));
                pt.y = cvRound(corners[i].y*(1 << shift));

                if( i != 0 )
                    cvLine( image, prev_pt, pt, color, 1, line_type, shift );

                cvLine( image, cvPoint(pt.x - r, pt.y - r),
                        cvPoint(pt.x + r, pt.y + r), color, line_width, line_type, shift );
                cvLine( image, cvPoint(pt.x - r, pt.y + r),
                        cvPoint(pt.x + r, pt.y - r), color, line_width, line_type, shift );
                cvCircle( image, pt, r+(1<<shift), color, line_width, line_type, shift );
                prev_pt = pt;
            }
        }
    }
}

// Returns the result of mod(mat(x,y), moduli) for each matrix element
cv::Mat modulo(const cv::Mat &mat, float n){

    cv::Mat ret(mat.rows, mat.cols, mat.type());

    for(int row=0; row<ret.rows; row++){
        for(int col=0; col<ret.cols; col++){
            float val = mat.at<float>(row, col);
            // note: std::fmod calculates the remainder, not arithmetic modulo
            ret.at<float>(row, col) = val - n * std::floor(val / n);
        }
    }

    return ret;
}

// Convert a 3xN matrix to a vector of Point3fs.
void matToPoints3f(const cv::Mat &mat, std::vector<cv::Point3f> &points){

    unsigned int nPoints = mat.cols;
    points.resize(nPoints);

    for(unsigned int i=0; i<nPoints; i++)
        points[i] = cv::Point3f(mat.at<float>(0, i), mat.at<float>(1, i), mat.at<float>(2, i));
}

// Convert a (Dim+1)xN matrix of homogenous points to a DimxN matrix of points in non-homogenous coordinates.
void convertMatFromHomogeneous(cv::Mat &src, cv::Mat &dst){
    unsigned int N = src.cols;
    unsigned int Dim = src.rows-1;
    dst.create(Dim, N, src.type());
    for(unsigned int i=0; i<N; i++){
        for(unsigned int j=0; j<Dim; j++)
            dst.at<float>(j,i) = src.at<float>(j,i)/src.at<float>(Dim,i);
    }

}

// Function to solve the hand-eye (or eye-in-hand) calibration problem.
// Finds [Omega | tau], to minimize ||[R_mark | t_mark][Omega | tau] - [Omega | tau][R | t]||^2
// Algorithm according to Tsai, Lenz, A new technique for fully autonomous and efficient 3d robotics hand-eye calibration
// DTU, 2014, Jakob Wilm
void handEyeCalibrationTsai(const std::vector<cv::Matx33f> R, const std::vector<cv::Vec3f> t, const std::vector<cv::Matx33f> R_mark, const std::vector<cv::Vec3f> t_mark, cv::Matx33f &Omega, cv::Vec3f &tau){

    unsigned int N = R.size();
    assert(N == R_mark.size());
    assert(N == t.size());
    assert(N == t_mark.size());

    // construct equations for rotation
    cv::Mat A(3*N, 3, CV_32F);
    cv::Mat b(3*N, 1, CV_32F);
    for(unsigned int i=0; i<N; i++){
        // angle axis representations
        cv::Vec3f rot;
        cv::Vec3f rot_mark;
        cv::Rodrigues(R[i], rot);
        cv::Rodrigues(R_mark[i], rot_mark);

        cv::Vec3f P = 2.0*sin(cv::norm(rot)/2.0)*cv::normalize(rot);
//std::cout << "P: " << std::endl << P << std::endl;
        cv::Vec3f P_mark = 2.0*sin(cv::norm(rot_mark)/2.0)*cv::normalize(rot_mark);
//std::cout << "P_mark: " << std::endl << P_mark << std::endl;
        cv::Vec3f sum = P+P_mark;
        cv::Mat crossProduct = (cv::Mat_<float>(3,3) << 0.0, -sum(2), sum(1), sum(2), 0.0, -sum(0), -sum(1), sum(0), 0.0);
//std::cout << "crossProduct: " << std::endl << crossProduct << std::endl;
        crossProduct.copyTo(A.rowRange(i*3, i*3+3));

        cv::Mat(P-P_mark).copyTo(b.rowRange(i*3, i*3+3));
    }

    // solve for rotation
    cv::Vec3f P_prime;
    cv::solve(A, b, P_prime, cv::DECOMP_SVD);
    cv::Vec3f P = 2.0*P_prime/(cv::sqrt(1.0 + cv::norm(P_prime)*cv::norm(P_prime)));
    float nP = cv::norm(P);
    cv::Mat crossProduct = (cv::Mat_<float>(3,3) << 0.0, -P(2), P(1), P(2), 0.0, -P(0), -P(1), P(0), 0.0);
    cv::Mat OmegaMat = (1.0-nP*nP/2.0)*cv::Mat::eye(3,3,CV_32F) + 0.5*(cv::Mat(P)*cv::Mat(P).t() + cv::sqrt(4.0 - nP*nP)*crossProduct);
    Omega = cv::Matx33f(OmegaMat);

    // construct equations for translation
    A.setTo(0.0);
    b.setTo(0.0);
    for(unsigned int i=0; i<N; i++){

        cv::Mat diff = cv::Mat(R_mark[i]) - cv::Mat::eye(3, 3, CV_32F);
        diff.copyTo(A.rowRange(i*3, i*3+3));

        cv::Mat diff_mark = cv::Mat(Omega*t[i] - t_mark[i]);
        diff_mark.copyTo(b.rowRange(i*3, i*3+3));
    }

    // solve for translation
    cv::solve(A, b, tau, cv::DECOMP_SVD);

    cv::Mat err_tau = b - (A*cv::Mat(tau));
    std::cout << err_tau << std::endl;
}

// Function to solve for the rotation axis from sets of 3D point coordinates of flat pattern feature points
// Algorithm according to Chen et al., Rotation axis calibration of a turntable using constrained global optimization, Optik 2014
// DTU, 2014, Jakob Wilm
void rotationAxisCalibration(const std::vector< std::vector<cv::Point3f> > Qcam, const std::vector<cv::Point3f> Qobj, cv::Vec3f &axis, cv::Vec3f &point){

    // number of frames (points on each arch)
    int l = Qcam.size();

    // number of points in each frame
    unsigned int mn = Qobj.size();

    assert(mn == Qcam[0].size());

    // construct matrix for axis determination
    cv::Mat M(6, 6, CV_32F, cv::Scalar(0));

    for(int k=0; k<l; k++){
        for(unsigned int idx=0; idx<mn; idx++){

//            float i = Qobj[idx].x+4;
//            float j = Qobj[idx].y+4;
            float i = Qobj[idx].x;
            float j = Qobj[idx].y;

            float x = Qcam[k][idx].x;
            float y = Qcam[k][idx].y;
            float z = Qcam[k][idx].z;

            M += (cv::Mat_<float>(6,6) << x*x, x*y, x*z, x, i*x, j*x,
                                            0, y*y, y*z, y, i*y, j*y,
                                            0,   0, z*z, z, i*z, j*z,
                                            0,   0,   0, 1,   i,   j,
                                            0,   0,   0, 0, i*i, i*j,
                                            0,   0,   0, 0,   0, j*j);

        }
    }

    cv::completeSymm(M, false);

    // solve for axis
    std::vector<float> lambda;
    cv::Mat u;
    cv::eigen(M, lambda, u);

    float minLambda = abs(lambda[0]);
    int idx = 0;
    for(unsigned int i=1; i<lambda.size(); i++){
        if(abs(lambda[i]) < minLambda){
            minLambda = lambda[i];
            idx = i;
        }
    }

    axis = u.row(idx).colRange(0, 3);
    axis = cv::normalize(axis);

    float nx = u.at<float>(idx, 0);
    float ny = u.at<float>(idx, 1);
    float nz = u.at<float>(idx, 2);
    float d  = u.at<float>(idx, 3);
    float dh = u.at<float>(idx, 4);
    float dv = u.at<float>(idx, 5);

//    // Paper version: c is initially eliminated
//    cv::Mat A(l*mn, mn+2, CV_32F, cv::Scalar(0.0));
//    cv::Mat bb(l*mn, 1, CV_32F);

//    for(int k=0; k<l; k++){
//        for(int idx=0; idx<mn; idx++){

//            float i = Qobj[idx].x;
//            float j = Qobj[idx].y;

//            float x = Qcam[k][idx].x;
//            float y = Qcam[k][idx].y;
//            float z = Qcam[k][idx].z;

//            float f = x*x + y*y + z*z + (2*x*nx + 2*y*ny + 2*z*nz)*(i*dh + j*dv);

//            int row = k*mn+idx;
//            A.at<float>(row, 0) = 2*x - (2*z*nx)/nz;
//            A.at<float>(row, 1) = 2*y - (2*z*ny)/nz;
//            A.at<float>(row, idx+2) = 1.0;

//            bb.at<float>(row, 0) = f + (2*z*d)/nz;
//        }
//    }

//    // solve for point
//    cv::Mat abe;
//    cv::solve(A, bb, abe, cv::DECOMP_SVD);

//    float a = abe.at<float>(0, 0);
//    float b = abe.at<float>(1, 0);
//    float c = -(nx*a+ny*b+d)/nz;

    // Our version: solve simultanously for a,b,c
    cv::Mat A(l*mn, mn+3, CV_32F, cv::Scalar(0.0));
    cv::Mat bb(l*mn, 1, CV_32F);

    for(int k=0; k<l; k++){
        for(int idx=0; idx<mn; idx++){

            float i = Qobj[idx].x;
            float j = Qobj[idx].y;

            float x = Qcam[k][idx].x;
            float y = Qcam[k][idx].y;
            float z = Qcam[k][idx].z;

            float f = x*x + y*y + z*z + (2*x*nx + 2*y*ny + 2*z*nz)*(i*dh + j*dv);

            int row = k*mn+idx;
            A.at<float>(row, 0) = 2*x;
            A.at<float>(row, 1) = 2*y;
            A.at<float>(row, 2) = 2*z;
            A.at<float>(row, idx+3) = 1.0;

            bb.at<float>(row, 0) = f;
        }
    }

    // solve for point
    cv::Mat abe;
    cv::solve(A, bb, abe, cv::DECOMP_SVD);

    float a = abe.at<float>(0, 0);
    float b = abe.at<float>(1, 0);
    float c = abe.at<float>(2, 0);

    point[0] = a;
    point[1] = b;
    point[2] = c;

}

// Function to fit two sets of corresponding pose data.
// Finds [Omega | tau], to minimize ||[R_mark | t_mark] - [Omega | tau][R | t]||^2
// Algorithm and notation according to Mili Shah, Comparing two sets of corresponding six degree of freedom data, CVIU 2011.
// DTU, 2013, Oline V. Olesen, Jakob Wilm
void fitSixDofData(const std::vector<cv::Matx33f> R, const std::vector<cv::Vec3f> t, const std::vector<cv::Matx33f> R_mark, const std::vector<cv::Vec3f> t_mark, cv::Matx33f &Omega, cv::Vec3f &tau){

    int N = R.size();
    assert(N == R_mark.size());
    assert(N == t.size());
    assert(N == t_mark.size());

    // Mean translations
    cv::Vec3f t_mean;
    cv::Vec3f t_mark_mean;
    for(int i=0; i<N; i++){
        t_mean += 1.0/N * t[i];
        t_mark_mean += 1.0/N * t_mark[i];
    }

    // Data with mean adjusted translations
    cv::Mat X_bar(3, 4*N, CV_32F);
    cv::Mat X_mark_bar(3, 4*N, CV_32F);
    for(int i=0; i<N; i++){
        cv::Mat(R[i]).copyTo(X_bar.colRange(i*4,i*4+3));
        cv::Mat(t[i] - t_mean).copyTo(X_bar.col(i*4+3));
        cv::Mat(R_mark[i]).copyTo(X_mark_bar.colRange(i*4,i*4+3));
        cv::Mat(t_mark[i] - t_mark_mean).copyTo(X_mark_bar.col(i*4+3));
    }
    //std::cout << X_bar << std::endl;
    // SVD
    cv::Mat W, U, VT;
    cv::SVDecomp(X_bar*X_mark_bar.t(), W, U, VT);

    cv::Matx33f D = cv::Matx33f::eye();
    if(cv::determinant(VT*U) < 0)
        D(3,3) = -1;

    // Best rotation
    Omega = cv::Matx33f(cv::Mat(VT.t()))*D*cv::Matx33f(cv::Mat(U.t()));

    // Best translation
    tau = t_mark_mean - Omega*t_mean;

}

// Forward distortion of points. The inverse of the undistortion in cv::initUndistortRectifyMap().
// Inspired by Pascal Thomet, http://code.opencv.org/issues/1387#note-11
// Convention for distortion parameters: http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
void initDistortMap(const cv::Matx33f cameraMatrix, const cv::Vec<float, 5> distCoeffs, const cv::Size size, cv::Mat &map1, cv::Mat &map2){

    float fx = cameraMatrix(0,0);
    float fy = cameraMatrix(1,1);
    float ux = cameraMatrix(0,2);
    float uy = cameraMatrix(1,2);

    float k1 = distCoeffs[0];
    float k2 = distCoeffs[1];
    float p1 = distCoeffs[2];
    float p2 = distCoeffs[3];
    float k3 = distCoeffs[4];

    map1.create(size, CV_32F);
    map2.create(size, CV_32F);

    for(int col = 0; col < size.width; col++){
        for(int row = 0; row < size.height; row++){

            // move origo to principal point and convert using focal length
            float x = (col-ux)/fx;
            float y = (row-uy)/fy;

            float xCorrected, yCorrected;

            //Step 1 : correct distortion
            float r2 = x*x + y*y;
            //radial
            xCorrected = x * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
            yCorrected = y * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
            //tangential
            xCorrected = xCorrected + (2.*p1*x*y + p2*(r2+2.*x*x));
            yCorrected = yCorrected + (p1*(r2+2.*y*y) + 2.*p2*x*y);

            //convert back to pixel coordinates
            float col_displaced = xCorrected * fx + ux;
            float row_displaced = yCorrected * fy + uy;

            // correct the vector in the opposite direction
            map1.at<float>(row,col) = col+(col-col_displaced);
            map2.at<float>(row,col) = row +(row-row_displaced);
        }
    }
}

// Downsample a texture which was created in virtual column/row space for a diamond pixel array projector
cv::Mat diamondDownsample(cv::Mat &pattern){

    cv::Mat pattern_diamond(pattern.rows,pattern.cols/2,CV_8UC3);

    for(int col = 0; col < pattern_diamond.cols; col++){
        for(int row = 0; row < pattern_diamond.rows; row++){

            pattern_diamond.at<cv::Vec3b>(row,col)=pattern.at<cv::Vec3b>(row,col*2+row%2);
        }
    }

    return pattern_diamond;

}


void mouseCallback(int evt, int x, int y, int flags, void* param){
    cv::Mat *im = (cv::Mat*) param;
    if (evt == CV_EVENT_LBUTTONDOWN) {
        if(im->type() == CV_8UC3){
            printf("%d %d: %d, %d, %d\n",
                   x, y,
                   (int)(*im).at<cv::Vec3b>(y, x)[0],
                    (int)(*im).at<cv::Vec3b>(y, x)[1],
                    (int)(*im).at<cv::Vec3b>(y, x)[2]);
        } else if (im->type() == CV_32F) {
            printf("%d %d: %f\n",
                   x, y,
                   im->at<float>(y, x));
        }
    }
}

void imshow(const char *windowName, cv::Mat im, unsigned int x, unsigned int y){

    // Imshow
    if(!cvGetWindowHandle(windowName)){
        int windowFlags = CV_GUI_EXPANDED | CV_WINDOW_KEEPRATIO;
        cv::namedWindow(windowName, windowFlags);
        cv::moveWindow(windowName, x, y);
    }
    cv::imshow(windowName, im);
}

void imagesc(const char *windowName, cv::Mat im){

    // Imshow with scaled image


}

cv::Mat histimage(cv::Mat histogram){

    cv::Mat histImage(512, 640, CV_8UC3, cv::Scalar(0));

    // Normalize the result to [ 2, histImage.rows-2 ]
    cv::normalize(histogram, histogram, 2, histImage.rows-2, cv::NORM_MINMAX, -1, cv::Mat());

    float bin_w = (float)histImage.cols/(float)histogram.rows;

    // Draw main histogram
    for(int i = 1; i < histogram.rows-10; i++){
        cv::line(histImage, cv::Point( bin_w*(i-1), histImage.rows - cvRound(histogram.at<float>(i-1)) ),
                 cv::Point( bin_w*(i), histImage.rows - cvRound(histogram.at<float>(i)) ),
                 cv::Scalar(255, 255, 255), 2, 4);
    }

    // Draw red max
    for(int i = histogram.rows-10; i < histogram.rows; i++){
        cv::line(histImage, cv::Point( bin_w*(i-1), histImage.rows - cvRound(histogram.at<float>(i-1)) ),
                 cv::Point( bin_w*(i), histImage.rows - cvRound(histogram.at<float>(i)) ),
                 cv::Scalar(0, 0, 255), 2, 4);
    }

    return histImage;
}

void hist(const char *windowName, cv::Mat histogram, unsigned int x, unsigned int y){

    // Display
    imshow(windowName, histimage(histogram), x, y);
    cv::Point(1,2);
}


void writeMat(cv::Mat const& mat, const char* filename, const char* varName, bool bgr2rgb){
    /*!
         *  \author Philip G. Lee <rocketman768@gmail.com>
         *  Write \b mat into \b filename
         *  in uncompressed .mat format (Level 5 MATLAB) for Matlab.
         *  The variable name in matlab will be \b varName. If
         *  \b bgr2rgb is true and there are 3 channels, swaps 1st and 3rd
         *  channels in the output. This is needed because OpenCV matrices
         *  are bgr, while Matlab is rgb. This has been tested to work with
         *  3-channel single-precision floating point matrices, and I hope
         *  it works on other types/channels, but not exactly sure.
         *  Documentation at <http://www.mathworks.com/help/pdf_doc/matlab/matfile_format.pdf>
         */
    int textLen = 116;
    char* text;
    int subsysOffsetLen = 8;
    char* subsysOffset;
    int verLen = 2;
    char* ver;
    char flags;
    int bytes;
    int padBytes;
    int bytesPerElement;
    int i,j,k,k2;
    bool doBgrSwap;
    char mxClass;
    int32_t miClass;
    uchar const* rowPtr;
    uint32_t tmp32;
    float tmp;
    FILE* fp;

    // Matlab constants.
    const uint16_t MI = 0x4d49; // Contains "MI" in ascii.
    const int32_t miINT8 = 1;
    const int32_t miUINT8 = 2;
    const int32_t miINT16 = 3;
    const int32_t miUINT16 = 4;
    const int32_t miINT32 = 5;
    const int32_t miUINT32 = 6;
    const int32_t miSINGLE = 7;
    const int32_t miDOUBLE = 9;
    const int32_t miMATRIX = 14;
    const char mxDOUBLE_CLASS = 6;
    const char mxSINGLE_CLASS = 7;
    const char mxINT8_CLASS = 8;
    const char mxUINT8_CLASS = 9;
    const char mxINT16_CLASS = 10;
    const char mxUINT16_CLASS = 11;
    const char mxINT32_CLASS = 12;
    const char mxUINT32_CLASS = 13;
    const uint64_t zero = 0; // Used for padding.

    fp = fopen( filename, "wb" );

    if( fp == 0 )
        return;

    const int rows = mat.rows;
    const int cols = mat.cols;
    const int chans = mat.channels();

    doBgrSwap = (chans==3) && bgr2rgb;

    // I hope this mapping is right :-/
    switch( mat.depth() ){
    case CV_8U:
        mxClass = mxUINT8_CLASS;
        miClass = miUINT8;
        bytesPerElement = 1;
        break;
    case CV_8S:
        mxClass = mxINT8_CLASS;
        miClass = miINT8;
        bytesPerElement = 1;
        break;
    case CV_16U:
        mxClass = mxUINT16_CLASS;
        miClass = miUINT16;
        bytesPerElement = 2;
        break;
    case CV_16S:
        mxClass = mxINT16_CLASS;
        miClass = miINT16;
        bytesPerElement = 2;
        break;
    case CV_32S:
        mxClass = mxINT32_CLASS;
        miClass = miINT32;
        bytesPerElement = 4;
        break;
    case CV_32F:
        mxClass = mxSINGLE_CLASS;
        miClass = miSINGLE;
        bytesPerElement = 4;
        break;
    case CV_64F:
        mxClass = mxDOUBLE_CLASS;
        miClass = miDOUBLE;
        bytesPerElement = 8;
        break;
    default:
        return;
    }

    //==================Mat-file header (128 bytes, page 1-5)==================
    text = new char[textLen]; // Human-readable text.
    memset( text, ' ', textLen );
    text[textLen-1] = '\0';
    const char* t = "MATLAB 5.0 MAT-file, Platform: PCWIN";
    memcpy( text, t, strlen(t) );

    subsysOffset = new char[subsysOffsetLen]; // Zeros for us.
    memset( subsysOffset, 0x00, subsysOffsetLen );
    ver = new char[verLen];
    ver[0] = 0x00;
    ver[1] = 0x01;

    fwrite( text, 1, textLen, fp );
    fwrite( subsysOffset, 1, subsysOffsetLen, fp );
    fwrite( ver, 1, verLen, fp );
    // Endian indicator. MI will show up as "MI" on big-endian
    // systems and "IM" on little-endian systems.
    fwrite( &MI, 2, 1, fp );
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //===================Data element tag (8 bytes, page 1-8)==================
    bytes = 16 + 24 + (8 + strlen(varName) + (8-(strlen(varName)%8))%8)
            + (8 + rows*cols*chans*bytesPerElement);
    fwrite( &miMATRIX, 4, 1, fp ); // Data type.
    fwrite( &bytes, 4, 1, fp); // Data size in bytes.
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //====================Array flags (16 bytes, page 1-15)====================
    bytes = 8;
    fwrite( &miUINT32, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );
    flags = 0x00; // Complex, logical, and global flags all off.

    tmp32 = 0;
    tmp32 = (flags << 8 ) | (mxClass);
    fwrite( &tmp32, 4, 1, fp );

    fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //===============Dimensions subelement (24 bytes, page 1-17)===============
    bytes = 12;
    fwrite( &miINT32, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );

    fwrite( &rows, 4, 1, fp );
    fwrite( &cols, 4, 1, fp );
    fwrite( &chans, 4, 1, fp );
    fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //==Array name (8 + strlen(varName) + (8-(strlen(varName)%8))%8 bytes, page 1-17)==
    bytes = strlen(varName);

    fwrite( &miINT8, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );
    fwrite( varName, 1, bytes, fp );

    // Pad to nearest 64-bit boundary.
    padBytes = (8-(bytes%8))%8;
    fwrite( &zero, 1, padBytes, fp );
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //====Matrix data (rows*cols*chans*bytesPerElement+8 bytes, page 1-20)=====
    bytes = rows*cols*chans*bytesPerElement;
    fwrite( &miClass, 4, 1, fp );
    fwrite( &bytes, 4, 1, fp );

    for( k = 0; k < chans; ++k )
    {
        if( doBgrSwap )
        {
            k2 = (k==0)? 2 : ((k==2)? 0 : 1);
        }
        else
            k2 = k;

        for( j = 0; j < cols; ++j )
        {
            for( i = 0; i < rows; ++i )
            {
                rowPtr = mat.data + mat.step*i;
                fwrite( rowPtr + (chans*j + k2)*bytesPerElement, bytesPerElement, 1, fp );
            }
        }
    }

    // Pad to 64-bit boundary.
    padBytes = (8-(bytes%8))%8;
    fwrite( &zero, 1, padBytes, fp );
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    fclose(fp);
    delete[] text;
    delete[] subsysOffset;
    delete[] ver;
}





}

