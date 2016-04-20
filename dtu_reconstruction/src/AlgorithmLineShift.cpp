#include "AlgorithmLineShift.h"
#include <cmath>
#include "cvtools.h"

#include <opencv2/imgproc/imgproc.hpp>

#ifndef log2f
#define log2f(x) (log(x)/log(2.0))
#endif

static unsigned int nLineShifts = 8; // number of columns over which each line is shifted

/*
 * The purpose of this function is to convert an unsigned
 * binary number to reflected binary Gray code.
 *
 * The operator >> is shift right. The operator ^ is exclusive or.
 * Source: http://en.wikipedia.org/wiki/Gray_code
 */
static unsigned int binaryToGray(unsigned int num) {
    return (num >> 1) ^ num;
}

/*
 * From Wikipedia: http://en.wikipedia.org/wiki/Gray_code
 * The purpose of this function is to convert a reflected binary
 * Gray code number to a binary number.
 */
static unsigned int grayToBinary(unsigned int num){
    unsigned int mask;
    for(mask = num >> 1; mask != 0; mask = mask >> 1)
        num = num ^ mask;
    return num;
}

/*
 * Return the Nth bit of an unsigned integer number
 */
static bool getBit(int decimal, int N){

    return decimal & 1 << (N-1);
}

/*
 * Return the number of bits set in an integer
 */
static int countBits(int n) {
  unsigned int c; // c accumulates the total bits set in v
  for (c = 0; n>0; c++)
    n &= n - 1; // clear the least significant bit set
  return c;
}

/*
 * Return the position of the least significant bit that is set
 */
static int leastSignificantBitSet(int x){
  if(x == 0)
      return 0;

  int val = 1;
  while(x>>=1)
      val++;

  return val;
}

static inline unsigned int powi(int num, unsigned int exponent){

    if(exponent == 0)
        return 1;

    float res = num;
    for(unsigned int i=0; i<exponent-1; i++)
        res *= num;

    return res;
}

static inline unsigned int twopowi(unsigned int exponent){

    return 1 << exponent;
}

// Algorithm
AlgorithmLineShift::AlgorithmLineShift(unsigned int _screenCols, unsigned int _screenRows) : Algorithm(_screenCols, _screenRows){

    int nTotalBits = ceilf(log2f((float)screenCols));

    // determine the necessary Gray code bits and add some robustness
    nGrayBits = nTotalBits - floorf(log2f((float)nLineShifts)) + 2;

    N = 2 + 2*nGrayBits + nLineShifts;

    // all on pattern
    cv::Mat allOn(1, screenCols, CV_8UC3, cv::Scalar::all(255));
    patterns.push_back(allOn);

    // all off pattern
    cv::Mat allOff(1, screenCols, CV_8UC3, cv::Scalar::all(0));
    patterns.push_back(allOff);


    // Gray code patterns
    for(unsigned int p=0; p<nGrayBits; p++){
        cv::Mat pattern(1, screenCols, CV_8UC3);
        cv::Mat patternInv(1, screenCols, CV_8UC3);

        for(unsigned int j=0; j<screenCols; j++){

            unsigned int jGray = binaryToGray(j);
            // Amplitude of channels
            int bit = (int)getBit(jGray, nTotalBits-p);
            pattern.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*bit,255.0*bit,255.0*bit);
            int invBit = bit^1;
            patternInv.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*invBit,255.0*invBit,255.0*invBit);
        }
        patterns.push_back(pattern);
        patterns.push_back(patternInv);
    }

    // line shifts
    for(unsigned int p=0; p<nLineShifts; p++){
        cv::Mat pattern(1, screenCols, CV_8UC3, cv::Scalar(0));

        for(unsigned int j=p; j<screenCols; j+= nLineShifts)
            pattern.at<cv::Vec3b>(0, j) = cv::Vec3b(255, 255, 255);

        patterns.push_back(pattern);
    }

}

cv::Mat AlgorithmLineShift::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

static cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt){
    assert(!img.empty());
    assert(img.channels() == 3);

    int x = (int)pt.x;
    int y = (int)pt.y;

    int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
    int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
    int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
    int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);

    float a = pt.x - (float)x;
    float c = pt.y - (float)y;

    uchar b = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[0] * a) * (1.f - c)
                           + (img.at<cv::Vec3b>(y1, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[0] * a) * c);
    uchar g = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[1] * a) * (1.f - c)
                           + (img.at<cv::Vec3b>(y1, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[1] * a) * c);
    uchar r = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[2] * a) * (1.f - c)
                           + (img.at<cv::Vec3b>(y1, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[2] * a) * c);

    return cv::Vec3b(b, g, r);
}

void getlineCenters(const cv::Mat& linesScanLine, const cv::Mat& codeScanLine, std::vector<cv::Vec2f>& lineCenters){

    int nCols = linesScanLine.cols;

    // finite derivative
    cv::Mat g(1, nCols, CV_32F);
    cv::Mat h(1, nCols, CV_32F);
    for(int i=2; i<nCols-2; i++){
//        g.at<float>(0, i) = linesScanLine.at<unsigned char>(0, i+2)+linesScanLine.at<unsigned char>(0, i+1)-
//                            linesScanLine.at<unsigned char>(0, i-1)-linesScanLine.at<unsigned char>(0, i-2);
        g.at<float>(0, i) = 1.0*linesScanLine.at<unsigned char>(0, i+2)+8.0*linesScanLine.at<unsigned char>(0, i+1)-
                            8.0*linesScanLine.at<unsigned char>(0, i-1)-1.0*linesScanLine.at<unsigned char>(0, i-2);
        h.at<float>(0, i) = -1.0*linesScanLine.at<unsigned char>(0, i+2)+16.0*linesScanLine.at<unsigned char>(0, i+1)-
                            30.0*linesScanLine.at<unsigned char>(0, i)+
                            16.0*linesScanLine.at<unsigned char>(0, i-1)-1.0*linesScanLine.at<unsigned char>(0, i-2);
    }
//    cvtools::writeMat(codeScanLine, "codeScanLine.mat", "codeScanLine");
//    cvtools::writeMat(linesScanLine, "linesScanLine.mat", "linesScanLine");
//    cvtools::writeMat(der, "der.mat", "der");

    for(int i=0; i<nCols-1; i++){
//        float fLeft = linesScanLine.at<unsigned char>(0, i-1);
        float fI = linesScanLine.at<unsigned char>(0, i);
//        float fRight = linesScanLine.at<unsigned char>(0, i+1);

        float gI = g.at<float>(0, i);
        float gRight = g.at<float>(0, i+1);

        float hI = h.at<float>(0, i);

        int codeI = codeScanLine.at<int>(0, i);
        //int codeRight = codeScanLine.at<int>(0, i+1);

        if((codeI != -1) && (fI > 10) && (gI >= 0.0) && (gRight <= 0.0) && (gRight < gI) && (hI < -1.0)){
            float delta = gI/(gI - gRight);
            lineCenters.push_back(cv::Vec2f(i + delta, codeI));
        }

    }

}

void AlgorithmLineShift::get3DPoints(SMCalibrationParameters calibration, const std::vector<cv::Mat>& frames0, const std::vector<cv::Mat>& frames1, std::vector<cv::Point3f>& Q, std::vector<cv::Vec3b>& color){

    assert(frames0.size() == N);
    assert(frames1.size() == N);

    int frameRows = frames0[0].rows;
    int frameCols = frames0[0].cols;

    // rectifying homographies (rotation+projections)
    cv::Size frameSize(frameCols, frameRows);
    cv::Mat R, T;
    // stereoRectify segfaults unless R is double precision
    cv::Mat(calibration.R1).convertTo(R, CV_64F);
    cv::Mat(calibration.T1).convertTo(T, CV_64F);
    cv::Mat R0, R1, P0, P1, QRect;
    cv::stereoRectify(calibration.K0, calibration.k0, calibration.K1, calibration.k1, frameSize, R, T, R0, R1, P0, P1, QRect, 0);

    // interpolation maps
    cv::Mat map0X, map0Y, map1X, map1Y;
    cv::initUndistortRectifyMap(calibration.K0, calibration.k0, R0, P0, frameSize, CV_32F, map0X, map0Y);
    cv::initUndistortRectifyMap(calibration.K1, calibration.k1, R1, P1, frameSize, CV_32F, map1X, map1Y);

    // gray-scale and remap
    std::vector<cv::Mat> frames0Rect(N);
    std::vector<cv::Mat> frames1Rect(N);
    for(int i=0; i<N; i++){
        cv::Mat temp;
        cv::cvtColor(frames0[i], temp, CV_BayerBG2GRAY);
        cv::remap(temp, frames0Rect[i], map0X, map0Y, CV_INTER_CUBIC);
        cv::cvtColor(frames1[i], temp, CV_BayerBG2GRAY);
        cv::remap(temp, frames1Rect[i], map1X, map1Y, CV_INTER_CUBIC);
    }

    //cvtools::writeMat(frames0Rect[0], "frames0Rect_0.mat", "frames0Rect_0");
    //cvtools::writeMat(frames0Rect[1], "frames0Rect_1.mat", "frames0Rect_1");
    //cvtools::writeMat(frames0Rect[20], "frames0Rect_20.mat", "frames0Rect_20");
    //cvtools::writeMat(frames0Rect[21], "frames0Rect_21.mat", "frames0Rect_21");

    // color debayer and remap
    cv::Mat color0Rect, color1Rect;
    cv::cvtColor(frames0[0], color0Rect, CV_BayerBG2RGB);
    cv::remap(color0Rect, color0Rect, map0X, map0Y, CV_INTER_CUBIC);

    cv::cvtColor(frames1[0], color1Rect, CV_BayerBG2RGB);
    cv::remap(color1Rect, color1Rect, map1X, map1Y, CV_INTER_CUBIC);

    int frameRectRows = frames0Rect[0].rows;
    int frameRectCols = frames0Rect[0].cols;

    // occlusion masks
    cv::Mat occlusion0Rect, occlusion1Rect;
    cv::subtract(frames0Rect[0], frames0Rect[1], occlusion0Rect);
    occlusion0Rect = (occlusion0Rect > 20) & (occlusion0Rect < 250);
    cv::subtract(frames1Rect[0], frames1Rect[1], occlusion1Rect);
    occlusion1Rect = (occlusion1Rect > 20) & (occlusion1Rect < 250);

//    cvtools::writeMat(occlusion0Rect, "occlusion0Rect.mat", "occlusion0Rect");
//    cvtools::writeMat(occlusion1Rect, "occlusion1Rect.mat", "occlusion1Rect");

    // erode occlusion masks
    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2));
    cv::erode(occlusion0Rect, occlusion0Rect, strel);
    cv::erode(occlusion1Rect, occlusion1Rect, strel);

//    // correct for texture modulation and ambient
//    cv::Mat A0 = frames0Rect[1];
//    cv::Mat M0 = frames0Rect[0]-frames0Rect[1];

//    cv::divide(256.0, M0, M0, CV_32F);
//    cv::Mat A1 = frames1Rect[1];
//    cv::Mat M1 = frames1Rect[0]-frames1Rect[1];
//    cv::divide(256.0, M1, M1, CV_32F);

//    for(int i=2; i<N; i++){
//        cv::multiply(frames0Rect[i]-A0, M0, frames0Rect[i], 1.0, CV_8UC1);
//        cv::multiply(frames1Rect[i]-A1, M1, frames1Rect[i], 1.0, CV_8UC1);
//    }

    //cvtools::writeMat(frames0Rect[22], "frames0Rect_22.mat", "frames0Rect_22");
    //cvtools::writeMat(frames0Rect[23], "frames0Rect_23.mat", "frames0Rect_23");

    // divide into Gray coding frames and line shift frames
    std::vector<cv::Mat> frames0GrayCode(frames0Rect.begin()+2, frames0Rect.begin()+2+2*nGrayBits);
    std::vector<cv::Mat> frames0LineShift(frames0Rect.begin()+2+2*nGrayBits, frames0Rect.end());
    std::vector<cv::Mat> frames1GrayCode(frames1Rect.begin()+2, frames1Rect.begin()+2+2*nGrayBits);
    std::vector<cv::Mat> frames1LineShift(frames1Rect.begin()+2+2*nGrayBits, frames1Rect.end());

    // decode patterns
    cv::Mat code0Gray(frameRectRows, frameRectCols, CV_32S, cv::Scalar(0));
    cv::Mat code1Gray(frameRectRows, frameRectCols, CV_32S, cv::Scalar(0));

    // into gray code
    for(int i=0; i<nGrayBits; i++){
        cv::Mat temp, bit0, bit1;

        cv::compare(frames0GrayCode[i*2], frames0GrayCode[i*2+1], temp, cv::CMP_GT);
        temp.convertTo(bit0, CV_32S, 1.0/255.0);
        cv::add(code0Gray, bit0*twopowi(nGrayBits-i-1), code0Gray, cv::noArray(), CV_32S);

        cv::compare(frames1GrayCode[i*2], frames1GrayCode[i*2+1], temp, cv::CMP_GT);
        temp.convertTo(bit1, CV_32S, 1.0/255.0);
        cv::add(code1Gray, bit1*twopowi(nGrayBits-i-1), code1Gray, cv::noArray(), CV_32S);
    }

    // convert to standard binary
    cv::Mat code0Binary(code0Gray.rows, code0Gray.cols, CV_32S, cv::Scalar(-1));
    cv::Mat code1Binary(code1Gray.rows, code1Gray.cols, CV_32S, cv::Scalar(-1));
    for(int r=0; r<frameRectRows; r++){
        for(int c=0; c<frameRectCols; c++){
            code0Binary.at<int>(r,c) = grayToBinary(code0Gray.at<int>(r,c));
            code1Binary.at<int>(r,c) = grayToBinary(code1Gray.at<int>(r,c));
        }
    }

    // set occluded pixels to -1
    for(int r=0; r<frameRectRows; r++){
        for(int c=0; c<frameRectCols; c++){
            if(occlusion0Rect.at<unsigned char>(r,c) == 0)
                code0Binary.at<int>(r,c) = -1;
            if(occlusion1Rect.at<unsigned char>(r,c) == 0)
                code1Binary.at<int>(r,c) = -1;
        }
    }

//cvtools::writeMat(code0Gray, "code0Gray.mat", "code0Gray");
//cvtools::writeMat(code1Gray, "code1Gray.mat", "code1Gray");
//cvtools::writeMat(code0Binary, "code0Binary.mat", "code0Binary");
//cvtools::writeMat(code1Binary, "code1Binary.mat", "code1Binary");

    // matching
    std::vector<cv::Vec2f> q0Rect, q1Rect;
    for(int s=0; s<nLineShifts; s++){

        cv::Mat lines0 = frames0LineShift[s];
        cv::Mat lines1 = frames1LineShift[s];

        for(int row=0; row<frameRectRows; row++){

            // line center data structure containing [x-coordinate (sub-px), region-code]
            std::vector<cv::Vec2f> lineCenters0, lineCenters1;

            // sorted, unique line centers
            getlineCenters(lines0.row(row), code0Binary.row(row), lineCenters0);
            getlineCenters(lines1.row(row), code1Binary.row(row), lineCenters1);

//             if(s==0 && row==1300){
//                std::cout << cv::Mat(lineCenters0) << std::endl;
//                std::cout << cv::Mat(lineCenters1) << std::endl;

//                cvtools::writeMat(lines0.row(row), "lines0.mat", "lines0");
//                cvtools::writeMat(lines1.row(row), "lines1.mat", "lines1");
//                cvtools::writeMat(code0Binary.row(row), "code0Binary.mat", "code0Binary");
//                cvtools::writeMat(code1Binary.row(row), "code1Binary.mat", "code1Binary");
//             }

            // match and store
            int i=0, j=0;
            while(i<lineCenters0.size() && j<lineCenters1.size()){

                if(lineCenters0[i][1] == lineCenters1[j][1]){
                    q0Rect.push_back(cv::Point2f(lineCenters0[i][0], row));
                    q1Rect.push_back(cv::Point2f(lineCenters1[j][0], row));
                    i += 1;
                    j += 1;
                } else if(lineCenters0[i][1] < lineCenters1[j][1]){
                    i += 1;
                } else if(lineCenters0[i][1] > lineCenters1[j][1]){
                    j += 1;
                }
            }

        }
    }

    int nMatches = q0Rect.size();

    if(nMatches < 1){
        Q.resize(0);
        color.resize(0);

        return;
    }

    // retrieve color information (at integer coordinates)
    color.resize(nMatches);
    for(int i=0; i<nMatches; i++){

        cv::Vec3b c0 = color0Rect.at<cv::Vec3b>(q0Rect[i][1], q0Rect[i][0]);
        cv::Vec3b c1 = color1Rect.at<cv::Vec3b>(q1Rect[i][1], q1Rect[i][0]);
//        cv::Vec3b c0 = getColorSubpix(color0Rect, q0Rect[i]);
//        cv::Vec3b c1 = getColorSubpix(color1Rect, q0Rect[i]);

        color[i] = 0.5*c0 + 0.5*c1;
    }

    // triangulate points
    cv::Mat QMatHomogenous, QMat;

    cv::triangulatePoints(P0, P1, q0Rect, q1Rect, QMatHomogenous);
    cvtools::convertMatFromHomogeneous(QMatHomogenous, QMat);

    // undo rectifying rotation
    cv::Mat R0Inv;
    cv::Mat(R0.t()).convertTo(R0Inv, CV_32F);
    QMat = R0Inv*QMat;

    cvtools::matToPoints3f(QMat, Q);

}
