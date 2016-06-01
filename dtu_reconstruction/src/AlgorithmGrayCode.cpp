#include "AlgorithmGrayCode.h"
#include <cmath>
#include "cvtools.h"

#include "white_balance.hpp"

#ifndef log2f
#define log2f(x) (log(x)/log(2.0))
#endif

//using namespace std;

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

//static int get_bit(int decimal, int N){

//    // Shifting the 1 for N-1 bits
//    int constant = 1 << (N-1);

//    // If the bit is set, return 1
//    if( decimal & constant )
//        return 1;
//    else
//        return 0;
//}

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
AlgorithmGrayCode::AlgorithmGrayCode(unsigned int _screenCols, unsigned int _screenRows) : Algorithm(_screenCols, _screenRows){

    Nbits = ceilf(log2f((float)screenCols));
    N = 2 + Nbits*2;

    std::cout << "NBits " << Nbits << std::endl;
    // all on pattern
    cv::Mat allOn(1, screenCols, CV_8UC3, cv::Scalar::all(255));
    patterns.push_back(allOn);

    // all off pattern
    cv::Mat allOff(1, screenCols, CV_8UC3, cv::Scalar::all(0));
    patterns.push_back(allOff);


    // horizontally encoding patterns
    for(unsigned int p=0; p<Nbits; p++){
        cv::Mat pattern(1, screenCols, CV_8UC3);
        cv::Mat patternInv(1, screenCols, CV_8UC3);

        for(unsigned int j=0; j<screenCols; j++){

            unsigned int jGray = binaryToGray(j);
            // Amplitude of channels
            int bit = (int)getBit(jGray, Nbits-p+1);
            pattern.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*bit,255.0*bit,255.0*bit);
            int invBit = bit^1;
            patternInv.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*invBit,255.0*invBit,255.0*invBit);
        }
        patterns.push_back(pattern);
        patterns.push_back(patternInv);
    /*    std::stringstream ss; ss << "/home/thso/gray_"; ss << p; ss << ".png";
        cv::imwrite(ss.str(), pattern);
        ss.str("");
        ss << "/home/thso/gray_"; ss << p; ss << ".png";
        cv::imwrite(ss.str(), patternInv);
        */
    }


}

cv::Mat AlgorithmGrayCode::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}


bool sortingLarger(cv::Vec4i i,cv::Vec4i j){ return (i[3]<j[3]);}
bool sortingEqual(cv::Vec4i i,cv::Vec4i j){ return (i[3]==j[3]);}
void getEdgeLabels(const cv::Mat& scanLine, int Nbits, std::vector<cv::Vec4i>& edges){

    int nCols = scanLine.cols;
    const int *data = scanLine.ptr<const int>(0);

    int labelLeft;
    int labelRight = data[0];

    // collect edges
    for(int col=1; col<nCols; col++){

        labelLeft = labelRight;
        labelRight = data[col];

        // labels need to be non-background, and differ in exactly one bit
        if(labelLeft != -1 && labelRight != -1 && (grayToBinary(labelRight) == grayToBinary(labelLeft)+1)){
            int orderingRelation = (labelLeft << Nbits) + labelRight;
            // store left label column
            edges.push_back(cv::Vec4i(col-1, labelLeft, labelRight, orderingRelation));
        }
    }

    // sort
    std::sort(edges.begin(), edges.end(), sortingLarger);

    // remove duplicates
    std::vector<cv::Vec4i>::iterator it;
    it = std::unique(edges.begin(), edges.end(), sortingEqual);
    edges.resize(std::distance(edges.begin(),it));
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

void AlgorithmGrayCode::get3DPoints(SMCalibrationParameters calibration, const std::vector<cv::Mat>& frames0, const std::vector<cv::Mat>& frames1, std::vector<cv::Point3f>& Q, std::vector<cv::Vec3b>& color){

    assert(frames0.size() == N);
    assert(frames1.size() == N);

//    for(int i=0; i<1920; i++){
//        std::cout << i << " " << binaryToGray(i) << " " << grayToBinary(binaryToGray(i)) << std::endl;
//    }

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

//    std::cout << "R0" << std::endl << R0 << std::endl;
//    std::cout << "P0" << std::endl << P0 << std::endl;
//    std::cout << "R1" << std::endl << R1 << std::endl;
//    std::cout << "P1" << std::endl << P1 << std::endl;

    // interpolation maps
    cv::Mat map0X, map0Y, map1X, map1Y;
    cv::initUndistortRectifyMap(calibration.K0, calibration.k0, R0, P0, frameSize, CV_32F, map0X, map0Y);
    cv::initUndistortRectifyMap(calibration.K1, calibration.k1, R1, P1, frameSize, CV_32F, map1X, map1Y);

    // gray-scale and remap
    std::vector<cv::Mat> frames0Rect(N);
    std::vector<cv::Mat> frames1Rect(N);
    for(unsigned int i=0; i<N; i++){
        cv::Mat temp;
     //   cv::cvtColor(frames0[i], temp, CV_BayerBG2GRAY);
         cv::cvtColor(frames0[i], temp, CV_BGR2GRAY);
        cv::remap(temp, frames0Rect[i], map0X, map0Y, CV_INTER_LINEAR);

  //      std::stringstream ss; ss << "/home/thso/gray_stl_left"; ss << i << ".png";
    //    cv::imwrite(ss.str(), frames0Rect[i]);

         cv::cvtColor(frames1[i], temp, CV_BGR2GRAY);
    //    cv::cvtColor(frames1[i], temp, CV_BayerBG2GRAY);
        cv::remap(temp, frames1Rect[i], map1X, map1Y, CV_INTER_LINEAR);

    //    std::stringstream ss1; ss1 << "/home/thso/gray_stl_right"; ss1 << i << ".png";
    //    cv::imwrite(ss1.str(), frames1Rect[i]);

    }


//    cvtools::writeMat(frames0Rect[0], "frames0Rect_0.mat", "frames0Rect_0");
//    cvtools::writeMat(frames0[0], "frames0_0.mat", "frames0_0");

//    cvtools::writeMat(frames0Rect[22], "frames0Rect_22.mat", "frames0Rect_22");
//    cvtools::writeMat(frames0Rect[23], "frames0Rect_23.mat", "frames0Rect_23");

//    cv::imwrite("frames0[0].png", frames0[0]);
//    cv::imwrite("frames0Rect[0].png", frames0Rect[0]);

//    cv::imwrite("frames1[0].png", frames1[0]);
//    cv::imwrite("frames1Rect[0].png", frames1Rect[0]);

    // color debayer and remap
    cv::Mat color0Rect, color1Rect;

//    frames0[0].convertTo(color0Rect, CV_8UC1, 1.0/256.0);
   // cv::cvtColor(frames0[0], color0Rect, CV_BayerBG2RGB);
    color0Rect = frames0[0];
    cv::remap(color0Rect, color0Rect, map0X, map0Y, CV_INTER_LINEAR);
    cv::xphoto::balanceWhite(color0Rect,color0Rect,cv::xphoto::WHITE_BALANCE_SIMPLE);
    cv::imwrite("/home/thso/color_left_stl.png", color0Rect);

  //    frames1[0].convertTo(color1Rect, CV_8UC1, 1.0/256.0);
  //  cv::cvtColor(frames1[0], color1Rect, CV_BayerBG2RGB);
    color1Rect = frames1[0];
    cv::remap(color1Rect, color1Rect, map1X, map1Y, CV_INTER_LINEAR);
    cv::xphoto::balanceWhite(color1Rect,color1Rect,cv::xphoto::WHITE_BALANCE_SIMPLE);
    cv::imwrite("/home/thso/color_right_stl.png", color1Rect);

    int frameRectRows = frames0Rect[0].rows;
    int frameRectCols = frames0Rect[0].cols;

//cvtools::writeMat(frames0Rect[0], "frames0Rect_0.mat", "frames0Rect_0");
//cvtools::writeMat(frames0Rect[1], "frames0Rect_1.mat", "frames0Rect_1");
//cvtools::writeMat(frames0Rect[20], "frames0Rect_20.mat", "frames0Rect_20");
//cvtools::writeMat(frames0Rect[21], "frames0Rect_21.mat", "frames0Rect_21");

    // occlusion masks
    cv::Mat occlusion0Rect, occlusion1Rect;
    cv::subtract(frames0Rect[0], frames0Rect[1], occlusion0Rect);
    occlusion0Rect = (occlusion0Rect > 10) & (occlusion0Rect < 250); //10 & 250
    cv::subtract(frames1Rect[0], frames1Rect[1], occlusion1Rect);
    occlusion1Rect = (occlusion1Rect > 10) & (occlusion1Rect < 250);


    // erode occlusion masks
    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2));
    cv::erode(occlusion0Rect, occlusion0Rect, strel);
    cv::erode(occlusion1Rect, occlusion1Rect, strel);

  //  cv::imwrite("/home/thso/occlusion0Rect.png", occlusion0Rect);
  //  cv::imwrite("/home/thso/occlusion1Rect.png", occlusion1Rect);

//cvtools::writeMat(frames0Rect[0], "frames0Rect_0.mat", "frames0Rect_0");
//cvtools::writeMat(frames0Rect[1], "frames0Rect_1.mat", "frames0Rect_1");
//cvtools::writeMat(frames0Rect[20], "frames0Rect_20.mat", "frames0Rect_20");
//cvtools::writeMat(frames0Rect[21], "frames0Rect_21.mat", "frames0Rect_21");

//    // correct for projector inversion error
//    cv::Mat W;
//    cv::add(frames0Rect[0], frames0Rect[1], W, cv::noArray(), CV_32F);
//    for(int i=2; i<N; i+=2){
//        cv::Mat S, E;
//        cv::add(frames0Rect[i], frames0Rect[i+1], S, cv::noArray(), CV_32F);
//        cv::subtract(W, S, E, cv::noArray(), CV_32F);
//        E *= 0.5;
//        cv::add(frames0Rect[i], E, frames0Rect[i], cv::noArray(), CV_16UC1);
//        cv::add(frames0Rect[i+1], E, frames0Rect[i+1], cv::noArray(), CV_16UC1);
//    }


//    // correct for texture modulation and ambient
//    cv::Mat A0 = frames0Rect[1];
//    cv::Mat M0 = frames0Rect[0]-frames0Rect[1];
////cvtools::writeMat(A0, "A0.mat", "A0");
////cvtools::writeMat(M0, "M0.mat", "M0");
////cvtools::writeMat(frames0Rect[20], "frames0Rect_20.mat", "frames0Rect_20");
////cvtools::writeMat(frames0Rect[21], "frames0Rect_21.mat", "frames0Rect_21");
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

//cvtools::writeMat(occlusion0Rect, "occlusion0Rect.mat", "occlusion0Rect");
//cvtools::writeMat(occlusion1Rect, "occlusion1Rect.mat", "occlusion1Rect");

    // decode patterns
    cv::Mat code0Rect(frameRectRows, frameRectCols, CV_32S, cv::Scalar(0));
    cv::Mat code1Rect(frameRectRows, frameRectCols, CV_32S, cv::Scalar(0));

    // into gray code
    for(unsigned int i=0; i<Nbits; i++){
        cv::Mat temp, bit0, bit1;

        cv::compare(frames0Rect[i*2+2], frames0Rect[i*2+3], temp, cv::CMP_GT);
        temp.convertTo(bit0, CV_32S, 1.0/255.0);
        cv::add(code0Rect, bit0*twopowi(Nbits-i-1), code0Rect, cv::noArray(), CV_32S);

        cv::compare(frames1Rect[i*2+2], frames1Rect[i*2+3], temp, cv::CMP_GT);
        temp.convertTo(bit1, CV_32S, 1.0/255.0);
        cv::add(code1Rect, bit1*twopowi(Nbits-i-1), code1Rect, cv::noArray(), CV_32S);
    }

//cvtools::writeMat(code0Rect, "code0Rect.mat", "code0Rect");
//cvtools::writeMat(code1Rect, "code1Rect.mat", "code1Rect");


//    // convert to standard binary
//    cv::Mat code0Binary(code0Rect.rows, code0Rect.cols, CV_32F);
//    cv::Mat code1Binary(code1Rect.rows, code1Rect.cols, CV_32F);
//    for(int r=0; r<frameRectRows; r++){
//        for(int c=0; c<frameRectCols; c++){
//            if(code0Rect.at<int>(r,c) != -1)
//                code0Binary.at<float>(r,c) = grayToBinary(code0Rect.at<int>(r,c));
//            if(code1Rect.at<int>(r,c) != -1)
//                code1Binary.at<float>(r,c) = grayToBinary(code1Rect.at<int>(r,c));
//        }
//    }

//cvtools::writeMat(code0Binary, "code0Binary.mat", "code0Binary");
//cvtools::writeMat(code1Binary, "code1Binary.mat", "code1Binary");

//    // threshold on vertical discontinuities (due to imperfect rectification)
//    cv::Mat edges0;
//    cv::Sobel(code0Binary, edges0, -1, 0, 1, 5);
//    occlusion0Rect = occlusion0Rect & (abs(edges0) < 50);

//    cv::Mat edges1;
//    cv::Sobel(code1Binary, edges1, -1, 0, 1, 5);
//    occlusion1Rect = occlusion1Rect & (abs(edges1) < 50);

//cvtools::writeMat(edges0, "edges0.mat", "edges0");
//cvtools::writeMat(edges1, "edges1.mat", "edges1");

    // set occluded pixels to -1
    for(int r=0; r<frameRectRows; r++){
        for(int c=0; c<frameRectCols; c++){
            if(occlusion0Rect.at<unsigned char>(r,c) == 0)
                code0Rect.at<int>(r,c) = -1;
            if(occlusion1Rect.at<unsigned char>(r,c) == 0)
                code1Rect.at<int>(r,c) = -1;
        }
    }

       cv::imwrite("/home/thso/code0Rect.png", code0Rect);
       cv::imwrite("/home/thso/code1Rect.png", code1Rect);

//    cvtools::writeMat(code0Rect, "code0Rect.mat", "code0Rect");
//    cvtools::writeMat(code1Rect, "code1Rect.mat", "code1Rect");

    // matching
    std::vector<cv::Vec2f> q0Rect, q1Rect;
    for(int row=0; row<frameRectRows; row++){

        // edge data structure containing [floor(column), labelLeft, labelRight, orderingRelation]
        std::vector<cv::Vec4i> edges0, edges1;

        // sorted, unique edges
        getEdgeLabels(code0Rect.row(row), Nbits, edges0);
        getEdgeLabels(code1Rect.row(row), Nbits, edges1);

        // match edges
        std::vector<cv::Vec4i> matchedEdges0, matchedEdges1;
        unsigned int i=0, j=0;
        while(i<edges0.size() && j<edges1.size()){

            if(edges0[i][3] == edges1[j][3]){
                matchedEdges0.push_back(edges0[i]);
                matchedEdges1.push_back(edges1[j]);
                i += 1;
                j += 1;
            } else if(edges0[i][3] < edges1[j][3]){
                i += 1;
            } else if(edges0[i][3] > edges1[j][3]){
                j += 1;
            }
        }

        // crude subpixel refinement
        // finds the intersection of linear interpolants in the positive/negative pattern
        for(unsigned int i=0; i<matchedEdges0.size(); i++){

            int level = Nbits - leastSignificantBitSet(matchedEdges0[i][1]^matchedEdges0[i][2]);

            // refine for camera 0
            float c0 = matchedEdges0[i][0];
            float c1 = c0+1;

            float pos0 = frames0Rect[2*level+2].at<unsigned char>(row, c0);
            float pos1 = frames0Rect[2*level+2].at<unsigned char>(row, c1);
            float neg0 = frames0Rect[2*level+3].at<unsigned char>(row, c0);
            float neg1 = frames0Rect[2*level+3].at<unsigned char>(row, c1);

            float col = c0 + (pos0 - neg0)/(neg1 - neg0 - pos1 + pos0);
            q0Rect.push_back(cv::Point2f(col, row));

            // refine for camera 1
            c0 = matchedEdges1[i][0];
            c1 = c0+1;

            pos0 = frames1Rect[2*level+2].at<unsigned char>(row, c0);
            pos1 = frames1Rect[2*level+2].at<unsigned char>(row, c1);
            neg0 = frames1Rect[2*level+3].at<unsigned char>(row, c0);
            neg1 = frames1Rect[2*level+3].at<unsigned char>(row, c1);

            col = c0 + (pos0 - neg0)/(neg1 - neg0 - pos1 + pos0);
            q1Rect.push_back(cv::Point2f(col, row));

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

        color[i] = c0;// 0.5*c0 + 0.5*c1;
    }

    // triangulate points
    cv::Mat QMatHomogenous, QMat;
//    cv::Mat C0 = P0.clone();
//    cv::Mat C1 = P1.clone();
//    C0.colRange(0, 3) = C0.colRange(0, 3)*R0;
//    C1.colRange(0, 3) = C1.colRange(0, 3)*R1.t();
    cv::triangulatePoints(P0, P1, q0Rect, q1Rect, QMatHomogenous);
    cvtools::convertMatFromHomogeneous(QMatHomogenous, QMat);

    // undo rectifying rotation
    cv::Mat R0Inv;
    cv::Mat(R0.t()).convertTo(R0Inv, CV_32F);
    QMat = R0Inv*QMat;

    cvtools::matToPoints3f(QMat, Q);

}
