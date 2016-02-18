#include "MacBethDetector.h"

static bool myfn(cv::RotatedRect i, cv::RotatedRect j) {
    return i.size.area()<j.size.area();
}

static bool coordinate_sort_y(cv::RotatedRect i, cv::RotatedRect j) {
    return i.center.y<j.center.y;
}

static bool coordinate_sort_x(cv::RotatedRect i, cv::RotatedRect j) {
    return i.center.x<j.center.x;
}

static bool distance_sort(float i, float j) {
    return i<j;
}

MacBethDetector::MacBethDetector()
{

}

/*
MacBethDetector::~MacBethDetector()
{

}


cv::Rect MacBethDetector::find_quad(cv::Mat contours, int min_size){
    std::vector<cv::Point> contours_poly;//( contours.size() );
    cv::Rect r;
    const int min_approx_level = 2, max_approx_level = MAX_CONTOUR_APPROX;
    int approx_level;
    for( approx_level = min_approx_level; approx_level <= max_approx_level; approx_level++ ){

        cv::approxPolyDP( contours, contours_poly, (float)approx_level, true );
        if( contours_poly.size() == 4 )
          break;
    }

  //  if(cv::isContourConvex(contours_poly[i])){
   cv::Rect temp = cv::boundingRect( cv::Mat(contours_poly) );
        if(temp.area() >= min_size){
            r = temp;
            std::cout << "x: " << r.x << " y: " << r.y << std::endl;
        }

      //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );

    return r;
}
*/

std::vector<cv::RotatedRect> MacBethDetector::findColorSquares(cv::Mat input_image, int method){

    cv::Mat macbeth_split[3];
    cv::Mat macbeth_split_thresh[3];

    cv::Mat pyr, timg, gray0(input_image.size(), CV_8U), gray;

       // down-scale and upscale the image to filter out the noise
       cv::pyrDown(input_image, pyr, cv::Size(input_image.cols/2, input_image.rows/2));
       cv::pyrUp(pyr, timg, input_image.size());

    for(int i = 0; i < 3; i++) {
        macbeth_split[i] = cv::Mat( cv::Size(input_image.cols, input_image.rows), input_image.depth(), 1 );
        macbeth_split_thresh[i] = cv::Mat( cv::Size(input_image.cols, input_image.rows), input_image.depth(), 1 );
    }

    //cv::split(input_image, macbeth_split);
    cv::split(timg, macbeth_split);

    int adaptive_method = CV_ADAPTIVE_THRESH_MEAN_C;
    int threshold_type = CV_THRESH_BINARY_INV;
    int block_size = int(round(MIN(input_image.cols,input_image.rows)*0.02));// | 1;
    std::cerr << "Using " << block_size << " as block size" << std::endl;

    double offset = 6;

    // do an adaptive threshold on each channel
    for(int i = 0; i < 3; i++) {

       cv::equalizeHist( macbeth_split[i], macbeth_split[i] );

       if(method == 0) cv::threshold(macbeth_split[i], macbeth_split_thresh[i], 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
       if(method > 0) cv::adaptiveThreshold(macbeth_split[i], macbeth_split_thresh[i], 255, adaptive_method, threshold_type, 9, offset);

       cv::dilate(macbeth_split_thresh[i], macbeth_split_thresh[i], cv::Mat(), cv::Point(-1,-1));
    }
/*
    cv::namedWindow("img1", 1 );
    cv::imshow("img1", macbeth_split_thresh[0]);
    cv::namedWindow("img2", 1 );
    cv::imshow("img2", macbeth_split_thresh[1]);
    cv::namedWindow("img3", 1 );
    cv::imshow("img3", macbeth_split_thresh[2]);
*/
    cv::Mat adaptive( cv::Size(input_image.cols, input_image.rows), CV_8U, 1 );

    // OR the binary threshold results together
    cv::bitwise_or(macbeth_split_thresh[0],macbeth_split_thresh[1],adaptive);
    cv::bitwise_or(macbeth_split_thresh[2],adaptive,adaptive);

//    cv::namedWindow("adaptive", 1 );
//    cv::imshow("adaptive", adaptive);

    int element_size = 2;//(block_size/10)+2;
    std::cerr << "Using " << element_size << " as element size" << std::endl;

    // do an opening on the threshold image
    cv::Mat element = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(element_size,element_size), cv::Point( -1,-1) );
    cv::morphologyEx(adaptive,adaptive,cv::MORPH_OPEN,element,cv::Point(-1,-1),1,cv::BORDER_CONSTANT);


    // find contours in the threshold image
    std::vector<std::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::findContours(adaptive,contours,hierarchy, cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);
    std::cout << contours.size() << " contours found!" << std::endl;
/*
    cv::RNG rng(12345);
    cv::cvtColor(adaptive, adaptive,CV_GRAY2RGB);
    for( int i = 0; i< contours.size(); i++ )
        {
          cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          cv::drawContours( adaptive, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );


        }

        cv::namedWindow("adaptive2", 1 );
        cv::imshow("adaptive2", adaptive);
       cv::waitKey(0);
*/
    int min_size = (input_image.cols*input_image.rows)/(MACBETH_SQUARES*100);

    std::vector<cv::RotatedRect> minRect;//( contours.size() );
    if(!contours.empty()) {
        /// Approximate contours to polygons + get bounding rects and circles
       std::vector<cv::Rect> boundRect( contours.size() );
       std::vector<std::vector<cv::Point> > contours_poly( contours.size() );

       /// Find the rotated rectangles and ellipses for each contour
        for( unsigned int i = 0; i < contours.size(); i++ ){
            if(method == 0){
            //Find rotated rectangles
            cv::RotatedRect rr = cv::minAreaRect(cv::Mat(contours[i]));
            if(rr.size.area()>= min_size){
                //Find all rectangular squares
              //  std::cout << "square fraction: " << std::abs(rr.size.height/rr.size.width) << std::endl;
              //  std::cout << "image fraction: " << double(input_image.cols)/double(input_image.rows) << std::endl;
                    if(std::abs(rr.size.height/rr.size.width) > 0.9 && std::abs(rr.size.height/rr.size.width) < 1.1){
                       // minRect[i] = rr;
                        minRect.push_back(rr);
                    }
                }
            }

            if(method == 1){
                cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i],  cv::arcLength(cv::Mat(contours[i]), true)*0.02, true );
                cv::Rect r = cv::boundingRect( cv::Mat(contours_poly[i]) );
                if(r.area() >= 500){
                    //Find all rectangular squares
                    if(std::abs(r.height - r.width) < 30.6){
                        cv::RotatedRect a;
                        a.angle = 0.0;
                        a.size.height = float(r.height);
                        a.size.width = float(r.width);
                        a.center.x = float(r.x + r.width/2);
                        a.center.y = float(r.y + r.height/2);
                        minRect.push_back(a);
                    }
                }

            }

        }
    }
            return minRect;
}

///Draw PCA Arrows
void MacBethDetector::drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    cv::line(img, p, q, colour, 2 , CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    cv::line(img, p, q, colour, 2, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    cv::line(img, p, q, colour, 2, CV_AA);
}

// Compute the board orientation using PCA
double MacBethDetector::board_orentation(std::vector<cv::RotatedRect> squares, cv::Mat img){
    std::vector<cv::Point> pts;
    for(unsigned int k = 0; k <squares.size(); k++){
        pts.push_back(squares[k].center);
    }

    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                               static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
//        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);
    }

    // Draw the principal components
    if(!img.empty()){
        cv::circle(img, cntr, 3, cv::Scalar(255, 0, 255), 2);
        cv::Point p1 = cntr + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
        cv::Point p2 = cntr - 0.02 * cv::Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
        drawAxis(img, cntr, p1, cv::Scalar(0, 255, 0), 1);
        drawAxis(img, cntr, p2, cv::Scalar(0, 0, 255), 5);
    }
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle * 180/CV_PI;

}

void MacBethDetector::median_filter(std::vector<cv::RotatedRect>& vec) {

    std::vector<cv::RotatedRect> copyVec = vec;
    vec.clear();
    std::sort(copyVec.begin(), copyVec.end(), myfn);
    cv::RotatedRect median_rect = copyVec[int(std::floor(copyVec.size()/2))];
    float median = median_rect.size.area();

    ///Compute variance
    double sum = 0.0;
    int length = 0;
    for( unsigned int j = 0; j< copyVec.size(); j++ ){
        sum += copyVec[j].size.area();
        length++;

    }
    float std_dev = sqrtf(sum/length);
    std::cout << "Median area: " << median << std::endl;
    std::cout << "Variance area: " << sum/length << std::endl;
    std::cout << "Std. dev area: " << std_dev << std::endl;

    for( unsigned int i = 0; i< copyVec.size(); i++ ){
       if(copyVec[i].size.area() < (median + 10* std_dev) && copyVec[i].size.area() > (median - 10* std_dev) ){
           vec.push_back(copyVec[i]);
       }
    }

}

void MacBethDetector::histogram_filtering(std::vector<cv::RotatedRect>& rect){

   ///Find max and minimum area
   cv::RotatedRect min = *std::min_element(rect.begin(),rect.end(), myfn);
   cv::RotatedRect max = *std::max_element(rect.begin(),rect.end(), myfn);

   std::array<std::vector<cv::RotatedRect>,4> hist;

   std::cout << "min area:" <<  min.size.area() << std::endl;
   std::cout << "max area:" <<  max.size.area() << std::endl;
   float interval = std::abs(max.size.area()-min.size.area());
   float bin_size = interval/4;

   std::cout << "interval:" <<  interval << std::endl;
   std::cout << "bin_size:" <<  bin_size << std::endl;
   std::cout << "interval 0: " << 0 << " to " << bin_size+ min.size.area() << std::endl;
   std::cout << "interval 1: " << bin_size+ min.size.area() << " to " << 2*bin_size+ min.size.area() << std::endl;
   std::cout << "interval 2: " << 2*bin_size+ min.size.area() << " to " << 3*bin_size+ min.size.area() << std::endl;
   std::cout << "interval 3: " << 3*bin_size+ min.size.area() << " to " << 4*bin_size+ min.size.area() << std::endl;

    for( unsigned int j = 0; j< rect.size(); j++ ){

         cv::RotatedRect r = rect.at(j);
        if(r.size.area() < (bin_size+ min.size.area()) ){
         // Insert to bin 0
            hist[0].push_back(r);
        }else if(r.size.area() >= bin_size+ min.size.area() || r.size.area() < 2*bin_size+ min.size.area()){
         // Insert to bin 1
            hist[1].push_back(r);
        }else if(2*r.size.area() >= bin_size+ min.size.area() || r.size.area() < 3*bin_size+ min.size.area()){
         // Insert to bin 2
            hist[2].push_back(r);
        }else if(3*r.size.area() >= bin_size+ min.size.area() || r.size.area() < 4*bin_size+ min.size.area()){
         // Insert to bin 3
            hist[3].push_back(r);
        }
    }

    std::cout << "bin 0 size: " << hist[0].size()<< std::endl;
    std::cout << "bin 1 size: " << hist[1].size()<< std::endl;
    std::cout << "bin 2 size: " << hist[2].size()<< std::endl;
    std::cout << "bin 3 size: " << hist[3].size()<< std::endl;

    unsigned int largest = 0;
    unsigned int index = 0;
    for( unsigned int k = 0; k< hist.size(); k++ ){
       if(hist[k].size() > largest){
           largest = hist[k].size();
           index = k;
       }
    }
    rect.clear();
    rect = hist[index];



}

bool MacBethDetector::colorSort(std::vector<cv::RotatedRect> &rect,  cv::Mat &img){

    if(img.empty()){
        std::cerr << "Input image is empty in colorSort()" << std::endl;
        return false;
    }

    //Map to store the correct order
    std::map<int,int> id_map;

    //Create Ground truth matrix
    cv::Mat macBeth_colors = cv::Mat(3, 24, CV_64F, ArrayTrueRGBValues);
    std::vector<cv::Point3f> mac_colors;
    for( int k = 0; k< macBeth_colors.cols; k++ ){
       cv::Point3f ptn(macBeth_colors.at<double>(0,k), macBeth_colors.at<double>(1,k),macBeth_colors.at<double>(2,k));
       mac_colors.push_back(ptn);
    }

    //Get rig_colors
    std::vector<cv::Point3f> rig_colors;
    for( unsigned int i = 0; i< rect.size(); i++ ){
        cv::Rect bounding_box = rect[i].boundingRect();
        // Setup a rectangle to define your region of interest
        cv::Rect myROI(bounding_box.x, bounding_box.y, bounding_box.width, bounding_box.height);
        //Get the color of each rectangle
        cv::Mat region = img(myROI);
        cv::Scalar mean_color = cv::mean(region);
        rig_colors.push_back(cv::Point3f( mean_color.val[2], mean_color.val[1],  mean_color.val[0]));
    }

  /*  if(rig_colors.size() != mac_colors.size()){
        std::cerr << "The number of Rig colors and Mac_colors are not identical! " << std::endl;
        return false;
    }
*/

    // Find nearest neighboors
    std::vector<double> rig_L2;
    std::vector<int> dst_ids;
    cv::flann::KDTreeIndexParams indexParams(4);
    cv::flann::Index kdtree(cv::Mat(mac_colors).reshape(1), indexParams);
    int knn = 3;
    for(unsigned int j = 0; j< rig_colors.size(); j++){
        cv::vector<float> query;
        query.push_back(rig_colors[j].x);
        query.push_back(rig_colors[j].y);
        query.push_back(rig_colors[j].z); //Insert the 3D point we need to find neighbours to the query
        std::vector<int> indices;
        std::vector<float> dists;
        kdtree.knnSearch(query, indices, dists, knn,cv::flann::SearchParams(64));

        std::cout << std::endl;
        std::cout << "ID: "  << j <<  " rig_color: [" << rig_colors[j].x << " ," << rig_colors[j].y << " ," << rig_colors[j].z << "]" << std::endl;
        double L2_rig = sqrtf(rig_colors[j].x*rig_colors[j].x + rig_colors[j].y*rig_colors[j].y + rig_colors[j].z*rig_colors[j].z);
        rig_L2.push_back(L2_rig);
        std::cout << "L2_rig: " << L2_rig << std::endl;

        std::cout << "Nearest Neighboor ID: "  << indices[0] << " mac_colors: [" << mac_colors[indices[0]].x << " ," << mac_colors[indices[0]].y << " ," << mac_colors[indices[0]].z << "]" << std::endl;
        double L2_mac = sqrtf(mac_colors[indices[0]].x*mac_colors[indices[0]].x + mac_colors[indices[0]].y*mac_colors[indices[0]].y + mac_colors[indices[0]].z*mac_colors[indices[0]].z);

        std::vector<int>::iterator it = std::find(dst_ids.begin(), dst_ids.end(), indices[0]);
        if(it!= dst_ids.end()){
           int index = std::distance(dst_ids.begin(), it);
           std::cout << "THE DST ID " <<  indices[0] << " IS ALREADY IDENTIFIED AS QUERY_ID " << index << " !!!!!" << std::endl;

           //Compare L2 metric -> select the one with smallest L2 distance
           std::map<int,float> L2_diff;  std::map<int,float> L2_diff_index;
           for(unsigned int t = 0; t<indices.size(); t++){
              //Compute L2 distance for nearest neighboors
              L2_mac = sqrtf(mac_colors[indices[t]].x*mac_colors[indices[t]].x + mac_colors[indices[t]].y*mac_colors[indices[t]].y + mac_colors[indices[t]].z*mac_colors[indices[t]].z);
              // Compute L2 for the old index
              double L2_rig_index = sqrtf(rig_colors[index].x*rig_colors[index].x + rig_colors[index].y*rig_colors[index].y + rig_colors[index].z*rig_colors[index].z);

              //Store all distances
              L2_diff.insert(std::pair<int, float>(indices[t],std::abs(L2_mac-L2_rig)));
              L2_diff_index.insert(std::pair<int, float>(indices[t],std::abs(L2_mac-L2_rig_index)));

           }

           float best_index = 200; float best = 200;
           auto it1 = L2_diff.begin();
           auto it2 = L2_diff_index.begin();

           while (it1 != L2_diff.end())
           {
               float dist_index =  it2->second;
               float dist =  it1->second;

               //Find smallest score
               if(best_index > dist_index)
                   best_index = dist_index;

               if(best > dist)
                   best = dist;




               std::cout << "dist_index: " << dist_index << " for index: " << it2->first <<  std::endl;
               std::cout << "dist: " << dist << " for index: " << it1->first <<  std::endl;

               ++it1;
               ++it2;
           }

           std::cout << "==============================================="<< std::endl;
           std::cout << "Best dst ID for: " << index << " is " << best_index << std::endl;
           std::cout << "Best dst ID for: " << j << " is " << best << std::endl;
           dst_ids.push_back(indices[0]);
           id_map.insert(std::pair<int,int>(index,best));

        }else{

           //Save used ids
           dst_ids.push_back(indices[0]);
           id_map.insert(std::pair<int,int>(j,indices[0]));
         }

        std::cout << std::endl;

}


   cv::RotatedRect a;
   a.angle = 0;
   a.size.height = 100;
   a.size.width = 100;
   a.center.x = -1;
   a.center.y = -1;

    std::vector<cv::RotatedRect> result;
    result.resize(24,a);
    for(std::map<int,int>::iterator it = id_map.begin(); it != id_map.end(); ++it){
       int query_id = it->first;
       int dst_id = it->second;
       std::cout << "QUERY ID:"  << query_id << " MAPS TO DST ID: " << dst_id << std::endl;
       cv::RotatedRect r = rect[query_id];
       result.at(dst_id) = r;
    }

    rect.clear();
    for(unsigned int y=0; y< result.size(); y++)
       rect.push_back(result[y]);

    return true;
}

void MacBethDetector::sortAndID(std::vector<cv::RotatedRect> rect,  cv::Mat img){

    //Get the rotation of the MacBeth colorchecker
    double orientation = 0;
    int landscape = -1;

    if(!img.empty()){
        orientation =  board_orentation(rect, img);
    }else{
        orientation =  board_orentation(rect);
    }

    //Initial sort squares according the there position
    if(orientation < 45 && orientation > -45){
        //Orientation = landscape
        landscape = 1;
        std::sort(rect.begin(),rect.end(), coordinate_sort_y);
    }else{
        landscape = 0;
        std::sort(rect.begin(),rect.end(),coordinate_sort_x);
    }

    colorSort(rect, img);

/*    double avg_height = 0;
    double avg_width = 0;
    double avg_angle = 0;
    cv::vector<cv::Point2f> pointsForSearch; //Insert all 2D points to this vector
    for( unsigned int i = 0; i< rect.size(); i++ ){
        cv::Point2f p; p.x = rect[i].center.x; p.y = rect[i].center.y;
        if(!img.empty()){
            std::stringstream ss; ss << i;
            cv::putText(img,ss.str(),cv::Point2f((p.x -rect[i].size.width/2), (p.y - rect[i].size.height/8)) ,CV_FONT_HERSHEY_COMPLEX_SMALL,0.8,cv::Scalar(0,0,255));
        }

        avg_height +=  rect[i].size.height;
        avg_width += rect[i].size.width;
        avg_angle += rect[i].angle;
        pointsForSearch.push_back(p);
    }

    //Compute average height, width and angle for all rectangles
    avg_height = avg_height/rect.size();
    avg_width = avg_width/rect.size();
    avg_angle = avg_angle=rect.size();

    std::cout << "==================================================== " << std::endl;
    std::cout << "Average square height = " << avg_height << std::endl;
    std::cout << "Average square width = " << avg_width << std::endl;
    std::cout << "Average square angle = " << avg_angle << std::endl;
    std::cout << "Board orientation in degrees: " << orientation << std::endl;
    std::cout << "Board orientation = " << (landscape == 1 ? "Landscape" : "Portrait") << std::endl;

    int numOfPoints = 5; //search for max 5 nn points
    cv::flann::KDTreeIndexParams indexParams(5);
    cv::flann::Index kdtree(cv::Mat(pointsForSearch).reshape(1), indexParams);

    //Map to store edge rectangle elements
    std::map<int, cv::Point2f> edge_elements;

    // Find nearest neighboors
    for(unsigned int j = 0; j< pointsForSearch.size(); j++ ){
        cv::vector<float> query;
        //  int j=15;
        query.push_back(pointsForSearch[j].x); //Insert the 2D point we need to find neighbours to the query
        query.push_back(pointsForSearch[j].y); //Insert the 2D point we need to find neighbours to the query
        std::vector<int> indices;
        std::vector<float> dists;
        kdtree.knnSearch(query, indices, dists, numOfPoints,cv::flann::SearchParams(64));

        //Erase the seed point in indices and dists vector
        for(std::vector<int>::iterator itr = indices.begin(); itr != indices.end();){
            if(pointsForSearch[j].x == pointsForSearch[*itr].x && pointsForSearch[j].y == pointsForSearch[*itr].y){
                itr = indices.erase(itr);
                int index = std::distance(indices.begin(),itr);
                std::vector<float>::iterator it = dists.begin();
                std::advance(it,index);
                dists.erase(it);
            }else{
                ++itr;
          }
        }

        //find the three neaest neighbors and dertermine if the point is an edge
        float average = std::accumulate(dists.begin(),dists.end(),0.0)/dists.size();
        int point_count = 0;
        for( unsigned int k = 0; k< indices.size(); k++ ){
            if(dists[k] < 1.3*average ){
                point_count++;
            }
        }

       // std::cout << "point_count: " << point_count << std::endl;
        //Store the edge elements
        //if(point_count < 4){
            edge_elements.insert(std::pair<int, cv::Point2f>(j, pointsForSearch[j]));

            if(!img.empty())cv::circle(img, pointsForSearch[j] , 6, cv::Scalar(0,255,0),2,8);
        //}//else{
         //   edge_elements.insert(std::pair<int, cv::Point2f>(point_count,pointsForSearch[j]));
       // }
    }

    std::cout << "edge_elements: " << edge_elements.size() << std::endl;
    std::map<int,cv::Point2f>::iterator  it;
    for(it = edge_elements.begin(); it!= edge_elements.end(); ++it){
        //int id = it->first;
        //cv::Point2f p1 = it->second;
      //  std::cout << "id: " << id << std::endl;
      //  std::cout << "MacBeth color: " << macBeth_colors.at<double>(0,id) << " " << macBeth_colors.at<double>(1,id) << " " << macBeth_colors.at<double>(2,id) << std::endl;
    //   cv::circle(input_image,p1, 3, cv::Scalar(255,255,0),2,8);
    }
*/
}

void MacBethDetector::findColorCenters(cv::Mat input_image, cv::Point upperleft, cv::Point lowerright){

  cv::RNG rng(12345);

  cv::Point p1(upperleft.x - 10, upperleft.y -10);
  cv::Point p2(lowerright.x + 10, lowerright.y +10);

  cv::Rect myROI(p1,p2);

  cv::Mat cropped = input_image(myROI);

  std::vector<cv::RotatedRect> rect = findColorSquares(cropped, 1);
  std::cout << "NUMBER OF SQUARES "  <<  rect.size() << std::endl;
  ///Remove elements that lies more than 10 * std_dev away
  median_filter(rect);

  //Not all MacBeth fields are found -> search for missing
  if(rect.size() != MACBETH_HEIGHT * MACBETH_WIDTH){
       std::cout << "FAILED TO DETECT "  << MACBETH_SQUARES - rect.size() << " SQUARES!" << std::endl;
       sortAndID(rect, cropped);
 }

  //Draw squares
  for( unsigned int i = 0; i< rect.size(); i++ ){
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
      // rotated rectangle
      cv::Point2f rect_points[4]; rect[i].points( rect_points );
      for( int j = 0; j < 4; j++ ){
         cv::line( cropped, rect_points[j], rect_points[(j+1)%4], color, 2, 8 );
      }
         cv::circle(cropped,rect[i].center, 1, cv::Scalar(255,255,255),2,8);
  }

   cv::namedWindow("find", 1 );
   cv::imshow("find", cropped);
   cv::waitKey();

}



