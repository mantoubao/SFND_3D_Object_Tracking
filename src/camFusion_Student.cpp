
#include <iostream>
#include <algorithm>
#include <numeric>
#include <assert.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    vector<double> distance;
    for(auto kptmatch:kptMatches)
    {
        cv::KeyPoint curr_kpt = kptsCurr.at(kptmatch.trainIdx);
        
        if (!boundingBox.roi.contains(curr_kpt.pt)){
            continue;
        }

        cv::KeyPoint prev_kpt = kptsPrev.at(kptmatch.queryIdx);
        double ptdist= cv::norm(curr_kpt.pt - prev_kpt.pt);

        distance.push_back(ptdist);

    }

    double mean_dist= accumulate(distance.begin(), distance.end(),0.0)/distance.size();
    // size_t dist_size= distance.size();
    // if(dist_size!=0){
    //     sort(distance.begin(), distance.end());
    //     if(dist_size%2==0){
    //         median_dist=(distance[dist_size/2-1]+distance[dist_size/2])/2;
    //     }
    //     else{
    //         median_dist=distance[dist_size/2];
    //     }

    // }

    //filter outlier based on median distance
    for(auto kptmatch:kptMatches){
        cv::KeyPoint curr_kpt = kptsCurr.at(kptmatch.trainIdx);
        if (!boundingBox.roi.contains(curr_kpt.pt)){
            continue;
        }

        cv::KeyPoint prev_kpt = kptsPrev.at(kptmatch.queryIdx);
        double ptdist= cv::norm(curr_kpt.pt - prev_kpt.pt);
        if(ptdist<mean_dist*1.2 ){
            boundingBox.kptMatches.push_back(kptmatch);
        }

    }

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> vdistratios;
    double minDist =100.0;
    for(auto itr1=kptMatches.begin(); itr1!=kptMatches.end()-1; ++itr1)
    {
        cv::KeyPoint curr_kpt_outer = kptsCurr.at(itr1->trainIdx);
        cv::KeyPoint prev_kpt_outer = kptsPrev.at(itr1->queryIdx);

        for(auto itr2=kptMatches.begin()+1; itr2!=kptMatches.end(); ++itr2)
        {
            cv::KeyPoint curr_kpt_inner = kptsCurr.at(itr2->trainIdx);
            cv::KeyPoint prev_kpt_inner = kptsPrev.at(itr2->queryIdx);

            double dist_curr=cv::norm(curr_kpt_outer.pt - curr_kpt_inner.pt);
            double dist_prev=cv::norm(prev_kpt_outer.pt-prev_kpt_inner.pt);

            if(dist_prev>std::numeric_limits<double>::epsilon() && dist_curr>=minDist){
                double distratio= dist_curr/dist_prev;
                vdistratios.push_back(distratio);
            }

        }
    }

    if(vdistratios.size()==0){
        TTC=NAN;
        return;
    }

    sort(vdistratios.begin(),vdistratios.end());
    double median_ratio=0.0;
    size_t vratio_size= vdistratios.size();
    
    if(vratio_size%2==0){
        median_ratio=(vdistratios[vratio_size/2-1]+vdistratios[vratio_size/2])/2;
    }
    else{
        median_ratio=vdistratios[vratio_size/2];
    }

    double deltaT= 1/frameRate;
    TTC=-deltaT/(1-median_ratio);
    
}

void computeMeanStdvariance(const std::vector<LidarPoint> & points, double & mean, double & stdvariance)
{
    double sum = 0.0;
    for(auto point:points){
        sum+=point.x;
    }
    mean=sum/points.size();

    double variance=0.0;
    for(auto point:points){
        variance+=(point.x-mean)*(point.x-mean)/points.size();
    }
    stdvariance=sqrt(variance);

}

double calMinX(const std::vector<LidarPoint> & points, const double mean, const double std, double thrsholdfactor){
    double MinX= 1e9;
    for(auto point:points){
        if(point.x < mean-thrsholdfactor*std || point.x> mean+thrsholdfactor*std )
            continue;

        MinX= point.x<MinX?point.x:MinX;

    }
    return MinX;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double deltaT= 1/frameRate;
    double pre_mean, pre_stdvar, cur_mean, cur_stdvar;

    computeMeanStdvariance(lidarPointsPrev,pre_mean,pre_stdvar);
    computeMeanStdvariance(lidarPointsCurr,cur_mean,cur_stdvar);

    double MinXPrev=calMinX(lidarPointsPrev, pre_mean,pre_stdvar,1.5);
    double MinXCurr=calMinX(lidarPointsCurr, cur_mean,cur_stdvar,1.5);

    TTC=MinXCurr*deltaT/(MinXPrev-MinXCurr);

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    const int pre_bb_size =prevFrame.boundingBoxes.size();
    const int cur_bb_size = currFrame.boundingBoxes.size();
    vector<vector<int>> matching_scores(pre_bb_size, vector<int>(cur_bb_size,0) );
    for(auto match : matches)
    {
        cv::KeyPoint pre_keypoint = prevFrame.keypoints[match.queryIdx];
        cv::Point pre_point (pre_keypoint.pt.x, pre_keypoint.pt.y);

        cv::KeyPoint cur_keypoint = currFrame.keypoints[match.trainIdx];
        cv::Point cur_point (cur_keypoint.pt.x, cur_keypoint.pt.y);

        int pre_bb_index=-1;
        for(int i = 0;i<pre_bb_size;++i){
            if(prevFrame.boundingBoxes[i].roi.contains(pre_point)){
                pre_bb_index = i;
                 break;
            }               
        }

        int cur_bb_index=-1;
        for(int i=0;i<cur_bb_size; ++i){
            if(currFrame.boundingBoxes[i].roi.contains(cur_point)){
                cur_bb_index=i;
                break;
            }
        }

        //ignore if keypoint not in any bounding boxes
        if(pre_bb_index==-1||cur_bb_index==-1)
            continue;

        matching_scores.at(pre_bb_index).at(cur_bb_index)+=1; 

    }
    
    for(int pre_matchingID=0;pre_matchingID<matching_scores.size();++pre_matchingID){
        auto max_idx=max_element(matching_scores.at(pre_matchingID).begin(),matching_scores.at(pre_matchingID).end());
        if (*max_idx>0){
            int cur_matchingID=distance(matching_scores.at(pre_matchingID).begin(),max_idx);
            //cout<<"cur_matchingID ="<<cur_matchingID<<"  matching_cur_index="<<pre_matchingID<<endl;
            bbBestMatches[pre_matchingID]=cur_matchingID;
        }
        
    }
        
}
