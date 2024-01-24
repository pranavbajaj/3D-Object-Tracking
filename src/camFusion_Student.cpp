
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set> 
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
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

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


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait, double *XMIN)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
    double ultmin = 1e8;
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
	if (xwmin < ultmin){ultmin = xwmin;}
	//cout << "Closest Lider point on ego car, x: " << xwmin << endl;
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }
    *XMIN = ultmin;
    //cout << "Xmin: " << ultmin << endl;

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

		
	
	
	vector<double> errors; 
	vector<cv::DMatch> matches;
	vector<cv::KeyPoint> currBBKepts; 
	cv::KeyPoint Curr;
	cv::KeyPoint Prev;
	
	for (auto it1 = kptMatches.begin(); it1 != kptMatches.end(); ++it1){
		if (boundingBox.roi.contains(kptsCurr[it1->trainIdx].pt)){
			Curr = kptsCurr[it1->trainIdx];
			Prev = kptsPrev[it1->queryIdx];			
			matches.push_back(*it1);
			errors.push_back(cv::norm(Curr.pt - Prev.pt));
			currBBKepts.push_back(Curr);
		}		
	}
	
	double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size(); 
	
	double threshold = 1.75 * mean;

	for (int i = 0; i < errors.size(); i++){
		if (errors[i] <= threshold){
			boundingBox.kptMatches.push_back(matches[i]);
			boundingBox.keypoints.push_back(currBBKepts[i]);
		}
	}

    
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
	vector<double> distRatios; 
	for(auto it1 = kptMatches.begin(); it1 != kptMatches.end() -1; ++it1){
		
		cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
		cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);
	
		for (auto it2 = kptMatches.begin() +  1; it2 != kptMatches.end(); ++it2){
			
			double minDist = 120.0;

			cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
			cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

			double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
			double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

			if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist){
				double distRatio = distCurr / distPrev;
				distRatios.push_back(distRatio);
			}
		}
	}

	if (distRatios.size() == 0){
		TTC = NAN;
		return;
	}	
	
	sort(distRatios.begin(), distRatios.end());

	// Medain Distance ratio calculation	
	long medIndex = floor(distRatios.size() / 2.0);
	double medianDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];
	
	// Mean distance ratio calculation
	/*
	double meanDistRatio = 0;
	int ptsConsidered = 0; 
	
	for (auto it = (distRatios.begin() + floor(distRatios.size() / 4)); it != (distRatios.end() - floor(distRatios.size() / 4)); ++it){
		meanDistRatio += *it;
		ptsConsidered++;
	}	
	meanDistRatio = meanDistRatio / ptsConsidered; 	
	*/
	
	TTC = (-1.0) /(frameRate * (1 - medianDistRatio));
   
}

// Compute TTC based on Lidar Points
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    float sumCurr = 0;
    float sumPrev = 0; 
    double meanCurr, meanPrev, medianCurr, medianPrev;
    int totalCurr = 0;
    int totalPrev = 0; 
    vector<float> currDis;
    vector<float> prevDis;
    bool meanMtd = false;

    for(auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it){
		prevDis.push_back(it->x);
		
	}

    for(auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it){
		currDis.push_back(it->x);
		
	}

  

    sort(prevDis.begin(), prevDis.end());
    sort(currDis.begin(), currDis.end());

    for (int i = floor(prevDis.size()/4); i <= prevDis.size()-floor(prevDis.size()/4); i++){
	sumPrev += prevDis[i];
	totalPrev += 1;
    }
    for (int i = floor(currDis.size()/4); i <= currDis.size()-floor(currDis.size()/4); i++){
	sumCurr += currDis[i];
	totalCurr += 1;
    }

    meanPrev = sumPrev/totalPrev;	
    meanCurr = sumCurr/totalCurr;

    long medIndex = floor(currDis.size() / 2.0);
    medianCurr = currDis.size() % 2 == 0 ? (currDis[medIndex - 1] + currDis[medIndex]) / 2.0 : currDis[medIndex];

    medIndex = floor(prevDis.size() / 2.0);
    medianPrev = prevDis.size() % 2 == 0 ? (prevDis[medIndex - 1] + prevDis[medIndex]) / 2.0 : prevDis[medIndex];
  

    if (meanMtd == true){TTC = meanCurr / (frameRate * (meanPrev - meanCurr));}
    else {TTC = medianCurr / (frameRate * (medianPrev - medianCurr));}
}

int rowMaxSearch(std::vector<std::vector<int>> &matrix, int row, int colSize){
	
	int max = 0;
	int ele;
	int index = 0;

	for (int i = 0; i < colSize; i++){
		ele = matrix[row][i];

		if (ele > max){
			max = ele;
			index = i;
		}
	}
	
	return index;
} 

int colMaxSearch(std::vector<std::vector<int>> &matrix, int col, int rowSize){
	
	int max = 0;
	int ele;
	int index = 0;

	for (int i = 0; i < rowSize; i++){
		ele = matrix[i][col];

		if (ele > max){
			max = ele;
			index = i;
		}
	}
	
	return index;
} 


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
	

	

	cv::KeyPoint prevKey, currKey; 	
	int currNoObj = currFrame.boundingBoxes.size();
	int prevNoObj = prevFrame.boundingBoxes.size(); 
	vector<vector<int>> relate (currNoObj, vector<int>(prevNoObj, 0));
	int box1ID, box2ID;
	int currObjID, prevObjID;

	for (auto it = matches.begin(); it != matches.end(); ++it){
		currKey = currFrame.keypoints[(*it).trainIdx];
		prevKey = prevFrame.keypoints[(*it).queryIdx];

		vector<int> box1;
		vector<int> box2;
		vector<int> currObjYOLOIds;
		vector<int> prevObjYOLOIds;

		for (auto it1 = currFrame.boundingBoxes.begin(); it1 != currFrame.boundingBoxes.end(); ++it1){

			float shrinkFactor = 0.20;
			cv::Rect smallerBox;
            		smallerBox.x = (*it1).roi.x + shrinkFactor * (*it1).roi.width / 2.0;
            		smallerBox.y = (*it1).roi.y + shrinkFactor * (*it1).roi.height / 2.0;
            		smallerBox.width = (*it1).roi.width * (1 - shrinkFactor);
            		smallerBox.height = (*it1).roi.height * (1 - shrinkFactor);

			if (smallerBox.contains(currKey.pt)){
				box1.push_back((*it1).boxID);
				currObjYOLOIds.push_back((*it1).classID);
			
			}
		}
		if (box1.size() == 1){
			box1ID = box1[0];
			currObjID = currObjYOLOIds[0];	
		}
		else {
			
			continue;
		}

		for (auto it2 = prevFrame.boundingBoxes.begin(); it2 != prevFrame.boundingBoxes.end(); ++it2){
			
			float shrinkFactor = 0.20;
			cv::Rect smallerBox;
            		smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            		smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            		smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            		smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

			if (smallerBox.contains(prevKey.pt)){
				box2.push_back((*it2).boxID);
				prevObjYOLOIds.push_back((*it2).classID);	
			}
		}
		if (box2.size() ==1){
			box2ID = box2[0];
			prevObjID = prevObjYOLOIds[0];
		}
		else {
			
			continue;	
		}
	
		relate[box1ID][box2ID] += 1;
	}
	
	if (currNoObj < prevNoObj){
		for (int row =  0; row < currNoObj; row++){
			int colIndex = rowMaxSearch(relate, row, prevNoObj); 
			if (relate[row][colIndex] == 0){continue;}			
			else {bbBestMatches.insert({row, colIndex});}

		}
	}
	else {
		for (int col = 0; col < prevNoObj; col++){
			int rowIndex = colMaxSearch(relate, col, currNoObj);
			if (relate[rowIndex][col] == 0){continue;}
			else {bbBestMatches.insert({rowIndex, col});}

		}
	}

 
}





























