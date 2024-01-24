# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## FP.1: Match 3D objects

The objective of this function is to track objects in consecutive frames. TO achieve it, the following steps are taken: 
1. A matrix, named "relate", is declared with zero as its initial values. The matrix's row indicates the BoxID of the objects in the current frame and the column indicates the BoxID of the objects in the previous frame. 
2. Loop through each matching keypoint pairs. When the key points in a pair lie only in one bounding box in the current and previous frame respectively, the respective cell value of the "relate" matrix is increased by 1. 
3. After going through each of the keypoint pairs, the "relate" matrix is analyzed. Depending upon the maximum value, rows are matched with respective columns (if row size < column size) or columns are matched with respective rows (if column size < row size). 

## FP.2: Compute Lidar-based TTC

#### Math behind Time-to-Collision (TTC)
* CAS-equipped vehicle is using a Lidar sensor to take distance measurements on preceding vehicles. The sensor in this scenario will give us the distance to the closest 3D point in the path of driving
	
<img src="images/llt-1.png" width="779" height="414" />

* Constant-Velocity Model
<img src="images/llt-2.png" width="779" height="414" />

* Lidar Point Cloud
  * The following image shows a Lidar point cloud as an overlay over a camera image taken in a highway scenario with a preceding vehicle directly in the path of driving. Distance to the sensor is color-coded (green is far away, red is close). Lidar points are shown in a top-view perspective and as an image overlay.
<img src="images/llt-3.png" width="779" height="414" />
  * All the point on the road are filterd out. 
<img src="images/llt-4.png" width="779" height="414" /> 

* To calculate TTC, only ego lane is considered. To calculate TTC, we fist need the distance to the preceding vehicle. From all the lidar points on the preceding vehicle mean distance or median distance is calculated. 
  * Median distance is preferred over the mean distance, as it helps to eliminate the effect of outliers in Lidar point data. Lidar points from the preceding vehicle are not accurate. They contain some outliers from the surrounding road. Because of this, the accuracy of the mean distance is hindered.
<img src="images/llt-5.png" width="779" height="414" /> 

## FP.3: Associate Keypoints Correspoindences with Bounding Boxes. 
* It is simply done by checking whether the corresponding keypoints are within the region of interest in the camera image. All matches which satisfy this condition are added to the Bounding box keypoint vector. 
* To eliminate the outliers, the robust mean of all the euclidean distances between keypoint matches is calculated, and then those points that are 1.75 times far from the mean are removed. 
* All the points which are considered, their corresponding keypoint matches are added to the Bounding box keypoint matches vector. 

## FP.4: Compute Camera-based TTC

#### Math behind Time-to-Collision (TTC)

* Constant-Velocity Model is considered. In the following figure, you can see how the height "H" of the preceding vehicle can be mapped onto the image place using perspective projection. We can see that the same height "H" maps to different heights "h0" and "h1" in the image plane, depending on the distance "d0" and "d1" of the vehicle. It is obvious that there is a geometric relation between "h", "H" and "d". 
  * "f" is focal length. 
<img src="images/ZCTT.png" width="779" height="414" /> 
<img src="images/ZCTT-1.png" width="779" height="414" /> 

* The only problem with this technique is that the height of the car can't be detected accurately. To overcome this problem, uniquely identifiable keypoints on preceding car are located. This keypoints can be easily tracked  from one frame to the next, we could use the distance between all keypoints on the vehicle relative to each other to compute a robust estimate of the height ratio in out TTC equation. The following figure illustrates the concept.
<img src="images/ZCTT-2.png" width="779" height="414" />

In (a), a set of keypoints has been detected and the relative distances between keypoints 1-7 have been computed. In (b), 4 keypoints have been matched between successive images (with keypoint 3 being a mismatch) using a higher-dimensional similarity measure called descriptor (more about that in the next lesson). The ratio of all relative distances between each other can be used to compute a reliable TTC estimate by replacing the height ratio h1/h0 with the mean or median of all distance ratios d0/d1.

* Median distance ratio is prefered over the mean distance ratio. Imagine a set of associated keypoint between two successive frames which contain a large number of mismatches. Computing the mean distance ratio would presumably lead to a faulty calculation of the TTC. A more robust way of computing the average of a dataset with outliers is to use the median instead.

## FP.5: Performance Evaluation 1 

* Comparsion between Lidar-based TTC and manually calculated TTC. 
	
<img src="images/FP5.PNG" width="779" height="414" /> 

  * Lidar-based TTC is roughly accurate over all the examples. As I have considered Median distance (as explained in FP.2), most of the errors due to outliers are estimated. 
  * There are large differences between Lidar-based TTC and manually calculated TTC in examples 11, 12, 16, 17, and 20. For calculating "Manually Calculated TTC", the minimum distance obtained from "show3DObjects()" is used. As the data can possibly have outliers, points on the road preceding to the preceding car, "Manually Calculated TTC" is not accurate.
  
## FP. 6 Performance Evaluation 2

* Project is run on different combination of dectector/descriptor. 
  * Detector types: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE and SIFT.
  * Discriptor types: BRISK, BRIEF, ORB, FREAK, AKAZE and SIFT. 
  
<img src="images/FP6-AKAZE_AKAZE.PNG" width="779" height="414" /> 
 
<img src="images/FP6-BRISK_BRIEF.PNG" width="779" height="414" /> 
<img src="images/FP6-BRISK_BRISK.PNG" width="779" height="414" /> 
<img src="images/FP6-BRISK_ORB.PNG" width="779" height="414" /> 
<img src="images/FP6-BRISK_FREAK.PNG" width="779" height="414" /> 
<img src="images/FP6-BRISK_SIFT.PNG" width="779" height="414" /> 
 
<img src="images/FP6-FAST_BRIEF.PNG" width="779" height="414" /> 
<img src="images/FP6-FAST_BRISK.PNG" width="779" height="414" />
<img src="images/FP6-FAST_ORB.PNG" width="779" height="414" />
<img src="images/FP6-FAST_FREAK.PNG" width="779" height="414" />
<img src="images/FP6-FAST_SIFT.PNG" width="779" height="414" />

<img src="images/FP6-HARRIS_BRIEF.PNG" width="779" height="414" />
<img src="images/FP6-HARRIS_BRISK.PNG" width="779" height="414" />
<img src="images/FP6-HARRIS_ORB.PNG" width="779" height="414" />
<img src="images/FP6-HARRIS_FREAK.PNG" width="779" height="414" />
<img src="images/FP6-HARRIS_SIFT.PNG" width="779" height="414" />

<img src="images/FP6-ORB_BRIEF.PNG" width="779" height="414" />
<img src="images/FP6-ORB_BRISK.PNG" width="779" height="414" />
<img src="images/FP6-ORB_SIFT.PNG" width="779" height="414" />

<img src="images/FP6-SHITOMASI_BRIEF.PNG" width="779" height="414" />
<img src="images/FP6-SHITOMASI_BRISK.PNG" width="779" height="414" />
<img src="images/FP6-SHITOMASI_ORB.PNG" width="779" height="414" />
<img src="images/FP6-SHITOMASI_FREAK.PNG" width="779" height="414" />
<img src="images/FP6-SHITOMASI_SIFT.PNG" width="779" height="414" />

<img src="images/FP6-SIFT_BRIEF.PNG" width="779" height="414" />
<img src="images/FP6-SIFT_BRISK.PNG" width="779" height="414" />
<img src="images/FP6-SIFT_FREAK.PNG" width="779" height="414" />
<img src="images/FP6-SIFT_SIFT.PNG" width="779" height="414" />

* BEST dectector/descriptor combination: 
  * SIFT/FREAK -> Square Root Mean Difference = 1.98.
  * SIFT/SIFT -> Square Root Mean Difference = 2.16.
  * SIFT/BRIEF -> Square Root Mean Difference = 2.28.
  * SIFT/BRISK -> Square Root Mean Difference = 2.27.
  * AKAZE/AKAZE -> Square Root Mean Difference = 2.43.
  * SHITOMASHI -> Square Root Mean Difference = 2.61.
  
  
  

 
 
  

  
