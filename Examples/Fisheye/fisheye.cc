/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;



cv::Mat mFisheyeK =  (cv::Mat_<double>(3 , 3) << 784.8073077255041, 0, 628.7589661896264,
                                                 0, 780.6384641050264, 514.8129402897023,
                                                 0, 0, 1);

cv::Mat mDistortion = (cv::Mat_<double>(4 , 1) << -0.0396157, 0.0634728, -0.112445, 0.0604191);



int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    cv::VideoCapture iCapture(0);
    iCapture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    iCapture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    if(!iCapture.isOpened()){
        cerr << "Failed to open camera" << endl;
    }

    // Main loop
    cv::Mat mImage;
    double nStartT = 0.0;
    for(;;)
    {
        iCapture >> mImage;
        
        // cout << "Image shape is: " << endl << mImage.size().width << " " << mImage.size().height << endl;
        // cv::fisheye::undistortImage(mImage, mImage, mFisheyeK, mDistortion, mFisheyeK);
        // cv::imshow("const cv::String &winname", mImage);
        // cv::waitKey(0);



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(mImage,nStartT);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();


        // Wait to load the next frame
        double T=0.03;

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
