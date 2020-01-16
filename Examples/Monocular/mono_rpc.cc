/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"
#include <dirent.h>
#include <sys/time.h>
#include <time.h>

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

double tic()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double)t.tv_sec + ((double)t.tv_usec) / 1000000.);
}

/// read image name
int getAbsoluteFiles(string directory, vector<string> &filesAbsolutePath) //参数1[in]要变量的目录  参数2[out]存储文件名
{
    DIR *dir = opendir(directory.c_str()); //打开目录   DIR-->类似目录句柄的东西
    if (dir == NULL)
    {
        cout << directory << " is not a directory or not exist!" << endl;
        return -1;
    }

    struct dirent *d_ent = NULL; //dirent-->会存储文件的各种属性
    char fullpath[128] = {0};
    char dot[3] = "."; //linux每个下面都有一个 .  和 ..  要把这两个都去掉
    char dotdot[6] = "..";

    while ((d_ent = readdir(dir)) != NULL) //一行一行的读目录下的东西,这个东西的属性放到dirent的变量中
    {
        if ((strcmp(d_ent->d_name, dot) != 0) && (strcmp(d_ent->d_name, dotdot) != 0)) //忽略 . 和 ..
        {
            if (d_ent->d_type == DT_DIR) //d_type可以看到当前的东西的类型,DT_DIR代表当前都到的是目录,在usr/include/dirent.h中定义的
            {

                string newDirectory = directory + string("/") + string(d_ent->d_name); //d_name中存储了子目录的名字
                if (directory[directory.length() - 1] == '/')
                {
                    newDirectory = directory + string(d_ent->d_name);
                }

                if (-1 == getAbsoluteFiles(newDirectory, filesAbsolutePath)) //递归子目录
                {
                    return -1;
                }
            }
            else //如果不是目录
            {
                string absolutePath = directory + string("/") + string(d_ent->d_name); //构建绝对路径
                if (directory[directory.length() - 1] == '/')                          //如果传入的目录最后是/--> 例如a/b/  那么后面直接链接文件名
                {
                    absolutePath = directory + string(d_ent->d_name); // /a/b/1.txt
                }
                filesAbsolutePath.push_back(absolutePath);
            }
        }
    }

    closedir(dir);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./mono_rpc path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    cout << "Creating orb-slam system ..." << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Main loop
    double startTime = tic();
    double imageLastTime = startTime;
    double timestamp = 0.0;
    cv::Mat gSafeImage;

    // image names
    vector<string> lvstrImageFilenames;
    string images_folder = argv[3];
    getAbsoluteFiles(images_folder, lvstrImageFilenames);
    sort(lvstrImageFilenames.begin(), lvstrImageFilenames.end());
    int nImages = lvstrImageFilenames.size();

    for (int ni = 0; ni < nImages; ni++)
    {
        cout << lvstrImageFilenames[ni] << endl;
        // Read image from file
        gSafeImage = cv::imread(lvstrImageFilenames[ni]);
        if (gSafeImage.empty())
        {
            cerr << "image empty!" << endl;
            return 1;
        }
        /// cut the lower part of the image
        gSafeImage = gSafeImage(cv::Rect(0, 0, 640, 350));

        /// add one white box to cover the sky area in the image
        int height = gSafeImage.rows;
        int width = gSafeImage.cols;
        int channels = gSafeImage.channels();
        for (int i = 0; i < 130; i++)
        {
            for (int j = 0; j < width; j++)
            {
                gSafeImage.at<cv::Vec3b>(i, j)[0] = 255;
                gSafeImage.at<cv::Vec3b>(i, j)[1] = 255;
                gSafeImage.at<cv::Vec3b>(i, j)[2] = 255;
            }
        }

        /// update timestamp
        timestamp += 0.03;

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(gSafeImage, timestamp);

        // sleep
        usleep(30000);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
