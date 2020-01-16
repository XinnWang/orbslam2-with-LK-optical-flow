# orbslam2 with LK optical flow
 This project is modified from orbslam2. All dependencies are consistent with orbslam2, try to support monocular camera now.

This project：

  1、improve the speed of orbslam2
  
  2、use optical flow to track the 3D points of the previous key frame, and non-key frames use PnPransac (the projection of the tracked 3D point on the current frame) to calculate the camera pose
  
# how to install:
If you have installed all the dependencies of orbslam2 and want to run on rgbd dataset, modify "run.sh" based on your directory name.

Then run:

chmod +x build.sh

chmod +x run.sh

./build.sh

./run_rpc.sh