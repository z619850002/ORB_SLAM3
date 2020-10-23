echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j

cd ../../../../


cd Examples/ROS/ORB_VIO
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
