ROOT=$(pwd)
cd ../../
rm -rf build/ install/ log/
colcon build
source install/setup.bash
cd $ROOT

