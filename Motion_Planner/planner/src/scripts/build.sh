source /opt/ros/noetic/setup.bash

DIR="$( cd "$( dirname "$0" )" && pwd )"
PROJECT_DIR=$DIR/..

mkdir $PROJECT_DIR/build
cd $PROJECT_DIR/build || exit

cmake -DCMAKE_INSTALL_PREFIX=/home/mikhail/install/pytorch-motion-planner/ ..
cmake --build . --target all -- -j8
#make install
