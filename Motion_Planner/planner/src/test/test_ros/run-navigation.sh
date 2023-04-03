#bash build.sh

INSTALL_DIR=/home/mikhail/install

source /opt/ros/noetic/setup.bash
source ${INSTALL_DIR}/hermesbot_simulation/setup.bash --extend
source ${INSTALL_DIR}/hermesbot_collision_checker/setup.bash --extend
source ${INSTALL_DIR}/hermesbot_navigation/setup.bash --extend
source ${INSTALL_DIR}/pytorch-motion-planner/setup.bash --extend

PYTHONPATH=/home/mikhail/research/pytorch-motion-planner/:$PYTHONPATH

roslaunch run_pytorch_navigation.launch vis:=true\
  visualization_config_path:=/home/mikhail/research/pytorch-motion-planner/launch/navigation_test.rviz
