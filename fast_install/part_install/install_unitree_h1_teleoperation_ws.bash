start_path=$(pwd)
cd ../../../..
mkdir -p unitree_h1_teleoperation_ws/src
cd unitree_h1_teleoperation_ws/src
git clone https://github.com/cyberbanana777/unitree_h1_teleoperation_ws.git .

chmod +x install_dependensies.bash
./install_dependensies.bash

cd ..
colcon build
source install/setup.bash

line_to_add="source \"$(pwd)/install/setup.bash\""
grep -qxF "$line_to_add" ~/.bashrc || echo "$line_to_add" >> ~/.bashrc

cd $start_path
unset start_path
