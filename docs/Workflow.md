

# PX4-autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl

# Setup Micro XRCE-DDS Agent & Client
MicroXRCEAgent udp4 -p 8888  Multi



# QGC













用pipx来安装
sudo apt-get install pipx

pipx install xxx
pipx ensurepath


### 安装必要的库
sudo apt install python3-kconfiglib


### 采用虚拟环境


sudo apt install python3-venv

python3 -m venv ~/xtd2_pyenv

source ~/xtd2_pyenv/bin/activate

make px4_sitl gz_x500

pip3 install kconfiglib
pip3 install jinja2
pip3 install pyyaml
pip3 install jsonschema
pip3 install empy==3.3.4
pip3 install pyros-genmsg
pip3 install future

pip3 install catkin_pkg
pip3 install lark

### 添加自己的模型
Therefore if you want to use your own model and run it in standalone mode, you will have to place its source code in ~/.simulation-gazebo.


### 多host通信




### ROS2 
Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.