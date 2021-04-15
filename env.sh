#!/bin/bash
#
# 自瞄代码环境配置工具
# 需要 Ubuntu 16.04，18.04 或 20.04 环境
# 有待进一步优化
#
# @author: frezcirno
#

START_DIR=`dirname $0`
START_DIR=`cd $START_DIR; pwd`
USER_ID=`id -u`

if [[ "$USER_ID" != "0" ]]; then
	echo "ERROR: You need to run the script as superuser (root account)."
	exit 1
fi

# echo -n "是否要更换 Ubuntu 镜像源至 mirrors.tuna.tsinghua.edu.cn [Y/n]? "
# read ANSWER
# if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
# sed -e "s/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g" \
# -e "s/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g" \
# -e "s/cn.mirrors.tuna.tsinghua.edu.cn/mirrors.tuna.tsinghua.edu.cn/g" \
# -i /etc/apt/sources.list
# apt update
# fi

echo "Installing packages......."
apt install -y build-essential cmake git vim libgtk2.0-dev libboost-dev libboost-thread-dev libusb-1.0-0-dev catkin lsb linux-headers-generic libdlib-dev libopencv-dev
snap install code --classic

# echo ""
# echo -n "是否要安装相机SDK [Y/n]? "
# read ANSWER
# if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
#     if [[ ! -f /tmp/sdk ]]; then
#         echo "Downloading Camera SDK......."
#         pushd "${START_DIR}" >>/dev/null
#         tar -zxf sdk.tgz -C ~
#         popd >>/dev/null
#     fi
#     echo "Installing Camera SDK......."
#     pushd /tmp/sdk >>/dev/null
#     chmod +x dahua/MVviewer_Ver2.2.5_Linux_x86_Build20200910.run
#     echo "yes\nyes\nyes\n" | ./dahua/MVviewer_Ver2.2.5_Linux_x86_Build20200910.run --nox11
#     mv mindvision/linuxSDK_V2.1.0.12 /opt/mindvision
#     popd >>/dev/null
# fi

echo ""
echo -n "是否要安装 Serial 库（需要访问 Github） [Y/n]? "
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
    echo "Downloading serial......."
    rm -rf /tmp/serial
    git clone --depth 1 https://github.com/wjwwood/serial.git /tmp/serial

    echo "Installing serial......."
    mkdir -p /tmp/serial/build
    pushd /tmp/serial/build >>/dev/null
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make install
    popd >>/dev/null
fi

echo "OK"
