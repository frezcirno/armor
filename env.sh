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

echo -n "是否要更换 Ubuntu 镜像源至 mirrors.tuna.tsinghua.edu.cn [Y/n]? "
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
sed -e "s/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g" \
-e "s/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g" \
-e "s/cn.mirrors.tuna.tsinghua.edu.cn/mirrors.tuna.tsinghua.edu.cn/g" \
-i /etc/apt/sources.list
apt update
fi

echo "Installing packages......."
apt install -y build-essential cmake git libgtk2.0-dev libboost-dev libboost-thread-dev libusb-1.0-0-dev catkin lsb linux-headers-generic libdlib-dev

echo ""
echo -n "是否要安装相机SDK [Y/n]? "
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
    if [[ ! -f ~/sdk ]]; then
        echo "Downloading Camera SDK......."
        pushd "${START_DIR}" >>/dev/null
        tar -zxf sdk.tgz -C ~
        popd >>/dev/null
    fi
    echo "Installing Camera SDK......."
    pushd ~/sdk >>/dev/null
    chmod +x dahua/MVviewer_Ver2.2.5_Linux_x86_Build20200910.run
    echo "yes\nyes\nyes\n" | ./dahua/MVviewer_Ver2.2.5_Linux_x86_Build20200910.run --nox11
    mv mindvision/linuxSDK_V2.1.0.12 /opt/mindvision
    popd >>/dev/null
fi

echo ""
echo -n "是否要安装 Serial 库（需要访问 Github） [Y/n]? "
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
    if [[ ! -f ~/serial ]]; then
        echo "Downloading serial......."
        git clone --depth 1 https://github.com/wjwwood/serial.git ~/serial
    fi

    echo "Installing serial......."
    mkdir -p ~/serial/build
    pushd ~/serial/build >>/dev/null
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make install
    popd >>/dev/null
fi

echo ""
echo -n "是否要安装 OpenCV-4.5.1 库（需要访问 Github） [Y/n]? "
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
    if [[ ! -f ~/opencv ]]; then
        echo "Downloading opencv......."
        git clone --depth 1 --branch 4.5.1 https://github.com/opencv/opencv.git ~/opencv
    fi

    echo "Installing opencv......."
    mkdir -p ~/opencv/build
    pushd ~/opencv/build >>/dev/null
    cmake -j8 ..
    make install 
    popd
fi

echo ""
echo -n "是否要下载最新版自瞄代码（需要访问 Github） [Y/n]? "
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
    if [[ ! -d ~/armor ]]; then
        echo "Downloading armor code......."
        git clone --branch master https://github.com/frezcirno/armor ~/armor
    fi
    echo "Copying TensorFlow libraries......."
    pushd "${START_DIR}" >>/dev/null
    tar -zxf tensorflow.tgz -C ~/armor
    popd >>/dev/null
    echo "Building executable......."
    mkdir -p ~/armor/build
    pushd ~/armor/build >>/dev/null
    cmake .. && make
    popd >>/dev/null
    echo "Download Armor code with TensorFlow C++ lib .......OK"
    echo "Location: ~/armor"
fi

# 守护进程目前存在一些问题
# echo ""
# echo -n "Download Armor Daemon [Y/n]? "
# read ANSWER
# if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
#     if [[ ! -d ~/daemon ]]; then
#         echo "Downloading daemon......."
#         pushd "${START_DIR}" >>/dev/null
#         tar -zxf daemon.tgz -C ~
#         popd >>/dev/null
#     fi
#     echo "Configure daemon......."
#     sed -i -e "s/\/root\/daemon\/daemonctl.py\ start\ --nohup\ --all//g" -e "s/exit\ 0/\/root\/daemon\/daemonctl.py\ start\ --nohup\ --all\nexit\ 0/g" /etc/rc.local 
#     chmod +x ~/daemon/daemonctl.py
#     ~/daemon/daemonctl.py reset
# fi

echo "OK"