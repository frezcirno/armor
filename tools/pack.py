#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import shutil
import os
import time

if os.path.exists("../release"):
    print("\033[32mexist release folder\033[0m")
else:
    print("\033[33mrelease folder not found. create new one.\033[0m")
    os.mkdir("../release")

if os.path.exists("../release/attack"):
    print("\033[31mattack folder under release! quit\033[0m")
    exit(-1)

current_time = time.localtime()
dst_path = time.strftime("../release/%Y-%m-%d+%H-%M-%S", current_time)
try:
    os.mkdir(dst_path)
    os.mkdir(dst_path + "/data")
    os.mkdir(dst_path + "/build")
    shutil.copytree("../src", dst_path + "/src")
    shutil.copytree("../test", dst_path + "/test")
    shutil.copytree("../inc", dst_path + "/inc")
    shutil.copytree("../tools", dst_path + "/tools")

    files = ['main.cpp', 'CMakeLists.txt', 'config.toml', 'README.md', 'run.sh']
    for _i in files:
        shutil.copyfile("../" + _i, dst_path + "/" + _i)

    for _file in os.listdir("../data"):
        if os.path.isfile("../data/" + _file):
            shutil.copyfile("../data/" + _file, dst_path + "/data/" + _file)

    os.mkdir(dst_path + "/data/raw")
    os.mkdir(dst_path + "/data/CalibrationImages")
    os.mkdir(dst_path + "/data/samples")
    os.mkdir(dst_path + "/data/samples/0")
    os.mkdir(dst_path + "/data/samples/1")
    os.mkdir(dst_path + "/data/video")

    with open(dst_path + "/data/video/auto", "w") as f:
        f.writelines("0\n")

    new_CMakeLists = ''
    with open(dst_path + '/CMakeLists.txt', 'r+') as f:
        for line in f:
            if line.find("cmake_minimum_required") != -1:
                new_CMakeLists += "cmake_minimum_required(VERSION 3.5)\r\n"
            else:
                new_CMakeLists += line

    with open(dst_path + '/CMakeLists.txt', 'w') as f:
        f.writelines(new_CMakeLists)

    with open(dst_path + time.strftime("/%Y-%m-%d+%H-%M-%S", current_time), 'w') as f:
        pass

    print("\033[33mzipping ...\033[0m")
    dst_folder_name = time.strftime("%Y-%m-%d+%H-%M-%S", current_time)
    os.system("cd ../release && mv %s attack && zip -r %s-attack.zip attack && mv attack %s" % (
        dst_folder_name, dst_folder_name, dst_folder_name))
    print("\033[32mzip complete\033[0m")

    print("\033[32mall done\033[0m")

except Exception as e:
    print(e)
    if os.path.exists(dst_path):
        shutil.rmtree(dst_path)
    if os.path.exists(dst_path + '.zip'):
        os.remove(dst_path + '.zip')
    print("\033[31mSomething error\033[0m")
