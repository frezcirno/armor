# 守护进程

本目录下为守护进程文件，守护进程用于保证在系统启动/主进程意外崩溃的时候自动启动主进程（执行识别、自瞄等的进程）

目前有两个版本的守护进程实现：

## 1. 2020版

大概是前人写的，包括如下文件：

| 文件            | 说明               | 备注                               |
|-----------------|------------------|------------------------------------|
| daemon.py       | 守护进程的一部分   |                                    |
| daemonctl.py    | 守护进程的控制程序 | 主要命令有start, killall, status等 |
| utils.py        | 空文件             |                                    |
| show_curve.py   | 不清楚             | 似乎是画图用的                     |
| setup_daemon.sh | 守护进程配置脚本   | 见下方                             |

- setup_daemon.sh 脚本用于配置守护进程，主要功能是

    1. 将自启命令添加到~/.profile文件中，以实现每次开机自动启动主进程

    2. 设置daemonctl.py的alias为"f*ck"，方便输入命令

## 2021新版

因为旧的守护进程不太优雅，21年改用更加强大的supervisor，包括如下文件

| 文件             | 说明               | 备注   |
|------------------|------------------|------|
| supervisord.conf | supervisor配置文件 | 见下方 |

supervisor的使用方法可以自行了解

- 配置

    1. 安装supervisor
    2. 根据实际情况修改supervisord.conf中的各项参数，然后将supervisord.conf拷贝到/etc/supervisor/supervisord.conf
    4. 重启supervisor

- 使用

    1. 启动主进程：`sudo supervisorctl start attack`
    2. 重启主进程：`sudo supervisorctl restart attack`
    3. 关闭主进程：`sudo supervisorctl stop attack`
    4. 查看主进程状态：`sudo supervisorctl status attack`