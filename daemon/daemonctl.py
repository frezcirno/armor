#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import pickle
import os
import re
import time

VERSION = '0.0.1'

_log_time = int(time.time())

# 配置列表
config_list = []

# 配置文件路径
config_file = os.path.join(os.path.dirname(__file__), "config.pkl")


def save(filename_, obj_):
    with open(filename_, 'wb') as f:
        pickle.dump(obj_, f)


def load(filename_):
    with open(filename_, 'rb') as f:
        return pickle.load(f)


def print_error(*content_):
    print("\033[31m", end='')
    print(*content_, "\033[0m")


def print_info(*content_):
    print("\033[32m", end='')
    print(*content_, "\033[0m")


def print_warn(*content_):
    print("\033[33m", end='')
    print(*content_, "\033[0m")


cwd = os.path.dirname(__file__)

config_template = {
    'daemon': '%s/../build/attack' % (cwd),
    'name': 'attack',
    'build_dir': '%s/../build/' % (cwd),
    'run': 'cd %s/../build/ && ./attack' % (cwd),
    'build': 'cd %s/../build/ && make -j7' % (cwd),
    'rebuild': 'cd %s/../build/ && rm -rf ./* && cmake .. && make' % (cwd),
}


def update_config_dict(_id, daemon_path_):
    _res = os.path.split(daemon_path_)
    config_list[_id]['daemon'] = daemon_path_
    config_list[_id]['name'] = _res[1]
    dir_name = _res[0]
    config_list[_id]['build_dir'] = '%s' % dir_name
    config_list[_id]['run'] = 'cd %s && ./%s' % (dir_name, _res[1])
    config_list[_id]['build'] = 'cd %s && make -j7' % dir_name
    config_list[_id]['rebuild'] = 'cd %s && rm -rf ./* && cmake .. && make -j7' % dir_name


def find_pid(name):
    """
    查找pid
    :param name:
    :return:
    """
    res = os.popen("ps -aux |grep %s |grep -v grep" % name).read()
    res = res.split("\n")
    pids = []
    for i in res[:-1]:
        u = re.findall(r"\s[0-9]+\s", i)
        if len(u) > 0:
            pids.append(u[0])
    return pids


def status():
    """
    查看状态
    :return:
    """
    res = check_config()
    pid_result = []
    for i in range(len(config_list)):
        print_info("[status %d]" % i)

        _daemon_pid = find_pid(r"'python3.*daemon.py' | grep %s | grep -v sh" % config_list[i]['daemon'])
        if len(_daemon_pid) == 0:
            print_warn("    daemon.py: not running")
        else:
            print_info("    daemon.py:", *_daemon_pid)
            pid_result.extend(_daemon_pid)
        if res[i]:
            _run_pid = find_pid(r"%s | grep -v sh | grep -v python3" % config_list[i]["name"])
            if len(_run_pid) == 0:
                print_warn("    %s: not running" % config_list[i]["daemon"])
            else:
                print_info("    %s:" % config_list[i]["daemon"], *_run_pid)
                pid_result.extend(_run_pid)
        else:
            print_error("    %s: invalid" % config_list[i]["daemon"])
    print()
    return pid_result


def check_config():
    res = []
    for i in range(len(config_list)):
        if not os.path.isfile(config_list[i]["daemon"]):
            print_error("ERROR: daemon path \'%s\' invalid" % config_list[i]["daemon"])
            res.append(False)
        else:
            res.append(True)
    return res


def config():
    for i in range(len(config_list)):
        print_info("[config_list %d]" % i)
        print_index = ['name', 'build_dir', 'run', 'build', 'rebuild']
        if not os.path.isfile(config_list[i]["daemon"]):
            print_error("%12s: %s (invalid)" % ("daemon", config_list[i]["daemon"]))
        else:
            print_info("%12s: %s" % ("daemon", config_list[i]["daemon"]))
        for key in print_index:
            print("%12s: %s" % (key, config_list[i][key]))
        print()
    print()


def set_daemon():
    """
    设置守护对象
    :return:
    """
    if args.path is None:
        print_error("[daemonctl] Use \'set -p <path> -i <index>\' to set daemon")
    else:
        if args.index is None:
            print_error("[daemonctl] Use \'set -p <path> -i <index>\' to set daemon")
        else:
            update_config_dict(args.index, args.path)
            print_info("Set complete")
            print()
            config()
            save(config_file, config_list)


def reset_daemon():
    global config_list
    try:
        os.remove(config_file)
    except:
        pass


def run():
    if args.index is None:
        print_error("[daemonctl] Use \'-i <index>\' to set index")
    else:
        if check_config()[args.index]:
            print(os.system("%s" % config_list[args.index]['build']))
            print(config_list[args.index]['run'])
            print()
            os.system("%s" % config_list[args.index]['build'])
            os.system("%s" % config_list[args.index]['run'])
            print()


def build():
    if args.index is None:
        print_error("[daemonctl] Use \'-i <index>\' to set index")
    else:
        if check_config()[args.index]:
            print(config_list[args.index]['build'])
            print()
            os.system("%s" % config_list[args.index]['build'])
            print()


def rebuild():
    if args.index is None:
        print_error("[daemonctl] Use \'-i <index>\' to set index")
    else:
        if check_config()[args.index]:
            print(config_list[args.index]['rebuild'])
            print()
            os.system("%s" % config_list[args.index]['rebuild'])
            print()


def kill_all():
    print_info("Before kill")
    pids = status()
    for _pid in pids:
        os.system("sudo kill -9 %s" % _pid)
    time.sleep(0.3)
    print_info("After kill")
    status()


def start():
    kill_all()
    _name = "nohup_%d" % _log_time 
    _nameerr = "nohuperr_%d" % _log_time 
    res = check_config()
    if args.all and args.nohup:
        for i in range(len(config_list)):
            cmd = "python3 %s %s" % (
                os.path.join(os.path.dirname(__file__), "daemon.py"), config_list[i]['daemon'])
            print_info(cmd)
            print_info()
            os.system("nohup `%s 1>\"/tmp/%s_%d.log\" 2>\"/tmp/%s_%d.log\"` & " % (cmd,_name,i,_nameerr,i) )
        return
    if res[args.index]:
        cmd = "python3 %s %s" % (
            os.path.join(os.path.dirname(__file__), "daemon.py"), config_list[args.index]['daemon'])
        print_info(cmd)
        print_info()
        if args.nohup:
            os.system("nohup `%s 1>\"/tmp/%s.log\" 2>\"/tmp/%s.log\"` &" % (cmd,_name,_nameerr))
        else:
            os.system(cmd)


def ps():
    for _name in ["daemon.py", *[_config_dict['name'] for _config_dict in config_list]]:
        cmd = "ps -aux |grep %s | grep -v grep" % _name
        print(cmd)
        os.system(cmd)
        print()
    print()


def print_help_info():
    print()
    print("SuperPower 2019 command tool")
    index = ['config_list', 'set', 'reset', 'find', 'status', 'ps', 'run', 'build', 'rebuild', 'start', 'killall', 'draw',
             'help']
    for key in index:
        print("%10s: %s" % (key, help_info[key]))
    print("""
examples:
<> set -p /home/sp/attack-small/build/attack
<> add -p /home/sp/attack-small/build/attack
<> set -p /home/sp/attack-small/build/attack -i 0
<> remove -i 1
<> ps
<> status -i 1
<> start --nohup --all
<> start --nohup -i 0
<> run
<> rebuild
""")
    print()


def add():
    if args.path is None:
        print_error("[daemonctl] Use \'set -p <path>\' to set daemon")
    else:
        config_list.append(config_template)
        update_config_dict(len(config_list) - 1, args.path)
        print_info("Set config_list %d complete" % (len(config_list) - 1))
        print()
        config()
        save(config_file, config_list)


def remove():
    config_list.pop(args.index)
    print_info("config_list %d has been removed" % args.index)
    print()
    config()
    save(config_file, config_list)


def draw():
    res = check_config()
    if res[args.index]:
        print("[draw %d]" % args.index)
        print()
        os.system(
            "%s %s" % (os.path.join(os.path.dirname(__file__), "show_curve.py"), config_list[args.index]['build_dir']))


if __name__ == '__main__':
    cmd_dict = {
        'killall': kill_all,
        'run': run,
        'build': build,
        'rebuild': rebuild,
        'find': status,
        'status': status,
        'config_list': config_list,
        'set': set_daemon,
        'reset': reset_daemon,
        'start': start,
        'ps': ps,
        'help': print_help_info,
        'add': add,
        'remove': remove,
        'draw': draw,
    }

    help_info = {
        'killall': '杀死守护进程和守护对象 / kill both daemon.py and attack',
        'run': '运行守护对象 / run attack',
        'build': '编译守护对象 / build attack',
        'rebuild': '重编译守护对象 / rebuild attack',
        'find': '查找守护进程和守护对象的pid/ find pids of daemon.py and attack',
        'status': '同 `find` / same as find ',
        'config_list': '打印当前配置信息 / print current configuration',
        'set': '-p <path> 设置守护对象绝对路径 / set what will be daemon',
        'reset': '恢复默认配置 / reset what will be daemon',
        'start': '开启守护进程; --nohup 设置后台开启; --all 设置运行所有配置',
        'ps': '使用`ps`查找 pid / run \'ps\' command',
        'help': '打印帮助信息 / print this help info',
        'draw': '绘制守护对象的弹道 / show shoot curve',
        'add': '增加一个配置; -p <path> 指定守护对象绝对路径',
        'remove': '删除一个配置; -i <int> 指定删除哪个配置, 默认为0',
        '-i': '-i <int> 指定作用于哪个配置, 默认为0',
    }

    if not os.path.exists(config_file):
        print_warn("[daemonctl] Config file not found. A default file is created.")
        save(config_file, [config_template])
        
    config_list = load(config_file)

    parser = argparse.ArgumentParser(description='SuperPower 2019 command tool', add_help=False)

    parser.add_argument("command", metavar="<command>", help="", choices=[key for key in cmd_dict])
    parser.add_argument('-p', '--path', metavar="<path>", help="path to program which will be daemon")
    parser.add_argument('--nohup', help="is with \'nohup\'", action='store_true')
    parser.add_argument('-i', '--index', default=0, type=int)
    parser.add_argument('--all', action='store_true')
    args = parser.parse_args()
    cmd_dict[args.command]()
