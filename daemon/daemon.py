#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import os
import sys

if __name__ == "__main__":
    log_path = os.path.join(os.path.dirname(__file__), "log.txt")

    while True:
        _res = os.path.split(sys.argv[1])
        cmd = 'cd %s && ./%s' % (_res[0], _res[1])
        os.system(cmd)
        print("[daemon] terminate")
        try:
            with open(log_path, 'a+') as f:
                f.writelines(time.strftime("%Y-%m-%d %H:%M:%S terminate\n", time.localtime()))
        except:
            pass
        time.sleep(0.1)
