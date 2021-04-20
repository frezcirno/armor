import socket
import cv2
import sys
import threading
import struct
import numpy

if __name__ == '__main__':
    address = ('localhost', 8880)

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(address)
    print('Bind UDP on 8880...')

    # 按照格式打包发送帧数和分辨率
    while True:
        data, addr = server.recvfrom(4)
        print('Received from %s:%s.' % addr)
        size = int.from_bytes(data, sys.byteorder)
        print(f'size {size}')
        if size:
            try:
                buf = b""  # 代表bytes类型
                while size:  # 读取每一张图片的长度
                    data, addr = server.recvfrom(4096)
                    size -= len(data)
                    buf += data  # 获取图片
                    print(f'read {len(data)}')
                mat = numpy.fromstring(buf, dtype='uint8')
                image = cv2.imdecode(mat, 1)  # 图像解码
                cv2.imshow('image', image)  # 展示图片
            except:
                pass
            finally:
                if(cv2.waitKey(0) == 27):  # 每10ms刷新一次图片，按‘ESC’（27）退出
                    server.close()
                    cv2.destroyAllWindows()
                    break
