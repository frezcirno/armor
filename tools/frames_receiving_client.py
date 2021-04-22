import socket
import cv2 as cv
import argparse
import numpy as np

# ref to `struct`: https://docs.python.org/3/library/struct.html
import struct

# data encoding format for communication through socket
# DATA: { [HEADER(fixed-sized)] [•••FRAME•••(size specified by HEADER)] }

# buffer size for the socket
_buffer_size = 4096
# format for packing (helps specify header size)
_packing_format = '!L'
# default server if not specified
_default_server = socket.gethostbyname(socket.gethostname())
# default port if not specified
_default_port = 8080
# message of the request to receive a frame
_request_message = '!FrameRequest'
# message for disconnection
_disconnection_message = '!Disconnect'
_encoding = 'utf-8'

def parsed_args():
    parser = argparse.ArgumentParser(description='Used to specify network parameters (e.g. port)')
    parser.add_argument('--server', nargs='?', help='the IPv4 address for the server', type=str)
    parser.add_argument('--port', nargs='?', help='the port to connect', type=int)
    args = parser.parse_args()

    # default settings values not provided
    if args.server is None:
        args.server = _default_server
    if args.port is None:
        args.port = _default_port

    return args


def main():
    args = parsed_args()

    # connect the server using socket
    server_address = (args.server, args.port)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(server_address)

    while True:
        client_socket.send(_request_message.encode(_encoding))

        # STEP1: decode the header
        # get the size of the header
        header_size = struct.calcsize(_packing_format)

        # receive the header in the packed form
        packed_header = b''
        while len(packed_header) < header_size:
            packed_header += client_socket.recv(_buffer_size)
        packed_header = packed_header[:header_size]
        
        # STEP2: decode the frame data, which is an opencv image to be displayed
        # unpack the header to get the image frame size
        # refer to the comment at the starting lines if you're puzzled
        frame_size = struct.unpack(_packing_format, packed_header)[0]
        
        # receive the frame data
        packed_frame = b''
        while len(packed_frame) < frame_size:
            packed_frame += client_socket.recv(_buffer_size)
        packed_frame = packed_frame[:frame_size]

        # decode the image from bytes
        frame = cv.imdecode(np.frombuffer(packed_frame), cv.IMREAD_COLOR)

        # show the image
        cv.imshow('RoboMaster', frame)
        cv.waitKey(1)

if __name__ == '__main__':
    main()
    # address = ('localhost', 8880)

    # server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # server.bind(address)
    # print('Bind UDP on 8880...')

    # # 按照格式打包发送帧数和分辨率
    # while True:
    #     data, addr = server.recvfrom(4)
    #     print('Received from %s:%s.' % addr)
    #     size = int.from_bytes(data, sys.byteorder)
    #     print(f'size {size}')
    #     if size:
    #         try:
    #             buf = b""  # 代表bytes类型
    #             while size:  # 读取每一张图片的长度
    #                 data, addr = server.recvfrom(4096)
    #                 size -= len(data)
    #                 buf += data  # 获取图片
    #                 print(f'read {len(data)}')
    #             mat = numpy.fromstring(buf, dtype='uint8')
    #             image = cv.imdecode(mat, 1)  # 图像解码
    #             cv.imshow('image', image)  # 展示图片
    #         except:
    #             pass
    #         finally:
    #             if(cv.waitKey(0) == 27):  # 每10ms刷新一次图片，按‘ESC’（27）退出
    #                 server.close()
    #                 cv.destroyAllWindows()
    #                 break
