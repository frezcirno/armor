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
# message for frame request
_frame_request_message = '!FrameRequest'
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

def pack(string):
    bytes = string.encode(_encoding)
    return struct.pack(_packing_format, len(bytes)) + bytes

def main():
    args = parsed_args()

    # connect the server using socket
    server_address = (args.server, args.port)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(server_address)
    print(f'Successfully connected to {server_address}')

    while True:
        # send frame request to the server
        packed_frame_request = pack(_frame_request_message)
        client_socket.send(packed_frame_request)
        print(f'Frame request sent: {len(packed_frame_request)}B')

        # STEP1: decode the header
        # get the size of the header
        header_size = struct.calcsize(_packing_format)

        # receive the header in the packed form
        packed_header = b''
        while len(packed_header) < header_size:
            packed_header += client_socket.recv(_buffer_size)
            print(f'Received {len(packed_header)}B of HEADER')
        packed_header = packed_header[:header_size]
        
        # unpack the header to get the image frame size
        # refer to the comment at the starting lines if you're puzzled
        frame_size = struct.unpack(_packing_format, packed_header)[0]
        print(f'Received HEADER: {frame_size}')
        
        # STEP2: decode the frame data, which is an opencv image to be displayed

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
    
    # disconnect from the server
    packed_disconnection = pack(_disconnection_message)
    client_socket.send(packed_disconnection)
    print(f'Disconnection request sent: {len(packed_disconnection)}B')

if __name__ == '__main__':
    main()
