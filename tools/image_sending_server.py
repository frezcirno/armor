import socket
import cv2 as cv
import argparse
import numpy as np
import threading

# ref to `struct`: https://docs.python.org/3/library/struct.html
import struct

# data encoding format for communication through socket
# DATA: { [HEADER(fixed-sized)] [•••MESSAGE•••(size specified by HEADER)] }

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

def handle_client(client_socket):
    camera = cv.VideoCapture(0)
    while True:
        # STEP1: decode the header
        # get the size of the header
        header_size = struct.calcsize(_packing_format)

        # receive the header in the packed form
        packed_header = b''
        while len(packed_header) < header_size:
            packed_header += client_socket.recv(_buffer_size)
            print(f'Received {len(packed_header)}B of HEADER')
        remaining_bytes = packed_header[header_size:]
        packed_header = packed_header[:header_size]

        # unpack the header to get the image frame size
        # refer to the comment at the starting lines if you're puzzled
        message_size = struct.unpack(_packing_format, packed_header)[0]
        print(f'Received HEADER: {message_size}')

        # STEP2: check the message
        packed_message = remaining_bytes
        while len(packed_message) < message_size:
            packed_message += client_socket.recv(_buffer_size)
            print(f'Received {len(packed_message)}B of HEADER')
        packed_message = packed_message[:message_size]
        # unpack to get the message
        message = packed_message.decode(_encoding)
        print(f"New message '{message}' received")

        # STEP3: perform different logics according to the message
        if message == _disconnection_message:
            break
        elif message == _frame_request_message:
            frame = camera.read()[1]
            packed_frame = cv.imencode('.jpg', frame)[1].tobytes()
            # the header which specifies the size of the frame
            packed_header = struct.pack(_packing_format, len(packed_frame))
            client_socket.send(packed_header + packed_frame)
            print(f'Image sent: {len(packed_header) + len(packed_frame)}B')


    client_socket.close()


def main():
    args = parsed_args()

    # connect the server using socket
    server_address = (args.server, args.port)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(server_address)

    server_socket.listen()
    print(f'Server {server_address} is running')

    while True:
        client_socket, address = server_socket.accept()
        print(f'New connection from {address}')
        
        thread = threading.Thread(target=handle_client, args=(client_socket,))
        thread.start()
        # '- 1' to exclude the server thread itself
        print(f'Number of active client threads: {threading.activeCount() - 1}')

if __name__ == '__main__':
    main()
