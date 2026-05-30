# =======================================================================================
#  Header
# =======================================================================================

import os
import ctypes
from pathlib import Path

# _base_path = str( Path(__file__).parent.resolve() )

UFR_OK = 0

UFR_START_BLANK=0
UFR_START_SERVER=1
UFR_START_CLIENT=3
UFR_START_PUBLISHER=4
UFR_START_SUBSCRIBER=5

# =======================================================================================
#  Class Link
# =======================================================================================

class Link(ctypes.Structure):
    dll = ctypes.CDLL(f"libufr.so")
    # dll.urf_sys_set_ld_path( bytes(_base_path, 'utf-8') );

    dll.ufr_close.argtypes = [ ctypes.c_void_p ]
    dll.ufr_close.restype = ctypes.c_int32

    dll.ufr_loop_ok.argtypes = [ ]
    dll.ufr_loop_ok.restype = ctypes.c_bool

    dll.ufr_recv_async.argtypes = [ ctypes.c_void_p ]
    dll.ufr_recv_async.restype = ctypes.c_int32

    # Meta
    dll.ufr_meta_item_nbytes.argtypes = [ ctypes.c_void_p ]
    dll.ufr_meta_item_nbytes.restype =  ctypes.c_int32

    # Get Scalar - 32bits
    dll.ufr_get_u32.argtypes = [ ctypes.c_void_p, ctypes.c_uint32 ]
    dll.ufr_get_u32.restype =  ctypes.c_uint32
    dll.ufr_get_i32.argtypes = [ ctypes.c_void_p, ctypes.c_int32 ]
    dll.ufr_get_i32.restype =  ctypes.c_int32
    dll.ufr_get_f32.argtypes = [ ctypes.c_void_p, ctypes.c_float ]
    dll.ufr_get_f32.restype =  ctypes.c_float

    # Get Scalar - 64bits
    dll.ufr_get_u64.argtypes = [ ctypes.c_void_p, ctypes.c_uint64 ]
    dll.ufr_get_u64.restype =  ctypes.c_uint64
    dll.ufr_get_i64.argtypes = [ ctypes.c_void_p, ctypes.c_int64 ]
    dll.ufr_get_i64.restype =  ctypes.c_int64
    dll.ufr_get_f64.argtypes = [ ctypes.c_void_p, ctypes.c_double ]
    dll.ufr_get_f64.restype =  ctypes.c_double

    dll.ufr_get_str.argtypes = [ ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int32 ]
    dll.ufr_get_str.restype =  ctypes.c_int32

    dll.ufr_get_rawptr.argtypes = [ ctypes.c_void_p]
    dll.ufr_get_rawptr.restype =  ctypes.c_void_p

    # Put - 32bits
    dll.ufr_put_u32.argtypes = [ ctypes.c_void_p, ctypes.c_uint32 ]
    dll.ufr_put_u32.restype =  ctypes.c_uint32
    dll.ufr_put_i32.argtypes = [ ctypes.c_void_p, ctypes.c_int32 ]
    dll.ufr_put_i32.restype =  ctypes.c_int32
    dll.ufr_put_f32.argtypes = [ ctypes.c_void_p, ctypes.c_float ]
    dll.ufr_put_f32.restype =  ctypes.c_int32

    dll.ufr_put_raw.argtypes = [ ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int32 ]
    dll.ufr_put_raw.restype =  ctypes.c_int32

    dll.ufr_put_raw.argtypes = [ ctypes.c_void_p ]
    dll.ufr_put_raw.restype =  ctypes.c_int32

    dll.ufr_put_file.argtypes = [ ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int32 ]
    dll.ufr_put_file.restype =  ctypes.c_int32

    

    # loop
    dll.ufr_loop_ok.argtypes = [ ]
    dll.ufr_loop_ok.restype =  ctypes.c_int32



    _fields_ = [
        ('data', ctypes.c_ubyte * 256)
    ]

    def __init__(self, text: str, type: int):
        error = Link.dll.ufr_new( ctypes.pointer(self), type, bytes(text,'utf-8') )
        if error != UFR_OK:
            error_msg = bytes(self.errstr).decode('utf-8').rstrip('\0')
            raise Exception(error_msg)

    def __del__(self):
        # self.close()
        pass

    def close(self):
        Link.dll.ufr_close( ctypes.pointer(self) )

    def __str__(self):
        api_name = Link.dll.ufr_api_name( ctypes.pointer(self) ).decode('utf-8')
        return api_name

    def recv(self):
        Link.dll.ufr_recv( ctypes.pointer(self) )

    def recv_with(self, link2, timeout_ms):
        res = Link.dll.ufr_recv_2s( ctypes.pointer(self), ctypes.pointer(link2), timeout_ms )
        return res == UFR_OK

    def recv_async(self):
        res = Link.dll.ufr_recv_async( ctypes.pointer(self) )
        return res == UFR_OK

    def is_error(self):
        return Link.dll.ufr_link_is_error( ctypes.pointer(self) )
        

    def write(self, value):
        Link.dll.ufr_write( ctypes.pointer(self), bytes(value, 'utf-8'), len(value) )

    def put(self, format, *args):
        index = 0
        state = 0
        for c in format:
            # State 0 : default
            if state == 0:
                if c == '#':
                    Link.dll.ufr_put_eof( ctypes.pointer(self) )
                elif c == '%':
                    state = 1
                elif c == '\n':
                    Link.dll.ufr_send( ctypes.pointer(self) )

            # State 0 : Found the caracter '%'. Example %d
            elif state == 1:
                # put integer
                if c == 'i' or c == 'd':
                    value = ctypes.c_int32( args[index] )
                    Link.dll.ufr_put_i32(ctypes.pointer(self), value, 1)
                    state = 0

                # put float
                elif c == 'f':
                    value = ctypes.c_float( args[index] )
                    Link.dll.ufr_put_f32(ctypes.pointer(self), value)
                    state = 0

                # put string
                elif c == 's':
                    value = args[index]
                    Link.dll.ufr_put_str(ctypes.pointer(self), bytes(value, 'utf-8'))
                    state = 0

                # put raw
                elif c == 'r':
                    value = args[index]
                    Link.dll.ufr_put_raw(ctypes.pointer(self), value, len(value))
                    state = 0
                else:
                    raise Exception(f"The variable %{c} is not allowed to serialize")

                index += 1

            # Error: state invalid
            else:
                raise Exception(f"State invalid")


    def get(self, format: str):
        resp = []
        state = 0
        for c in format:
            # State 0 : default
            if state == 0:
                if c == '^' or c == '>':
                    Link.dll.ufr_recv(ctypes.pointer(self))
                elif c == '\n':
                    Link.dll.ufr_get_eof(ctypes.pointer(self))
                elif c == '%':
                    state = 1
               
            # State 1 : Found the caracter '%'. Example %d
            elif state == 1:
                if c == 'i' or c == 'd':
                    var = Link.dll.ufr_get_i32(ctypes.pointer(self), ctypes.c_int32(0))
                    resp.append(var)
                    state = 0
                elif c == 'f':
                    var = Link.dll.ufr_get_f32(ctypes.pointer(self), ctypes.c_float(0))
                    resp.append(var)
                    state = 0
                elif c == 's':
                    size = Link.dll.ufr_meta_item_nbytes(ctypes.pointer(self)) + 1   # +1 : para o \0
                    buffer = ctypes.create_string_buffer(b"", size)
                    Link.dll.ufr_get_str( ctypes.pointer(self), ctypes.pointer(buffer), size)
                    text = bytes(buffer).decode('utf-8').rstrip('\0')
                    resp.append(text)
                    state = 0
                elif c == 'p':
                    ptr = Link.dll.ufr_get_rawptr(ctypes.pointer(self))
                    resp.append(ptr)
                    state = 0
                elif c == 'r':
                    size = Link.dll.ufr_meta_item_nbytes(ctypes.pointer(self))
                    buffer = (ctypes.c_ubyte * size)()
                    Link.dll.ufr_get_raw( ctypes.pointer(self), ctypes.pointer(buffer), size)
                    resp.append(buffer)
                    state = 0
                else:
                    raise Exception(f"The variable %{c} is not allowed to unpack")    
            
            # Error: state invalid
            else:
                raise Exception(f"Wrong state for get")

        # case just one, return scalar value
        if len(resp) == 1:
            return resp[0]
        else:
            return resp

    def get_cv_image(self):
        import numpy as np
        msg = self.get("%d %d %d %p")
        # print(msg)

        im_type = msg[0]
        im_rows = msg[1]
        im_cols = msg[2]
        im_data = msg[3]
        if im_type == 16:
            im_canal = 3
            image = np.ctypeslib.as_array(
                ctypes.cast(im_data, ctypes.POINTER(ctypes.c_ubyte)),
                shape=(im_rows, im_cols, im_canal)
            )
        elif im_type == 2:
            im_canal = 1
            image = np.ctypeslib.as_array(
                ctypes.cast(im_data, ctypes.POINTER(ctypes.c_ushort)),
                shape=(im_rows, im_cols, im_canal)
            )
        else:
            raise Exception("Invalid im_type : ", im_type)

        return image
    
    def recv_cv_image(self):
        self.recv()
        return self.get_cv_image()

    def send_cv_image(self, image):
        success, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        byte_data = buffer.tobytes()
        # Link.dll.ufr_put_file(ctypes.pointer(self), 'image/jpeg', byte.)


# =======================================================================================
#  Public Functions
# =======================================================================================

def Subscriber(text: str):
    return Link(text, UFR_START_SUBSCRIBER)

def Publisher(text: str):
    return Link(text, UFR_START_PUBLISHER)

def Server(text: str):
    return Link(text, UFR_START_SERVER)

def Client(text: str):
    return Link(text, UFR_START_CLIENT)


def subscriber(text: str):
    return Link(text, UFR_START_SUBSCRIBER)

def publisher(text: str):
    return Link(text, UFR_START_PUBLISHER)

def server(text: str):
    return Link(text, UFR_START_SERVER)

def client(text: str):
    return Link(text, UFR_START_CLIENT)


def loop_ok():
    return Link.dll.ufr_loop_ok()

def loop():
    return Link.dll.ufr_loop_ok()

# =======================================================================================
#  Tests
# =======================================================================================

"""
import sys
import signal

print("OPA")

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
"""