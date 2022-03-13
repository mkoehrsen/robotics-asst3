import socket
import struct

from . import messages
from .util import synchronized

def listen(port):
    s_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_sock.bind((socket.gethostname(), port))
    s_sock.listen(0)
    conn, addr = s_sock.accept()
    return Connection(conn)
    
def connect(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    return Connection(sock)

class Connection(object):

    def __init__(self, sock):
        self.sock = sock
        self.reader = sock.makefile("rb")
        self.writer = sock.makefile("wb")

    @synchronized
    def send(self, device, message):
        wrote = 0
        wrote += self.writer.write(struct.pack("<B", device))
        wrote += message.write_into(self.writer)
        self.writer.flush()
        return wrote
    
    @synchronized
    def receive(self):
        """ Blocks until a message appears. """
        device, = struct.unpack("<B", self.reader.read(1))
        return device, messages.read_next_message(self.reader)
        
    def close(self):
        self.sock.close()
        
    def closed(self):
        return self.sock.fileno() == -1