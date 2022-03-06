import threading

def create_read_exactly(reader):
    """
    Creates a read method that ensures exactly SIZE bytes are read. The spec
    only guarantees that "up to" SIZE bytes will be read, although that's not
    true in practice with buffered streams.
    """
    def fn(size):
        ret = reader.read(size)
        assert len(ret) == size, "Expected %d bytes, got %d." % (size, len(ret))
        return ret
    return fn
    
def synchronized(fn):
    lock = threading.Lock()
    def wrapper(*args, **kwargs):
        with lock:
            return fn(*args, **kwargs)
    return wrapper