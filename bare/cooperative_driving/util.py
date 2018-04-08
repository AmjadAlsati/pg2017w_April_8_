import platform
from contextlib import contextmanager

_RASPI_ARCH = 'armv7l'

def on_raspi():
    return platform.machine() == _RASPI_ARCH

def noop(*args, **kwd):
    pass

@contextmanager
def quitting(thing):
    try:
        yield thing
    finally:
        thing.quit()
