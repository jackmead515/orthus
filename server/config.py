import os
from distutils.util import strtobool

HTTP_PORT = 8000
STREAM_CONFIG = None
RECORD_DIRECTORY = None
DEBUG = False

def required_env(key):
    value = os.getenv(key)
    
    if value is None:
        raise Exception(f'{key} is not set in environment variables')
    
    return value


def initialize():
    global STREAM_CONFIG, RECORD_DIRECTORY, DEBUG, HTTP_PORT
    
    HTTP_PORT = int(os.getenv('HTTP_PORT', HTTP_PORT))

    STREAM_CONFIG = required_env('STREAM_CONFIG')
    
    DEBUG = os.getenv('DEBUG', 'False').lower() == 'true'
    
    RECORD_DIRECTORY = required_env('RECORD_DIRECTORY')
