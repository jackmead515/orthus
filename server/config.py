import os
from distutils.util import strtobool

HTTP_PORT = 8000
RECORD_DIRECTORY = None
DEBUG = False

def required_env(key):
    value = os.getenv(key)
    
    if value is None:
        raise Exception(f'{key} is not set in environment variables')
    
    return value


def initialize():
    global RECORD_DIRECTORY, DEBUG, HTTP_PORT
    
    HTTP_PORT = int(os.getenv('HTTP_PORT', HTTP_PORT))

    DEBUG = os.getenv('DEBUG', 'False').lower() == 'true'
    
    RECORD_DIRECTORY = required_env('RECORD_DIRECTORY')
