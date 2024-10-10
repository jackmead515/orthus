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

    STREAM_CONFIG = {
        'stream_name': required_env('STREAM_NAME'),
        'stream_host': required_env('STREAM_HOST'),
        'stream_port': required_env('STREAM_PORT'),
        'infps': required_env('INFPS'),
        'inres': required_env('INRES'),
        'outres': required_env('OUTRES'),
        'outfps': required_env('OUTFPS'),
        'bitrate': required_env('BITRATE'),
        'threads': required_env('THREADS'),
        'vsync': required_env('VSYNC'),
        'segment_time': required_env('SEGMENT_TIME'),
        'video_index': required_env('VIDEO_INDEX'),
        'archive':  strtobool(required_env('ARCHIVE')),
        'grayscale':  strtobool(required_env('GRAYSCALE')),
    }
    
    DEBUG = os.getenv('DEBUG', 'False').lower() == 'true'
    
    RECORD_DIRECTORY = required_env('RECORD_DIRECTORY')
