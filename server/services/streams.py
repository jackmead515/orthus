import logging
import threading
import subprocess
import time
import shlex
import signal
from subprocess import TimeoutExpired

import config

_stream = None
_streams_lock = threading.Lock()


def kill_stream():
    global _stream, _streams_lock

    with _streams_lock:
        if _stream is not None:
            _stream.send_signal(signal.SIGINT)
            try:
                exit_code = _stream.wait(timeout=5)
            except TimeoutExpired:
                _stream.kill()

        _stream = None

        # final any ffmpeg processes that may have been left running
        subprocess.run(['pkill', 'ffmpeg'])


def launch_stream():
    global _stream, _streams_lock
    
    kill_stream()

    with _streams_lock:
        logging.info(f'Running ffmpeg pipeline: {shlex.split(config.STREAM_CONFIG)}')

        if config.DEBUG:
            _stream = subprocess.Popen(shlex.split(config.STREAM_CONFIG), shell=True, cwd=config.RECORD_DIRECTORY)
        else:
            _stream = subprocess.Popen(shlex.split(config.STREAM_CONFIG), shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=config.RECORD_DIRECTORY)