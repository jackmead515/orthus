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

        # final any orthus processes that may have been left running
        subprocess.run(['pkill', 'orthus'])


def launch_stream(stream_config: str):
    global _stream, _streams_lock
    
    kill_stream()

    with _streams_lock:
        logging.info(f'Running pipeline: {shlex.split(stream_config)}')

        if config.DEBUG:
            _stream = subprocess.Popen(shlex.split(stream_config), shell=True, cwd=config.RECORD_DIRECTORY)
        else:
            _stream = subprocess.Popen(shlex.split(stream_config), shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=config.RECORD_DIRECTORY)