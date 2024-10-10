import logging
import threading
import subprocess
import time

import controllers.streams as stream_controller

import config

_stream = None
_streams_lock = threading.Lock()


def kill_stream():
    global _stream, _streams_lock

    with _streams_lock:
        if _stream is not None:
            timeout = 5
            while not _stream.poll():
                _stream.kill()
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    raise Exception('Failed to kill stream')
        _stream = None
        
        # final any ffmpeg processes that may have been left running
        subprocess.run(['pkill', 'ffmpeg'])


def launch_stream():
    global _stream, _streams_lock

    with _streams_lock:
        
        if _stream is not None:
            timeout = 5
            while not _stream.poll():
                _stream.kill()
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    raise Exception('Failed to kill stream')

        command = stream_controller.get_pi_usb_encoding_pipeline(config.STREAM_CONFIG, config.RECORD_DIRECTORY)

        logging.info(f'Running ffmpeg pipeline: {command}')

        if config.DEBUG:
            _stream = subprocess.Popen(command, shell=True)
        else:
            _stream = subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)