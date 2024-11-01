# generate Flask boilerplate
from flask import Flask
from flask_cors import CORS

app = Flask(__name__)
cors = CORS(app, resources={r"/api/*": {"origins": "*"}})

import config

config.initialize()

from routes import stream

app.register_blueprint(stream.mod)

@app.route('/')
def index():
    return app.send_static_file('index.html')


@app.route('/jsmpeg.min.js')
def jsmpeg():
    return app.send_static_file('jsmpeg.min.js')


if __name__ == '__main__':
    """
    STREAM_NAME=zedorthus \
    STREAM_HOST=0.0.0.0 \
    STREAM_PORT=3131 \
    STREAM_CONFIG="ffmpeg -an -f v4l2 -i /dev/video0 -video_size 2560x720 -framerate 30 -threads 4 -f tee -map 0:v -f segment -vcodec mjpeg -qscale:v 3 -segment_time 10 -reset_timestamps 1 -strftime 1 zedorthus_%s.mp4 -map 0:v -f mpegts -vcodec mpeg1video -qscale:v 18 -vf \"crop=(iw/2):ih:0:0\" udp://0.0.0.0:3131" \
    RECORD_DIRECTORY=/data \
    python3 main.py
    
    curl -X POST http://localhost:8000/api/stream
    """
    
    app.run(
        host='0.0.0.0',
        port=config.HTTP_PORT,
        threaded=True,
        debug=False
    )
    
    