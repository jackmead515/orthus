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
    STREAM_NAME='webcam' \
    STREAM_HOST='localhost' \
    STREAM_PORT='3131' \
    INFPS='30' \
    INRES='640x480' \
    OUTRES='640x480' \
    OUTFPS='30' \
    BITRATE='1024k' \
    THREADS='0' \
    VSYNC='2' \
    SEGMENT_TIME='10' \
    VIDEO_INDEX='0' \
    ARCHIVE='1' \
    GRAYSCALE='0' \
    RECORD_DIRECTORY='/tmp' \
    python3 main.py
    
    curl -X POST http://localhost:8000/api/stream
    """
    
    app.run(
        host='0.0.0.0',
        port=config.HTTP_PORT,
        threaded=True,
        debug=False
    )
    
    