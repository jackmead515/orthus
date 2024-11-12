# generate Flask boilerplate
from flask import Flask
from flask_cors import CORS

app = Flask(__name__)
cors = CORS(app, resources={r"/api/*": {"origins": "*"}})

import config

config.initialize()

from routes import stream as stream_routes
from routes import app as app_routes

app.register_blueprint(stream_routes.mod)
app.register_blueprint(app_routes.mod)

if __name__ == '__main__':
    """    
    curl -X POST http://localhost:8000/api/stream
    """
    
    app.run(
        host='0.0.0.0',
        port=config.HTTP_PORT,
        threaded=True,
        debug=False
    )
    
    