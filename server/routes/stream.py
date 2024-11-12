from flask import Blueprint

import services.streams as stream_service

mod = Blueprint('stream', __name__)

@mod.route('/api/stream', methods=['POST'])
def create_stream():

    # build stream_config
    stream_config = "orthus --help"
    
    stream_service.launch_stream(stream_config)
    
    return 'OK', 200


@mod.route('/api/stream', methods=['DELETE'])
def delete_stream():
    stream_service.kill_stream()
    
    return 'OK', 200