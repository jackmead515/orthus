from flask import Blueprint, Flask

mod = Blueprint('app', __name__)

@mod.route('/')
def index():
    return Flask.send_static_file('index.html')


@mod.route('/jsmpeg.min.js')
def jsmpeg():
    return Flask.send_static_file('jsmpeg.min.js')
