
from flask import Flask
from flask_socketio import SocketIO
from app.web.api import setup_api

def run_server(sensor_svc, mode_ctrl):
app = Flask(__name__, static_folder="../static", template_folder="../static")
socketio = SocketIO(app, cors_allowed_origins="*")
setup_api(app, socketio, sensor_svc, mode_ctrl)
socketio.run(app, host="0.0.0.0", port=5000)
