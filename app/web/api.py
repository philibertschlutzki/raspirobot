from flask import jsonify, request, send_from_directory

def setup_api(app, socketio, sensor_svc, mode_ctrl):
@app.route('/sensors')
def get_sensors():
with sensor_svc.data_lock:
return jsonify(sensor_svc.data)

    @app.route('/')
    def index():
        return send_from_directory(app.static_folder, 'index.html')
    
    @socketio.on('mode_change')
    def on_mode_change(msg):
        mode = msg.get('mode')
        if mode in mode_ctrl.modes:
            mode_ctrl.current = mode
            socketio.emit('mode', {'mode': mode})
    ```

---

## Frontend

Legen Sie unter `app/static/` folgende Dateien an:

- **index.html**: Dashboard mit Canvas-Visualisierung und Modus-Schaltern  
- **js/app.js**: WebSocket-Verbindung, Fetch-Aufrufe an `/sensors`, UI-Updates  
- **css/style.css**: Layout und Responsive-Design  

(_Beispiel: index.html lädt Socket.IO, verbindet, zeigt Sensordaten und Buttons zum Wechseln zwischen Patrol und Follow._)

---

## Systemd-Service

Datei: `systemd/robot.service`
