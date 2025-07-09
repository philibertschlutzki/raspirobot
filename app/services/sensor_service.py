from threading import Thread, Lock
class SensorService(Thread):
    def __init__(self, ultrasonic, microwave):
        super().__init__()
        self.ultrasonic = ultrasonic
        self.microwave = microwave
        self.data_lock = Lock()
        self.data = {}
    def run(self):
        while True:
            d_front  = self.ultrasonic.read_distance('front')
            d_right  = self.ultrasonic.read_distance('right')
            d_back   = self.ultrasonic.read_distance('back')
            d_left   = self.ultrasonic.read_distance('left')
            person   = self.microwave.detect_motion()
            with self.data_lock:
                self.data = {'front': d_front, 'right': d_right,
                             'back': d_back,  'left': d_left,
                             'person': person}
            time.sleep(0.05)  # 20 Hz