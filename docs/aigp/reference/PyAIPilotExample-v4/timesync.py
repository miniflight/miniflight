import time
import threading

from pymavlink import mavutil

TIMESYNC_REQUEST_HZ = 10

class TimeSync:

    def __init__(self, mavlink_connection, data):
        self.mavlink_conn = mavlink_connection
        self.data = data
        self.thread = None
        self.is_running = False

    @classmethod
    def create_timesync(cls, mavlink_connection, data):
        ts = cls(mavlink_connection, data)
        ts.thread = threading.Thread(
            target=ts.timesync_loop,
            daemon = False
        )
        ts.is_running = True
        ts.thread.start()
        return ts

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def timesync_loop(self):
        while self.is_running:
            now = int(time.time_ns())
            self.mavlink_conn.mav.timesync_send(
                now,  # tc1 = client time
                0     # ts1 = 0 (request)
            )
            time.sleep(1.0 / TIMESYNC_REQUEST_HZ)
