import fastdds
import sys
import os

class VehicleControlWriterListener(fastdds.DataWriterListener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
    
    def on_publication_matched(self, datawriter, info):
        if self.publisher.shutdown_event.is_set():
            return
        if 0 < info.current_count_change:
            print(f"VehicleControlMsg发布者匹配到订阅者 {info.last_subscription_handle}")
            self.publisher.vehiclecontrol_matched_readers += 1
            self.publisher.check_all_ready()
        else:
            print(f"VehicleControlMsg发布者取消匹配 {info.last_subscription_handle}")
            self.publisher.vehiclecontrol_matched_readers -= 1
