import time
import threading
import fastdds
from msg.ImageMsg import ImageMsg
from threading import Lock
import numpy as np
from common.utils import RED, GREEN, YELLOW, BLUE, RESET
import os, sys
import cv2
import ctypes

class ImageMsgReaderListener(fastdds.DataReaderListener):
    def __init__(self, publisher, camera_name, img_height=480, img_width=640, infer=False):
        super().__init__()
        self.publisher = publisher
        self.camera_name = camera_name 
        self.frame_count = 0
        self.last_print_time = 0
        self.match_count = 0
        self.first_match_time = 0
        self.current_time_idx = 0
        

        self._buffer = None
        self.img_height = img_height
        self.img_width = img_width
        self.channels = 4
        
        self.target_fps = 10
        self.interval = 1.0 / self.target_fps
        self.latest_data = None 
        self.data_lock = Lock() 
        self.infer = infer
        if not self.infer:
            self.timer_thread = threading.Thread(target=self.publish_image_to_frame_collector, daemon=True)
            self.timer_thread.start()
        print(f"[ImageMsgReaderListener][{self.camera_name}] created!!!!!!!!!!!!!!!!!")
    
    def on_subscription_matched(self, datareader, info):
        current_time = time.time()
        if self.publisher.shutdown_event.is_set():
            return
            
        if 0 < info.current_count_change:
            self.match_count += 1
            if self.first_match_time == 0:
                self.first_match_time = current_time
                
            print(f"\n[{self.camera_name}] 匹配到发布者 (Domain 11)")
            print(f"[{self.camera_name}]   匹配计数: {self.match_count}")
            
            self.publisher.image_matched_writers[self.camera_name] += 1
        else:
            print(f"\n[{self.camera_name}] 取消匹配")
            self.publisher.image_matched_writers[self.camera_name] -= 1

    def get_image_from_data(self, data):
        """使用 get_buffer() 方法快速转换"""
        st = time.time()
        vec = data.image_data()
        size = vec.size()
        
        if size == 0:
            return b""
        
        if self._buffer is None or len(self._buffer) != size:
            self._buffer = bytearray(size)
        
        try:
            buf_obj = vec.get_buffer()
            
            if hasattr(buf_obj, '__int__'):
                src_ptr = int(buf_obj)
            else:
                src_ptr = ctypes.cast(buf_obj, ctypes.c_void_p).value

            dst_ptr = (ctypes.c_char * size).from_buffer(self._buffer)
            ctypes.memmove(ctypes.addressof(dst_ptr), src_ptr, size)
            image = np.frombuffer(self._buffer, dtype=np.uint8)
            image = image.reshape(self.img_height, self.img_width, self.channels)[:, :, :3]
            image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
            image = np.transpose(image, (2, 0, 1))
            return image
            
        except Exception as e:
            print(f"[{self.camera_name}] get_buffer 方法失败: {e}")
            return bytearray(vec)
    
    def on_data_available(self, datareader):
        if self.publisher.shutdown_event.is_set():
            return
            
        try:
            sample_info = fastdds.SampleInfo()
            data = ImageMsg.ImageMsg()
            ret = datareader.take_next_sample(data, sample_info)
            
            if ret == fastdds.RETCODE_OK and sample_info.valid_data: 
                self.frame_count += 1
                if self.infer:
                    with self.publisher.image_data_lock[self.camera_name]:
                        self.publisher.latest_images[self.camera_name] = self.get_image_from_data(data).copy()
                        self.publisher.get_new_images[self.camera_name] = True
                        # print(f"[{self.camera_name}] 缓存图像数据，当前帧数: {self.frame_count} shape: {self.publisher.latest_images[self.camera_name].shape}")
                else:
                    with self.data_lock:
                        self.latest_data = self.get_image_from_data(data).copy()
             


        except Exception as e:
            print(f"[{self.camera_name}] 缓存图像数据时出错: {str(e)[:50]}")
            if not self.publisher.shutdown_event.is_set() and self.frame_count % 100 == 0:
                print(f"[{self.camera_name}] 缓存图像数据时出错: {str(e)[:50]}")
    
    def publish_image_to_frame_collector(self):
        """定时器线程：严格按10Hz频率处理并发送缓存的最新图像数据"""
        while not self.publisher.shutdown_event.is_set():
            process_start_time = time.time()
            image_data = None
            with self.data_lock:
                if self.latest_data is not None:
                    image_data = self.latest_data.copy()  # 再次拷贝，避免后续修改

            if image_data is not None:
                if self.publisher.is_robot_moving and self.publisher.frame_collector.data_recorder.is_recording:
                    t_time = time.time()

                    #print(f"[ImageMsgReaderListener] send image {self.camera_name} idx={self.current_time_idx} current time : {t_time}")
                    self.publisher.frame_collector.update_image(
                        cam_name=self.camera_name,  # 确保相机名正确传递
                        img_data=image_data,
                        idx=self.current_time_idx,
                        ts=t_time
                    )
                                    
                    self.current_time_idx += 1
            
            elapsed_time = time.time() - process_start_time
            sleep_time = max(0, self.interval - elapsed_time)
            time.sleep(sleep_time)
        
            # print(f"[{self.camera_name}] publish_image_to_frame_collector loop time: {end_time - start_time:.6f} shape : {image_data.shape}")


        
