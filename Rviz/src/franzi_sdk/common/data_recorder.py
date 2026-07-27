import threading
import time
import numpy as np
import queue
import h5py
import traceback
import cv2
from common.utils import RED, GREEN, YELLOW, BLUE, RESET

class DataRecorder:
    """数据记录器：接收完整帧（含原始图像），在后台线程处理图像并写入HDF5
    核心职责：数据记录、文件管理、资源清理、图像预处理
    支持灵活配置：根据模块配置动态创建数据集（无图像/pose/action则不创建对应数据集）
    新增：批量写入功能（默认100帧批量，参数可调）
    """
    def __init__(self, batch_size: int = 100):  # 新增批量大小参数
        # 核心初始化
        self.data_queue = queue.Queue(maxsize=1000)
        self.start_confirm = threading.Event()
        self.stop_confirm = threading.Event()
        self.stop_event = threading.Event()  # 线程终止事件
        self.is_recording = False
        
        # 文件/数据集句柄
        self.current_file = None
        self.current_filename = None
        self.dataset_handles = {}
        
        # 状态计数器
        self.frame_count = 0
        self.segment_id = 0
        
        # 线程锁
        self.file_lock = threading.Lock()
        
        # 图像目标尺寸配置（统一管理）
        self.img_target_shape = (3, 240, 320)  # (C, H, W)
        
        # ========== 新增：批量写入相关配置 ==========
        self.batch_size = batch_size  # 批量写入大小（可调）
        self.batch_cache = {          # 批量缓存容器
            "robot_pose": [],
            "robot_action": [],
            "head_image": [],
            "left_arm_image": [],
            "right_arm_image": []
        }
        
        # 后台线程（daemon=True）
        self.save_thread = threading.Thread(target=self.save_worker, daemon=True)
        self.save_thread.start()
        
        # 简化初始化日志
        print(f"数据记录器初始化完成 | 图像目标尺寸: {self.img_target_shape} | 批量写入大小: {self.batch_size}")

    def save_worker(self):
        """后台工作线程：处理队列中的数据"""
        while not self.stop_event.is_set():
            try:
                item = self.data_queue.get(timeout=0.5)
                try:  # 嵌套try，确保task_done必执行
                    if item == "START":
                        self._handle_start()
                    elif item == "STOP":
                        self._handle_stop()
                    elif isinstance(item, dict):
                        self._handle_full_frame(item)
                finally:
                    self.data_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"后台线程异常: {e}")
                traceback.print_exc()
        
        # 线程退出前：处理队列剩余数据
        self._process_remaining_queue()

    def _process_remaining_queue(self):
        """处理队列中剩余的数据"""
        print(f"开始处理队列剩余数据，当前队列长度: {self.data_queue.qsize()}")
        while not self.data_queue.empty():
            try:
                item = self.data_queue.get(block=False)
                try:
                    if item == "START":
                        self._handle_start()
                    elif item == "STOP":
                        self._handle_stop()
                    elif isinstance(item, dict):
                        self._handle_full_frame(item)
                finally:
                    self.data_queue.task_done()
            except queue.Empty:
                break
            except Exception as e:
                print(f"处理剩余数据异常: {e}")
                traceback.print_exc()
        print("队列剩余数据处理完成")

    # ========== 新增：批量写入函数 ==========
    def _batch_write_data(self):
        """批量写入缓存中的数据到HDF5"""
        if not self.current_file or len(self.batch_cache["robot_pose"]) == 0:
            return
        
        try:
            batch_len = len(self.batch_cache["robot_pose"])
            start_idx = self.frame_count - batch_len  # 起始索引
            end_idx = self.frame_count                # 结束索引
            
            # 1. 批量写入数值型数据
            for key in ["robot_pose", "robot_action"]:
                if key in self.dataset_handles and len(self.batch_cache[key]) > 0:
                    batch_data = np.array(self.batch_cache[key])
                    self.dataset_handles[key].resize((end_idx, batch_data.shape[1]))
                    self.dataset_handles[key][start_idx:end_idx] = batch_data
            
            # 2. 批量写入图像数据
            img_keys = ["head_image", "left_arm_image", "right_arm_image"]
            for img_key in img_keys:
                if img_key in self.dataset_handles and len(self.batch_cache[img_key]) > 0:
                    batch_data = np.array(self.batch_cache[img_key])
                    self.dataset_handles[img_key].resize((end_idx,) + self.img_target_shape)
                    self.dataset_handles[img_key][start_idx:end_idx] = batch_data
            
            print(f"{GREEN}批量写入{batch_len}帧数据 | 起始索引:{start_idx} 结束索引:{end_idx}{RESET}")
            
            # 清空批量缓存
            for key in self.batch_cache:
                self.batch_cache[key].clear()
                
        except Exception as e:
            print(f"{RED}批量写入失败: {e}{RESET}")
            traceback.print_exc()

    def _handle_start(self):
        """创建HDF5文件和数据集：匹配传入的数据key"""
        with self.file_lock:
            if self.current_file:
                self.current_file.close()
            
            # 创建文件路径
            self.current_filename = f"/home/Thor/mcl/datasets/episode_{self.segment_id}.hdf5"
            
            try:
                self.current_file = h5py.File(self.current_filename, "w")
                self.dataset_handles.clear()

                # 创建数值型数据集
                self.dataset_handles["robot_pose"] = self.current_file.create_dataset(
                    "robot_pose",
                    shape=(0, 26),
                    maxshape=(None, 26),
                    dtype=np.float64,
                    chunks=True,
                    compression="gzip"
                )
                self.dataset_handles["robot_action"] = self.current_file.create_dataset(
                    "robot_action",
                    shape=(0, 26),
                    maxshape=(None, 26),
                    dtype=np.float64,
                    chunks=True,
                    compression="gzip"
                )

                # 创建图像数据集
                img_datasets = ["head_image", "left_arm_image", "right_arm_image"]
                for img_name in img_datasets:
                    self.dataset_handles[img_name] = self.current_file.create_dataset(
                        img_name,
                        shape=(0,) + self.img_target_shape,
                        maxshape=(None,) + self.img_target_shape,
                        dtype=np.uint8,
                        chunks=(1,) + self.img_target_shape,
                        compression="gzip"
                    )
                
                # 创建progress数据集
                self.dataset_handles["progress"] = self.current_file.create_dataset(
                    "progress",
                    shape=(0, 1),          
                    maxshape=(None, 1),    
                    dtype=np.float64,      
                    chunks=True,
                    compression="gzip"
                )

                self.frame_count = 0
                # 清空批量缓存（防止残留）
                for key in self.batch_cache:
                    self.batch_cache[key].clear()
                print(f"创建记录文件成功: {self.current_filename}")
                print(f"已创建数据集: {list(self.dataset_handles.keys())}")
                self.start_confirm.set()
                self.is_recording = True  # 确保开始记录状态正确
            
            except Exception as e:
                print(f"创建文件失败: {e}")
                traceback.print_exc()
                self.current_file = None
                self.start_confirm.clear()
                self.is_recording = False

    def _handle_stop(self):
        """关闭文件并释放资源"""
        with self.file_lock:
            if self.current_file:
                try:
                    # ========== 新增：停止前写入剩余的批量缓存 ==========
                    if len(self.batch_cache["robot_pose"]) > 0:
                        print(f"{YELLOW}停止记录，写入剩余{len(self.batch_cache['robot_pose'])}帧缓存数据{RESET}")
                        self._batch_write_data()
                    
                    print(f"\n记录完成 | 分段ID: {self.segment_id} | 总帧数: {self.frame_count}")
                    
                    # 写入progress数据
                    if self.frame_count > 0:
                        progress_values = np.linspace(0.0, 1.0, self.frame_count).reshape(-1, 1)
                        self.dataset_handles["progress"].resize((self.frame_count, 1))
                        self.dataset_handles["progress"][:] = progress_values
                        print(f"  progress数据已写入: ({self.frame_count}, 1) | 范围: 0.0 ~ 1.0")

                    # 刷新并关闭文件
                    self.current_file.flush()
                    self.current_file.close()
                    self.stop_confirm.set()
                    print(f"文件已关闭: {self.current_filename}")
                except Exception as e:
                    print(f"关闭文件错误: {e}")
                    traceback.print_exc()
                    self.stop_confirm.clear()
                
                self.current_file = None
                self.dataset_handles.clear()
                self.segment_id += 1
                self.is_recording = False

    def _handle_full_frame(self, full_frame):
        """写入完整帧数据：先缓存，达到批量大小后批量写入"""
        if not self.current_file:
            print("警告：尝试写入数据但当前无打开的文件")
            return
        
        try:
            with self.file_lock:
                # 1. 处理并缓存单帧数据（替代原逐帧写入逻辑）
                start_time = time.time()
                
                # 2. 缓存robot_pose
                if "robot_pose" in full_frame and "robot_pose" in self.dataset_handles:
                    pose_data = full_frame["robot_pose"]
                    if pose_data.shape == (26,):
                        self.batch_cache["robot_pose"].append(pose_data)
                    else:
                        print(f"警告：robot_pose维度错误，期望(26,)，实际{pose_data.shape}")
                
                # 3. 缓存robot_action
                if "robot_action" in full_frame and "robot_action" in self.dataset_handles:
                    action_data = full_frame["robot_action"]
                    if action_data.shape == (26,):
                        self.batch_cache["robot_action"].append(action_data)
                    else:
                        print(f"警告：robot_action维度错误，期望(26,)，实际{action_data.shape}")
                
                # 4. 处理并缓存图像数据
                img_keys = ["head_image", "left_arm_image", "right_arm_image"]
                for img_key in img_keys:
                    if img_key in full_frame and img_key in self.dataset_handles:
                        raw_img = full_frame[img_key]
                        self.batch_cache[img_key].append(raw_img)
                
                # 5. 更新帧计数
                self.frame_count += 1
                
                # 6. 达到批量大小则批量写入
                if len(self.batch_cache["robot_pose"]) >= self.batch_size:
                    self._batch_write_data()
                
                # 调试日志（每10帧打印一次，避免刷屏）
                if self.frame_count % 10 == 0:
                    print(f"已缓存 {self.frame_count} 帧数据 | 当前批量缓存: {len(self.batch_cache['robot_pose'])} 帧")
                
                end_time = time.time()
                # print(f"{RED}缓存一帧数据耗时: {end_time - start_time:.6f} 秒{RESET}")  

        except Exception as e:
            print(f"{RED}缓存帧失败: {e}{RESET}")
            traceback.print_exc()

    def record_full_frame(self, full_frame):
        """对外接口：接收完整帧数据并加入队列"""
        if not self.is_recording:
            print("警告：尝试写入数据但未开始记录")
            return False
        try:
            self.data_queue.put(full_frame, block=True, timeout=1.0)
            return True
        except queue.Full:
            print(f"队列已满，丢弃帧 | 当前队列长度: {self.data_queue.qsize()}")
            return False

    def start_recording(self):
        """启动记录"""
        if not self.is_recording:
            self.start_confirm.clear()
            try:
                self.data_queue.put("START", block=True, timeout=1.0)
                if self.start_confirm.wait(timeout=2.0):
                    print("开始记录数据（已确认）")
                    self.is_recording = True
                    return True
                else:
                    print("启动超时：Recorder未完成文件创建")
                    self.is_recording = False
                    return False
            except queue.Full:
                print("启动失败：队列已满，无法放入START指令")
                self.is_recording = False
                return False
        print("警告：记录已在运行中")
        return True

    def stop_recording(self):
        """停止记录"""
        if self.is_recording:
            self.stop_confirm.clear()
            try:
                self.data_queue.put("STOP", block=True, timeout=1.0)
                if self.stop_confirm.wait(timeout=5.0):
                    print("停止记录，已关闭文件（已确认）")
                    self.is_recording = False
                    return True
                else:
                    print("停止超时：Recorder未完成文件关闭")
                    self.is_recording = False
                    return False
            except queue.Full:
                print("停止失败：队列已满，无法放入STOP指令")
                return False
        print("警告：记录未在运行中")
        return True

    def cleanup(self):
        """清理资源"""
        print("开始清理资源，等待数据写入完成...")
        self.is_recording = False
        
        try:
            self.data_queue.put("STOP", block=True, timeout=1.0)
        except queue.Full:
            print("队列已满，强制处理剩余数据...")
        
        self.stop_event.set()
        
        if self.save_thread.is_alive():
            self.save_thread.join(timeout=10.0)
            if self.save_thread.is_alive():
                print("警告：后台线程未正常退出")
        
        with self.file_lock:
            # ========== 新增：清理前写入剩余缓存 ==========
            if self.current_file and len(self.batch_cache["robot_pose"]) > 0:
                print(f"{YELLOW}清理资源，写入剩余{len(self.batch_cache['robot_pose'])}帧缓存数据{RESET}")
                self._batch_write_data()
                
            if self.current_file:
                try:
                    self.current_file.flush()
                    self.current_file.close()
                    print(f"已关闭文件: {self.current_filename}")
                except Exception as e:
                    print(f"关闭文件异常: {e}")
                    traceback.print_exc()
                finally:
                    self.current_file = None
                    self.dataset_handles.clear()
        
        self.frame_count = 0
        print("[数据记录器] 资源清理完成，所有数据已写入磁盘")

# DataRecorder单独测试
if __name__ == "__main__":
    # 创建记录器实例（可自定义批量大小，比如改为50）
    recorder = DataRecorder(batch_size=20)  # 新增批量大小参数
    
    # 启动记录
    if not recorder.start_recording():
        print("Recorder启动失败")
        exit(1)
    
    # 模拟发送100帧数据
    print("\n开始模拟写入100帧数据...")
    for i in range(100):
        # 模拟原始图像字节数据 (480,640,4)
        raw_img_bytes = np.random.randint(0, 255, (480, 640, 4), dtype=np.uint8).tobytes()
        
        # 构造完整帧数据（匹配数据集key）
        one_step_data = {
            "robot_pose": np.random.rand(26).astype(np.float64),
            "robot_action": np.random.rand(26).astype(np.float64),
            "head_image": raw_img_bytes,
            "left_arm_image": raw_img_bytes,
            "right_arm_image": raw_img_bytes,
        }
        
        # 写入数据
        success = recorder.record_full_frame(one_step_data)
        if not success:
            print(f"第{i+1}帧写入失败")
        
        # 模拟少量延迟
        time.sleep(0.001)
    
    # 停止记录并清理
    print("\n停止记录...")
    recorder.stop_recording()
    recorder.cleanup()
    
    # 验证文件是否创建成功
    import os
    test_file = f"/home/rossum/chenjx/mcl/vr_control_data_record/datasets/episode_0.hdf5"
    if os.path.exists(test_file):
        print(f"\n测试成功：文件已创建 {test_file}")
        # 简单验证文件内容
        with h5py.File(test_file, "r") as f:
            print("文件中的数据集：")
            for key in f.keys():
                print(f"  {key}: {f[key].shape}")
    else:
        print(f"\n测试失败：文件未创建 {test_file}")
