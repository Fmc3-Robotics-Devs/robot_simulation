import threading
import time
import numpy as np
import h5py
import os
from common.data_recorder import DataRecorder
from common.utils import RED, GREEN, YELLOW, BLUE, RESET

# ===================== 极简版FrameCollector =====================
class FrameCollector:
    """
    极简帧收集器：仅维护当前帧/上一帧数据，无复杂对齐
    - 无缓存队列，仅保存当前帧和上一帧
    - 当前帧无数据时，自动用上一帧兜底
    - 10Hz固定频率推送数据至DataRecorder
    """
    def __init__(self, freq: int = 10, data_recorder=None):
        # 基础配置
        self.data_recorder = data_recorder
        self.interval = 1.0 / freq  # 10Hz → 0.1秒
        
        # 核心数据：当前帧 + 上一帧（兜底用）
        self.lock = threading.Lock()
        # 当前帧数据
        self.current_action = None
        self.current_pose = None
        self.current_imgs = {"head_image": None, "left_arm_image": None, "right_arm_image": None}
        # 上一帧数据（兜底）
        self.last_action = None
        self.last_pose = None
        self.last_imgs = {"head_image": None, "left_arm_image": None, "right_arm_image": None}

        self.action_idx = 0
        self.pose_idx = 0
        self.img_idx = {"head_image": 0, "left_arm_image": 0, "right_arm_image": 0}
        
        # 线程控制
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._collect_loop, daemon=True)
        self.thread.start()
        print(f"帧收集器启动 | 频率{freq}Hz")

    # ========== 仅更新当前帧数据（极简） ==========
    def update_action(self, action: np.ndarray, idx: int, ts: float):
        """更新当前动作帧"""
        # print(f"{GREEN}update action shape: {action.shape} idx={idx} ts={ts}{RESET}")
        with self.lock:
            self.current_action = action.copy()
            self.action_idx = idx

    def update_pose(self, pose: np.ndarray, idx: int, ts: float):
        """更新当前位姿帧"""
        # print(f"{YELLOW}update pose shape: {pose.shape} idx={idx} ts={ts}{RESET}")
        with self.lock:
            self.current_pose = pose.copy()
            self.pose_idx = idx

    def update_image(self, cam_name: str, img_data: bytes, idx: int, ts: float):
        """更新指定相机的当前图像帧"""
        # print(f"{RED}update image {cam_name} shape: {len(img_data)} idx={idx} ts={ts}{RESET}")
        if cam_name in self.current_imgs:
            with self.lock:
                self.current_imgs[cam_name] = img_data
                self.img_idx[cam_name] = idx

    # ========== 核心循环：取当前帧/上一帧兜底 ==========
    def _collect_loop(self):
        start_time = time.time()
        while not self.stop_event.is_set():
            start_ts = time.time()
            
            # 1. 加锁读取当前帧，无数据则用上一帧兜底
            with self.lock:
                # 动作兜底
                action = self.current_action if self.current_action is not None else self.last_action
                # 位姿兜底
                pose = self.current_pose if self.current_pose is not None else self.last_pose
                # 图像兜底
                imgs = {}
                for cam in self.current_imgs:
                    imgs[cam] = self.current_imgs[cam] if self.current_imgs[cam] is not None else self.last_imgs[cam]

                # 2. 有有效数据才推送（至少动作/位姿/图像都有兜底数据）
                if action is not None and pose is not None and all(v is not None for v in imgs.values()):
                    if self.data_recorder and hasattr(self.data_recorder, "record_full_frame"):
                        self.data_recorder.record_full_frame({
                            "robot_pose": pose.astype(np.float64),
                            "robot_action": action.astype(np.float64),
                            "head_image": imgs["head_image"],
                            "left_arm_image": imgs["left_arm_image"],
                            "right_arm_image": imgs["right_arm_image"]
                        })

                # 3. 更新上一帧为当前帧（为下一次兜底做准备）
                self.last_action = self.current_action
                self.last_pose = self.current_pose
                self.last_imgs = {k: v for k, v in self.current_imgs.items()}
                # 清空当前帧（等待下一次update）
                self.current_action = None
                self.current_pose = None
                self.current_imgs = {k: None for k in self.current_imgs}

            # 4. 精准控制10Hz频率
            elapsed = time.time() - start_ts
            time.sleep(max(0, self.interval - elapsed))
        end_time = time.time()
        # print(f"{BLUE}collect loop elapsed: {end_time - start_time:.6f}{RESET}")

    # ========== 极简清理逻辑 ==========
    def cleanup(self):
        print("[FrameCollector] 停止采集...")
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        print("[FrameCollector] 清理完成")

# ===================== 极简测试代码 =====================
if __name__ == "__main__":
    print("="*50 + " 极简版测试 " + "="*50)
    SAMPLE_NUM = 160  # 总帧数
    SAMPLE_FREQ = 10  # 10Hz

    # 1. 初始化Recorder
    data_recorder = DataRecorder()
    if not data_recorder.start_recording():
        print("❌ Recorder启动失败")
        exit(1)

    # 2. 初始化极简收集器
    collector = FrameCollector(freq=SAMPLE_FREQ, data_recorder=data_recorder)

    # 3. 模拟数据推送（仅更新当前帧，无复杂idx/ts）
    def sim_data():
        for i in range(SAMPLE_NUM):
            # 推送动作/位姿/图像（当前帧）
            collector.update_action(np.random.rand(26), idx=i, ts=i/SAMPLE_FREQ)
            collector.update_pose(np.random.rand(26), idx=i, ts=i/SAMPLE_FREQ)
            collector.update_image("head_image", np.random.randint(0,255,(480,640,4),np.uint8).tobytes(), idx=i, ts=i/SAMPLE_FREQ)
            collector.update_image("left_arm_image", np.random.randint(0,255,(480,640,4),np.uint8).tobytes(), idx=i, ts=i/SAMPLE_FREQ)
            collector.update_image("right_arm_image", np.random.randint(0,255,(480,640,4),np.uint8).tobytes(), idx=i, ts=i/SAMPLE_FREQ)
            time.sleep(1/SAMPLE_FREQ)  # 10Hz

    # 启动模拟线程
    threading.Thread(target=sim_data, daemon=True).start()

    # 4. 等待采集完成
    print(f"\n⏳ 采集{int(SAMPLE_NUM/SAMPLE_FREQ)}秒...")
    time.sleep(SAMPLE_NUM/SAMPLE_FREQ + 0.5)  # 加0.5秒确保最后一帧处理

    # 5. 停止采集+保存
    print("\n🛑 停止采集")
    collector.cleanup()
    data_recorder.stop_recording()
    data_recorder.cleanup()

    # 6. 验证文件
    test_file = "/home/rossum/chenjx/mcl/vr_control_data_record/datasets/episode_0.hdf5"
    if os.path.exists(test_file):
        with h5py.File(test_file, "r") as f:
            print(f"\n✅ 测试成功 | 实际帧数：{f['robot_action'].shape[0]}（预期：{SAMPLE_NUM}）")
    else:
        print("\n❌ 测试失败：文件未生成")

    print("="*50 + " 测试结束 " + "="*50)
