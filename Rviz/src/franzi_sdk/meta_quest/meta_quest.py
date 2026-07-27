import numpy as np
import time
import os
from collections import deque
#from oculus_reader.oculus_reader.reader import OculusReader
from .oculus_reader.oculus_reader.reader import OculusReader
from scipy.spatial.transform import Rotation as R  # 用于旋转矩阵转欧拉角

class MetaQuest:
    def __init__(self):
        # 初始化VR阅读器
        try:
            self.reader = OculusReader()
        except Exception as e:
            raise RuntimeError(f"初始化失败: {e}") from e

        # 参考位姿缓存（恢复头显h，+手柄l/r/L/R）
        # 键说明：h=头显 l=左手柄（头显坐标系） r=右手柄（头显坐标系） L=左手柄（世界坐标系） R=右手柄（世界坐标系）
        self.ref_pose = {
            "h": None,    # 头显参考位姿（用于头显增量计算）
            "l": None,    # 左手柄-头显坐标系参考位姿
            "r": None,    # 右手柄-头显坐标系参考位姿
            "L": None,    # 左手柄-世界坐标系参考位姿
            "R": None     # 右手柄-世界坐标系参考位姿
        }

        # 原始数据缓存
        self.transforms = None
        self.buttons = None

        # 坐标变换配置（拆分手柄的头显/世界坐标系，增加头显变换矩阵）
        self.translation_scaling_factor = 1.0    # 平移缩放系数（可调整减小增量幅度）
        self.rotation_smoothing_threshold = 0.1  # 旋转增量滤波阈值（小于该值设为0，减少抖动）
        self.translation_smoothing_threshold = 0.001  # 平移增量滤波阈值（米）
        
        # 头显变换矩阵（头部控制专用）
        self.head_Trm = np.eye(4)                # 头显坐标变换矩阵
        # 左臂变换矩阵（区分头显/世界坐标系）
        self.left_arm_head_Trm = np.eye(4)       # 左臂-头显坐标系变换矩阵
        # self.left_arm_world_Trm = np.eye(4)      # 左臂-世界坐标系变换矩阵
        """
        self.left_arm_world_Trm = np.array([
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])      # 左臂-世界坐标系变换矩阵 VR与机械臂反向
        """
        self.left_arm_world_Trm = np.array([
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1]
        ])      # 左臂-世界坐标系变换矩阵  VR与机械臂同向向
        
        # 右臂变换矩阵（区分头显/世界坐标系）
        self.right_arm_head_Trm = np.eye(4)      # 右臂-头显坐标系变换矩阵
        #self.right_arm_world_Trm = np.eye(4)     # 右臂-世界坐标系变换矩阵
        """
        self.right_arm_world_Trm = np.array([
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])      # 左臂-世界坐标系变换矩阵 VR与机械臂反向
        """
        self.right_arm_world_Trm = np.array([
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1]
        ])      # 右臂-世界坐标系变换矩阵  VR与机械臂同向向

    def update(self):
        """更新VR原始数据（位姿+按键）"""
        try:
            self.transforms, self.buttons = self.reader.get_transformations_and_buttons()
        except Exception as e:
            self.transforms, self.buttons = None, None
            print(f"数据更新异常: {e}")

    # -------------------------- 通用工具函数 --------------------------
    def _get_btn(self, key, default=None, is_bool=False):
        """
        通用按键/遥杆获取（兼容tuple/list/np.ndarray/数值类型）
        :param key: 按键名
        :param default: 默认值
        :param is_bool: 是否为布尔型按键（X/Y/A/B）
        :return: 布尔值（bool键）/ 列表（数值键：LG/RG/LTr/RTr/遥杆）
        """
        if self.buttons is None:
            return False if is_bool else ([0.0] if default is None else default)
        
        val = self.buttons.get(key)
        if val is None:
            return False if is_bool else ([0.0] if default is None else default)
        
        if is_bool:
            return bool(val)
        else:
            # 统一转为float列表，兼容tuple（遥杆）/list/np.ndarray
            if isinstance(val, (list, tuple, np.ndarray)):
                return [float(v) for v in val]
            else:
                return [float(val)]

    def _matrix_to_cartesian(self, mat, degrees=True):
        """
        通用4x4矩阵转笛卡尔坐标（x,y,z,roll,pitch,yaw）
        :param mat: 4x4齐次位姿矩阵
        :param degrees: 欧拉角是否返回角度（默认True）
        :return: (x, y, z, roll, pitch, yaw) 笛卡尔坐标（x/y/z单位：米）
        """
        # 确保输入是有效矩阵
        if mat is None or not isinstance(mat, np.ndarray):
            mat = np.eye(4)
        
        # 平移分量（米）
        x, y, z = mat[0, 3], mat[1, 3], mat[2, 3]
        # 旋转矩阵转欧拉角（zyx顺序，匹配机械臂常用定义）
        rot_mat = mat[:3, :3]
        rotation = R.from_matrix(rot_mat)
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=degrees)
        return np.array([x, y, z, roll, pitch, yaw], dtype=float)

    def _get_raw_pose_matrix(self, pose_key):
        """获取原始4x4位姿矩阵（支持h/l/r/L/R），确保返回有效矩阵"""
        if pose_key not in ["h", "l", "r", "L", "R"]:
            return np.eye(4)
        # 确保transforms非空，且返回的是拷贝（避免原数据被修改）
        if self.transforms is None:
            return np.eye(4)
        raw_mat = self.transforms.get(pose_key, np.eye(4))
        return raw_mat.copy() if isinstance(raw_mat, np.ndarray) else np.eye(4)

    def _get_transform_matrix(self, pose_key):
        """根据pose_key匹配坐标变换矩阵（区分头显/手柄的头显/世界坐标系）"""
        if pose_key == "h":  # 头显
            return self.head_Trm
        elif pose_key == "l":  # 左手柄-头显坐标系
            return self.left_arm_head_Trm
        elif pose_key == "L":  # 左手柄-世界坐标系
            return self.left_arm_world_Trm
        elif pose_key == "r":  # 右手柄-头显坐标系
            return self.right_arm_head_Trm
        elif pose_key == "R":  # 右手柄-世界坐标系
            return self.right_arm_world_Trm
        else:  # 其他（无变换）
            return np.eye(4)

    def _get_cartesian_increment(self, pose_key, trigger_check):
        """
        通用笛卡尔增量计算（头显/手柄累计增量：实时位姿 - 按键按下时刻基准位姿）
        :param pose_key: 位姿键（支持h/l/r/L/R）
        :param trigger_check: 按键触发检查函数（如self.is_y_pressed/self.is_l_trig_pressed）
        :return: (dx, dy, dz, dr, dp, dyaw) 笛卡尔增量（dx/dy/dz：米；dr/dp/dyaw：度）
        """
        # 1. 按键未按下时：重置基准位姿，返回0增量
        if not trigger_check():
            self.ref_pose[pose_key] = None
            return np.zeros((6,), dtype=float)

        # 2. 获取当前位姿矩阵（确保有效）
        current_mat = self._get_raw_pose_matrix(pose_key)

        # 3. 首次按下按键：记录当前位姿作为基准位姿，返回0增量
        if self.ref_pose[pose_key] is None:
            self.ref_pose[pose_key] = current_mat.copy()
            return np.zeros((6,), dtype=float)

        # 4. 获取基准位姿（按键按下时刻的位姿），校验有效性
        ref_mat = self.ref_pose[pose_key]
        if ref_mat is None or not isinstance(ref_mat, np.ndarray):
            self.ref_pose[pose_key] = current_mat.copy()
            return np.zeros((6,), dtype=float)

        # 5. 获取变换矩阵
        Trm = self._get_transform_matrix(pose_key)

        # 6. 计算平移增量（实时位姿 - 基准位姿，累计增量）
        delta_pose = current_mat[:3, 3] - ref_mat[:3, 3]
        delta_pos_homog = np.append(delta_pose, 1)
        delta_pos_transformed = (Trm @ delta_pos_homog) * self.translation_scaling_factor
        dx, dy, dz = delta_pos_transformed[:3]

        # 7. 平移增量滤波（小值置0，减少抖动）
        dx = 0.0 if abs(dx) < self.translation_smoothing_threshold else dx
        dy = 0.0 if abs(dy) < self.translation_smoothing_threshold else dy
        dz = 0.0 if abs(dz) < self.translation_smoothing_threshold else dz

        # 8. 计算旋转增量（实时位姿 - 基准位姿，累计增量）
        delta_rot = current_mat[:3, :3] @ np.linalg.inv(ref_mat[:3, :3])
        delta_rot_homog = np.eye(4)
        delta_rot_homog[:3, :3] = delta_rot
        delta_rot_transformed = Trm @ delta_rot_homog @ np.linalg.inv(Trm)
        
        # 旋转矩阵转欧拉角（度）
        rotation = R.from_matrix(delta_rot_transformed[:3, :3])
        dr, dp, dyaw = rotation.as_euler('zyx', degrees=True)

        # 9. 旋转增量滤波（小值置0，减少抖动）
        dr = 0.0 if abs(dr) < self.rotation_smoothing_threshold else dr
        dp = 0.0 if abs(dp) < self.rotation_smoothing_threshold else dp
        dyaw = 0.0 if abs(dyaw) < self.rotation_smoothing_threshold else dyaw

        return np.array([dx, dy, dz, dr, dp, dyaw], dtype=float)


    def _get_cartesian_increment_R(self, pose_key, trigger_check):
        """
        通用笛卡尔增量计算（头显/手柄累计增量：实时位姿 - 按键按下时刻基准位姿）
        :param pose_key: 位姿键（支持h/l/r/L/R）
        :param trigger_check: 按键触发检查函数（如self.is_y_pressed/self.is_l_trig_pressed）
        :return: (dx, dy, dz, dr, dp, dyaw) 笛卡尔增量（dx/dy/dz：米；dr/dp/dyaw：度）
        """
        # 1. 按键未按下时：重置基准位姿，返回0增量
        if not trigger_check():
            self.ref_pose[pose_key] = None
            return np.zeros((6,), dtype=float)

        # 2. 获取当前位姿矩阵（确保有效）
        current_mat = self._get_raw_pose_matrix(pose_key)

        # 3. 首次按下按键：记录当前位姿作为基准位姿，返回0增量
        if self.ref_pose[pose_key] is None:
            self.ref_pose[pose_key] = current_mat.copy()
            return np.zeros((6,), dtype=float)

        # 4. 获取基准位姿（按键按下时刻的位姿），校验有效性
        ref_mat = self.ref_pose[pose_key]
        if ref_mat is None or not isinstance(ref_mat, np.ndarray):
            self.ref_pose[pose_key] = current_mat.copy()
            return np.zeros((6,), dtype=float)

        # 5. 获取变换矩阵
        Trm = self._get_transform_matrix(pose_key)

        # 6. 计算平移增量（实时位姿 - 基准位姿，累计增量）
        delta_pose = current_mat[:3, 3] - ref_mat[:3, 3]
        delta_pos_homog = np.append(delta_pose, 1)
        delta_pos_transformed = (Trm @ delta_pos_homog) * self.translation_scaling_factor
        dx, dy, dz = delta_pos_transformed[:3]

        # 7. 平移增量滤波（小值置0，减少抖动）
        dx = 0.0 if abs(dx) < self.translation_smoothing_threshold else dx
        dy = 0.0 if abs(dy) < self.translation_smoothing_threshold else dy
        dz = 0.0 if abs(dz) < self.translation_smoothing_threshold else dz

        # 8. 计算旋转增量（实时位姿 - 基准位姿，累计增量）
        delta_rot = current_mat[:3, :3] @ np.linalg.inv(ref_mat[:3, :3])
        delta_rot_homog = np.eye(4)
        delta_rot_homog[:3, :3] = delta_rot
        delta_rot_transformed = Trm @ delta_rot_homog @ np.linalg.inv(Trm)
        
        rotation = R.from_matrix(delta_rot_transformed[:3, :3])
        #print(f"rotation {type(rotation)}")
        #print(f"rotation {rotation.as_matrix()}")
        return rotation

    def _get_cartesian_increment_R_no_need_pressed(self, pose_key):
        """
        通用笛卡尔增量计算（头显/手柄累计增量：实时位姿 - 按键按下时刻基准位姿）
        :param pose_key: 位姿键（支持h/l/r/L/R）
        :return: (dx, dy, dz, dr, dp, dyaw) 笛卡尔增量（dx/dy/dz：米；dr/dp/dyaw：度）
        """
        # 2. 获取当前位姿矩阵（确保有效）
        current_mat = self._get_raw_pose_matrix(pose_key)

        # 3. 首次按下按键：记录当前位姿作为基准位姿，返回0增量
        if self.ref_pose[pose_key] is None:
            self.ref_pose[pose_key] = current_mat.copy()
            return np.zeros((6,), dtype=float)

        # 4. 获取基准位姿（按键按下时刻的位姿），校验有效性
        ref_mat = self.ref_pose[pose_key]
        if ref_mat is None or not isinstance(ref_mat, np.ndarray):
            self.ref_pose[pose_key] = current_mat.copy()
            return np.zeros((6,), dtype=float)

        # 5. 获取变换矩阵
        Trm = self._get_transform_matrix(pose_key)

        # 6. 计算平移增量（实时位姿 - 基准位姿，累计增量）
        delta_pose = current_mat[:3, 3] - ref_mat[:3, 3]
        delta_pos_homog = np.append(delta_pose, 1)
        delta_pos_transformed = (Trm @ delta_pos_homog) * self.translation_scaling_factor
        dx, dy, dz = delta_pos_transformed[:3]

        # 7. 平移增量滤波（小值置0，减少抖动）
        dx = 0.0 if abs(dx) < self.translation_smoothing_threshold else dx
        dy = 0.0 if abs(dy) < self.translation_smoothing_threshold else dy
        dz = 0.0 if abs(dz) < self.translation_smoothing_threshold else dz

        # 8. 计算旋转增量（实时位姿 - 基准位姿，累计增量）
        delta_rot = current_mat[:3, :3] @ np.linalg.inv(ref_mat[:3, :3])
        delta_rot_homog = np.eye(4)
        delta_rot_homog[:3, :3] = delta_rot
        delta_rot_transformed = Trm @ delta_rot_homog @ np.linalg.inv(Trm)
        
        rotation = R.from_matrix(delta_rot_transformed[:3, :3])
        #print(f"rotation {type(rotation)}")
        #print(f"rotation {rotation.as_matrix()}")
        return rotation


    # -------------------------- 按键状态接口（规范化命名） --------------------------
    def is_x_pressed(self):
        """X键按下状态"""
        return self._get_btn("X", is_bool=True)

    def is_y_pressed(self):
        """Y键按下状态（头显增量触发）"""
        return self._get_btn("Y", is_bool=True)

    def is_a_pressed(self):
        """A键按下状态"""
        return self._get_btn("A", is_bool=True)

    def is_b_pressed(self):
        """B键按下状态"""
        return self._get_btn("B", is_bool=True)

    def is_l_trig_pressed(self):
        """左握把（LT）按下状态"""
        return self._get_btn("LG")[0]

    def is_r_trig_pressed(self):
        """右握把（RT）按下状态"""
        return self._get_btn("RG")[0]

    # -------------------------- 夹爪状态接口 --------------------------
    def get_left_gripper(self):
        """左夹爪值（0~1）"""
        return self._get_btn("LTr")[0]

    def get_right_gripper(self):
        """右夹爪值（0~1）"""
        return self._get_btn("RTr")[0]

    # -------------------------- 摇杆接口（头/底盘控制专用） --------------------------
    def get_left_joystick(self):
        """左手摇杆值（X/Y：-1~1）→ 底盘控制专用"""
        js_val = self._get_btn("leftJS", default=[0.0, 0.0])
        return (js_val[0], js_val[1])

    def get_right_joystick(self):
        """右手摇杆值（X/Y：-1~1）→ 头部控制辅助"""
        js_val = self._get_btn("rightJS", default=[0.0, 0.0])
        return (js_val[0], js_val[1])

    # joystick是摇杆按下状态，VR中的key是"LJ"和"RJ"，控制数据删除逻辑所以返回bool类型,--预留--
    def is_l_joystick_pressed(self):
        return self._get_btn("LJ", is_bool=True)

    def is_r_joystick_pressed(self):
        return self._get_btn("RJ", is_bool=True)

    # -------------------------- 原始位姿接口 --------------------------
    def get_head_pose(self):
        """头显原始位姿（x/y/z：米；roll/pitch/yaw：度）"""
        return self._matrix_to_cartesian(self._get_raw_pose_matrix("h"))

    def get_left_arm_pose(self):
        """左手柄原始位姿（头显坐标系）"""
        return self._matrix_to_cartesian(self._get_raw_pose_matrix("l"))

    def get_right_arm_pose(self):
        """右手柄原始位姿（头显坐标系）"""
        return self._matrix_to_cartesian(self._get_raw_pose_matrix("r"))

    def get_left_arm_world_pose(self):
        """左手柄原始位姿（世界坐标系）"""
        return self._matrix_to_cartesian(self._get_raw_pose_matrix("L"))

    def get_right_arm_world_pose(self):
        """右手柄原始位姿（世界坐标系）"""
        return self._matrix_to_cartesian(self._get_raw_pose_matrix("R"))

    # -------------------------- 增量接口（头显+手柄） --------------------------
    def get_head_increment(self):
        """头显增量（X键按下时计算帧间增量，用于身体控制）"""
        return self._get_cartesian_increment("h", self.is_x_pressed)

    def get_left_arm_increment(self):
        """左手柄增量（头显坐标系，LG按下时计算帧间增量）"""
        return self._get_cartesian_increment("l", self.is_x_pressed)

    def get_right_arm_increment(self):
        """右手柄增量（头显坐标系，RG按下时计算帧间增量）"""
        return self._get_cartesian_increment("r", self.is_x_pressed)

    def get_left_arm_world_increment(self):
        """左手柄增量（世界坐标系，LG按下时计算帧间增量）"""
        return self._get_cartesian_increment("L", self.is_x_pressed)

    def get_right_arm_world_increment(self):
        """右手柄增量（世界坐标系，RG按下时计算帧间增量）"""
        return self._get_cartesian_increment("R", self.is_x_pressed)

    def get_left_arm_world_increment_R(self):
        """左手柄增量（世界坐标系，LG按下时计算帧间增量）"""
        return self._get_cartesian_increment_R("L", self.is_x_pressed)

    def get_right_arm_world_increment_R(self):
        """右手柄增量（世界坐标系，RG按下时计算帧间增量）"""
        return self._get_cartesian_increment_R("R", self.is_x_pressed)
    
    def get_left_arm_world_increment_R_no_need_pressed(self):
        """左手柄增量（世界坐标系，LG按下时计算帧间增量）"""
        return self._get_cartesian_increment_R_no_need_pressed("L")

    def get_right_arm_world_increment_R_no_need_pressed(self):
        """右手柄增量（世界坐标系，RG按下时计算帧间增量）"""
        return self._get_cartesian_increment_R_no_need_pressed("R")


# -------------------------- 测试打印函数（含帧率可视化） --------------------------
def print_vr_data(quest, target_fps: float = 20.0):
    """
    格式化打印VR数据（单位：平移→厘米；旋转→度），新增帧率可视化
    核心逻辑：
    - 头部控制：Y键按下时头显帧间增量 + 右手摇杆辅助
    - 底盘控制：左手摇杆（无增量）
    - 双臂控制：LG/RG按下时，手柄帧间增量（头显/世界坐标系）
    新增帧率可视化：
    - 即时FPS（最近30帧滑动平均）、平均FPS、目标FPS、单次循环耗时
    """
    # ========== 帧率统计（闭包+滑动窗口，避免全局变量，保证线程安全） ==========
    # 初始化帧率统计变量（仅首次调用时执行）
    if not hasattr(print_vr_data, "frame_count"):
        print_vr_data.frame_count = 0  # 总帧数
        print_vr_data.start_time = time.monotonic()  # 总开始时间
        print_vr_data.last_times = deque(maxlen=30)  # 最近30帧的时间戳（滑动窗口）
        print_vr_data.target_fps = target_fps  # 目标帧率
        print_vr_data.target_cycle = 1.0 / target_fps  # 目标单次循环耗时（秒）
    
    # 1. 更新帧率统计
    current_time = time.monotonic()
    print_vr_data.last_times.append(current_time)
    print_vr_data.frame_count += 1

    # 2. 计算帧率指标
    # 平均FPS（总帧数/总耗时）
    total_elapsed = current_time - print_vr_data.start_time
    avg_fps = print_vr_data.frame_count / total_elapsed if total_elapsed > 0 else 0.0
    # 即时FPS（最近30帧滑动平均）
    if len(print_vr_data.last_times) >= 2:
        instant_fps = len(print_vr_data.last_times) / (print_vr_data.last_times[-1] - print_vr_data.last_times[0])
        # 单次循环耗时（毫秒）
        cycle_elapsed = (print_vr_data.last_times[-1] - print_vr_data.last_times[-2]) * 1000
    else:
        instant_fps = 0.0
        cycle_elapsed = 0.0
    # 帧率误差（百分比）
    fps_error = abs(instant_fps - print_vr_data.target_fps) / print_vr_data.target_fps * 100 if print_vr_data.target_fps > 0 else 0.0

    # ========== 清屏+基础打印 ==========
    os.system('cls' if os.name == 'nt' else 'clear')
    
    # 1. 基础状态（新增帧率信息）
    print("="*90)
    print(f"目标帧率: {print_vr_data.target_fps:.1f}Hz (目标耗时: {print_vr_data.target_cycle*1000:.1f}ms)")
    print(f"            即时FPS: {instant_fps:.1f}Hz | 平均FPS: {avg_fps:.1f}Hz | "
          f"单次耗时: {cycle_elapsed:.1f}ms | 帧率误差: {fps_error:.1f}%")
    print("="*90)

    # 2. 按键按下状态
    print("\n【按键按下状态】")
    trigger_buttons = [
        ("X键", quest.is_x_pressed()),
        ("Y键（头显增量触发）", quest.is_y_pressed()),
        ("A键（底盘控制触发）", quest.is_a_pressed()),
        ("B键（记录切换）", quest.is_b_pressed()),
        ("LT握把（左臂控制）", quest.is_l_trig_pressed()),
        ("RT握把（右臂控制）", quest.is_r_trig_pressed()),
    ]
    for name, state in trigger_buttons:
        print(f"  {name:<20}: {'按下' if state else '松开'}")

    # 3. 夹爪状态
    print("\n【夹爪状态】（0.00~1.00）")
    left_grip = quest.get_left_gripper()
    right_grip = quest.get_right_gripper()
    print(f"  左夹爪值: {left_grip:.2f}")
    print(f"  右夹爪值: {right_grip:.2f}")

    # 4. 摇杆数据（头/底盘控制专用）
    print("\n【摇杆数据】（X/Y范围：-1.0~1.0）")
    left_js = quest.get_left_joystick()
    right_js = quest.get_right_joystick()
    print(f"  左手摇杆（底盘控制）: X={left_js[0]:.2f}, Y={left_js[1]:.2f}")
    print(f"  右手摇杆（头部辅助）: X={right_js[0]:.2f}, Y={right_js[1]:.2f}")

    # 辅助打印函数（单位转换：米→厘米，增加异常兜底）
    def print_cartesian(name, pose, is_increment=False):
        # 兜底：确保pose是有效numpy数组
        if not isinstance(pose, np.ndarray) or pose.shape != (6,):
            pose = np.zeros((6,), dtype=float)
        
        x, y, z, r, p, yaw = pose
        unit = "Δcm" if is_increment else "cm"
        x_cm = x * 100  # 米转厘米
        y_cm = y * 100
        z_cm = z * 100
        print(f"    {name}: X={x_cm:.2f}{unit}, Y={y_cm:.2f}{unit}, Z={z_cm:.2f}{unit} | Roll={r:.2f}°, Pitch={p:.2f}°, Yaw={yaw:.2f}°")

    # 5. 原始位姿
    print("\n【原始位姿】（平移：cm；旋转：°）")
    print("  ├─ 头显（头部控制参考）:")
    print_cartesian("头显", quest.get_head_pose())
    print("  ├─ 左臂（LT控制）:")
    print_cartesian("左臂-头显坐标系", quest.get_left_arm_pose())
    print_cartesian("左臂-世界坐标系", quest.get_left_arm_world_pose())
    print("  └─ 右臂（RT控制）:")
    print_cartesian("右臂-头显坐标系", quest.get_right_arm_pose())
    print_cartesian("右臂-世界坐标系", quest.get_right_arm_world_pose())

    # 6. 增量数据（头显+手柄帧间增量）
    print("\n【帧间增量】（平移：Δcm；旋转：°）")
    print("  ├─ 头显增量（Y键按下时更新）:")
    print_cartesian("头显", quest.get_head_increment(), is_increment=True)
    print("  ├─ 左臂增量（LT按下时更新）:")
    print_cartesian("左臂-头显坐标系", quest.get_left_arm_increment(), is_increment=True)
    print_cartesian("左臂-世界坐标系", quest.get_left_arm_world_increment(), is_increment=True)
    print("  └─ 右臂增量（RT按下时更新）:")
    print_cartesian("右臂-头显坐标系", quest.get_right_arm_increment(), is_increment=True)
    print_cartesian("右臂-世界坐标系", quest.get_right_arm_world_increment(), is_increment=True)

    print("\n" + "="*90)
    print("核心控制逻辑：")
    print("  → 头部控制：X键按下时头显帧间增量 + 右手摇杆辅助控制")
    print("  → 底盘控制：左手摇杆（X/Y：-1~1）直接控制（无增量）")
    print("  → 左臂控制：LT握把按下时，左手柄帧间增量（头显/世界坐标系可选）")
    print("  → 右臂控制：RT握把按下时，右手柄帧间增量（头显/世界坐标系可选）")
    print("  → 增量优化：帧间差计算+小值滤波，解决增量大/抖动问题")
    print("单位说明：平移（cm）、旋转（°）、夹爪（0~1）、遥杆（-1~1）、耗时（ms）")
    print("="*90)

def precise_sleep(dt: float, slack_time: float=0.001, time_func=time.monotonic):
    """
    Use hybrid of time.sleep and spinning to minimize jitter.
    Sleep dt - slack_time seconds first, then spin for the rest.
    """
    t_start = time_func()
    if dt > slack_time:
        time.sleep(dt - slack_time)
    t_end = t_start + dt
    while time_func() < t_end:
        pass
    return

def precise_wait(t_end: float, slack_time: float=0.001, time_func=time.monotonic):
    t_start = time_func()
    t_wait = t_end - t_start
    if t_wait > 0:
        t_sleep = t_wait - slack_time
        if t_sleep > 0:
            time.sleep(t_sleep)
        while time_func() < t_end:
            pass
    return

if __name__ == "__main__":
    # 1. 配置循环频率（参数化控制，替换固定的 0.05 秒）
    TARGET_FPS = 50.0  # 目标刷新频率：50Hz → 单次循环50ms（可修改为50/100Hz等）
    CYCLE_DURATION = 1.0 / TARGET_FPS  # 单次循环目标时长（秒）

    try:
        # 初始化VR设备
        quest = MetaQuest()
        print(f"MetaQuest VR设备初始化成功！")
        print(f"循环频率设置为: {TARGET_FPS}Hz (单次循环时长: {CYCLE_DURATION*1000:.1f}ms)\n")
        time.sleep(1)

        # 2. 初始化循环的「下一次目标结束时间」
        t_next = time.monotonic() + CYCLE_DURATION

        # 循环读取并打印数据（稳定频率版）
        while True:
            # 核心操作：更新VR数据 + 打印
            quest.update()
            print_vr_data(quest, target_fps=TARGET_FPS)

            # 3. 精确等待到下一个目标时间（保证循环周期稳定）
            precise_wait(t_next)
            # 更新下一次的目标结束时间
            t_next += CYCLE_DURATION

    except KeyboardInterrupt:
        print("\n\n测试结束：用户手动退出")
    except RuntimeError as e:
        print(f"\n初始化失败：{e}")
    except Exception as e:
        print(f"\n运行出错：{e}")
        import traceback
        traceback.print_exc()  # 打印详细错误栈（调试用）


### 说明：删除is_recording状态，这份代码中不处理任何逻辑，只负责对外提供VR数据
