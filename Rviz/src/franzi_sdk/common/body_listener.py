import time
import numpy as np
import fastdds
from msg.BodyStateMsg import BodyState
from msg.BodyCmdAckMsg import BodyCmdAck
from common.utils import RED, GREEN, YELLOW, BLUE, RESET

class BodyCmdWriterListener(fastdds.DataWriterListener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
    
    def on_publication_matched(self, datawriter, info):
        if self.publisher.shutdown_event.is_set():
            return
        if 0 < info.current_count_change:
            print(f"BodyCmd发布者匹配到订阅者 {info.last_subscription_handle}")
            self.publisher.bodycmd_matched_readers += 1
            self.publisher.check_all_ready()
        else:
            print(f"BodyCmd发布者取消匹配 {info.last_subscription_handle}")
            self.publisher.bodycmd_matched_readers -= 1

class BodyStateAskWriterListener(fastdds.DataWriterListener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
    
    def on_publication_matched(self, datawriter, info):
        if self.publisher.shutdown_event.is_set():
            return
        if 0 < info.current_count_change:
            print(f"BodyStateAsk发布者匹配到订阅者 {info.last_subscription_handle}")
            self.publisher.bodystateask_matched_readers += 1
            self.publisher.check_all_ready()
        else:
            print(f"BodyStateAsk发布者取消匹配 {info.last_subscription_handle}")
            self.publisher.bodystateask_matched_readers -= 1

class BodyStateReaderListener(fastdds.DataReaderListener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.send_obs_idx = 0
    
    def on_subscription_matched(self, datareader, info):
        if self.publisher.shutdown_event.is_set():
            return
        if 0 < info.current_count_change:
            print(f"BodyState订阅者匹配到发布者 {info.last_publication_handle}")
            self.publisher.bodystate_matched_writers += 1
            self.publisher.check_all_ready()
        else:
            print(f"BodyState订阅者取消匹配 {info.last_publication_handle}")
            self.publisher.bodystate_matched_writers -= 1
    
    def on_data_available(self, datareader):
        if self.publisher.shutdown_event.is_set():
            return
            
        try:
            sample_info = fastdds.SampleInfo()
            data = BodyState.BodyPoseStateMsg()
            
            ret = datareader.take_next_sample(data, sample_info)
            if ret == fastdds.RETCODE_OK and sample_info.valid_data and not self.publisher.shutdown_event.is_set():
                
                left_pose = data.left_arm_pose()
                right_pose = data.right_arm_pose()
                waist_pose = data.waist_pose()
                head_pose = data.head_pose()
                # 提取身体观察数据（腰部位姿）
                obs_waist_pose = np.array([
                    waist_pose.x(), waist_pose.y(), waist_pose.z(),
                    waist_pose.rx(), waist_pose.ry(), waist_pose.rz()
                ])
                
                # 提取头部观察数据（头部位姿）
                obs_head_pose = np.array([
                    head_pose.x(), head_pose.y(), head_pose.z(),
                    head_pose.rx(), head_pose.ry(), head_pose.rz()
                ])
                
                # 提取左臂观察数据（左臂位姿和夹爪）
                obs_left_arm_pose = np.array([
                    left_pose.x(), left_pose.y(), left_pose.z(),
                    left_pose.rx(), left_pose.ry(), left_pose.rz()
                ])

                obs_right_arm_pose = np.array([
                    right_pose.x(), right_pose.y(), right_pose.z(),
                    right_pose.rx(), right_pose.ry(), right_pose.rz()
                ])
                
                obs_left_gripper = data.left_gripper_angle()
                obs_right_gripper = data.right_gripper_angle()
                #print(f"obs_left_arm_pose : {obs_left_arm_pose}")
                #print(f"obs_right_arm_pose: {obs_right_arm_pose}")
                #print(f"BodyState 订阅者接收数据:\n left gripper = {obs_left_gripper} \n right gripper = {obs_right_gripper}")

                
                if not self.publisher.control_started:
                    # print(f"正在将接收到的BodyState设置为新的基准位姿...")
                    self.publisher.set_base_poses_from_body_state(data)
                
                with self.publisher.state_lock:
                    self.publisher.latest_state = data
                    
                    if self.publisher.control_active and not self.publisher.control_started:
                        self.publisher.new_state_available = True
                        # print(f"状态已更新，准备启动控制循环")
                
                if self.publisher.frame_collector.data_recorder.is_recording:
                    robot_obs_pose = np.concatenate([
                        obs_waist_pose, obs_head_pose,
                        obs_left_arm_pose, np.array([obs_left_gripper]),
                        obs_right_arm_pose, np.array([obs_right_gripper])
                    ])
                    t_time = time.time()
                    #print(f"[BodyStateReaderListener] send obs idx={self.send_obs_idx} current time : {t_time}")
                    self.publisher.frame_collector.update_pose(
                        pose=robot_obs_pose,
                        idx=self.send_obs_idx,
                        ts=t_time
                    )
                    self.send_obs_idx += 1       
        except Exception as e:
            if not self.publisher.shutdown_event.is_set():
                print(f"处理BodyState消息时发生错误: {e}")
                import traceback
                traceback.print_exc()

class BodyCmdAckReaderListener(fastdds.DataReaderListener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
    
    def on_subscription_matched(self, datareader, info):
        if self.publisher.shutdown_event.is_set():
            return
        if 0 < info.current_count_change:
            print(f"BodyCmdAck订阅者匹配到发布者 {info.last_publication_handle}")
            self.publisher.bodycmdack_matched_writers += 1
            self.publisher.check_all_ready()
        else:
            print(f"BodyCmdAck订阅者取消匹配 {info.last_publication_handle}")
            self.publisher.bodycmdack_matched_writers -= 1
    
    def on_data_available(self, datareader):
        if self.publisher.shutdown_event.is_set():
            return
            
        try:
            sample_info = fastdds.SampleInfo()
            data = BodyCmdAck.BodyCmdAckMsg()
            
            ret = datareader.take_next_sample(data, sample_info)
            if ret == fastdds.RETCODE_OK and sample_info.valid_data and not self.publisher.shutdown_event.is_set():
                
                # 减少打印频率
                # print(f"\n[接收BodyCmdAck], 序列号: {data.sequence_number()}, 时间戳: {data.timestamp()}")
                try:
                    ack_status = data.get_cmd_ack()
                    # print(f"   确认状态: {'成功' if ack_status else '失败'}")
                except:
                    pass
                    # print("   确认状态: 未知")
                
                with self.publisher.ack_lock:
                    self.publisher.ack_received = True
                    # print(f"ACK已接收")
                    
        except Exception as e:
            if not self.publisher.shutdown_event.is_set():
                print(f"处理BodyCmdAck消息时发生错误: {e}")
                import traceback
                traceback.print_exc()
