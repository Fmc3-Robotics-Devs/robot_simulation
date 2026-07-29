import time
import numpy as np
import fastdds
from msg.AgentCmdMsg import AgentCmdMsg
from msg.AgentStateMsg import AgentStateMsg
from common.utils import RED, GREEN, YELLOW, BLUE, RESET

class AgentCmdWriterListener(fastdds.DataWriterListener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher

    def on_publication_matched(self, datawriter, info):
        if self.publisher.shutdown_event.is_set():
            return
        if 0 < info.current_count_change:
            print(f"AgentCmd发布者匹配到订阅者 {info.last_subscription_handle}")
            self.publisher.agentcmd_matched_readers += 1
            self.publisher.check_all_ready()
        else:
            print(f"AgentCmd发布者取消匹配 {info.last_subscription_handle}")
            self.publisher.agentcmd_matched_readers -= 1


class AgentStateReaderListener(fastdds.DataReaderListener):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.send_obs_idx = 0

    def on_subscription_matched(self, datareader, info):
        if self.publisher.shutdown_event.is_set():
            return
        if 0 < info.current_count_change:
            print(f"AgentState订阅者匹配到发布者 {info.last_publication_handle}")
            self.publisher.agentstate_matched_writers += 1
            self.publisher.check_all_ready()
        else:
            print(f"AgentState订阅者取消匹配 {info.last_publication_handle}")
            self.publisher.agentstate_matched_writers -= 1

    def on_data_available(self, datareader):
        if self.publisher.shutdown_event.is_set():
            return

        try:
            sample_info = fastdds.SampleInfo()
            data = AgentStateMsg.AgentStateMsg()

            ret = datareader.take_next_sample(data, sample_info)
            if ret == fastdds.RETCODE_OK and sample_info.valid_data and not self.publisher.shutdown_event.is_set():
                
                with self.publisher.agent_state_lock:
                    self.publisher.latest_agent_state = data
                    self.publisher.get_new_agent_state = True

                self.publisher.has_received_state = True

                if hasattr(self.publisher, 'frame_collector') and self.publisher.frame_collector.data_recorder.is_recording:
                    try:
                        waist_pose = data.waist_pose()
                        head_pose = data.head_pose()
                        left_pose = data.left_arm_pose()
                        right_pose = data.right_arm_pose()

                        obs_waist_pose = np.array([
                            waist_pose.x(), waist_pose.y(), waist_pose.z(),
                            waist_pose.rx(), waist_pose.ry(), waist_pose.rz()
                        ])

                        obs_head_pose = np.array([
                            head_pose.x(), head_pose.y(), head_pose.z(),
                            head_pose.rx(), head_pose.ry(), head_pose.rz()
                        ])

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

                        robot_obs_pose = np.concatenate([
                            obs_waist_pose, obs_head_pose,
                            obs_left_arm_pose, np.array([obs_left_gripper]),
                            obs_right_arm_pose, np.array([obs_right_gripper])
                        ])

                        t_time = time.time()
                        self.publisher.frame_collector.update_pose(
                            pose=robot_obs_pose,
                            idx=self.send_obs_idx,
                            ts=t_time
                        )
                        self.send_obs_idx += 1
                    except Exception as e:
                        if not self.publisher.shutdown_event.is_set():
                            print(f"记录AgentState数据时发生错误: {e}")
        except Exception as e:
            if not self.publisher.shutdown_event.is_set():
                print(f"处理AgentState消息时发生错误: {e}")
                import traceback
                traceback.print_exc()