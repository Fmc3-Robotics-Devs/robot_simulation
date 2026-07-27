import os
import signal
import sys

signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)

# current_dir = os.path.dirname(os.path.abspath(__file__))
# sys.path.insert(0, current_dir)

current_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(current_dir)  # 获取父目录（inobot/）
sys.path.insert(0, current_dir)
sys.path.insert(0, project_dir)  # 把项目根目录加入路径

from meta_quest.meta_quest import MetaQuest
from multi_topic_publisher import MultiTopicPublisher


def main():
    print("=" * 60)
    print("        VR 双臂遥操作")
    print("=" * 60)

    quest = None
    publisher = None

    try:
        quest = MetaQuest()
        print("MetaQuest VR 设备初始化成功！")

        publisher = MultiTopicPublisher(quest=quest)
        publisher.run()

    except KeyboardInterrupt:
        print("\n\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if publisher:
            publisher.delete()
        print("程序结束")


if __name__ == "__main__":
    main()
