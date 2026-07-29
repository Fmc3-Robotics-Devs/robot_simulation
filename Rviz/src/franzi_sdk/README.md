包安装方式
1:conda create -n sdk_vr python==3.12
2:conda activate sdk_vr
3:pip install -e .
4:pip install numpy pure-python-adb pyyaml scipy
可以根据：environment.yml 创建conda环境
5:python data_collector/data_collector.py #开启遥操作
