# Python Client Demo

## arm_client_demo.py（SdkServer / ArmClient）

连接 INBC `SdkServerModule`，默认 **8000**。SDK 首次 import 会自动输出 `core` 日志；二开自行挂载 `logging.Handler` 承接即可。

修改脚本顶部的 `HOST`、`PORT`、`POLL_SEC` 等常量后运行：

```bash
python demo/arm_client_demo.py
```

固件需已启动 SdkServerModule（默认端口 8000）。

错误码与解决办法见 `errors.py`；建连失败或业务 `status!=0` 时 SDK 会自动打日志。
