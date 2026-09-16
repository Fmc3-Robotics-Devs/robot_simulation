# PEM Pipeline 演示稿

- `pem_pipeline.pptx`：8 页、16:9 的中文演示稿，文字和图形可编辑。
- `pem_pipeline.pdf`：由 PPTX 导出的演示版，已嵌入中文字体。

内容：流程总览、项目理解、仿真基线、迭代优化、现场验证、经验记录机制、工程师对话示例、闭环总结。

素材来源：

- `../PEM Project/Video_Stacker.mp4`：00:08、00:16 的现场参考画面。
- `../PEM Project/20260906 Measurements.pdf`：第 1 页测量图。
- `../PEM Project/`：项目文件清单，包括第二段视频、极片图纸和 `tray.stl`。
- `../renders/pem_overview.mp4`：00:15、02:20、03:45 的仿真截帧。
- 用户指定的 `Screenshot from 2026-09-14 17-40-13.png`：第 6 页记录机制截图。
- 用户指定的 `Screenshot from 2026-09-14 17-22-42.png`：第 7 页工程师对话截图。

两张用户截图以原始图片嵌入；无需依赖外部图片路径。PDF 采用静态视频截帧。
当前 PEM 演示是极片叠搬运入 tray；流程图描述目标工作方式，不表示优化接入和现场验收已经完成。详细说明保留在 PPT 讲者备注中。

导出需要 LibreOffice（本次使用 24.2.7.2）和 Noto Sans CJK SC 字体。从仓库根目录执行：

```bash
libreoffice --headless --convert-to pdf --outdir Mujoco/presentation Mujoco/presentation/pem_pipeline.pptx
pdfinfo Mujoco/presentation/pem_pipeline.pdf
pdffonts Mujoco/presentation/pem_pipeline.pdf
```

已验证：PPTX/PDF 均为 8 页、两张截图原样嵌入、PDF 字体已嵌入，并逐页检查导出的画面。
