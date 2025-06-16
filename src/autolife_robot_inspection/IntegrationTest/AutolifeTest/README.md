# AutolifeRobotInspection

```
.
├── MediumOnnx/ # 语音模型文件
│ ├── zh_CN-huayan-medium.onnx
│ └── zh_CN-huayan-medium.onnx.json
│
├── pkl/ # 预录制的控制指令文件
│ ├── command_easy.pkl
│ └── command_full2.pkl
│
├── scripts/ # 主要功能脚本
│ ├── init.py
│ ├── camera_test.py # 摄像头测试脚本
│ └── robot_reset.py # 机器人复位控制脚本
│
└── utils/ # 工具模块
├── README.md
└── test_main.py # 主测试程序
```



## 主要功能

1. **语音控制**：
   - 使用 `MediumOnnx` 中文语音模型进行语音合成
   - 支持语音状态播报和指令提示
2. **硬件控制**：
   - 摄像头测试功能（`camera_test.py`）
   - 机器人复位控制（`robot_reset.py`）
   - 多模块状态监控
3. **指令回放**：
   - 支持从 `.pkl` 文件回放预录制的控制指令

## 使用说明

1. **语音系统初始化**：
   
   ```python
   from utils import PiperVoice
   
   # 初始化语音模块
   voice = PiperVoice(
       hw_api, 
       "MediumOnnx/zh_CN-huayan-medium.onnx"
   )
   
   # 配置音频设备
   hw_api.initialize([
       'mod_microphone_a311',
       'mod_speaker_a311' 
   ])
   hw_api.speaker_set_volume(100)
   
   # 测试语音
   voice.play_voice("系统初始化完成")
   ```
   
2. **主系统启动**：

   ```python
   # 后台运行并设置实时优先级
   nohup chrt -r 99 python -u test_main.py > test.log &
   
   # 查看日志
   tail -f test.log
   ```

3. **模块独立测试**：

   ```python
   # 摄像头测试
   python scripts/camera_test.py
   
   # 机器人复位
   python scripts/robot_reset.py
   ```
   
   

