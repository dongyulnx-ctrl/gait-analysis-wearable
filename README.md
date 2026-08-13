# 基于可穿戴多传感协同判别的步态分析系统

> 本科毕业设计（测控技术与仪器专业，2026 届）
> 一套「可穿戴多传感器采集 → 华为云 OBS → 上位机实时可视化 + CNN-Transformer 步态分类」的端到端系统。

## 项目简介

帕金森病等神经系统疾病的早期诊断中，步态特征是一项关键临床指标。本系统通过可穿戴设备同步采集**双 IMU（惯性测量单元）、双通道 EMG（表面肌电）、双足底压力**三类信号，经 STM32 打包上传至华为云对象存储（OBS），上位机实时拉取并调用深度学习模型，对 **5 类步态**进行在线分类与可视化：

| 编号 | 步态类型 |
|:---:|:---|
| 1 | 正常步态 |
| 2 | 剪刀步态 |
| 3 | 拖曳步态 |
| 4 | 冻结步态 |
| 5 | 慌张步态 |

## 系统架构

```
┌────────────────────┐    无线/串口    ┌──────────────┐    MQTT/HTTP    ┌────────────────┐
│ 可穿戴采集端        │ ──────────────► │  STM32F103   │ ──────────────► │  华为云 OBS     │
│  IMU×2 · EMG×2     │                 │ 数据打包/上传 │                 │  (对象存储)     │
│  足底压力 ADC×2     │                 └──────────────┘                 └───────┬────────┘
└────────────────────┘                                                          │ 实时拉取
                                                                                ▼
                                                                       ┌────────────────┐
                                                                       │  PyQt5 上位机   │
                                                                       │  波形可视化     │
                                                                       │  CNN-Transformer│
                                                                       │  5 类步态分类   │
                                                                       └────────────────┘
```

## 目录结构

```
gait-analysis-wearable/
├── hardware/
│   ├── embedded/           # STM32F103 嵌入式工程（Keil MDK5）
│   │   └── USART/          #   源码：BSP / CMSIS / FWLib / USER
│   └── PCB/                # 电路设计（Altium + 嘉立创EDA）
│       ├── EMG_V02.SchDoc  #   EMG 采集板原理图
│       ├── EMG_V02.PcbDoc  #   EMG 采集板 PCB
│       ├── *.epro2         #   主板工程（嘉立创EDA）
│       └── BOM_*.xlsx      #   物料清单
└── software/
    ├── huawei_shwo_v1.7.py   # 上位机主程序（连华为云 OBS 实时接收）
    ├── huawei_show_v仿真.py  # 仿真版（读取本地 CSV 回放，不连云）
    ├── 模型参数/             # 训练好的模型权重 + 归一化参数
    │   ├── best_gait_model_5cls.pth
    │   ├── imu_mean.npy
    │   └── imu_std.npy
    └── 示例数据/             # 仿真版演示用的一段正常步态数据
        └── 2026年5月13日正常步态视频测试.csv
```

## 硬件部分

- **主控**：STM32F103（标准外设库 StdPeriphLib）
- **传感器**：
  - IMU ×2：ICM-42670（六轴陀螺仪 + 加速度计，脚踝 / 足部各一）
  - 磁力计：QMC5883P（姿态解算辅助）
  - EMG ×2：表面肌电采集（蓝左 / 绿右）
  - 足底压力：薄膜压力传感器 ×2（ADC 采集）
- **开发环境**：Keil MDK5，工程入口 `hardware/embedded/USART/USER/USART.uvprojx`

核心代码位于 `USER/`（`main.c`、`fusion_send.c` 数据融合与打包、`emg_feature.c` / `imu_feature.c` 特征提取、`adc.c` 压力采集等）。

## 软件部分

- **上位机**：PyQt5 + pyqtgraph，多线程架构（OBS 下载线程 / 推理线程 / UI 刷新）
- **云端链路**：华为云 OBS Python SDK（`esdk-obs-python`），实时监听桶内新文件并解析
- **模型**：CNN-Transformer（1D CNN 特征提取 + Transformer Encoder + 全连接分类）

## 快速开始

### 依赖安装

```bash
pip install PyQt5 pyqtgraph numpy torch pandas esdk-obs-python
```

### 1. 仿真版（无需云端，最快体验）

```bash
cd software
python huawei_show_v仿真.py
```

脚本会读取 `示例数据/` 下的 CSV，按真实时间戳回放并驱动界面。

### 2. 主程序（连华为云 OBS 实时接收）

1. 编辑 `software/huawei_shwo_v1.7.py`，填入你自己的 OBS 凭证：
   ```python
   AK = "YOUR_ACCESS_KEY".strip()
   SK = "YOUR_SECRET_KEY".strip()
   BUCKET_NAME = "your-bucket-name".strip()
   ```
2. 运行：
   ```bash
   cd software
   python huawei_shwo_v1.7.py
   ```

## 注意事项

- ⚠️ **云凭证请自备**：代码中的 `AK/SK` 已替换为占位符，请填入自己的华为云 OBS 凭证，**切勿将真实密钥提交到公开仓库**。
- 📦 **关于模型**：仓库提供了训练好的权重（`best_gait_model_5cls.pth`）与归一化参数，但**模型结构定义（`model.py`）与训练代码未开源**。如需复现推理，请参考论文自行实现 `cnn_transformer` 结构后再加载权重。
- 模型缺失时，上位机仍可正常启动并展示传感器波形，仅推理功能不可用。

## 开源协议

[MIT License](LICENSE)

## 作者

董鸿儒（Hongru Dong）
