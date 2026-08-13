import sys
import os
import time
import csv
import json
import traceback
import tempfile
import datetime
import math
import numpy as np
import torch
from collections import deque
from queue import Queue
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from obs import ObsClient

# ================= 深度学习模型配置区 =================
# 项目根目录（本脚本所在目录），改为相对路径以方便移植
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(_BASE_DIR, "模型参数")

# ================= OBS 配置区 =================
# 【重要】请填入你自己的华为云 OBS 凭证，切勿把真实 AK/SK 提交到公开仓库
AK = "YOUR_ACCESS_KEY".strip()
SK = "YOUR_SECRET_KEY".strip()
BUCKET_NAME = "dtu-data-bucket".strip()
FOLDER_PREFIX = "dtu_data/"
BASE_DIR = _BASE_DIR
CSV_FILE = os.path.join(BASE_DIR, "5.18答辩演示.csv")
DATA_POINTS_PER_ROW = 23
RESET_TOLERANCE = 100
EMG_WINDOW_SIZE = 10

# ================= OBS 数据解析逻辑 =================
def sg(obj, key, default=None):
    if obj is None: return default
    if isinstance(obj, dict): return obj.get(key, default)
    return getattr(obj, key, default)

def find_raw_text(obj):
    if isinstance(obj, str): return obj.strip() if obj.strip().upper().startswith("AA") else None
    elif isinstance(obj, dict):
        for v in obj.values():
            r = find_raw_text(v)
            if r: return r
    elif isinstance(obj, list):
        for item in obj:
            r = find_raw_text(item)
            if r: return r
    return None

def parse_data(raw_str, last_timestamp):
    real_text = ""
    try:
        data = json.loads(raw_str)
        found = find_raw_text(data)
        if found: real_text = found
    except: pass
    if not real_text: real_text = raw_str.strip()
    if not real_text: return None, last_timestamp
    parts = real_text.split()
    if parts and parts[0].upper() == "AA": parts = parts[1:]
    parsed = []
    cur_last = last_timestamp
    for i in range(0, len(parts), DATA_POINTS_PER_ROW):
        group = parts[i:i + DATA_POINTS_PER_ROW]
        if len(group) < DATA_POINTS_PER_ROW: continue
        try:
            row = []
            for idx, val in enumerate(group):
                num = float(val)
                if idx in [1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15]: row.append(num / 1000.0)
                elif idx in [7, 8, 9, 16, 17, 18]: row.append(num / 100.0)
                else: row.append(num)
            ts = row[0]
            if cur_last is not None and ts < cur_last - RESET_TOLERANCE: continue
            else: cur_last = ts
            parsed.append(row)
        except ValueError: continue
    return parsed if parsed else None, cur_last

# ================= OBS 后台下载线程 =================
class OBSWorkerThread(QThread):
    log_signal = pyqtSignal(str)
    def __init__(self, data_queue, emg_rms_queue, imu1_angle_queue, imu2_angle_queue, imu1_predict_queue, pressure_queue):
        super().__init__()
        self.data_queue = data_queue
        self.emg_rms_queue = emg_rms_queue
        self.imu1_angle_queue = imu1_angle_queue
        self.imu2_angle_queue = imu2_angle_queue      # 【新增】IMU2 角度队列
        self.imu1_predict_queue = imu1_predict_queue
        self.pressure_queue = pressure_queue
        self.is_running = True
        self.last_timestamp = None
        self.emg1_history = deque(maxlen=EMG_WINDOW_SIZE)
        self.emg2_history = deque(maxlen=EMG_WINDOW_SIZE)

    def _calculate_rms(self, data):
        if not data: return 0.0
        return round(math.sqrt(sum([x ** 2 for x in data]) / len(data)), 2)

    def run(self):
        os.makedirs(BASE_DIR, exist_ok=True)
        if not os.path.exists(CSV_FILE):
            with open(CSV_FILE, "w", newline="", encoding="utf-8-sig") as f:
                f.write("Timestamp_ms,IMU1_AccX,IMU1_AccY,IMU1_AccZ,IMU1_GyroX,IMU1_GyroY,IMU1_GyroZ,IMU1_AngleX,IMU1_AngleY,IMU1_AngleZ,IMU2_AccX,IMU2_AccY,IMU2_AccZ,IMU2_GyroX,IMU2_GyroY,IMU2_GyroZ,IMU2_AngleX,IMU2_AngleY,IMU2_AngleZ,EMG1_RMS,EMG2_RMS,ADC1_Pressure,ADC2_Pressure\n")
        print("=" * 50); print("[*] OBS 后台下载线程启动..."); self.log_signal.emit("[*] OBS 线程启动...")
        obs_client = ObsClient(access_key_id=AK, secret_access_key=SK, server="https://obs.cn-north-4.myhuaweicloud.com")
        start_time_utc = datetime.datetime.utcnow()
        marker_baton = None
        resp = obs_client.listObjects(BUCKET_NAME, prefix=FOLDER_PREFIX, max_keys=1000)
        if resp.status < 300:
            contents = sg(sg(resp, 'body'), 'contents') or []
            for obj in contents:
                lm = sg(obj, 'lastModified'); lm_dt = lm if isinstance(lm, datetime.datetime) else datetime.datetime.min
                if lm_dt > start_time_utc: self._process_file(obs_client, sg(obj, 'key', ''))
                marker_baton = sg(obj, 'key', '')
        self.log_signal.emit("[*] 初始化完成，开始实时监听...")
        while self.is_running:
            try:
                resp = obs_client.listObjects(BUCKET_NAME, prefix=FOLDER_PREFIX, marker=marker_baton, max_keys=1000)
                if resp.status < 300:
                    contents = sg(sg(resp, 'body'), 'contents') or []
                    for obj in contents:
                        key = sg(obj, 'key', '')
                        if key: self._process_file(obs_client, key)
                        marker_baton = key
                time.sleep(0.2)
            except Exception as e: self.log_signal.emit(f"[❌ {e}]"); time.sleep(0.2)
        print("[*] OBS 线程停止。"); obs_client.close()

    def _process_file(self, client, file_key):
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".json"); temp_path = temp_file.name; temp_file.close()
        file_resp = client.getObject(BUCKET_NAME, file_key, downloadPath=temp_path)
        if file_resp is None or file_resp.status >= 300:
            if os.path.exists(temp_path): os.remove(temp_path)
            return
        content = ""
        try:
            with open(temp_path, 'r', encoding='utf-8') as f: content = f.read()
        finally:
            if os.path.exists(temp_path): os.remove(temp_path)
        parsed, self.last_timestamp = parse_data(content, self.last_timestamp)
        if parsed:
            with open(CSV_FILE, "a", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)
                for row in parsed:
                    emg1_bias_corrected = row[19] - 998.0; emg2_bias_corrected = row[20] - 998.0
                    self.emg1_history.append(emg1_bias_corrected); self.emg2_history.append(emg2_bias_corrected)
                    row[19] = self._calculate_rms(self.emg1_history); row[20] = self._calculate_rms(self.emg2_history)
                    writer.writerow(row)
                    self.data_queue.put(row[:7])
                    self.emg_rms_queue.put((row[0], row[19], row[20]))
                    self.imu1_angle_queue.put((row[0], row[7], row[8], row[9]))
                    # 【新增】提取 IMU2 角度 row[16,17,18] 抛给队列
                    self.imu2_angle_queue.put((row[0], row[16], row[17], row[18]))
                    self.imu1_predict_queue.put(row[1:7])
                    self.pressure_queue.put((row[0], row[21], row[22]))

    def stop(self): self.is_running = False

# ================= 实时深度学习预测后台线程 =================
class PredictWorkerThread(QThread):
    def __init__(self, imu_queue, shared_results_deque, model, imu_mean, imu_std):
        super().__init__()
        self.imu_queue = imu_queue
        self.shared_results_deque = shared_results_deque
        self.model = model
        self.imu_mean = imu_mean
        self.imu_std = imu_std
        self.buf = deque(maxlen=300)
        self.is_running = True
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def run(self):
        if self.model is None: return
        print("[*] 实时预测线程启动...")
        while self.is_running:
            while not self.imu_queue.empty():
                try: self.buf.append(self.imu_queue.get_nowait())
                except: break
            if len(self.buf) >= 246:
                data = np.array(list(self.buf)[-246:])
                data = (data - self.imu_mean) / self.imu_std
                data = torch.tensor(data.transpose((1, 0)), dtype=torch.float32).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    outputs = self.model(data)
                    _, predicted = torch.max(outputs, 1)
                    res_class = predicted.cpu().numpy()[0] + 1
                self.shared_results_deque.append((time.time(), res_class))
            time.sleep(0.5)

    def stop(self): self.is_running = False

# ================= UI 界面组件 =================
class MedicalCardWidget(QtWidgets.QWidget):
    def __init__(self): super().__init__(); self.initUI()
    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self); layout.setSpacing(0); layout.setContentsMargins(0, 0, 0, 0)
        medical_content = [("姓名：", "张三"), ("性别：", "男"), ("年龄：", "60岁"), ("入院时间：", "2026.3.8"), ("主治医生：", "李医生"), ("时间：", self.get_current_time())]
        for i, (label, value) in enumerate(medical_content):
            row_widget = QtWidgets.QWidget(); row_widget.setMinimumHeight(30); row_layout = QtWidgets.QHBoxLayout(row_widget); row_layout.setContentsMargins(10, 0, 10, 0)
            label_widget = QtWidgets.QLabel(label); label_widget.setStyleSheet("color: #fff; font-weight: bold;font-size: 30px;")
            value_widget = QtWidgets.QLabel(value); value_widget.setStyleSheet("color: #fff;font-size: 30px;")
            if label == "时间：": value_widget.setStyleSheet("color: #00ff88; font-weight: bold;font-size: 30px;")
            row_layout.addWidget(label_widget, 1); row_layout.addStretch(1); row_layout.addWidget(value_widget, 1)
            layout.addWidget(row_widget)
            if i < len(medical_content) - 1:
                line = QtWidgets.QFrame(); line.setFrameShape(QtWidgets.QFrame.HLine); line.setStyleSheet("background-color: rgba(255,255,255,0.1);"); line.setFixedHeight(1); layout.addWidget(line)
    def get_current_time(self): return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

class GaitTimeSeriesWidget(QtWidgets.QWidget):
    def __init__(self, shared_results_deque):
        super().__init__(); self.results_buf = shared_results_deque; self.initUI()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.update_plot); self.timer.start(1000)
    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(2, 2, 2, 2); layout.setSpacing(2)
        pg.setConfigOptions(background='#2b2b3c', foreground='w', antialias=True)
        self.pw = pg.PlotWidget(title="分类结果时序图"); self.pw.showGrid(x=True, y=True, alpha=0.2); self.pw.setYRange(0.5, 5.5); self.pw.setXRange(0, 60)
        custom_ticks = [(1, "正常步态"), (2, "剪刀步态"), (3, "拖曳步态"), (4, "冻结步态"), (5, "慌张步态")]
        self.pw.getAxis('left').setTicks([custom_ticks]); layout.addWidget(self.pw)
    def update_plot(self):
        if not self.results_buf: return
        now = time.time(); times = [t for t, _ in self.results_buf]; res = [r for _, r in self.results_buf]
        rel_times = [now - t for t in times]; filtered = [(rt, r) for rt, r in zip(rel_times, res) if rt <= 60]
        if not filtered: return
        rt, r = zip(*filtered)
        color_map = {1: '#FF6B6B', 2: '#4ECDC4', 3: '#45B7D1', 4: '#FF9F1C', 5: '#9B59B6'}
        colors = [color_map.get(val, '#FFFFFF') for val in r]
        self.pw.clear()
        self.pw.plot(rt, r, pen=None, symbol='o', symbolSize=8, symbolBrush=colors)
        self.pw.plot(rt, r, pen=pg.mkPen(color='#FFFFFF', width=2))
    def stop_timer(self): self.timer.stop()

class GaitBarWidget(QtWidgets.QWidget):
    def __init__(self, shared_results_deque):
        super().__init__(); self.results_buf = shared_results_deque; self.initUI()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.update_plot); self.timer.start(1000)
    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(2, 2, 2, 2); layout.setSpacing(2)
        pg.setConfigOptions(background='#2b2b3c', foreground='w', antialias=True)
        self.pw = pg.PlotWidget(title="病情百分比统计"); self.pw.showGrid(x=True, y=False, alpha=0.2); self.pw.setYRange(0, 100); self.pw.setXRange(0, 6)
        bar_ticks = [(1, "正常步态"), (2, "剪刀步态"), (3, "拖曳步态"), (4, "冻结步态"), (5, "慌张步态")]
        self.pw.getAxis('bottom').setTicks([bar_ticks]); layout.addWidget(self.pw)
    def update_plot(self):
        if not self.results_buf: return
        now = time.time(); recent_res = [r for t, r in self.results_buf if (now - t) <= 60]
        if not recent_res: return
        counts = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}
        for r in recent_res:
            if r in counts: counts[r] += 1
        total = sum(counts.values())
        percentages = {k: (v / total) * 100 if total > 0 else 0 for k, v in counts.items()}
        self.pw.clear()
        x = np.array([1, 2, 3, 4, 5]); y = [percentages.get(i, 0) for i in x]
        color_map = {1: '#FF6B6B', 2: '#4ECDC4', 3: '#45B7D1', 4: '#FF9F1C', 5: '#9B59B6'}
        brushes = [pg.mkBrush(color_map.get(i, '#FFFFFF')) for i in x]
        bg = pg.BarGraphItem(x=x, height=y, width=0.6, brushes=brushes); self.pw.addItem(bg)
    def stop_timer(self): self.timer.stop()

class MedicineHistoryWidget(QtWidgets.QWidget):
    def __init__(self): super().__init__(); self.initUI()
    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self); layout.setSpacing(0); layout.setContentsMargins(0, 0, 0, 0)
        for name, medicine, date in [("李一", "康复阶段", "2026.4.10"), ("李二", "治疗阶段", "2026.3.3")]:
            row = QtWidgets.QWidget(); row.setMinimumHeight(25); rl = QtWidgets.QHBoxLayout(row); rl.setContentsMargins(10, 0, 10, 0)
            for t in [name, medicine, date]: l = QtWidgets.QLabel(t); l.setStyleSheet("color: #fff;bold;font-size: 30px;"); rl.addWidget(l)
            layout.addWidget(row)

class PressurePlotWidget(QtWidgets.QWidget):
    def __init__(self, pressure_queue):
        super().__init__()
        self.pressure_queue = pressure_queue
        self.max_points = 500
        self.buf_time = deque(maxlen=self.max_points)
        self.buf_adc1 = deque(maxlen=self.max_points)
        self.buf_adc2 = deque(maxlen=self.max_points)
        self.initUI()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(30)

    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        pg.setConfigOptions(background='#2b2b3c', foreground='w', antialias=True)
        self.pw = pg.PlotWidget(title="足底压力")
        self.pw.showGrid(x=True, y=True, alpha=0.3)
        self.pw.setYRange(0, 5000)
        self.curve_adc1 = self.pw.plot(pen=pg.mkPen('#FFD700', width=2), name="ADC1", clipToView=True, autoDownsample=True)
        self.curve_adc2 = self.pw.plot(pen=pg.mkPen('#FF4500', width=2), name="ADC2", clipToView=True, autoDownsample=True)
        layout.addWidget(self.pw)

    def _tick(self):
        if self.pressure_queue.qsize() > 100:
            for _ in range(self.pressure_queue.qsize() - 50):
                try: self.pressure_queue.get_nowait()
                except: break
        got_data = False
        for _ in range(1):
            try:
                ts, adc1, adc2 = self.pressure_queue.get_nowait()
                self.buf_time.append(ts); self.buf_adc1.append(adc1); self.buf_adc2.append(adc2)
                got_data = True
            except: break
        if got_data:
            self.curve_adc1.setData(self.buf_time, self.buf_adc1)
            self.curve_adc2.setData(self.buf_time, self.buf_adc2)

    def stop_timer(self): self.timer.stop()

class EMGRmsPlotWidget(QtWidgets.QWidget):
    def __init__(self, emg_rms_queue):
        super().__init__()
        self.emg_rms_queue = emg_rms_queue
        self.max_points = 500
        self.buf_time = deque(maxlen=self.max_points)
        self.buf_emg1_rms = deque(maxlen=self.max_points)
        self.buf_emg2_rms = deque(maxlen=self.max_points)
        self.initUI()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(30)

    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        pg.setConfigOptions(background='#2b2b3c', foreground='w', antialias=True)
        self.pw_emg = pg.PlotWidget(title="肌肉活动强度")
        self.pw_emg.showGrid(x=True, y=True, alpha=0.3)
        self.curve_emg1 = self.pw_emg.plot(pen=pg.mkPen('#00ff88', width=2), name="EMG1_RMS", clipToView=True, autoDownsample=True)
        self.curve_emg2 = self.pw_emg.plot(pen=pg.mkPen('#FF9F1C', width=2), name="EMG2_RMS", clipToView=True, autoDownsample=True)
        layout.addWidget(self.pw_emg)

    def _tick(self):
        if self.emg_rms_queue.qsize() > 100:
            for _ in range(self.emg_rms_queue.qsize() - 50):
                try: self.emg_rms_queue.get_nowait()
                except: break
        got_data = False
        for _ in range(1):
            try:
                ts, rms1, rms2 = self.emg_rms_queue.get_nowait()
                self.buf_time.append(ts)
                self.buf_emg1_rms.append(rms1)
                self.buf_emg2_rms.append(rms2)
                got_data = True
            except: break
        if got_data:
            self.curve_emg1.setData(self.buf_time, self.buf_emg1_rms)
            self.curve_emg2.setData(self.buf_time, self.buf_emg2_rms)

    def stop_timer(self): self.timer.stop()

# ================= IMU1 加速度 + 角速度（移到右侧） =================
class IMU1AccGyroWidget(QtWidgets.QWidget):
    def __init__(self, data_queue):
        super().__init__(); self.data_queue = data_queue; self.max_points = 500
        self.buf_time = deque(maxlen=self.max_points); self.buf_ax = deque(maxlen=self.max_points); self.buf_ay = deque(maxlen=self.max_points); self.buf_az = deque(maxlen=self.max_points)
        self.buf_gx = deque(maxlen=self.max_points); self.buf_gy = deque(maxlen=self.max_points); self.buf_gz = deque(maxlen=self.max_points)
        self.initUI(); self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self._tick); self.timer.start(30)

    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(2, 2, 2, 2); layout.setSpacing(2)
        pg.setConfigOptions(background='#2b2b3c', foreground='w', antialias=True)
        self.pw_acc = pg.PlotWidget(title="加速度")
        self.pw_acc.showGrid(x=True, y=True, alpha=0.3); self.pw_acc.setYRange(-2, 2)
        self.curve_ax = self.pw_acc.plot(pen=pg.mkPen('r', width=2), name="Acc_X", clipToView=True, autoDownsample=True)
        self.curve_ay = self.pw_acc.plot(pen=pg.mkPen('g', width=2), name="Acc_Y", clipToView=True, autoDownsample=True)
        self.curve_az = self.pw_acc.plot(pen=pg.mkPen('b', width=2), name="Acc_Z", clipToView=True, autoDownsample=True)
        self.pw_gyro = pg.PlotWidget(title="角速度")
        self.pw_gyro.showGrid(x=True, y=True, alpha=0.3); self.pw_gyro.setYRange(-8, 8)
        self.curve_gx = self.pw_gyro.plot(pen=pg.mkPen('y', width=2), name="Gyro_X", clipToView=True, autoDownsample=True)
        self.curve_gy = self.pw_gyro.plot(pen=pg.mkPen('c', width=2), name="Gyro_Y", clipToView=True, autoDownsample=True)
        self.curve_gz = self.pw_gyro.plot(pen=pg.mkPen('m', width=2), name="Gyro_Z", clipToView=True, autoDownsample=True)
        layout.addWidget(self.pw_acc, stretch=1); layout.addWidget(self.pw_gyro, stretch=1)

    def _tick(self):
        if self.data_queue.qsize() > 100:
            for _ in range(self.data_queue.qsize() - 50):
                try: self.data_queue.get_nowait()
                except: break
        got_data = False
        for _ in range(1):
            try:
                row = self.data_queue.get_nowait()
                self.buf_time.append(row[0]); self.buf_ax.append(row[1]); self.buf_ay.append(row[2]); self.buf_az.append(row[3])
                self.buf_gx.append(row[4]); self.buf_gy.append(row[5]); self.buf_gz.append(row[6])
                got_data = True
            except: break
        if got_data:
            self.curve_ax.setData(self.buf_time, self.buf_ax); self.curve_ay.setData(self.buf_time, self.buf_ay); self.curve_az.setData(self.buf_time, self.buf_az)
            self.curve_gx.setData(self.buf_time, self.buf_gx); self.curve_gy.setData(self.buf_time, self.buf_gy); self.curve_gz.setData(self.buf_time, self.buf_gz)

    def stop_timer(self): self.timer.stop()

# ================= IMU1 三轴角度（移到中间） =================
class IMU1AnglePlotWidget(QtWidgets.QWidget):
    def __init__(self, imu1_angle_queue):
        super().__init__(); self.imu1_angle_queue = imu1_angle_queue; self.max_points = 500
        self.buf_time = deque(maxlen=self.max_points); self.buf_ax = deque(maxlen=self.max_points); self.buf_ay = deque(maxlen=self.max_points); self.buf_az = deque(maxlen=self.max_points)
        self.initUI(); self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self._tick); self.timer.start(30)

    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(2, 2, 2, 2); layout.setSpacing(2)
        pg.setConfigOptions(background='#2b2b3c', foreground='w', antialias=True)
        self.pw = pg.PlotWidget(title="脚踝姿态")
        self.pw.showGrid(x=True, y=True, alpha=0.3); self.pw.setYRange(-180, 180)
        self.curve_ax = self.pw.plot(pen=pg.mkPen('r', width=2), name="Angle_X", clipToView=True, autoDownsample=True)
        self.curve_ay = self.pw.plot(pen=pg.mkPen('g', width=2), name="Angle_Y", clipToView=True, autoDownsample=True)
        self.curve_az = self.pw.plot(pen=pg.mkPen('#00bfff', width=2), name="Angle_Z", clipToView=True, autoDownsample=True)
        layout.addWidget(self.pw)

    def _tick(self):
        if self.imu1_angle_queue.qsize() > 100:
            for _ in range(self.imu1_angle_queue.qsize() - 50):
                try: self.imu1_angle_queue.get_nowait()
                except: break
        got_data = False
        for _ in range(1):
            try:
                ts, ax, ay, az = self.imu1_angle_queue.get_nowait()
                self.buf_time.append(ts); self.buf_ax.append(ax); self.buf_ay.append(ay); self.buf_az.append(az); got_data = True
            except: break
        if got_data:
            self.curve_ax.setData(self.buf_time, self.buf_ax); self.curve_ay.setData(self.buf_time, self.buf_ay); self.curve_az.setData(self.buf_time, self.buf_az)

    def stop_timer(self): self.timer.stop()

# ================= 【新增】IMU2 三轴角度 =================
class IMU2AnglePlotWidget(QtWidgets.QWidget):
    def __init__(self, imu2_angle_queue):
        super().__init__(); self.imu2_angle_queue = imu2_angle_queue; self.max_points = 500
        self.buf_time = deque(maxlen=self.max_points); self.buf_ax = deque(maxlen=self.max_points); self.buf_ay = deque(maxlen=self.max_points); self.buf_az = deque(maxlen=self.max_points)
        self.initUI(); self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self._tick); self.timer.start(30)

    def initUI(self):
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(2, 2, 2, 2); layout.setSpacing(2)
        pg.setConfigOptions(background='#2b2b3c', foreground='w', antialias=True)
        self.pw = pg.PlotWidget(title="足部姿态")
        self.pw.showGrid(x=True, y=True, alpha=0.3); self.pw.setYRange(-180, 180)
        self.curve_ax = self.pw.plot(pen=pg.mkPen('r', width=2), name="Angle_X", clipToView=True, autoDownsample=True)
        self.curve_ay = self.pw.plot(pen=pg.mkPen('g', width=2), name="Angle_Y", clipToView=True, autoDownsample=True)
        self.curve_az = self.pw.plot(pen=pg.mkPen('#FF69B4', width=2), name="Angle_Z", clipToView=True, autoDownsample=True)
        layout.addWidget(self.pw)

    def _tick(self):
        if self.imu2_angle_queue.qsize() > 100:
            for _ in range(self.imu2_angle_queue.qsize() - 50):
                try: self.imu2_angle_queue.get_nowait()
                except: break
        got_data = False
        for _ in range(1):
            try:
                ts, ax, ay, az = self.imu2_angle_queue.get_nowait()
                self.buf_time.append(ts); self.buf_ax.append(ax); self.buf_ay.append(ay); self.buf_az.append(az); got_data = True
            except: break
        if got_data:
            self.curve_ax.setData(self.buf_time, self.buf_ax); self.curve_ay.setData(self.buf_time, self.buf_ay); self.curve_az.setData(self.buf_time, self.buf_az)

    def stop_timer(self): self.timer.stop()

# ================= 主窗口 =================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.data_queue = Queue(maxsize=5000)
        self.emg_rms_queue = Queue(maxsize=5000)
        self.imu1_angle_queue = Queue(maxsize=5000)
        self.imu2_angle_queue = Queue(maxsize=5000)       # 【新增】
        self.imu1_predict_queue = Queue(maxsize=5000)
        self.pressure_queue = Queue(maxsize=5000)
        self.shared_predict_results = deque(maxlen=100)
        self.initUI()
        self.obs_thread = None
        self.predict_thread = None
        self.running = False
        self.load_ai_model()

    def load_ai_model(self):
        self.model = None; self.imu_mean = None; self.imu_std = None
        try:
            from model import cnn_transformer
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.imu_mean = np.load(os.path.join(MODEL_DIR, 'imu_mean.npy'))
            self.imu_std = np.load(os.path.join(MODEL_DIR, 'imu_std.npy'))
            self.model = cnn_transformer(output_dim=5, len_channel=6, input_dim=32, hidden_dim=256, num_heads=2, dropout=0.2).to(device)
            self.model.load_state_dict(torch.load(os.path.join(MODEL_DIR, 'best_gait_model_5cls.pth'), map_location=device, weights_only=False))
            self.model.eval()
            print("[*] AI 预测模型加载成功！")
        except Exception as e:
            print(f"[❌ AI 模型加载失败: {e}]，将仅展示传感器波形。")

    def initUI(self):
        self.setWindowTitle("实时步态分类 - 华为云OBS版")
        self.resize(1600, 900)
        self.setStyleSheet("QMainWindow { background-color: #1e1e2f; } QLabel { font-size: 14px; }")
        central_widget = QtWidgets.QWidget(); self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget); main_layout.setSpacing(10); main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.addWidget(self.create_left_widget(), 3)
        main_layout.addWidget(self.create_center_widget(), 5)
        main_layout.addWidget(self.create_right_widget(), 2)
        self.statusBar().showMessage("状态：未运行")

    def create_left_widget(self):
        widget = QtWidgets.QWidget(); widget.setStyleSheet("QWidget { background-color: rgba(30, 30, 47, 0.8); border-radius: 8px; }")
        layout = QtWidgets.QVBoxLayout(widget); layout.setSpacing(10); layout.setContentsMargins(10, 10, 10, 10)
        layout.addWidget(self.create_section("病历卡", MedicalCardWidget()), 4)
        self.gait_time_series = GaitTimeSeriesWidget(self.shared_predict_results)
        layout.addWidget(self.create_section("步态类型", self.gait_time_series), 3)
        self.emg_rms_plot = EMGRmsPlotWidget(self.emg_rms_queue)
        layout.addWidget(self.create_section("表面肌电信号", self.emg_rms_plot), 3)
        return widget

    def create_center_widget(self):
        widget = QtWidgets.QWidget(); widget.setStyleSheet("QWidget { background-color: rgba(30, 30, 47, 0.8); border-radius: 8px; }")
        layout = QtWidgets.QVBoxLayout(widget); layout.setSpacing(10); layout.setContentsMargins(10, 10, 10, 10)
        self.gait_bar = GaitBarWidget(self.shared_predict_results)
        layout.addWidget(self.create_section("病情统计", self.gait_bar), 4)
        control_widget = self.create_control_widget(); control_widget.setFixedHeight(60); layout.addWidget(control_widget)
        # 【修改】中间区域改为：IMU1 角度 + IMU2 角度
        self.imu1_angle_plot = IMU1AnglePlotWidget(self.imu1_angle_queue)
        layout.addWidget(self.create_section("脚踝姿态", self.imu1_angle_plot), 3)
        self.imu2_angle_plot = IMU2AnglePlotWidget(self.imu2_angle_queue)
        layout.addWidget(self.create_section("足部姿态", self.imu2_angle_plot), 3)
        return widget

    def create_right_widget(self):
        widget = QtWidgets.QWidget(); widget.setStyleSheet("QWidget { background-color: rgba(30, 30, 47, 0.8); border-radius: 8px; }")
        layout = QtWidgets.QVBoxLayout(widget); layout.setSpacing(10); layout.setContentsMargins(10, 10, 10, 10)
        layout.addWidget(self.create_section("治疗阶段", MedicineHistoryWidget()), 4)
        # 【修改】右侧改为：IMU1 加速度 + 角速度
        self.imu1_acc_gyro_plot = IMU1AccGyroWidget(self.data_queue)
        layout.addWidget(self.create_section("加速度 角速度", self.imu1_acc_gyro_plot), 3)
        self.pressure_plot = PressurePlotWidget(self.pressure_queue)
        layout.addWidget(self.create_section("足底压力", self.pressure_plot), 3)
        return widget

    def create_section(self, title, content_widget):
        widget = QtWidgets.QWidget(); widget.setStyleSheet("QWidget { background-color: rgba(43, 43, 60, 0.8); border-radius: 6px; }")
        layout = QtWidgets.QVBoxLayout(widget); layout.setContentsMargins(0, 0, 0, 0); layout.setSpacing(0)
        if title:
            tw = QtWidgets.QWidget(); tw.setFixedHeight(30); tw.setStyleSheet("QWidget { background-color: rgba(0, 0, 0, 0.3); border-top-left-radius: 6px; border-top-right-radius: 6px; }")
            tl = QtWidgets.QHBoxLayout(tw); tl.setContentsMargins(10, 0, 10, 0)
            tlbl = QtWidgets.QLabel(title); tlbl.setStyleSheet("color: #fff; font-weight: bold;font-size: 25px;"); tl.addWidget(tlbl); tl.addStretch()
            layout.addWidget(tw)
        content_widget.setStyleSheet("QWidget { background-color: transparent; }")
        layout.addWidget(content_widget)
        return widget

    def create_control_widget(self):
        widget = QtWidgets.QWidget(); layout = QtWidgets.QHBoxLayout(widget); layout.setContentsMargins(10, 10, 10, 10)
        self.run_button = QtWidgets.QPushButton("开始实时接收")
        self.run_button.setStyleSheet("QPushButton { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #667eea, stop:1 #764ba2); color: white; border: 1px solid rgba(255, 255, 255, 0.3); border-radius: 5px; padding: 10px 20px; font-weight: bold; min-height: 20px; } QPushButton:hover { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #764ba2, stop:1 #667eea); }")
        self.run_button.clicked.connect(self.toggle_running)
        layout.addWidget(self.run_button)
        return widget

    def handle_log(self, msg): print(f"[UI] {msg}"); self.statusBar().showMessage(msg)

    def toggle_running(self):
        if not self.running:
            # 【修改】传入 imu2_angle_queue
            self.obs_thread = OBSWorkerThread(self.data_queue, self.emg_rms_queue, self.imu1_angle_queue, self.imu2_angle_queue, self.imu1_predict_queue, self.pressure_queue)
            self.obs_thread.log_signal.connect(self.handle_log)
            self.obs_thread.start()
            if self.model is not None:
                self.predict_thread = PredictWorkerThread(self.imu1_predict_queue, self.shared_predict_results, self.model, self.imu_mean, self.imu_std)
                self.predict_thread.start()
            self.running = True; self.run_button.setText("停止接收")
        else:
            if self.obs_thread: self.obs_thread.stop(); self.obs_thread.wait()
            if self.predict_thread: self.predict_thread.stop(); self.predict_thread.wait()
            self.running = False; self.run_button.setText("开始实时接收")

    def closeEvent(self, event):
        if self.obs_thread: self.obs_thread.stop(); self.obs_thread.wait()
        if self.predict_thread: self.predict_thread.stop(); self.predict_thread.wait()
        self.imu1_acc_gyro_plot.stop_timer()
        self.emg_rms_plot.stop_timer()
        self.imu1_angle_plot.stop_timer()
        self.imu2_angle_plot.stop_timer()  # 【新增】
        self.gait_time_series.stop_timer(); self.gait_bar.stop_timer()
        self.pressure_plot.stop_timer()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    font = QtGui.QFont("Microsoft YaHei", 8); app.setFont(font)
    main_window = MainWindow(); main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__': main()
