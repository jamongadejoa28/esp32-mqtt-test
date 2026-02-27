import sys
import socket
import struct
import numpy as np
import cv2
import time
from collections import deque

import paho.mqtt.client as mqtt

has_mqtt = True

from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QTextEdit,
    QFrame,
    QPushButton,
    QLineEdit,
)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap, QIntValidator

try:
    from pyzbar.pyzbar import decode

    has_pyzbar = True
except ImportError:
    decode = None
    has_pyzbar = False

from dotenv import load_dotenv
import os

# ROOT 경로의 mcu/.env 파일을 로딩
load_dotenv(os.path.join(os.path.dirname(__file__), "../mcu/.env"))

UDP_IP = "0.0.0.0"
UDP_PORT = 8021
# Python GUI와 Mosquitto는 같은 WSL2 인스턴스에서 실행되므로 localhost 사용
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC_PUB_CONTROL = "device/control"
MQTT_TOPIC_SUB_SENSOR = "device/sensor"


class UDPReceiverThread(QThread):
    """ESP32-CAM에서 수신된 이미지 조각을 합치는 스레드"""

    image_received = pyqtSignal(np.ndarray, bool)
    log_msg = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.5)
        self.running = True
        self.image_buffer = {}

    def run(self):
        self.log_msg.emit(f"UDP 수신 서버 시작됨... (포트: {UDP_PORT})")
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
                if data.startswith(b"IMG"):
                    header_format = "<cHHH"
                    frame_type, image_id, total_chunks, chunk_index = struct.unpack(
                        header_format, data[3:10]
                    )
                    is_trigger = frame_type == b"T"
                    payload = data[10:]

                    # 새 프레임이 오면 이전 미완성 프레임의 조각들은 즉시 폐기
                    old_keys = [k for k in self.image_buffer.keys() if k != image_id]
                    for k in old_keys:
                        del self.image_buffer[k]

                    if image_id not in self.image_buffer:
                        self.image_buffer[image_id] = {
                            "chunks": {},
                            "is_trigger": is_trigger,
                            "last_update": time.time(),
                        }

                    self.image_buffer[image_id]["chunks"][chunk_index] = payload

                    # 프레임이 조립될 수 있는지 확인
                    if len(self.image_buffer[image_id]["chunks"]) == total_chunks:
                        full_img_data = b"".join(
                            self.image_buffer[image_id]["chunks"][i]
                            for i in range(total_chunks)
                        )
                        np_arr = np.frombuffer(full_img_data, np.uint8)
                        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if img is not None:
                            self.image_received.emit(
                                img, self.image_buffer[image_id]["is_trigger"]
                            )

                        del self.image_buffer[image_id]

            except socket.timeout:
                continue
            except Exception as e:
                self.log_msg.emit(f"[ERROR] UDP 통신 에러: {e}")

    def stop(self):
        self.running = False
        self.quit()
        self.wait()


class MQTTReceiverThread(QThread):
    sensor_triggered = pyqtSignal()
    dc_stopped = pyqtSignal()
    log_msg = pyqtSignal(str)

    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT):
        super().__init__()
        self.broker = broker
        self.port = port
        self.client = None
        self.running = True

    def run(self):
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:
            self.client = mqtt.Client()

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.log_msg.emit(f"Connecting MQTT Broker : {self.broker}")
            self.client.connect(self.broker, self.port, 60)
            while self.running:
                self.client.loop(0.1)
        except Exception as e:
            self.log_msg.emit(f"[ERROR] MQTT Connection failed: {e}")

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.log_msg.emit(f"[MQTT] Connected to Broker: {self.broker}")
            self.client.subscribe(MQTT_TOPIC_SUB_SENSOR)
        else:
            self.log_msg.emit(f"[MQTT] Failed to connect, rc: {rc}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8")
        if topic == MQTT_TOPIC_SUB_SENSOR:
            if payload == "DETECTED":
                self.sensor_triggered.emit()
            elif payload == "DC_STOPPED":
                self.dc_stopped.emit()

    def publish_command(self, cmd):
        if self.client and self.client.is_connected():
            self.client.publish(MQTT_TOPIC_PUB_CONTROL, cmd)

    def stop(self):
        self.running = False
        if self.client:
            self.client.disconnect()
        self.quit()
        self.wait()


class QRInventoryApp(QWidget):
    def __init__(self):
        super().__init__()

        # 인식된 QR 데이터를 큐(Queue) 형태로 저장
        self.qr_queue = deque()
        self.processed_history = []

        self.last_scan_time = 0
        self.last_scanned_qr = ""

        self.initUI()
        self.is_processing = False

        self.udp_thread = UDPReceiverThread()
        self.udp_thread.image_received.connect(self.process_image)
        self.udp_thread.log_msg.connect(self.append_log)
        self.udp_thread.start()

        self.mqtt_thread = MQTTReceiverThread()
        self.mqtt_thread.sensor_triggered.connect(self.handle_sensor_detection)
        self.mqtt_thread.dc_stopped.connect(self.handle_dc_stopped)
        self.mqtt_thread.log_msg.connect(self.append_log)
        self.mqtt_thread.start()

    def initUI(self):
        self.setWindowTitle("QR 물류 자동 분류기 (PyQt6)")
        self.resize(900, 600)

        main_layout = QHBoxLayout()

        left_layout = QVBoxLayout()
        self.img_label = QLabel(
            "카메라 스트리밍 대기중...\n(화면이 들어오지 않았습니다)"
        )
        self.img_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.img_label.setStyleSheet(
            "background-color: #2F2F2F; color: #FFFFFF; font-size: 16px; border: 2px solid #555;"
        )
        self.img_label.setMinimumSize(480, 360)
        left_layout.addWidget(self.img_label)

        right_layout = QVBoxLayout()

        # 1. 모터 제어 패널 추가
        control_frame = QFrame()
        control_frame.setStyleSheet("background-color: #f1f3f4; border-radius: 8px;")
        control_layout = QVBoxLayout()
        control_layout.addWidget(QLabel("<b>DC모터 (컨베이어) 제어</b>"))

        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("모터 속도 (0~255):"))
        self.input_speed = QLineEdit("0")
        self.input_speed.setValidator(QIntValidator(0, 255))
        speed_layout.addWidget(self.input_speed)
        control_layout.addLayout(speed_layout)

        btn_layout = QHBoxLayout()
        self.btn_start = QPushButton("▶ 시작")
        self.btn_stop = QPushButton("■ 정지")
        self.btn_stop.setEnabled(False)

        self.btn_start.clicked.connect(self.on_start_clicked)
        self.btn_stop.clicked.connect(self.on_stop_clicked)

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        control_layout.addLayout(btn_layout)

        # 가상 센서 버튼 행 추가
        sensor_layout = QHBoxLayout()
        self.btn_sensor_sim = QPushButton("🟢 센서 ON (임시)")
        self.btn_sensor_sim.setStyleSheet(
            "background-color: #28a745; color: white; font-weight: bold;"
        )
        self.btn_sensor_sim.clicked.connect(self.on_sensor_sim_clicked)
        sensor_layout.addWidget(QLabel("가상 센서 트리거:"))
        sensor_layout.addWidget(self.btn_sensor_sim)
        control_layout.addLayout(sensor_layout)

        control_frame.setLayout(control_layout)
        right_layout.addWidget(control_frame)

        frame = QFrame()
        frame.setStyleSheet("background-color: #e8f0fe; border-radius: 10px;")
        frame_layout = QVBoxLayout()

        self.count_label = QLabel("== 인식된 QR 큐 ==\n비어있음")
        self.count_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #1a73e8; padding: 10px;"
        )
        self.count_label.setAlignment(Qt.AlignmentFlag.AlignTop)
        frame_layout.addWidget(self.count_label)
        frame.setLayout(frame_layout)
        right_layout.addWidget(frame)

        right_layout.addWidget(QLabel("시스템 로그:"))
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet(
            "background-color: #1e1e1e; color: #00ff00; font-family: Consolas;"
        )
        right_layout.addWidget(self.log_view)

        main_layout.addLayout(left_layout, 6)
        main_layout.addLayout(right_layout, 4)

        self.setLayout(main_layout)

    def process_image(self, img, is_trigger):
        if self.is_processing:
            return
        self.is_processing = True

        try:
            if not has_pyzbar:
                detector = cv2.QRCodeDetector()
                data, bbox, _ = detector.detectAndDecode(img)
                if data:
                    if bbox is not None:
                        points = bbox[0].astype(int)
                        for i in range(len(points)):
                            cv2.line(
                                img,
                                tuple(points[i]),
                                tuple(points[(i + 1) % len(points)]),
                                (0, 255, 0),
                                3,
                            )
                    self.handle_qr_data(data)
            else:
                decoded_objects = decode(img)
                if decoded_objects:
                    for obj in decoded_objects:
                        pts = obj.polygon
                        if len(pts) == 4:
                            pts = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
                            cv2.polylines(img, [pts], True, (255, 0, 0), 4)

                        qr_data = obj.data.decode("utf-8")
                        self.handle_qr_data(qr_data)

            rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w

            qt_img = QImage(
                rgb_img.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
            )
            pixmap = QPixmap.fromImage(qt_img)
            scaled_pixmap = pixmap.scaled(
                self.img_label.width(),
                self.img_label.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            self.img_label.setPixmap(scaled_pixmap)
        finally:
            self.is_processing = False

    def handle_qr_data(self, data):
        current_time = time.time()
        # 동일 QR코드는 2초 이내 중복 인식 방지
        if data == self.last_scanned_qr and (current_time - self.last_scan_time) < 2.0:
            return

        self.last_scanned_qr = data
        self.last_scan_time = current_time

        self.append_log(f"✅ [QR 신규 인식] 큐에 추가됨: {data}")
        self.qr_queue.append(data)
        self.update_display()

    def on_start_clicked(self):
        speed_val = self.input_speed.text()
        if not speed_val:
            speed_val = "0"
            self.input_speed.setText("0")

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.input_speed.setEnabled(False)

        self.mqtt_thread.publish_command(f"DC_START:{speed_val}")
        self.append_log(f"▶ 컨베이어 시작됨 (속도: {speed_val})")

    def on_stop_clicked(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.input_speed.setEnabled(True)

        self.mqtt_thread.publish_command("DC_STOP")
        self.append_log("■ 컨베이어 정지됨")

    def on_sensor_sim_clicked(self):
        self.mqtt_thread.publish_command("SENSOR_SIMULATE")
        self.append_log("🟢 가상 근접 센서 신호(SENSOR_SIMULATE) 전송")

    def handle_dc_stopped(self):
        self.append_log("⏹️ 시나리오 분류완료: 1초경과 자동 정지 수행됨")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.input_speed.setEnabled(True)

    def handle_sensor_detection(self):
        self.append_log("🚨 [이벤트] 근접 센서 물체 도달 감지!")
        if len(self.qr_queue) > 0:
            item = self.qr_queue.popleft()
            self.append_log(f"  -> QR 값 '{item}' 큐에서 제거 (분류 처리 확정)")
            self.processed_history.append(item)

            # QR 내용에 따라 양방향 제어 로직
            # 예시: 'esp32'가 포함되어 있으면 왼쪽으로, 없으면 오른쪽으로 분류
            if "esp32" in item.lower():
                self.append_log(f"  -> '{item}' : (왼쪽 분류)")
                self.mqtt_thread.publish_command("MOVE_LEFT")
            else:
                self.append_log(f"  -> '{item}' : (오른쪽 분류)")
                self.mqtt_thread.publish_command("MOVE_RIGHT")
        else:
            self.append_log(
                "  -> [경고] 처리할 QR 큐가 비어있습니다. 분류기를 헛돌립니다."
            )

        self.update_display()

    def update_display(self):
        display_text = "== 대기중인 파싱 큐 ==\n"
        if not self.qr_queue:
            display_text += "비어있음\n"
        else:
            for idx, item in enumerate(self.qr_queue):
                display_text += f"{idx+1}. {item}\n"

        display_text += "\n== 분류 완료 내역 (최근 5건) ==\n"
        for idx, item in enumerate(self.processed_history[-5:]):
            display_text += f"- {item}\n"

        self.count_label.setText(display_text)

    def append_log(self, text):
        self.log_view.append(text)
        self.log_view.verticalScrollBar().setValue(
            self.log_view.verticalScrollBar().maximum()
        )

    def closeEvent(self, event):
        self.udp_thread.stop()
        self.mqtt_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = QRInventoryApp()
    ex.show()
    sys.exit(app.exec())
