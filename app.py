from flask import Flask, jsonify, request, render_template
import serial
import serial.tools.list_ports
import threading
import cv2
import time
import mediapipe as mp
import base64
from io import BytesIO
from PIL import Image
import pygame  # Import pygame for music playback
import os

app = Flask(__name__)

# 串口配置
SERIAL_BAUD = 57600
arduino = None
serial_lock = threading.Lock()
is_connected = False
current_port = None

# 计时及状态控制
last_detection_time = 0
last_action_sent = 0  # 上一次发指令的动作，用于避免重复发

# 初始化MediaPipe
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# 音乐文件路径
MUSIC_FILE_PATH = r"C:\Users\yongs\Desktop\Michael Jackson - Billie Jean.flac"

# 验证音乐文件是否存在
if not os.path.exists(MUSIC_FILE_PATH):
    print(f"错误：音乐文件 {MUSIC_FILE_PATH} 不存在！")

# 连接Arduino（示例）
def send_command_to_arduino(command):
    global arduino, serial_lock, is_connected
    if not is_connected or not arduino or not arduino.is_open:
        return False, "未连接到Arduino", None
    try:
        with serial_lock:
            arduino.write(command.encode())
            time.sleep(0.1)
            response = arduino.readline().decode().strip()
        return True, "命令已发送", response
    except Exception as e:
        return False, f"发送失败: {str(e)}", None

# 播放音乐的函数
def play_music():
    try:
        pygame.mixer.init()
        pygame.mixer.music.load(MUSIC_FILE_PATH)
        pygame.mixer.music.play()
        print(f"正在播放音乐: {MUSIC_FILE_PATH}")
    except Exception as e:
        print(f"音乐播放失败: {str(e)}")

# 连接串口
def init_arduino(port):
    global arduino, is_connected, current_port
    try:
        if arduino and arduino.is_open:
            arduino.close()
        arduino = serial.Serial(port, SERIAL_BAUD, timeout=1)
        time.sleep(2)
        is_connected = True
        current_port = port
        return True, "成功连接到 Arduino"
    except Exception as e:
        is_connected = False
        current_port = None
        return False, str(e)

# 获取串口列表
@app.route('/api/ports', methods=['GET'])
def get_ports():
    try:
        ports = list(serial.tools.list_ports.comports())
        port_list = [{"name": port.device, "description": port.description} for port in ports]
        return jsonify({"status": "success", "ports": port_list})
    except Exception as e:
        return jsonify({"status": "error", "message": f"获取串口失败: {str(e)}"}), 500

# 连接指定串口
@app.route('/api/connect', methods=['POST'])
def api_connect():
    data = request.get_json()
    port = data.get('port')
    if not port:
        return jsonify({"status": "error", "message": "请选择串口"}), 400
    success, message = init_arduino(port)
    if success:
        return jsonify({"status": "success", "message": message})
    else:
        return jsonify({"status": "error", "message": message}), 500

# 断开串口
@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    global arduino, is_connected, current_port
    try:
        if arduino and arduino.is_open:
            arduino.close()
        is_connected = False
        current_port = None
        return jsonify({"status": "success", "message": "已断开连接"})
    except Exception as e:
        return jsonify({"status": "error", "message": f"断开失败: {str(e)}"})

# 获取连接状态
@app.route('/api/status', methods=['GET'])
def api_status():
    return jsonify({"status": "connected" if is_connected else "disconnected", "port": current_port})

# 启动摄像头识别线程
@app.route('/start_camera', methods=['GET'])
def start_camera():
    try:
        threading.Thread(target=detect_motion_from_camera, daemon=True).start()
        return jsonify({"status": "success", "message": "摄像头已启动"}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": f"启动摄像头失败: {str(e)}"}), 500

# 根路径，渲染主页
@app.route('/')
def index():
    return render_template('index.html')

# 移动控制接口
@app.route('/api/move', methods=['POST'])
def move():
    data = request.get_json()
    direction = data.get('direction')

    if direction == 'light':
        success, message, response = send_command_to_arduino('123')
    elif direction == 'exit':
        success, message, response = send_command_to_arduino('-1')
    elif direction == 'calibrate':
        success, message, response = send_command_to_arduino('1')
    elif direction == 'dance':
        success, message, response = send_command_to_arduino('5')
        if success:
            # Play music in a separate thread
            threading.Thread(target=play_music, daemon=True).start()
    else:
        return jsonify({"status": "error", "message": "不支持的方向"}), 400

    if success:
        result = {"status": "success", "message": message}
        if response:
            result["arduino_response"] = response
        return jsonify(result)
    else:
        return jsonify({"status": "error", "message": message}), 500

# 识别逻辑：检测左手、右手、双手
def recognize_action(landmarks):
    nose = landmarks[0]
    shoulder_left = landmarks[11]
    shoulder_right = landmarks[12]
    wrist_left = landmarks[15]
    wrist_right = landmarks[16]

    if wrist_left.y < nose.y and wrist_right.y < nose.y:
        return 5
    elif wrist_left.y < shoulder_left.y:
        return 2
    elif wrist_right.y < shoulder_right.y:
        return 4
    return 0

# 摄像头检测主逻辑
def detect_motion_from_camera():
    global last_action_sent
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("摄像头未打开！")
        return

    last_time = time.time()
    detection_interval = 3  # 秒
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        current_time = time.time()
        if current_time - last_time >= detection_interval:
            last_time = current_time
            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                action_id = recognize_action(landmarks)
                if action_id != 0 and action_id != last_action_sent:
                    perform_action(action_id)
                    print(f"检测到动作：{action_id}，自动发指令")
                    if action_id == 5:
                        # Play music when action_id is 5 (dance)
                        threading.Thread(target=play_music, daemon=True).start()
                    last_action_sent = action_id
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# 执行动作
def perform_action(action_id):
    send_command_to_arduino(str(action_id))

# 转换图像为base64
def image_to_base64(img):
    _, img_encoded = cv2.imencode('.png', img)
    img_bytes = img_encoded.tobytes()
    return base64.b64encode(img_bytes).decode('utf-8')

# 获取摄像头视频帧的 API
@app.route('/get_camera_feed', methods=['GET'])
def get_camera_feed():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        img_base64 = image_to_base64(frame)
        cap.release()
        return jsonify({'image': img_base64})
    else:
        cap.release()
        return jsonify({'status': 'error', 'message': '无法读取摄像头帧'}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)