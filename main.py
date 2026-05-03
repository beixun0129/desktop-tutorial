
from maix import camera, display, image, nn, app, comm, uart
import struct, os, math, time

report_on = True
APP_CMD_DETECT_RES = 0x02

UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200
UART_FRAME_HEADER = b"\xAA\xBB\xCC\xDD"

# 初始化串口
# 注意：设备号 (如 "/dev/ttyS1") 和波特率 (如 115200) 需要根据你实际使用的引脚和单片机配置进行修改
try:
    # 不同的 Maix 硬件串口号可能不同，常见有 /dev/ttyS0, /dev/ttyS1 等
    serial_port = uart.UART(UART_DEVICE, UART_BAUD)
    print("UART initialized successfully.")
except Exception as e:
    print("Failed to initialize UART:", e)
    serial_port = None

RED_CONFIRM_FRAMES = 5
_red_streak = 0
GREEN_CONFIRM_FRAMES = 5
_green_streak = 0
_last_light_cx = None
_last_light_cy = None
_last_light_label = None
DEBUG_UART = True
_last_uart_cmd = None
_uart_err_cnt = 0
UART_MODE = "binary"
UART_SEND_INTERVAL_MS = 1000
_last_uart_tx_ms = 0
_last_dist_cm = 0
UART_RX_DEBUG = False
_uart_rx_buf = bytearray()

CAM_HEIGHT_M = 1.7
CAM_PITCH_DEG = 15.0
CAM_VFOV_DEG = 60.0
MAX_DIST_M = 50.0

def _hexdump(b):
    return " ".join(f"{x:02X}" for x in b)

def _uart_cmd_name(cmd):
    return {
        0x00: "WALK",
        0x01: "STOP_RED",
        0x02: "STOP_CAR",
        0x03: "WALK_GREEN",
        0x04: "WAIT_CROSSWALK",
        0x05: "WAIT_CROSSWALK_PERSON",
    }.get(cmd, f"UNKNOWN(0x{cmd:02X})")

def _estimate_ground_distance_m(v, img_h):
    try:
        if img_h <= 0:
            return None
        cy = img_h * 0.5
        vfov = math.radians(CAM_VFOV_DEG)
        fy = (img_h * 0.5) / math.tan(vfov * 0.5)
        pitch = math.radians(CAM_PITCH_DEG)
        theta = pitch + math.atan((v - cy) / fy)
        if theta <= 0.0:
            return None
        d = CAM_HEIGHT_M / math.tan(theta)
        if d <= 0.0:
            return None
        if d > MAX_DIST_M:
            return None
        return d
    except Exception:
        return None

def _try_parse_uart_frames():
    global _uart_rx_buf
    out = []
    while True:
        b = _uart_rx_buf
        if not b:
            break
        i_old = b.find(b"\xAA\xBB")
        i_new = b.find(UART_FRAME_HEADER)
        if i_old < 0 and i_new < 0:
            if len(b) > 3:
                _uart_rx_buf = b[-3:]
            break
        if i_old >= 0 and (i_new < 0 or i_old <= i_new):
            i = i_old
            need = i + 6
            if len(b) < need:
                break
            frame = bytes(b[i:need])
            if frame[0] == 0xAA and frame[1] == 0xBB and frame[5] == 0xCC:
                cmd = frame[2]
                dist_cm = frame[3] | (frame[4] << 8)
                out.append(("old", cmd, dist_cm, True, frame))
                _uart_rx_buf = b[need:]
                continue
            _uart_rx_buf = b[i+1:]
            continue
        else:
            i = i_new
            need = i + 8
            if len(b) < need:
                break
            frame = bytes(b[i:need])
            checksum_ok = (sum(frame[:-1]) % 255) == frame[-1]
            cmd = frame[4]
            dist_cm = frame[5] | (frame[6] << 8)
            out.append(("new", cmd, dist_cm, checksum_ok, frame))
            _uart_rx_buf = b[need:]
            continue
    return out

def encode_objs(objs):
    '''
        encode objs info to bytes body for protocol
        2B x(LE) + 2B y(LE) + 2B w(LE) + 2B h(LE) + 2B idx + 4B score(float) ...
    '''
    body = b''
    for obj in objs:
        body += struct.pack("<hhHHHf", obj.x, obj.y, obj.w, obj.h, obj.class_id, obj.score)
    return body

model_path = "model_257742.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/maixhub/257742/model_257742.mud"
detector = nn.YOLOv5(model=model_path)

cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
# 开启自动曝光和自动白平衡 (如果硬件和驱动支持)
# 通常 Maix 库的 Camera 默认是开启的，但可以显式调用确保其处于自动模式
# try:
#     cam.set_auto_exp(True)
# except Exception as e:
#     print("Set auto exposure failed:", e)

dis = display.Display()

p = comm.CommProtocol(buff_size = 1024)

while not app.need_exit():
    # msg = p.get_msg()

    img = cam.read()
    
    # 获取图像宽度和高度用于处理
    img_w = img.width()
    img_h = img.height()
    
    # 提取画面上半部分（通常是红绿灯出现的位置）的亮度
    # 这里简单地取一个中心靠上的区域来评估环境亮度，你可以根据实际摄像头安装角度调整
    # roi: (x, y, w, h)
    roi_x = img_w // 4
    roi_y = 0
    roi_w = img_w // 2
    roi_h = img_h // 3
    
    # 裁剪出感兴趣的区域 (ROI)
    # 注意：这里的 image 库操作可能因具体的 maix 版本而略有不同
    # 如果 crop 不可用，可以考虑在整张图上计算或者使用其他 API
    try:
        # 获取 ROI 区域进行颜色统计
        # 很多图像库可以通过计算 HSV 的 V (亮度) 通道来判断是否过曝
        # 但为了不严重影响帧率，且 YOLO 已经做了大部分工作，
        # 我们这里演示如何结合传统视觉 (HSV) 来辅助验证红绿灯状态。
        
        # 假设我们想对模型检测出的红绿灯区域，用 HSV 做二次确认
        pass
    except Exception as e:
        pass

    objs = detector.detect(img, conf_th = 0.25, iou_th = 0.45)

    # 过滤掉远处的行人 (person) 和车辆 (car)
    filtered_objs = []
    for obj in objs:
        label = detector.labels[obj.class_id]
        
        # 使用 HSV 颜色空间进行红绿灯颜色辅助验证
        # 当模型检测到是红绿灯（或无法确定具体颜色，只有 'traffic_light' 大类时）
        # 可以截取该框的图像，转换到 HSV 空间，计算红色或绿色像素的比例
        if label in ["red", "green"]:
            # 确保坐标在图像范围内
            x1 = max(0, obj.x)
            y1 = max(0, obj.y)
            x2 = min(img.width(), obj.x + obj.w)
            y2 = min(img.height(), obj.y + obj.h)
            
            # 只有当框有效时才处理
            if x2 > x1 and y2 > y1:
                try:
                    # 注意：Maix 视觉库(maix.image)的具体 API 会有所不同
                    # 这里提供一种常见的思路：获取框内的统计信息
                    # 比如：img.get_statistics(roi=(x1, y1, x2-x1, y2-y1))
                    # 或者如果有 find_blobs，可以指定颜色阈值 (L, A, B) 或 (H, S, V)
                    
                    # 由于当前使用的是基于神经网络的检测 (YOLO)，模型本身已经区分了 'red' 和 'green'。
                    # 如果你在极亮/极暗环境下，发现模型区分错误（比如红灯识别成了绿灯），
                    # 可以在这里加入传统 CV (如 HSV 阈值) 进行二次校验：
                    
                    # 伪代码示例：
                    # roi_img = img.crop(x1, y1, x2-x1, y2-y1)
                    # roi_hsv = roi_img.to_hsv()
                    # red_pixels = roi_hsv.find_color(red_hsv_threshold)
                    # if red_pixels > threshold: 
                    #     label = "red_verified"
                    pass
                except Exception as e:
                    pass
                    
        if label in ["person", "car"]:
            # 如果行人和车辆的宽度或高度小于某个阈值（说明在远处），则忽略不识别
            # 这里的 30 可以根据实际距离感受进行调大或调小
            if obj.w < 30 or obj.h < 30:
                continue
        filtered_objs.append(obj)
    objs = filtered_objs

    # ===== 状态决策逻辑开始 =====
    # 主体是“行人”，需要根据摄像头检测到的红绿灯、车辆、斑马线等做状态决策
    # 状态：例如 "WALK" (走), "STOP" (停), "WAIT" (等待/观察)
    
    current_action = "WALK" # 默认状态
    
    has_red_light = False
    has_green_light = False
    has_car_nearby = False
    has_crosswalk = False
    has_person = False
    nearest_car_dist_m = None
    nearest_crosswalk_dist_m = None
    nearest_person_dist_m = None
    best_light = None
    
    # 分析当前帧检测到的所有有效目标
    for obj in objs:
        label = detector.labels[obj.class_id]
        if label == "red":
            has_red_light = True
            if best_light is None or obj.score > best_light.score:
                best_light = obj
        elif label == "green":
            has_green_light = True
            if best_light is None or obj.score > best_light.score:
                best_light = obj
        elif label == "person":
            has_person = True
            d = _estimate_ground_distance_m(obj.y + obj.h, img_h)
            if d is not None:
                if nearest_person_dist_m is None or d < nearest_person_dist_m:
                    nearest_person_dist_m = d
        elif label == "car":
            # 判断车是否离得很近（通过检测框大小，或在画面下方判断）
            # 假设面积大于某个阈值，或者 y 坐标靠下，说明车很近很危险
            if obj.w * obj.h > 2000 or obj.y > img.height() // 2:
                has_car_nearby = True
            d = _estimate_ground_distance_m(obj.y + obj.h, img_h)
            if d is not None:
                if nearest_car_dist_m is None or d < nearest_car_dist_m:
                    nearest_car_dist_m = d
        elif label == "crosswalk":
            has_crosswalk = True
            d = _estimate_ground_distance_m(obj.y + obj.h, img_h)
            if d is not None:
                if nearest_crosswalk_dist_m is None or d < nearest_crosswalk_dist_m:
                    nearest_crosswalk_dist_m = d

    # 决策树 (以行人的视角)
    # 我们定义一套简单的协议发送给单片机，比如：
    # 0x00: WALK (正常走)
    # 0x01: STOP_RED (红灯停)
    # 0x02: STOP_CAR (车近停)
    # 0x03: WALK_GREEN (绿灯行)
    # 0x04: WAIT_CROSSWALK (斑马线观察)
    uart_cmd = 0x00
    
    light_moving = False
    if best_light is not None:
        label = detector.labels[best_light.class_id]
        cx = best_light.x + best_light.w * 0.5
        cy = best_light.y + best_light.h * 0.5
        if _last_light_cx is not None and _last_light_cy is not None:
            dx = cx - _last_light_cx
            dy = cy - _last_light_cy
            thr = max(10.0, min(img_w, img_h) * 0.06)
            if dx * dx + dy * dy > thr * thr:
                light_moving = True
        _last_light_cx = cx
        _last_light_cy = cy
        _last_light_label = label
    else:
        _last_light_cx = None
        _last_light_cy = None
        _last_light_label = None
    
    if has_red_light and not light_moving:
        _red_streak = min(_red_streak + 1, RED_CONFIRM_FRAMES)
    else:
        _red_streak = max(_red_streak - 1, 0)
    if has_green_light and not light_moving:
        _green_streak = min(_green_streak + 1, GREEN_CONFIRM_FRAMES)
    else:
        _green_streak = max(_green_streak - 1, 0)
    red_confirmed = _red_streak >= RED_CONFIRM_FRAMES
    green_confirmed = _green_streak >= GREEN_CONFIRM_FRAMES

    if red_confirmed:
        current_action = "STOP (Red Light)"
        uart_cmd = 0x01
    elif has_car_nearby:
        current_action = "STOP (Car Nearby)"
        uart_cmd = 0x02
    elif green_confirmed:
        current_action = "WALK (Green Light)"
        uart_cmd = 0x03
    elif has_crosswalk and has_person:
        current_action = "WAIT (Crosswalk + Person)"
        uart_cmd = 0x05
    elif has_crosswalk:
        current_action = "WAIT (Crosswalk)"
        uart_cmd = 0x04
    else:
        current_action = "WALK"
        uart_cmd = 0x00

    # 将决策结果打印在画面左上角，方便观察
    img.draw_string(10, 10, f"Action: {current_action}", color=image.COLOR_GREEN if "WALK" in current_action else image.COLOR_RED)
    if has_red_light and not red_confirmed:
        img.draw_string(10, 30, f"Red pending: {_red_streak}/{RED_CONFIRM_FRAMES}", color=image.COLOR_RED)
    if has_green_light and not green_confirmed:
        img.draw_string(10, 70, f"Green pending: {_green_streak}/{GREEN_CONFIRM_FRAMES}", color=image.COLOR_GREEN)
    dist_m = None
    candidates = []
    if nearest_car_dist_m is not None:
        candidates.append(nearest_car_dist_m)
    if nearest_person_dist_m is not None:
        candidates.append(nearest_person_dist_m)
    if nearest_crosswalk_dist_m is not None:
        candidates.append(nearest_crosswalk_dist_m)
    if candidates:
        dist_m = min(candidates)
    dist_cm = 0
    if dist_m is not None:
        dist_cm = int(dist_m * 100 + 0.5)
        if dist_cm < 0:
            dist_cm = 0
        if dist_cm > 65535:
            dist_cm = 65535
        img.draw_string(10, 50, f"Dist: {dist_m:.2f}m", color=image.COLOR_GREEN)
    
    # 通过串口发送决策结果给单片机
    if serial_port:
        try:
            if UART_RX_DEBUG:
                rx = serial_port.read()
                if rx:
                    _uart_rx_buf.extend(rx)
                    for proto, cmd, rx_dist_cm, ok, frame in _try_parse_uart_frames():
                        ok_tag = "OK" if ok else "CHK_FAIL"
                        print(f"[UART RX] {proto} {ok_tag} cmd={_uart_cmd_name(cmd)} dist_cm={rx_dist_cm} data={_hexdump(frame)}")
            now_ms = int(time.time() * 1000)
            
            # 当状态发生变化，或者距离发生明显变化（例如大于 50cm），或者达到心跳发送间隔时，发送数据
            dist_changed_significantly = abs(dist_cm - _last_dist_cm) > 50
            should_send = (uart_cmd != _last_uart_cmd) or dist_changed_significantly or (now_ms - _last_uart_tx_ms >= UART_SEND_INTERVAL_MS)
            
            if should_send:
                if UART_MODE == "ascii":
                    data_to_send = f"AA BB {uart_cmd:02X} {dist_cm}\n".encode("ascii")
                else:
                    payload = struct.pack("<BH", uart_cmd, dist_cm)
                    frame = UART_FRAME_HEADER + payload
                    checksum = sum(frame) % 255
                    data_to_send = frame + struct.pack("B", checksum)
                ret = serial_port.write(data_to_send)
                _last_uart_cmd = uart_cmd
                _last_dist_cm = dist_cm
                _last_uart_tx_ms = now_ms
                if DEBUG_UART:
                    dist_tag = "NA" if dist_m is None else f"{dist_m:.2f}m"
                    ret_tag = "" if ret is None else f" n={ret}"
                    print(f"[UART TX] cmd={_uart_cmd_name(uart_cmd)} dist={dist_tag}{ret_tag} data={_hexdump(data_to_send)}")
        except Exception as e:
            _uart_err_cnt += 1
            if DEBUG_UART and _uart_err_cnt <= 5:
                print(f"[UART ERROR] write failed: {e!r}")
            
    # ===== 状态决策逻辑结束 =====

    if len(objs) > 0 and report_on:
        body = encode_objs(objs)
        p.report(APP_CMD_DETECT_RES, body)

    for obj in objs:
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = image.COLOR_RED)
        msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
        img.draw_string(obj.x, obj.y, msg, color = image.COLOR_RED)
    dis.show(img)
