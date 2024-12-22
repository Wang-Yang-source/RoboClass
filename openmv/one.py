import sensor, time
from pyb import UART, LED

# 打开所有 LED 灯
LED(1).on()
LED(2).on()
LED(3).on()

# 颜色跟踪阈值 (L Min, L Max, A Min, A Max, B Min, B Max)
thresholds = [
    (13, 30, 29, 47, 16, 28),  # 红色阈值
    (19, 32, -59, -18, 13, 36),  # 绿色阈值
    (8, 37, -12, 8, -36, -7)  # 蓝色阈值
]

# 巡线检测参数
THRESHOLD = (0, 50)  # 二值化阈值，需根据实际情况调整
ROI = (60, 60, 240, 160)  # 感兴趣区域

# 初始化相机
sensor.reset()  # 重置感光元件
sensor.set_pixformat(sensor.RGB565)  # 设置颜色格式为 RGB565
sensor.set_framesize(sensor.QVGA)  # 设置分辨率为 QVGA
sensor.skip_frames(time=2000)  # 跳过一些帧，等待传感器稳定
sensor.set_auto_gain(False)  # 关闭自动增益
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
clock = time.clock()

# 初始化 UART
uart = UART(3, baudrate=115200, bits=8, parity=None, stop=1)

# 颜色检测函数
def blob():
    img = sensor.snapshot()  # 捕获图像
    img.lens_corr(1.8)  # 镜头畸变校正
    for i, threshold in enumerate(thresholds):
        for blob in img.find_blobs([threshold], pixels_threshold=200, area_threshold=200, merge=True):
            # 发送颜色信息通过 UART
            uart.write(f"{i}\n")
            print(i)
            # 绘制矩形框和中心十字标记
            img.draw_rectangle(blob.rect(), color=(255 * (i == 0), 255 * (i == 1), 255 * (i == 2)))
            img.draw_cross(blob.cx(), blob.cy())

def line():
    img = sensor.snapshot()  # 捕获图像
    line_segments = img.find_line_segments(roi=ROI, merge_distance=20, max_theta_difference=20)

    # 用于分类线段的列表
    left_lines = []
    right_lines = []

    for line in line_segments:
        angle = line.theta()
        # 根据线段的角度分类
        if angle < 90:  # 偏左的线段
            left_lines.append(line)
        elif angle > 90:  # 偏右的线段
            right_lines.append(line)
        img.draw_line(line.line(), color=(255, 0, 0))  # 绘制线段

# 主循环
while True:
    clock.tick()
    # blob()  # 执行颜色检测
    line()  # 执行巡线检测
    # print("FPS:", clock.fps())  # 打印帧率
