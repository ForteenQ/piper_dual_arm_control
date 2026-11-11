import os
import cv2
import numpy as np

# A4 纸尺寸 (300 DPI): 210mm x 297mm -> 2480 x 3508 像素
DPI = 300
MM_TO_PX = DPI / 25.4  # ~11.811 px/mm

WIDTH_PX = int(210 * MM_TO_PX)   # 2480
HEIGHT_PX = int(297 * MM_TO_PX)  # 3508

canvas = np.ones((HEIGHT_PX, WIDTH_PX, 3), dtype=np.uint8) * 255
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

def get_tag_img(tag_id, size_px):
    img = np.zeros((size_px, size_px), dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(aruco_dict, tag_id, size_px, 1)
    elif hasattr(cv2.aruco, 'drawMarker'):
        cv2.aruco.drawMarker(aruco_dict, tag_id, size_px, img, 1)
    if len(img.shape) == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img

# 标题与说明
title = "AprilTag 36h11 25mm Dedicated Array (A4 - 300 DPI)"
subtitle = "Tailored for 30mm Cubes. Outer cut-box: exactly 30x30mm. Print scale: 100% (Do not fit)."
cv2.putText(canvas, title, (100, 100), cv2.FONT_HERSHEY_DUPLEX, 1.2, (20, 20, 20), 2, cv2.LINE_AA)
cv2.putText(canvas, subtitle, (100, 150), cv2.FONT_HERSHEY_DUPLEX, 0.75, (90, 90, 90), 2, cv2.LINE_AA)

# 绘制精确 100mm 标尺
ruler_x = 100
ruler_y = 210
ruler_len_px = int(100 * MM_TO_PX)
cv2.line(canvas, (ruler_x, ruler_y), (ruler_x + ruler_len_px, ruler_y), (0, 0, 0), 4)
cv2.line(canvas, (ruler_x, ruler_y - 20), (ruler_x, ruler_y + 20), (0, 0, 0), 4)
cv2.line(canvas, (ruler_x + ruler_len_px, ruler_y - 20), (ruler_x + ruler_len_px, ruler_y + 20), (0, 0, 0), 4)
cv2.putText(canvas, "<--- 100.0 mm Physical Ruler (Check with ruler after print) --->", 
            (ruler_x + 30, ruler_y - 25), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0, 0, 0), 2, cv2.LINE_AA)

# 密集网格排版：5 列 x 6 行 = 30 个 25mm 标签
# 每个标签外围灰色裁剪框为 30mm x 30mm (完美对应 30mm 方块顶面)
cols = 5
rows = 6
tag_size_mm = 25.0
box_size_mm = 30.0  # 剪裁框 30mm

start_x_mm = 20.0
start_y_mm = 28.0
pitch_x_mm = 35.0   # 列间距
pitch_y_mm = 43.0   # 行间距

# ID 分配：前 3 行全为 ID 0 (共 15 个，单目标抓取用)；后 3 行分配 ID 1, 2, 3 (分拣备用)
id_list = [0]*15 + [1]*5 + [2]*5 + [3]*5

count = 0
for r in range(rows):
    for c in range(cols):
        tid = id_list[count]
        
        # 30mm 裁剪框左上角
        box_x_mm = start_x_mm + c * pitch_x_mm
        box_y_mm = start_y_mm + r * pitch_y_mm
        
        # 25mm 标签居中放置在 30mm 框内 (四周留 2.5mm 白边)
        tag_x_mm = box_x_mm + 2.5
        tag_y_mm = box_y_mm + 2.5
        
        # 像素坐标
        tag_x_px = int(tag_x_mm * MM_TO_PX)
        tag_y_px = int(tag_y_mm * MM_TO_PX)
        tag_s_px = int(tag_size_mm * MM_TO_PX)
        
        box_x_px = int(box_x_mm * MM_TO_PX)
        box_y_px = int(box_y_mm * MM_TO_PX)
        box_s_px = int(box_size_mm * MM_TO_PX)
        
        # 贴入 25mm 标签
        tag_img = get_tag_img(tid, tag_s_px)
        canvas[tag_y_px:tag_y_px+tag_s_px, tag_x_px:tag_x_px+tag_s_px] = tag_img
        
        # 绘制 30mm x 30mm 灰色裁剪框 (带微虚线效果)
        cv2.rectangle(canvas, (box_x_px, box_y_px), (box_x_px + box_s_px, box_y_px + box_s_px), (170, 170, 170), 2)
        
        # 标签说明
        label_txt = f"ID: {tid} (25mm)"
        cv2.putText(canvas, label_txt, (box_x_px, box_y_px + box_s_px + int(6 * MM_TO_PX)), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, (60, 60, 60), 1, cv2.LINE_AA)
        
        count += 1

script_dir = os.path.dirname(os.path.abspath(__file__))
out_png = os.path.join(script_dir, "apriltag_multi_a4_print.png")
cv2.imwrite(out_png, canvas)
print("SUCCESS: 25mm Array A4 Image generated at " + out_png)
