'''
Author: potato potato@potato.com
Date: 2025-12-04 13:22:57
LastEditors: potato potato@potato.com
LastEditTime: 2025-12-04 13:34:04
FilePath: /RC_2026/RC_2026_1/rc26_perception/scripts/export_onnx.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3
"""
将 YOLOv8 .pt 模型导出为 ONNX 格式
用法: python3 export_onnx.py /path/to/yolov8s.pt
"""
import sys
import os

def main():
    if len(sys.argv) < 2:
        # 默认路径
        pt_path = "/home/potato/RC_2026/yolov8s.pt"
    else:
        pt_path = sys.argv[1]
    
    if not os.path.exists(pt_path):
        print(f"Error: Model file not found: {pt_path}")
        sys.exit(1)
    
    try:
        from ultralytics import YOLO
    except ImportError:
        print("Installing ultralytics...")
        os.system("pip3 install ultralytics")
        from ultralytics import YOLO
    
    # 输出路径
    output_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    onnx_path = os.path.join(output_dir, "models", "yolov8s.onnx")
    
    print(f"Loading model: {pt_path}")
    model = YOLO(pt_path)
    
    print(f"Exporting to ONNX: {onnx_path}")
    # opset=12, simplify=True, dynamic=False for OpenCV 4.5.4 compatibility
    model.export(format="onnx", imgsz=640, opset=12, simplify=True, dynamic=False)
    
    # 移动到目标位置
    exported_onnx = pt_path.replace(".pt", ".onnx")
    if os.path.exists(exported_onnx) and exported_onnx != onnx_path:
        import shutil
        shutil.move(exported_onnx, onnx_path)
        print(f"Moved to: {onnx_path}")
    
    print("Done!")

if __name__ == "__main__":
    main()
