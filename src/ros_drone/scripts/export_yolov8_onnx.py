#!/usr/bin/env python3
"""从 Ultralytics best_v8.pt 导出 YOLOv8 ONNX，供 detector 包 C++ 节点使用。"""
import argparse
import os

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pt", default=os.path.join(os.path.dirname(__file__), "..", "..", "..", "best_v8.pt"),
                   help="best_v8.pt 路径")
    p.add_argument("--out", default="", help="输出 .onnx 路径（默认同目录 best_v8.onnx）")
    args = p.parse_args()
    pt = os.path.abspath(args.pt)
    out = args.out or os.path.join(os.path.dirname(pt), "best_v8.onnx")
    out = os.path.abspath(out)
    from ultralytics import YOLO
    model = YOLO(pt)
    model.export(format="onnx", imgsz=640, simplify=True)
    # ultralytics 默认写到 pt 同目录，文件名与权重名一致仅后缀改 .onnx
    default_onnx = os.path.splitext(pt)[0] + ".onnx"
    if os.path.abspath(default_onnx) != out and os.path.isfile(default_onnx):
        import shutil
        shutil.move(default_onnx, out)
    print("ONNX:", out if os.path.isfile(out) else default_onnx)

if __name__ == "__main__":
    main()
