#!/usr/bin/env python3
"""YOLOv8 ONNX 탐지기 (onnxruntime + OpenCV + numpy만 사용, ultralytics·GPU 불필요)

로봇 docker(JetRover, Jetson Nano JetPack 4.6)의 torch는 CPU 전용이고 ultralytics가 없어서,
Colab에서 best.pt를 ONNX로 내보내 이미 설치된 onnxruntime으로 돌린다.
나중에 rovpang_vision/object_detector.py가 이 클래스를 그대로 가져다 쓴다.

  det = YoloOnnx('best.onnx', conf=0.5)
  for d in det.detect(bgr_image):   # d = (class_name, confidence, x1, y1, x2, y2)
      ...
"""
import ast

import cv2
import numpy as np
import onnxruntime as ort


class YoloOnnx:
    def __init__(self, model_path, conf=0.5, iou=0.45, threads=4):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads
        self.session = ort.InferenceSession(model_path, sess_options=opts,
                                            providers=['CPUExecutionProvider'])
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.size = int(inp.shape[2])            # 내보낼 때의 imgsz (예: 320, 640)
        self.conf = conf
        self.iou = iou
        meta = self.session.get_modelmeta().custom_metadata_map
        self.names = ast.literal_eval(meta['names']) if 'names' in meta else {}

    def letterbox(self, img):
        """비율을 유지해 size x size 로 줄이고 남는 곳은 회색(114)으로 채운다."""
        h, w = img.shape[:2]
        r = min(self.size / h, self.size / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        top = (self.size - nh) // 2
        left = (self.size - nw) // 2
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[top:top + nh, left:left + nw] = resized
        return canvas, r, left, top

    def detect(self, bgr):
        """BGR 영상 -> [(이름, 신뢰도, x1, y1, x2, y2), ...] (원본 영상 픽셀 좌표)"""
        img, r, left, top = self.letterbox(bgr)
        blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0   # BGR->RGB, NCHW
        out = self.session.run(None, {self.input_name: np.ascontiguousarray(blob)})[0][0]
        out = out.T                                  # (후보 수, 4 + 클래스 수): cx, cy, w, h, 점수들
        scores = out[:, 4:]
        cls = scores.argmax(axis=1)
        confs = scores[np.arange(len(scores)), cls]
        keep = confs >= self.conf
        if not keep.any():
            return []
        boxes, confs, cls = out[keep, :4], confs[keep], cls[keep]
        x1 = (boxes[:, 0] - boxes[:, 2] / 2 - left) / r
        y1 = (boxes[:, 1] - boxes[:, 3] / 2 - top) / r
        w = boxes[:, 2] / r
        h = boxes[:, 3] / r
        idx = cv2.dnn.NMSBoxes(np.stack([x1, y1, w, h], 1).tolist(), confs.tolist(), self.conf, self.iou)
        H, W = bgr.shape[:2]
        result = []
        for i in np.array(idx).flatten():
            bx1 = int(np.clip(x1[i], 0, W - 1))
            by1 = int(np.clip(y1[i], 0, H - 1))
            bx2 = int(np.clip(x1[i] + w[i], 0, W - 1))
            by2 = int(np.clip(y1[i] + h[i], 0, H - 1))
            name = self.names.get(int(cls[i]), str(int(cls[i])))
            result.append((name, float(confs[i]), bx1, by1, bx2, by2))
        return result
