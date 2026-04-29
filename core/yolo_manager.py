#!/usr/bin/env python3
# coding=utf-8

import base64
import io
import threading
from typing import Dict, Any, List, Optional, Callable


class YOLOManager:
    def __init__(self, model_name: str = "yolov8n.pt", confidence: float = 0.5, debug: bool = False):
        self.model_name = model_name
        self.confidence = confidence
        self.debug_mode = debug

        self.model = None
        self.is_available = False
        self.is_running = False

        # Callbacks
        self.detection_callback: Optional[Callable[[List[Dict[str, Any]]], None]] = None
        self.status_callback: Optional[Callable[[str], None]] = None

        # Load model in background so startup isn't blocked
        threading.Thread(target=self._load_model, daemon=True).start()

    def _load_model(self):
        try:
            from ultralytics import YOLO
            if self.debug_mode:
                print(f"🔍 Loading YOLO model: {self.model_name}")
            self.model = YOLO(self.model_name)
            self.is_available = True
            if self.debug_mode:
                print(f"✅ YOLO model loaded: {self.model_name}")
            if self.status_callback:
                self.status_callback("Ready")
        except ImportError:
            print("⚠️  ultralytics not installed — run: pip install ultralytics")
            if self.status_callback:
                self.status_callback("ultralytics not installed")
        except Exception as e:
            print(f"❌ YOLO model load error: {e}")
            if self.status_callback:
                self.status_callback(f"Error: {e}")

    def set_detection_callback(self, callback: Callable[[List[Dict[str, Any]]], None]):
        self.detection_callback = callback

    def set_status_callback(self, callback: Callable[[str], None]):
        self.status_callback = callback

    def set_confidence(self, confidence: float):
        self.confidence = max(0.1, min(1.0, confidence))

    def run_detection(self, image_data: str) -> Dict[str, Any]:
        """
        Run YOLO detection on a base64-encoded JPEG image.
        Fires detection_callback with results when done.
        Returns immediately (runs in background thread).
        """
        if not self.is_available or self.model is None:
            return {"success": False, "error": "YOLO model not available"}

        if self.is_running:
            return {"success": False, "error": "Detection already running"}

        def _detect():
            self.is_running = True
            try:
                from PIL import Image

                img_bytes = base64.b64decode(image_data)
                pil_image = Image.open(io.BytesIO(img_bytes)).convert("RGB")

                results = self.model(pil_image, conf=self.confidence, verbose=False)

                detections = []
                for result in results:
                    boxes = result.boxes
                    if boxes is None:
                        continue
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        label = result.names[cls_id]
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                        detections.append({
                            "label": label,
                            "confidence": round(conf, 3),
                            "bbox": [round(x1), round(y1), round(x2), round(y2)]
                        })

                if self.debug_mode:
                    print(f"🔍 YOLO: {len(detections)} detections")
                    for d in detections:
                        print(f"   {d['label']} ({d['confidence']:.2f})")

                if self.detection_callback:
                    self.detection_callback(detections)

            except Exception as e:
                if self.debug_mode:
                    print(f"❌ YOLO detection error: {e}")
            finally:
                self.is_running = False

        threading.Thread(target=_detect, daemon=True).start()
        return {"success": True, "message": "Detection started"}

    def cleanup(self):
        self.model = None
        self.is_available = False
