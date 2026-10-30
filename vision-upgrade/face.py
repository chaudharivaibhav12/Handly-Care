"""Find faces and the forehead point in a color image.

Uses YuNet, a small face detector that ships with OpenCV (the model file is in models/,
so nothing is downloaded). For each face it gives a box and 5 landmarks: both eyes, the
nose tip and both mouth corners. The forehead is found from the landmarks:

    forehead = middle of the eyes - 0.55 * (middle of the mouth - middle of the eyes)

so it stays on the forehead when the head tilts.

Try it with your laptop webcam (no robot needed):
    python face.py --webcam
    python face.py --image some_photo.jpg
"""
from __future__ import annotations

import argparse
import os
from dataclasses import dataclass

import cv2
import numpy as np

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "face_detection_yunet_2023mar.onnx")

try:   # OpenCV 5 prints a harmless warning about DNN targets; keep the console readable
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except AttributeError:
    pass


@dataclass
class Face:
    box: tuple              # x, y, w, h (pixels)
    score: float
    landmarks: np.ndarray   # 5 x 2: right eye, left eye, nose tip, right mouth corner, left mouth corner
    forehead: tuple         # (u, v) pixel
    eye_mouth_px: float     # eye-to-mouth distance in pixels: how big the face looks

    @property
    def patch_radius(self) -> int:
        """Window size for reading forehead depth: about 1 cm on a real face."""
        return max(3, int(round(0.12 * self.eye_mouth_px)))

    @property
    def mid_face(self) -> tuple:
        """A point between the eyes and the mouth (nose/cheeks): a backup depth reading."""
        eyes = (self.landmarks[0] + self.landmarks[1]) / 2
        mouth = (self.landmarks[3] + self.landmarks[4]) / 2
        p = (eyes + mouth) / 2
        return float(p[0]), float(p[1])


def forehead_from_landmarks(landmarks: np.ndarray, up_ratio: float = 0.55):
    """Forehead pixel and face size from YuNet's 5 landmarks."""
    lm = np.asarray(landmarks, dtype=np.float64).reshape(5, 2)
    eyes = (lm[0] + lm[1]) / 2
    mouth = (lm[3] + lm[4]) / 2
    down = mouth - eyes
    fh = eyes - up_ratio * down
    return (float(fh[0]), float(fh[1])), float(np.linalg.norm(down))


class FaceFinder:
    def __init__(self, model_path: str = MODEL, score_threshold: float = 0.75, up_ratio: float = 0.55):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"face model not found: {model_path}")
        if not hasattr(cv2, "FaceDetectorYN"):
            raise RuntimeError("this OpenCV has no FaceDetectorYN; run: pip install -U opencv-python")
        self.up_ratio = up_ratio
        self._det = cv2.FaceDetectorYN.create(model_path, "", (320, 320), score_threshold, 0.3, 5000)
        self._size = None

    def detect(self, bgr: np.ndarray) -> list[Face]:
        """All faces, biggest first."""
        h, w = bgr.shape[:2]
        if self._size != (w, h):
            self._det.setInputSize((w, h))
            self._size = (w, h)
        _, raw = self._det.detect(bgr)
        faces = []
        if raw is None:
            return faces
        for f in raw:
            lm = f[4:14].reshape(5, 2)
            forehead, eye_mouth = forehead_from_landmarks(lm, self.up_ratio)
            if eye_mouth < 4:   # a few pixels: too small to be useful
                continue
            faces.append(Face(tuple(float(v) for v in f[:4]), float(f[14]), lm.astype(np.float64),
                              forehead, eye_mouth))
        faces.sort(key=lambda fc: -fc.box[2] * fc.box[3])
        return faces


def draw_faces(img: np.ndarray, faces: list[Face], chosen: Face | None = None, lines=()) -> np.ndarray:
    out = img.copy()
    for fc in faces:
        x, y, w, h = (int(round(v)) for v in fc.box)
        color = (0, 220, 0) if fc is chosen else (160, 160, 160)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        for px, py in fc.landmarks:
            cv2.circle(out, (int(px), int(py)), 2, (255, 200, 0), -1)
        u, v = (int(round(c)) for c in fc.forehead)
        cv2.drawMarker(out, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.circle(out, (u, v), fc.patch_radius, (0, 0, 255), 1)
    for i, text in enumerate(lines):
        cv2.putText(out, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(out, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out


def _main():
    ap = argparse.ArgumentParser(description="Show the face box, landmarks and forehead point.")
    ap.add_argument("--webcam", type=int, nargs="?", const=0, help="camera index (default 0)")
    ap.add_argument("--image", help="a photo to try instead of the webcam")
    ap.add_argument("--out", default="out/face_test.png")
    args = ap.parse_args()
    finder = FaceFinder()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit(f"can't read {args.image}")
        faces = finder.detect(img)
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        cv2.imwrite(args.out, draw_faces(img, faces, faces[0] if faces else None, [f"{len(faces)} face(s)"]))
        print(f"{len(faces)} face(s). Saved {args.out}")
        return

    cap = cv2.VideoCapture(args.webcam if args.webcam is not None else 0)
    if not cap.isOpened():
        raise SystemExit("can't open the webcam")
    print("Press q to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        faces = finder.detect(frame)
        text = [f"{len(faces)} face(s)"] + ([f"forehead pixel {faces[0].forehead[0]:.0f}, {faces[0].forehead[1]:.0f}"] if faces else [])
        cv2.imshow("forehead finder", draw_faces(frame, faces, faces[0] if faces else None, text))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    _main()
