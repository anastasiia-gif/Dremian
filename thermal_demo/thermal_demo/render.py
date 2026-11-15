"""
render.py — turn a grayscale thermal frame + detection state into an annotated
color JPEG for the web UI.

Uses a thermal-style colormap so it reads as a thermal camera, draws the
detection box, the Kalman-predicted position, a lock indicator and a HUD.
Pure numpy for the colormap; uses cv2 only for drawing + JPEG encode (both are
available on Raspberry Pi OS via `pip install opencv-python-headless`).
"""

import numpy as np

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


def _thermal_colormap(gray_u8):
    """Map 0-255 grayscale to a thermal palette (black->purple->red->yellow->white)."""
    if HAVE_CV2:
        return cv2.applyColorMap(gray_u8, cv2.COLORMAP_INFERNO)
    # numpy fallback: simple inferno-like ramp
    g = gray_u8.astype(np.float32) / 255.0
    r = np.clip(1.5 * g, 0, 1)
    gr = np.clip(1.5 * g - 0.4, 0, 1)
    b = np.clip(2.0 * g - 1.0, 0, 1) + np.clip(0.5 - g, 0, 0.5)
    img = np.stack([b, gr, r], axis=-1)  # BGR
    return (img * 255).astype(np.uint8)


def annotate(frame_u8, meas, state, locked, frame_idx,
             algo_ms, e2e_ms, cmd_dps, lock_count, total_count):
    """Return annotated BGR image (numpy HxWx3 uint8)."""
    img = _thermal_colormap(frame_u8)
    h, w = frame_u8.shape

    if HAVE_CV2:
        # Kalman-predicted track position (green crosshair).
        if state is not None:
            sx, sy = int(state[0]), int(state[1])
            color = (60, 220, 60) if locked else (60, 200, 220)
            cv2.drawMarker(img, (sx, sy), color, cv2.MARKER_CROSS, 22, 2)
            # detection box around the lock
            if locked:
                cv2.rectangle(img, (sx - 16, sy - 16), (sx + 16, sy + 16), color, 2)
                cv2.putText(img, "TARGET LOCK", (sx + 20, sy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        # raw detection (small orange dot) if present
        if meas is not None:
            mx, my = int(meas[0]), int(meas[1])
            cv2.circle(img, (mx, my), 3, (40, 140, 240), -1)

        # HUD bar
        cv2.rectangle(img, (0, 0), (w, 54), (20, 20, 20), -1)
        state_txt = "LOCKED" if locked else "SEARCHING"
        scol = (60, 220, 60) if locked else (60, 200, 240)
        cv2.putText(img, f"{state_txt}", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, scol, 2, cv2.LINE_AA)
        cv2.putText(img, f"frame {frame_idx}   lock {lock_count}/{total_count}",
                    (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, f"algo {algo_ms:4.1f} ms   e2e {e2e_ms:4.1f} ms",
                    (w - 230, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"PN cmd {cmd_dps:+5.1f} deg/s",
                    (w - 230, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 220, 180), 1, cv2.LINE_AA)
    return img


def to_jpeg(img_bgr, quality=80):
    """Encode BGR image to JPEG bytes."""
    if HAVE_CV2:
        ok, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            return buf.tobytes()
    # Fallback: raw PPM (browsers don't show PPM, but keeps server alive without cv2)
    return None
