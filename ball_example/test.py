import cv2
import numpy as np

# All built-in ArUco & AprilTag dictionaries to try
ALL_DICTS = {
    "DICT_4X4_50":      cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":     cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":     cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000":    cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50":      cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100":     cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250":     cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000":    cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50":      cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100":     cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250":     cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000":    cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50":      cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100":     cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250":     cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000":    cv2.aruco.DICT_7X7_1000,
    "APRILTAG_16h5":    cv2.aruco.DICT_APRILTAG_16h5,
    "APRILTAG_25h9":    cv2.aruco.DICT_APRILTAG_25h9,
    "APRILTAG_36h10":   cv2.aruco.DICT_APRILTAG_36h10,
    "APRILTAG_36h11":   cv2.aruco.DICT_APRILTAG_36h11,
}

def preprocess(gray):
    """Simple Otsu threshold + small cleanup."""
    # blur to reduce noise
    gray = cv2.medianBlur(gray, 5)
    # Otsu binarization
    _, thresh = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    # close small holes
    kernel = np.ones((3,3), np.uint8)
    clean  = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
    return clean

def main():
    # default detector params
    params = cv2.aruco.DetectorParameters()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⛔ Cannot open camera")
        return

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        thresh = preprocess(gray)

        vis = frame.copy()
        found = False

        # try each dictionary until one works
        for name, dict_id in ALL_DICTS.items():
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            corners, ids, _ = cv2.aruco.detectMarkers(
                thresh, aruco_dict, parameters=params
            )
            if ids is not None:
                # annotate detection
                print(f"✔️ Detected with {name}, IDs: {ids.flatten().tolist()}")
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                # draw corners & centroids
                for pts, mid in zip(corners, ids.flatten()):
                    pts = pts.reshape((4,2)).astype(int)
                    for i, (x,y) in enumerate(pts):
                        cv2.circle(vis, (x,y), 4, (0,255,0), -1)
                        cv2.putText(vis, str(i), (x+5,y-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0,255,0), 2)
                    cx, cy = pts.mean(axis=0).astype(int)
                    cv2.circle(vis, (cx,cy), 5, (0,0,255), -1)
                    cv2.putText(vis, f"id={mid}", (cx+5,cy+5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0,0,255), 2)
                # show which dict we used
                cv2.putText(vis, name, (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (255,0,0), 2)
                found = True
                break

        if not found:
            cv2.putText(vis, "No marker detected", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0,0,255), 2)

        # display
        cv2.imshow("Live (press q to quit)", vis)
        cv2.imshow("Thresholded", thresh)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()