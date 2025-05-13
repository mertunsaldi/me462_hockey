import cv2

def main():
    # 0 = default camera. Change to 1, 2, ... to use other connected cameras. 4 worked for me.

    cap = cv2.VideoCapture(4)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 'q' to exit.")
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame.")
            break

        # Optionally resize the frame
        # frame = cv2.resize(frame, (640, 480))

        # Display the resulting frame
        cv2.imshow('Camera Feed', frame)

        # Wait 1 ms and check if 'q' key is pressed to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture and destroy windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
