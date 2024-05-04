import cv2
import os
from datetime import datetime
from camthreader import *
def mouse_callback(event, x, y, flags, param):
    global capture_requested
    if event == cv2.EVENT_LBUTTONDOWN:
        capture_requested = True
def take_photo():
    cap = WebcamVideoStream(src=0).start()
    cv2.namedWindow("camara")
    cv2.setMouseCallback("camara", mouse_callback)
    global capture_requested
    capture_requested = False

    while True:
        frame = cap.read()
        cv2.imshow("camara", frame)
        if capture_requested:
            now = datetime.now()
            timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
            downloads_folder = os.path.join(os.path.expanduser('~'), 'Downloads')
            filename = os.path.join(downloads_folder, f"foto_{timestamp}.jpg")
            cv2.imwrite(filename, frame)
            print(f"¡Foto tomada y guardada como {filename}!")
            capture_requested = False
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("¡Listo!")
    take_photo()