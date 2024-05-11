import cv2
import os
from datetime import datetime
from camthreader import *
def record_video():
    cap = WebcamVideoStream(src=0).start()
    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    downloads_folder = os.path.join(os.path.expanduser('~'), 'Downloads')
    video_filename = os.path.join(downloads_folder, f"video_{timestamp}.avi")
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_filename, fourcc, 20.0, (160, 120))
    print("Comenzando a grabar... (Presiona 'q' para detener)")

    while True:
        frame = cap.read()
        out.write(frame)
        cv2.imshow('Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"¡Video guardado como {video_filename}!")

if __name__ == "__main__":
    record_video()