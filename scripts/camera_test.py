import time
from flask import Flask, Response
import cv2
from picamera2 import Picamera2

app = Flask(__name__)

# --- CAMERA SETUP (The Native Way) ---
print("📷 Starting Camera...")
picam2 = Picamera2()

# Configure the camera for low latency streaming
# We request BGR888 format which OpenCV loves
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "BGR888"}
)
picam2.configure(config)
picam2.start()
print("✅ Camera Running!")

def generate_frames():
    while True:
        try:
            # Capture the latest image directly into a numpy array
            # This is much faster than cv2.VideoCapture
            frame = picam2.capture_array()

            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            
            frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            print(f"Error capturing frame: {e}")
            time.sleep(0.1)

@app.route('/')
def index():
    return '''
    <html>
      <head>
        <title>Native Pi Car</title>
        <style>body { background-color: #222; color: #eee; text-align: center; font-family: sans-serif; }</style>
      </head>
      <body>
        <h1>Robot Vision</h1>
        <img src="/video_feed" style="border: 4px solid #4CAF50; border-radius: 8px;">
      </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Note: debug=False is important to prevent double-loading the camera
    app.run(host='0.0.0.0', port=5001, debug=False)
