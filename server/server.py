from flask import Flask, request
import datetime
import os

app = Flask(__name__)

# Root endpoint untuk test di browser
@app.route('/')
def home():
    return '''
    <h1>ESP32-CAM Upload Server</h1>
    <p>Server is running!</p>
    <p>POST images to: <code>/upload</code></p>
    <p>Images will be saved in: <code>{}</code></p>
    '''.format(os.path.abspath('uploaded_images'))

@app.route('/upload', methods=['POST'])
def upload():
    try:
        # Log incoming request
        client_ip = request.remote_addr
        print(f"\n[{datetime.datetime.now().strftime('%H:%M:%S')}] Upload request from: {client_ip}")

        # Create folder if not exists
        if not os.path.exists('uploaded_images'):
            os.makedirs('uploaded_images')
            print(f"  -> Created folder: uploaded_images")

        # Generate filename dengan timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"uploaded_images/image_{timestamp}.jpg"

        # Save image
        data = request.data
        print(f"  -> Receiving {len(data)} bytes...")

        with open(filename, 'wb') as f:
            f.write(data)

        # Log success
        print(f"  -> SUCCESS! Image saved: {filename}\n")

        return 'OK', 200

    except Exception as e:
        print(f"  -> ERROR: {str(e)}\n")
        return f'ERROR: {str(e)}', 500

if __name__ == '__main__':
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    print("=" * 60)
    print("ESP32-CAM Upload Server Starting...")
    print(f"Hostname: {hostname}")
    print(f"Local IP: {local_ip}")
    print("Server will listen on: http://0.0.0.0:5000")
    print("")
    print("Test URLs:")
    print(f"  - http://localhost:5000")
    print(f"  - http://127.0.0.1:5000")
    print(f"  - http://{local_ip}:5000")
    print("")
    print("Upload endpoint: POST /upload")
    print("=" * 60)

    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        print(f"ERROR: Server failed to start - {e}")
        input("Press Enter to exit...")