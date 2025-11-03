from flask import Flask, request, jsonify
import datetime
import os
import json

app = Flask(__name__)

# Global session counter (untuk development testing)
CURRENT_SESSION = "session_001"

# Root endpoint untuk test di browser
@app.route('/')
def home():
    return '''
    <h1>ESP32-CAM Upload Server (Development)</h1>
    <p>Server is running!</p>
    <h3>Endpoints:</h3>
    <ul>
        <li>GET <code>/session/current</code> - Get current session ID</li>
        <li>POST <code>/upload/image</code> - Upload image (multipart/form-data)</li>
        <li>POST <code>/upload/meta</code> - Upload metadata (application/json)</li>
    </ul>
    <p>Images folder: <code>{}</code></p>
    <p>Metadata folder: <code>{}</code></p>
    '''.format(
        os.path.abspath(f'uploaded_images/{CURRENT_SESSION}'),
        os.path.abspath(f'uploaded_metadata/{CURRENT_SESSION}')
    )

@app.route('/session/current', methods=['GET'])
def get_session():
    """
    Endpoint untuk mendapatkan session_id saat ini.
    Simulasi backend API untuk development.
    """
    try:
        client_ip = request.remote_addr
        print(f"\n[{datetime.datetime.now().strftime('%H:%M:%S')}] Session request from: {client_ip}")

        response = {
            "status": "success",
            "message": "Session retrieved successfully",
            "data": {
                "last_completed_session": None,
                "next_session_id": CURRENT_SESSION,
                "is_first_session": True
            }
        }

        print(f"  -> Returning session_id: {CURRENT_SESSION}\n")
        return jsonify(response), 200

    except Exception as e:
        print(f"  -> ERROR: {str(e)}\n")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/upload/image', methods=['POST'])
def upload_image():
    """
    Endpoint untuk upload image dengan multipart/form-data.
    Body: photo_id (string), file (binary)
    """
    try:
        client_ip = request.remote_addr
        print(f"\n[{datetime.datetime.now().strftime('%H:%M:%S')}] Image upload from: {client_ip}")

        # Validasi photo_id
        photo_id = request.form.get('photo_id')
        if not photo_id:
            print(f"  -> ERROR: Missing photo_id in form data\n")
            return jsonify({"status": "error", "message": "Missing photo_id"}), 400

        # Validasi file
        if 'file' not in request.files:
            print(f"  -> ERROR: Missing file in form data\n")
            return jsonify({"status": "error", "message": "Missing file"}), 400

        file = request.files['file']
        if file.filename == '':
            print(f"  -> ERROR: Empty filename\n")
            return jsonify({"status": "error", "message": "Empty filename"}), 400

        # Create folder structure: uploaded_images/session_XXX/
        image_folder = f'uploaded_images/{CURRENT_SESSION}'
        if not os.path.exists(image_folder):
            os.makedirs(image_folder)
            print(f"  -> Created folder: {image_folder}")

        # Save dengan nama photo_id.jpg
        filename = f"{image_folder}/{photo_id}.jpg"
        file.save(filename)

        file_size = os.path.getsize(filename)
        print(f"  -> Photo ID: {photo_id}")
        print(f"  -> File size: {file_size} bytes")
        print(f"  -> SUCCESS! Image saved: {filename}\n")

        response = {
            "status": "success",
            "photo_id": photo_id,
            "session_id": CURRENT_SESSION,
            "path": f"{CURRENT_SESSION}/images/{photo_id}.jpg"
        }

        return jsonify(response), 200

    except Exception as e:
        print(f"  -> ERROR: {str(e)}\n")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/upload/meta', methods=['POST'])
def upload_metadata():
    """
    Endpoint untuk upload metadata dengan JSON body.
    Body: {"photo_id": "001", "session_id": "session_001", "group_id": "1"}
    """
    try:
        client_ip = request.remote_addr
        print(f"\n[{datetime.datetime.now().strftime('%H:%M:%S')}] Metadata upload from: {client_ip}")

        # Parse JSON body
        if not request.is_json:
            print(f"  -> ERROR: Content-Type is not application/json\n")
            return jsonify({"status": "error", "message": "Content-Type must be application/json"}), 400

        data = request.get_json()

        # Validasi required fields
        required_fields = ['photo_id', 'session_id', 'group_id']
        for field in required_fields:
            if field not in data:
                print(f"  -> ERROR: Missing field '{field}'\n")
                return jsonify({"status": "error", "message": f"Missing field: {field}"}), 400

        photo_id = data['photo_id']
        session_id = data['session_id']
        group_id = data['group_id']

        # Create folder structure: uploaded_metadata/session_XXX/
        metadata_folder = f'uploaded_metadata/{session_id}'
        if not os.path.exists(metadata_folder):
            os.makedirs(metadata_folder)
            print(f"  -> Created folder: {metadata_folder}")

        # Save metadata sebagai JSON file (untuk development testing)
        metadata_file = f"{metadata_folder}/{photo_id}.json"
        with open(metadata_file, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"  -> Photo ID: {photo_id}")
        print(f"  -> Session ID: {session_id}")
        print(f"  -> Group ID: {group_id}")
        print(f"  -> SUCCESS! Metadata saved: {metadata_file}\n")

        response = {
            "status": "success",
            "photo_id": photo_id,
            "session_id": session_id,
            "path": f"{session_id}/metadata/{photo_id}.json"
        }

        return jsonify(response), 200

    except Exception as e:
        print(f"  -> ERROR: {str(e)}\n")
        return jsonify({"status": "error", "message": str(e)}), 500

# Legacy endpoint (keep untuk backward compatibility)
@app.route('/upload', methods=['POST'])
def upload():
    try:
        # Log incoming request
        client_ip = request.remote_addr
        print(f"\n[{datetime.datetime.now().strftime('%H:%M:%S')}] [LEGACY] Upload request from: {client_ip}")

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