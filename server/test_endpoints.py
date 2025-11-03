"""
Test script untuk menguji semua endpoints Flask server
Usage: python test_endpoints.py
"""

import requests
import io
import json
from PIL import Image

# Server URL
BASE_URL = "http://localhost:5000"

def create_test_image():
    """Create a simple test image in memory"""
    img = Image.new('RGB', (640, 480), color=(73, 109, 137))
    img_bytes = io.BytesIO()
    img.save(img_bytes, format='JPEG')
    img_bytes.seek(0)
    return img_bytes

def test_session_current():
    """Test GET /session/current"""
    print("\n" + "="*60)
    print("TEST 1: GET /session/current")
    print("="*60)

    try:
        response = requests.get(f"{BASE_URL}/session/current")
        print(f"Status Code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")

        if response.status_code == 200:
            data = response.json()
            session_id = data['data']['next_session_id']
            print(f"\n[SUCCESS] Session ID: {session_id}")
            return session_id
        else:
            print(f"\n[FAILED] Status: {response.status_code}")
            return None

    except Exception as e:
        print(f"\n[ERROR] {str(e)}")
        return None

def test_upload_image(photo_id, session_id):
    """Test POST /upload/image"""
    print("\n" + "="*60)
    print(f"TEST 2: POST /upload/image (photo_id={photo_id})")
    print("="*60)

    try:
        # Create test image
        img_bytes = create_test_image()

        # Prepare multipart form data
        files = {
            'file': (f'{photo_id}.jpg', img_bytes, 'image/jpeg')
        }
        data = {
            'photo_id': photo_id
        }

        response = requests.post(f"{BASE_URL}/upload/image", files=files, data=data)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")

        if response.status_code == 200:
            print(f"\n[SUCCESS] Image uploaded: {photo_id}.jpg")
            return True
        else:
            print(f"\n[FAILED] Status: {response.status_code}")
            return False

    except Exception as e:
        print(f"\n[ERROR] {str(e)}")
        return False

def test_upload_metadata(photo_id, session_id, group_id):
    """Test POST /upload/meta"""
    print("\n" + "="*60)
    print(f"TEST 3: POST /upload/meta (photo_id={photo_id}, group_id={group_id})")
    print("="*60)

    try:
        # Prepare JSON payload
        payload = {
            "photo_id": photo_id,
            "session_id": session_id,
            "group_id": str(group_id)
        }

        headers = {
            'Content-Type': 'application/json'
        }

        response = requests.post(f"{BASE_URL}/upload/meta", json=payload, headers=headers)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")

        if response.status_code == 200:
            print(f"\n[SUCCESS] Metadata uploaded for {photo_id}")
            return True
        else:
            print(f"\n[FAILED] Status: {response.status_code}")
            return False

    except Exception as e:
        print(f"\n[ERROR] {str(e)}")
        return False

def test_complete_workflow():
    """Test complete workflow: session -> image -> metadata"""
    print("\n" + "="*60)
    print("TEST 4: Complete Workflow (3 photos, 2 groups)")
    print("="*60)

    # Step 1: Get session ID
    session_id = test_session_current()
    if not session_id:
        print("\n[FAILED] Workflow FAILED at session retrieval")
        return False

    # Step 2: Upload multiple images with metadata
    test_cases = [
        ("001", 1),  # Photo 1, Group 1
        ("002", 1),  # Photo 2, Group 1
        ("003", 2),  # Photo 3, Group 2 (setelah belok)
    ]

    success_count = 0
    for photo_id, group_id in test_cases:
        # Upload image
        if test_upload_image(photo_id, session_id):
            # Upload metadata (only if image success)
            if test_upload_metadata(photo_id, session_id, group_id):
                success_count += 1

    print("\n" + "="*60)
    print(f"Workflow Summary: {success_count}/{len(test_cases)} successful")
    print("="*60)

    if success_count == len(test_cases):
        print("\n[SUCCESS] Complete workflow SUCCESS!")
        print("\nCheck folders:")
        print("  - uploaded_images/session_001/")
        print("  - uploaded_metadata/session_001/")
        return True
    else:
        print("\n[FAILED] Some tests failed")
        return False

def main():
    print("\n" + "="*60)
    print("ESP32-CAM Upload Server - Endpoint Testing")
    print("="*60)
    print(f"Target Server: {BASE_URL}")
    print("\nMake sure Flask server is running!")
    print("Run: python server.py")
    print("="*60)

    input("\nPress Enter to start testing...")

    # Run complete workflow test
    test_complete_workflow()

    print("\n" + "="*60)
    print("Testing completed!")
    print("="*60)

if __name__ == "__main__":
    main()
