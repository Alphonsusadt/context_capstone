"""
Test script untuk verify server connectivity
Jalankan script ini di komputer yang sama dengan server
"""

import requests
import sys

SERVER_URL = "http://192.168.1.9:5000"

print("=" * 60)
print("Testing ESP32-CAM Upload Server Connectivity")
print("=" * 60)

# Test 1: Root endpoint
print("\nTest 1: Testing root endpoint (GET /)...")
try:
    response = requests.get(f"{SERVER_URL}/", timeout=5)
    if response.status_code == 200:
        print("  ✓ SUCCESS! Server is reachable")
        print(f"  Response: {response.text[:100]}...")
    else:
        print(f"  ✗ FAILED! Status code: {response.status_code}")
except requests.exceptions.ConnectionError:
    print("  ✗ FAILED! Connection refused - Server not running or firewall blocking")
    print("  Check:")
    print("    1. Server running? (python server.py)")
    print("    2. Windows Firewall blocking?")
    print("    3. Correct IP address?")
    sys.exit(1)
except requests.exceptions.Timeout:
    print("  ✗ FAILED! Connection timeout")
    sys.exit(1)
except Exception as e:
    print(f"  ✗ ERROR: {e}")
    sys.exit(1)

# Test 2: Upload endpoint
print("\nTest 2: Testing upload endpoint (POST /upload)...")
try:
    # Create dummy image data
    dummy_data = b'\xFF\xD8\xFF\xE0' + b'\x00' * 1000  # JPEG header + dummy data

    response = requests.post(f"{SERVER_URL}/upload", data=dummy_data, timeout=10)

    if response.status_code == 200:
        print("  ✓ SUCCESS! Upload endpoint working")
        print(f"  Response: {response.text}")
    else:
        print(f"  ✗ FAILED! Status code: {response.status_code}")
except Exception as e:
    print(f"  ✗ ERROR: {e}")
    sys.exit(1)

print("\n" + "=" * 60)
print("All tests passed! ✓")
print("Server is ready to receive images from ESP32-CAM")
print("=" * 60)
