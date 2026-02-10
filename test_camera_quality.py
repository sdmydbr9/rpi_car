#!/usr/bin/env python3
"""
Test script for camera quality settings bug fix.
Verifies that camera resolution changes via WebSocket are properly accepted
and echoed back in WxH format (not falling back to 'low').

Usage:
    python3 test_camera_quality.py [--host HOST] [--port PORT]

Requirements:
    pip install python-socketio[client] requests
"""
import sys
import time
import json
import argparse
import threading

try:
    import socketio
except ImportError:
    print("❌ python-socketio not installed. Run: pip install 'python-socketio[client]'")
    sys.exit(1)

# ─────────────────────────────────────────────
# Test configuration
# ─────────────────────────────────────────────
TIMEOUT = 10  # seconds to wait for each response

class CameraQualityTest:
    def __init__(self, host, port):
        self.base_url = f"http://{host}:{port}"
        self.sio = socketio.Client(logger=False, engineio_logger=False)
        self.results = []
        self.response_event = threading.Event()
        self.last_response = None
        self.camera_specs = None
        self.specs_event = threading.Event()
        
        # Setup handlers
        self.sio.on('camera_config_response', self._on_config_response)
        self.sio.on('camera_specs_sync', self._on_specs_sync)
        self.sio.on('connect', self._on_connect)
    
    def _on_connect(self):
        print(f"✅ Connected to {self.base_url}")
    
    def _on_specs_sync(self, data):
        self.camera_specs = data
        print(f"📷 Camera specs received: {json.dumps(data, indent=2)}")
        self.specs_event.set()
    
    def _on_config_response(self, data):
        self.last_response = data
        print(f"   📨 Response: {json.dumps(data, indent=2)}")
        self.response_event.set()
    
    def connect(self):
        print(f"\n{'='*60}")
        print(f"🔌 Connecting to {self.base_url}...")
        print(f"{'='*60}")
        try:
            self.sio.connect(self.base_url, transports=['websocket'])
            # Wait for camera specs
            if not self.specs_event.wait(timeout=TIMEOUT):
                print("⚠️  No camera_specs_sync received (might be OK if camera not available)")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def disconnect(self):
        self.sio.disconnect()
        print("\n🔌 Disconnected")
    
    def send_config_and_wait(self, config: dict) -> dict:
        """Send camera_config_update and wait for response."""
        self.response_event.clear()
        self.last_response = None
        print(f"\n   📤 Sending: {json.dumps(config)}")
        self.sio.emit('camera_config_update', config)
        if not self.response_event.wait(timeout=TIMEOUT):
            return None
        return self.last_response
    
    def test(self, name: str, config: dict, expected_resolution: str, expect_status: str = 'ok'):
        """Run a single test case."""
        print(f"\n{'─'*60}")
        print(f"🧪 TEST: {name}")
        print(f"{'─'*60}")
        response = self.send_config_and_wait(config)
        
        if response is None:
            self._record(name, False, "No response received (timeout)")
            return
        
        actual_resolution = response.get('current_config', {}).get('resolution', 'N/A')
        actual_status = response.get('status', 'N/A')
        
        # Check resolution
        res_ok = actual_resolution == expected_resolution
        status_ok = actual_status == expect_status
        
        passed = res_ok and status_ok
        
        details = []
        if not res_ok:
            details.append(f"resolution: expected '{expected_resolution}', got '{actual_resolution}'")
        if not status_ok:
            details.append(f"status: expected '{expect_status}', got '{actual_status}'")
        
        self._record(name, passed, "; ".join(details) if details else f"resolution='{actual_resolution}', status='{actual_status}'")
    
    def _record(self, name, passed, details):
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"\n   {status}: {name}")
        if details:
            print(f"   Details: {details}")
        self.results.append((name, passed, details))
    
    def run_all_tests(self):
        """Run the full test suite."""
        if not self.connect():
            print("❌ Cannot run tests - connection failed")
            return False
        
        try:
            # ─── Test 1: WxH format '640x480' ───
            self.test(
                "WxH format: 640x480",
                {"resolution": "640x480", "jpeg_quality": 70, "framerate": 30},
                expected_resolution="640x480"
            )
            
            # ─── Test 2: WxH format '1920x1080' ───
            self.test(
                "WxH format: 1920x1080",
                {"resolution": "1920x1080", "jpeg_quality": 70, "framerate": 30},
                expected_resolution="1920x1080"
            )
            
            # ─── Test 3: WxH format '1296x972' ───
            self.test(
                "WxH format: 1296x972",
                {"resolution": "1296x972", "jpeg_quality": 70, "framerate": 30},
                expected_resolution="1296x972"
            )
            
            # ─── Test 4: Legacy key 'low' (backward compat) ───
            self.test(
                "Legacy key: 'low' → normalized to '640x480'",
                {"resolution": "low", "jpeg_quality": 70, "framerate": 30},
                expected_resolution="640x480"
            )
            
            # ─── Test 5: Legacy key 'high' (backward compat) ───
            self.test(
                "Legacy key: 'high' → normalized to '1920x1080'",
                {"resolution": "high", "jpeg_quality": 70, "framerate": 30},
                expected_resolution="1920x1080"
            )
            
            # ─── Test 6: Legacy key 'medium' (backward compat) ───
            self.test(
                "Legacy key: 'medium' → normalized to '1280x720'",
                {"resolution": "medium", "jpeg_quality": 70, "framerate": 30},
                expected_resolution="1280x720"
            )
            
            # ─── Test 7: JPEG quality change only ───
            self.test(
                "JPEG quality change only (resolution unchanged)",
                {"jpeg_quality": 90},
                expected_resolution="1280x720"  # unchanged from last test
            )
            
            # ─── Test 8: Reset back to 640x480 ───
            self.test(
                "Reset to 640x480",
                {"resolution": "640x480", "jpeg_quality": 70, "framerate": 30},
                expected_resolution="640x480"
            )
            
            # ─── Test 9: Verify persistence ───
            print(f"\n{'─'*60}")
            print(f"🧪 TEST: Verify persistence file")
            print(f"{'─'*60}")
            try:
                import os
                config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.camera_config.json')
                if os.path.exists(config_path):
                    with open(config_path, 'r') as f:
                        persisted = json.load(f)
                    actual = persisted.get('resolution', 'N/A')
                    passed = actual == '640x480'
                    self._record(
                        "Persistence: .camera_config.json has correct resolution",
                        passed,
                        f"persisted resolution='{actual}'"
                    )
                else:
                    self._record("Persistence: .camera_config.json exists", False, "File not found")
            except Exception as e:
                self._record("Persistence check", False, str(e))
            
        finally:
            self.disconnect()
        
        return self.print_summary()
    
    def print_summary(self):
        """Print test summary and return True if all passed."""
        print(f"\n{'='*60}")
        print(f"📊 TEST SUMMARY")
        print(f"{'='*60}")
        
        passed = sum(1 for _, p, _ in self.results if p)
        total = len(self.results)
        
        for name, p, details in self.results:
            status = "✅" if p else "❌"
            print(f"  {status} {name}")
        
        print(f"\n  {'='*40}")
        all_passed = passed == total
        if all_passed:
            print(f"  ✅ ALL {total} TESTS PASSED")
        else:
            print(f"  ❌ {passed}/{total} PASSED, {total - passed} FAILED")
        print(f"  {'='*40}\n")
        
        return all_passed


def main():
    parser = argparse.ArgumentParser(description="Test camera quality settings")
    parser.add_argument('--host', default='localhost', help='Server host (default: localhost)')
    parser.add_argument('--port', default=5000, type=int, help='Server port (default: 5000)')
    args = parser.parse_args()
    
    tester = CameraQualityTest(args.host, args.port)
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
