import subprocess
import time
import sys

class PiCarNetworkManager:
    def __init__(self):
        # Your exact NetworkManager profile names
        self.hotspot_profile = "CarHotspot"
        self.wifi_profile = "wifi2"

    def enable_hotspot(self):
        """Switches the PiCar to Hotspot mode."""
        print(f"📡 Disabling autoconnect for '{self.wifi_profile}'...")
        try:
            # 1. Disable Wi-Fi autoconnect so it doesn't fight the hotspot (Blocking command)
            subprocess.run(
                ["sudo", "nmcli", "connection", "modify", self.wifi_profile, "autoconnect", "no"],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            print(f"📡 Disconnecting Wi-Fi and starting Hotspot: '{self.hotspot_profile}'...")
            # 2. Bring up hotspot (Non-blocking command so the script survives the drop)
            subprocess.Popen(
                ["sudo", "nmcli", "connection", "up", self.hotspot_profile],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("✅ Hotspot activation command sent.")
            return True
        except Exception as e:
            print(f"❌ Failed to activate hotspot: {e}")
            return False

    def enable_wifi(self):
        """Switches the PiCar back to home Wi-Fi."""
        print(f"🌐 Enabling autoconnect for '{self.wifi_profile}'...")
        try:
            # 1. Re-enable Wi-Fi autoconnect (Blocking command)
            subprocess.run(
                ["sudo", "nmcli", "connection", "modify", self.wifi_profile, "autoconnect", "yes"],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            print(f"🌐 Disconnecting Hotspot and joining Wi-Fi: '{self.wifi_profile}'...")
            # 2. Bring up Wi-Fi (Non-blocking command)
            subprocess.Popen(
                ["sudo", "nmcli", "connection", "up", self.wifi_profile],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("✅ Wi-Fi activation command sent.")
            return True
        except Exception as e:
            print(f"❌ Failed to activate Wi-Fi: {e}")
            return False

# ==========================================
# 🛠️ TERMINAL SIMULATION/TESTING BLOCK
# ==========================================
if __name__ == "__main__":
    # This allows you to run: python3 network_core.py hotspot
    # or: python3 network_core.py wifi

    manager = PiCarNetworkManager()

    if len(sys.argv) > 1:
        command = sys.argv[1].lower()

        if command == "hotspot":
            manager.enable_hotspot()
        elif command == "wifi":
            manager.enable_wifi()
        else:
            print("⚠️ Unknown command. Use 'hotspot' or 'wifi'.")
    else:
        print("🚗 PiCar Network Manager Simulator")
        print("Simulating a switch to Hotspot in 3 seconds...")
        time.sleep(3)
        manager.enable_hotspot()

        # Note: If you run this simulation over SSH via Wi-Fi,
        # your terminal will freeze right here as the network drops!
