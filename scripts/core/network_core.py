"""Minimal NetworkManager profile switching for the disarmed control server."""

from __future__ import annotations

import os
import subprocess


class PiCarNetworkManager:
    def __init__(self) -> None:
        self.hotspot_profile = os.environ.get(
            "RC_HOTSPOT_PROFILE", "CarHotspot"
        )
        self.wifi_profile = os.environ.get("RC_WIFI_PROFILE", "wifi2")

    @staticmethod
    def _modify_autoconnect(profile: str, enabled: bool) -> None:
        subprocess.run(
            [
                "sudo",
                "nmcli",
                "connection",
                "modify",
                profile,
                "autoconnect",
                "yes" if enabled else "no",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )

    @staticmethod
    def _activate(profile: str) -> None:
        subprocess.Popen(
            ["sudo", "nmcli", "connection", "up", profile],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    def enable_hotspot(self) -> bool:
        try:
            self._modify_autoconnect(self.wifi_profile, False)
            self._activate(self.hotspot_profile)
            return True
        except (OSError, subprocess.SubprocessError):
            return False

    def enable_wifi(self) -> bool:
        try:
            self._modify_autoconnect(self.wifi_profile, True)
            self._activate(self.wifi_profile)
            return True
        except (OSError, subprocess.SubprocessError):
            return False
