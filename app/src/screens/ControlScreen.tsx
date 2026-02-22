import React, { useState, useEffect } from 'react';
import {
  View,
  Text,
  TextInput,
  TouchableOpacity,
  StyleSheet,
  Alert,
  SafeAreaView,
  ScrollView,
} from 'react-native';
import * as socketClient from '../lib/socketClient';

interface ControlState {
  steeringAngle: number;
  throttle: boolean;
  brake: boolean;
  gear: string;
  speed: number;
  rpm: number;
  temperature: number;
}

export default function ControlScreen() {
  const [serverIp, setServerIp] = useState('192.168.4.1');
  const [isConnected, setIsConnected] = useState(false);
  const [controlState, setControlState] = useState<ControlState>({
    steeringAngle: 0,
    throttle: false,
    brake: false,
    gear: 'N',
    speed: 0,
    rpm: 0,
    temperature: 0,
  });

  useEffect(() => {
    // Set up telemetry callback
    socketClient.onTelemetry((data) => {
      setControlState((prev) => ({
        ...prev,
        speed: data.speed,
        rpm: data.rpm,
        gear: data.gear,
        steeringAngle: data.steer_angle,
        temperature: data.temperature || 0,
      }));
    });
  }, []);

  useEffect(() => {
    // Subscribe to connection state changes (handles hotspot/no-internet scenarios)
    socketClient.onConnectionStateChange((connected) => {
      console.log(`[ControlScreen] Connection state changed: ${connected ? 'CONNECTED' : 'DISCONNECTED'}`);
      setIsConnected(connected);
    });
  }, []);

  const handleConnect = async () => {
    try {
      await socketClient.connectToServer(serverIp);
      setIsConnected(true);
      Alert.alert('Connected', `Successfully connected to ${serverIp}`);
    } catch (error) {
      Alert.alert('Connection Failed', `Could not connect to ${serverIp}`);
    }
  };

  const handleDisconnect = () => {
    socketClient.disconnectFromServer();
    setIsConnected(false);
    Alert.alert('Disconnected', 'Disconnected from server');
  };

  const handleThrottle = (pressed: boolean) => {
    setControlState((prev) => ({ ...prev, throttle: pressed }));
    socketClient.emitThrottle(pressed);
  };

  const handleBrake = (pressed: boolean) => {
    setControlState((prev) => ({ ...prev, brake: pressed }));
    socketClient.emitBrake(pressed);
  };

  const handleSteering = (direction: 'left' | 'right' | 'center') => {
    const angles = { left: -45, center: 0, right: 45 };
    const angle = angles[direction];
    setControlState((prev) => ({ ...prev, steeringAngle: angle }));
    socketClient.emitSteering(angle);
  };

  const handleGearChange = (gear: string) => {
    setControlState((prev) => ({ ...prev, gear }));
    socketClient.emitGearChange(gear);
  };

  return (
    <SafeAreaView style={styles.container}>
      <ScrollView contentContainerStyle={styles.scrollContent}>
        <Text style={styles.title}>RPi Car Control</Text>

        {/* Connection Section */}
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Connection</Text>
          <TextInput
            style={styles.input}
            placeholder="Server IP"
            value={serverIp}
            onChangeText={setServerIp}
            editable={!isConnected}
          />
          <TouchableOpacity
            style={[styles.button, isConnected && styles.buttonDisconnect]}
            onPress={isConnected ? handleDisconnect : handleConnect}
          >
            <Text style={styles.buttonText}>
              {isConnected ? 'Disconnect' : 'Connect'}
            </Text>
          </TouchableOpacity>
          <Text style={[styles.status, isConnected && styles.statusConnected]}>
            Status: {isConnected ? 'Connected' : 'Disconnected'}
          </Text>
        </View>

        {/* Telemetry Section */}
        {isConnected && (
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>Telemetry</Text>
            <Text style={styles.telemetry}>Speed: {controlState.speed.toFixed(1)} km/h</Text>
            <Text style={styles.telemetry}>RPM: {controlState.rpm}</Text>
            <Text style={styles.telemetry}>Gear: {controlState.gear}</Text>
            <Text style={styles.telemetry}>Steering: {controlState.steeringAngle}°</Text>
            <Text style={styles.telemetry}>Temp: {controlState.temperature.toFixed(1)}°C</Text>
          </View>
        )}

        {/* Control Section */}
        {isConnected && (
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>Controls</Text>

            {/* Steering */}
            <View style={styles.controlGroup}>
              <Text style={styles.controlLabel}>Steering</Text>
              <View style={styles.buttonRow}>
                <TouchableOpacity
                  style={styles.controlButton}
                  onPress={() => handleSteering('left')}
                >
                  <Text style={styles.controlButtonText}>← Left</Text>
                </TouchableOpacity>
                <TouchableOpacity
                  style={styles.controlButton}
                  onPress={() => handleSteering('center')}
                >
                  <Text style={styles.controlButtonText}>↑ Center</Text>
                </TouchableOpacity>
                <TouchableOpacity
                  style={styles.controlButton}
                  onPress={() => handleSteering('right')}
                >
                  <Text style={styles.controlButtonText}>→ Right</Text>
                </TouchableOpacity>
              </View>
            </View>

            {/* Throttle & Brake */}
            <View style={styles.controlGroup}>
              <Text style={styles.controlLabel}>Throttle & Brake</Text>
              <View style={styles.buttonRow}>
                <TouchableOpacity
                  style={[styles.controlButton, controlState.throttle && styles.activeButton]}
                  onPressIn={() => handleThrottle(true)}
                  onPressOut={() => handleThrottle(false)}
                >
                  <Text style={styles.controlButtonText}>Gas</Text>
                </TouchableOpacity>
                <TouchableOpacity
                  style={[styles.controlButton, controlState.brake && styles.activeButton]}
                  onPressIn={() => handleBrake(true)}
                  onPressOut={() => handleBrake(false)}
                >
                  <Text style={styles.controlButtonText}>Brake</Text>
                </TouchableOpacity>
              </View>
            </View>

            {/* Gears */}
            <View style={styles.controlGroup}>
              <Text style={styles.controlLabel}>Gears</Text>
              <View style={styles.gearRow}>
                {['R', 'N', '1', '2', '3'].map((gear) => (
                  <TouchableOpacity
                    key={gear}
                    style={[
                      styles.gearButton,
                      controlState.gear === gear && styles.activeGear,
                    ]}
                    onPress={() => handleGearChange(gear)}
                  >
                    <Text style={styles.gearButtonText}>{gear}</Text>
                  </TouchableOpacity>
                ))}
              </View>
            </View>

            {/* Emergency Stop */}
            <TouchableOpacity
              style={styles.emergencyButton}
              onPress={() => socketClient.emitEmergencyStop()}
            >
              <Text style={styles.emergencyButtonText}>EMERGENCY STOP</Text>
            </TouchableOpacity>
          </View>
        )}
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#1a1a1a',
  },
  scrollContent: {
    padding: 20,
  },
  title: {
    fontSize: 28,
    fontWeight: 'bold',
    color: '#fff',
    textAlign: 'center',
    marginBottom: 20,
  },
  section: {
    backgroundColor: '#2a2a2a',
    borderRadius: 10,
    padding: 15,
    marginBottom: 15,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: 'bold',
    color: '#fff',
    marginBottom: 10,
  },
  input: {
    backgroundColor: '#fff',
    padding: 12,
    borderRadius: 5,
    marginBottom: 10,
    fontSize: 16,
  },
  button: {
    backgroundColor: '#4CAF50',
    padding: 15,
    borderRadius: 5,
    alignItems: 'center',
  },
  buttonDisconnect: {
    backgroundColor: '#f44336',
  },
  buttonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: 'bold',
  },
  status: {
    color: '#f44336',
    marginTop: 10,
    fontSize: 14,
    textAlign: 'center',
  },
  statusConnected: {
    color: '#4CAF50',
  },
  telemetry: {
    color: '#fff',
    fontSize: 16,
    marginBottom: 5,
  },
  controlGroup: {
    marginBottom: 20,
  },
  controlLabel: {
    color: '#fff',
    fontSize: 16,
    marginBottom: 10,
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  controlButton: {
    backgroundColor: '#555',
    padding: 15,
    borderRadius: 5,
    flex: 1,
    marginHorizontal: 5,
    alignItems: 'center',
  },
  controlButtonText: {
    color: '#fff',
    fontSize: 14,
    fontWeight: 'bold',
  },
  activeButton: {
    backgroundColor: '#4CAF50',
  },
  gearRow: {
    flexDirection: 'row',
    justifyContent: 'space-around',
  },
  gearButton: {
    backgroundColor: '#555',
    padding: 15,
    borderRadius: 5,
    width: 50,
    alignItems: 'center',
  },
  gearButtonText: {
    color: '#fff',
    fontSize: 18,
    fontWeight: 'bold',
  },
  activeGear: {
    backgroundColor: '#2196F3',
  },
  emergencyButton: {
    backgroundColor: '#f44336',
    padding: 20,
    borderRadius: 5,
    alignItems: 'center',
    marginTop: 10,
  },
  emergencyButtonText: {
    color: '#fff',
    fontSize: 18,
    fontWeight: 'bold',
  },
});
