import { useEffect, useRef, useCallback } from 'react';
import * as httpClient from '../lib/httpClient';

interface UseAutoAccelerationOptions {
  enabled: boolean;
  gear: string;
  isEngineRunning: boolean;
  currentSpeed: number;
}

// Gear speed limits (matching backend)
const GEAR_LIMITS: Record<string, number> = {
  'R': 80,
  'N': 0,
  '1': 40,
  '2': 60,
  '3': 80,
  'S': 100,
};

/**
 * Hook to manage client-side auto-acceleration
 * Incrementally increases throttle based on selected gear
 */
export const useAutoAcceleration = ({
  enabled,
  gear,
  isEngineRunning,
  currentSpeed,
}: UseAutoAccelerationOptions): void => {
  const accelIntervalRef = useRef<number | null>(null);
  const currentSpeedRef = useRef(currentSpeed);

  // Update ref when current speed changes
  useEffect(() => {
    currentSpeedRef.current = currentSpeed;
  }, [currentSpeed]);

  useEffect(() => {
    // Clear any existing interval
    if (accelIntervalRef.current) {
      clearInterval(accelIntervalRef.current);
      accelIntervalRef.current = null;
    }

    // Stop if conditions aren't met
    if (!enabled || !isEngineRunning || gear === 'N' || gear === 'R') {
      httpClient.emitThrottle(false);
      return;
    }

    const gearLimit = GEAR_LIMITS[gear] || 0;

    // Start acceleration interval
    accelIntervalRef.current = window.setInterval(() => {
      const speed = currentSpeedRef.current;

      if (speed < gearLimit) {
        // Increment throttle
        httpClient.emitThrottle(true);
      } else {
        // At limit, maintain throttle
        httpClient.emitThrottle(true);
      }
    }, 200); // Increment every 200ms for smooth acceleration

    return () => {
      if (accelIntervalRef.current) {
        clearInterval(accelIntervalRef.current);
        accelIntervalRef.current = null;
      }
      httpClient.emitThrottle(false);
    };
  }, [enabled, isEngineRunning, gear]);
};

export default useAutoAcceleration;
