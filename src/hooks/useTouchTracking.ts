import { useRef, useEffect, useCallback } from "react";

/**
 * useTouchTracking — Multitouch-safe touch tracking hook for gaming controls.
 *
 * Each interactive element gets its own instance. The hook tracks the SPECIFIC
 * finger (by touch.identifier) that started on this element, so multiple elements
 * can be touched simultaneously without interfering with each other.
 *
 * Uses native addEventListener with { passive: false } to guarantee
 * e.preventDefault() works reliably across all mobile browsers.
 */

export interface TouchCallbacks {
  onTouchStart?: (touch: Touch, e: TouchEvent) => void;
  onTouchMove?: (touch: Touch, e: TouchEvent) => void;
  onTouchEnd?: (touch: Touch | null, e: TouchEvent) => void;
}

export function useTouchTracking(
  elementRef: React.RefObject<HTMLElement | null>,
  callbacks: TouchCallbacks,
  enabled: boolean = true
) {
  // Track the identifier of the finger currently interacting with this element
  const activeTouchIdRef = useRef<number | null>(null);

  // Store callbacks in refs to avoid re-registering listeners on every render
  const callbacksRef = useRef(callbacks);
  callbacksRef.current = callbacks;

  const enabledRef = useRef(enabled);
  enabledRef.current = enabled;

  // Find a touch by identifier in a TouchList
  const findTouchById = useCallback((touchList: TouchList, id: number): Touch | null => {
    for (let i = 0; i < touchList.length; i++) {
      if (touchList[i].identifier === id) {
        return touchList[i];
      }
    }
    return null;
  }, []);

  useEffect(() => {
    const el = elementRef.current;
    if (!el) return;

    const handleTouchStart = (e: TouchEvent) => {
      if (!enabledRef.current) return;

      // If we're already tracking a finger on this element, ignore new touches
      if (activeTouchIdRef.current !== null) return;

      // Find the new touch that started on this element
      for (let i = 0; i < e.changedTouches.length; i++) {
        const touch = e.changedTouches[i];
        // Verify the touch target is within our element
        if (el.contains(touch.target as Node)) {
          activeTouchIdRef.current = touch.identifier;
          e.preventDefault();
          e.stopPropagation();
          callbacksRef.current.onTouchStart?.(touch, e);
          return;
        }
      }
    };

    const handleTouchMove = (e: TouchEvent) => {
      if (activeTouchIdRef.current === null) return;

      // Find OUR specific finger in changedTouches
      const touch = findTouchById(e.changedTouches, activeTouchIdRef.current);
      if (touch) {
        e.preventDefault();
        callbacksRef.current.onTouchMove?.(touch, e);
      }
    };

    const handleTouchEnd = (e: TouchEvent) => {
      if (activeTouchIdRef.current === null) return;

      const touch = findTouchById(e.changedTouches, activeTouchIdRef.current);
      if (touch) {
        activeTouchIdRef.current = null;
        e.preventDefault();
        callbacksRef.current.onTouchEnd?.(touch, e);
      }
    };

    const handleTouchCancel = (e: TouchEvent) => {
      if (activeTouchIdRef.current === null) return;

      const touch = findTouchById(e.changedTouches, activeTouchIdRef.current);
      if (touch) {
        activeTouchIdRef.current = null;
        e.preventDefault();
        callbacksRef.current.onTouchEnd?.(null, e);
      }
    };

    // { passive: false } is CRITICAL — it allows e.preventDefault() to work.
    // Without this, mobile browsers (especially Chrome) treat touch listeners
    // as passive by default and ignore preventDefault().
    const opts: AddEventListenerOptions = { passive: false };

    el.addEventListener("touchstart", handleTouchStart, opts);
    el.addEventListener("touchmove", handleTouchMove, opts);
    el.addEventListener("touchend", handleTouchEnd, opts);
    el.addEventListener("touchcancel", handleTouchCancel, opts);

    return () => {
      el.removeEventListener("touchstart", handleTouchStart);
      el.removeEventListener("touchmove", handleTouchMove);
      el.removeEventListener("touchend", handleTouchEnd);
      el.removeEventListener("touchcancel", handleTouchCancel);
      activeTouchIdRef.current = null;
    };
  }, [elementRef, findTouchById]);

  // Expose a way to check if this element is currently being touched
  const isActive = useCallback(() => activeTouchIdRef.current !== null, []);

  // Force-release the tracked touch (useful for disable/unmount edge cases)
  const forceRelease = useCallback(() => {
    activeTouchIdRef.current = null;
  }, []);

  return { isActive, forceRelease, activeTouchIdRef };
}
