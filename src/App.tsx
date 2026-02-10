import { Toaster } from "@/components/ui/toaster";
import { Toaster as Sonner } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { useEffect } from "react";
import Index from "./pages/Index";
import NotFound from "./pages/NotFound";
import { OrientationLock } from "./components/OrientationLock";

const queryClient = new QueryClient();

// Gaming UI Polish - Disable unwanted interactions
const disableGameDisruptiveActions = () => {
  // Disable context menu
  document.addEventListener("contextmenu", (e) => e.preventDefault());

  // Disable text selection via keyboard
  document.addEventListener("selectstart", (e) => {
    if ((e.target as HTMLElement)?.contentEditable !== "true") {
      e.preventDefault();
    }
  });

  // Disable double-click text selection
  document.addEventListener("dblclick", (e) => {
    if ((e.target as HTMLElement)?.contentEditable !== "true") {
      const selection = window.getSelection();
      if (selection) {
        selection.removeAllRanges();
      }
    }
  });

  // Disable drag-and-drop
  document.addEventListener("dragstart", (e) => e.preventDefault());
  document.addEventListener("drop", (e) => e.preventDefault());
  document.addEventListener("dragover", (e) => e.preventDefault());

  // Prevent default touch behaviors (scroll, pull-to-refresh, overscroll bounce)
  // This is the critical handler for gaming — blocks ALL browser touch interpretation
  // unless the element is explicitly marked as scrollable.
  document.addEventListener("touchmove", (e) => {
    const target = e.target as HTMLElement;
    if (!target.closest('[data-scrollable="true"]')) {
      e.preventDefault();
    }
  }, { passive: false });

  // Prevent Safari gesturestart/gesturechange (pinch-zoom on iOS)
  document.addEventListener("gesturestart", (e) => e.preventDefault());
  document.addEventListener("gesturechange", (e) => e.preventDefault());

  // Prevent double-tap zoom on all touch devices
  let lastTouchEnd = 0;
  document.addEventListener("touchend", (e) => {
    const now = Date.now();
    if (now - lastTouchEnd <= 300) {
      e.preventDefault();
    }
    lastTouchEnd = now;
  }, { passive: false });

  // Request Wake Lock to prevent screen dimming during gameplay
  const requestWakeLock = async () => {
    try {
      if ("wakeLock" in navigator) {
        await (navigator as any).wakeLock.request("screen");
      }
    } catch {
      // Wake Lock not supported or denied — non-critical
    }
  };

  // Request wake lock on first interaction
  const onFirstInteraction = () => {
    requestWakeLock();
    document.removeEventListener("touchstart", onFirstInteraction);
    document.removeEventListener("click", onFirstInteraction);
  };
  document.addEventListener("touchstart", onFirstInteraction, { once: true });
  document.addEventListener("click", onFirstInteraction, { once: true });

  // Re-acquire wake lock when page becomes visible again (e.g. tab switch back)
  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState === "visible") {
      requestWakeLock();
    }
  });
};

const App = () => {
  useEffect(() => {
    disableGameDisruptiveActions();
  }, []);

  return (
    <QueryClientProvider client={queryClient}>
      <TooltipProvider>
        <Toaster />
        <Sonner />
        <OrientationLock />
        <BrowserRouter>
          <Routes>
            <Route path="/" element={<Index />} />
            {/* ADD ALL CUSTOM ROUTES ABOVE THE CATCH-ALL "*" ROUTE */}
            <Route path="*" element={<NotFound />} />
          </Routes>
        </BrowserRouter>
      </TooltipProvider>
    </QueryClientProvider>
  );
};

export default App;
