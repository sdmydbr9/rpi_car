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

  // Prevent default touch behaviors
  document.addEventListener("touchmove", (e) => {
    // Allow scrolling on scrollable elements
    const target = e.target as HTMLElement;
    if (!target.closest('[data-scrollable="true"]')) {
      // Only prevent if not explicitly marked as scrollable
    }
  }, { passive: false });
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
