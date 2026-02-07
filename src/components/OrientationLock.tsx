import { useEffect, useState } from "react";

export const OrientationLock = () => {
  const [isPortrait, setIsPortrait] = useState(false);

  useEffect(() => {
    // Check initial orientation
    const checkOrientation = () => {
      const isPort =
        window.innerHeight > window.innerWidth ||
        window.matchMedia("(orientation: portrait)").matches;
      setIsPortrait(isPort);
    };

    checkOrientation();

    // Listen for orientation changes
    window.addEventListener("orientationchange", checkOrientation);
    window.addEventListener("resize", checkOrientation);
    window.matchMedia("(orientation: portrait)").addListener(checkOrientation);

    return () => {
      window.removeEventListener("orientationchange", checkOrientation);
      window.removeEventListener("resize", checkOrientation);
      window
        .matchMedia("(orientation: portrait)")
        .removeListener(checkOrientation);
    };
  }, []);

  if (!isPortrait) {
    return null;
  }

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-background/95 backdrop-blur-sm">
      <div className="flex flex-col items-center justify-center gap-8 px-8 py-12">
        {/* Animated Rotation Icon */}
        <div className="relative w-24 h-24 flex items-center justify-center">
          <svg
            className="w-full h-full text-primary animate-pulse"
            fill="none"
            viewBox="0 0 100 100"
            strokeWidth={1.5}
            stroke="currentColor"
          >
            <path
              d="M 50 10 A 40 40 0 0 1 80 80 M 85 75 L 80 80 L 75 75"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
            <path
              d="M 50 90 A 40 40 0 0 1 20 20 M 15 25 L 20 20 L 25 25"
              strokeLinecap="round"
              strokeLinejoin="round"
              opacity={0.3}
            />
          </svg>
          <div className="absolute inset-0 animate-spin [animation-duration:3s] border-2 border-transparent border-t-primary border-r-primary rounded-full" />
        </div>

        {/* Text Content */}
        <div className="text-center space-y-4 max-w-sm">
          <h1 className="text-3xl font-bold text-foreground tracking-wider">
            ROTATE DEVICE
          </h1>
          <p className="text-muted-foreground text-lg font-light leading-relaxed">
            This application is optimized for landscape orientation. Please
            rotate your device to continue.
          </p>
        </div>

        {/* Status Indicator */}
        <div className="flex items-center gap-3 mt-6 px-6 py-3 rounded-sm border border-primary/30 bg-primary/5">
          <div className="w-2 h-2 bg-primary rounded-full animate-pulse"></div>
          <span className="text-sm text-primary font-medium">
            Waiting for rotation...
          </span>
        </div>
      </div>
    </div>
  );
};
