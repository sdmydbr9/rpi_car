import { useState, useRef, useCallback } from "react";

interface SteeringWheelProps {
  onAngleChange: (angle: number) => void;
  angle: number;
}

export const SteeringWheel = ({ onAngleChange, angle }: SteeringWheelProps) => {
  const wheelRef = useRef<HTMLDivElement>(null);
  const [isDragging, setIsDragging] = useState(false);
  const startAngleRef = useRef(0);
  const startTouchAngleRef = useRef(0);

  const calculateAngle = useCallback((clientX: number, clientY: number) => {
    if (!wheelRef.current) return 0;
    const rect = wheelRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    return Math.atan2(clientY - centerY, clientX - centerX) * (180 / Math.PI);
  }, []);

  const handleTouchStart = useCallback((e: React.TouchEvent) => {
    e.preventDefault();
    setIsDragging(true);
    const touch = e.touches[0];
    startTouchAngleRef.current = calculateAngle(touch.clientX, touch.clientY);
    startAngleRef.current = angle;
  }, [angle, calculateAngle]);

  const handleTouchMove = useCallback((e: React.TouchEvent) => {
    if (!isDragging) return;
    e.preventDefault();
    const touch = e.touches[0];
    const currentTouchAngle = calculateAngle(touch.clientX, touch.clientY);
    let deltaAngle = currentTouchAngle - startTouchAngleRef.current;
    
    // Normalize delta
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;
    
    let newAngle = startAngleRef.current + deltaAngle;
    // Clamp between -90 and 90
    newAngle = Math.max(-90, Math.min(90, newAngle));
    onAngleChange(newAngle);
  }, [isDragging, calculateAngle, onAngleChange]);

  const handleTouchEnd = useCallback(() => {
    setIsDragging(false);
    // Return to center
    onAngleChange(0);
  }, [onAngleChange]);

  return (
    <div className="flex flex-col items-center justify-center h-full p-0.5 overflow-hidden">
      <div
        ref={wheelRef}
        className="relative cursor-grab active:cursor-grabbing touch-none select-none w-[min(28vw,10rem)] aspect-[5/4]"
        onTouchStart={handleTouchStart}
        onTouchMove={handleTouchMove}
        onTouchEnd={handleTouchEnd}
        onMouseDown={(e) => {
          setIsDragging(true);
          startTouchAngleRef.current = calculateAngle(e.clientX, e.clientY);
          startAngleRef.current = angle;
        }}
        onMouseMove={(e) => {
          if (!isDragging) return;
          const currentTouchAngle = calculateAngle(e.clientX, e.clientY);
          let deltaAngle = currentTouchAngle - startTouchAngleRef.current;
          if (deltaAngle > 180) deltaAngle -= 360;
          if (deltaAngle < -180) deltaAngle += 360;
          let newAngle = startAngleRef.current + deltaAngle;
          newAngle = Math.max(-90, Math.min(90, newAngle));
          onAngleChange(newAngle);
        }}
        onMouseUp={() => {
          setIsDragging(false);
          onAngleChange(0);
        }}
        onMouseLeave={() => {
          if (isDragging) {
            setIsDragging(false);
            onAngleChange(0);
          }
        }}
      >
        {/* Steering Wheel Container */}
        <div
          className="steering-wheel relative w-full h-full"
          style={{ transform: `rotate(${angle}deg)` }}
        >
          {/* Outer Ring */}
          <div className="w-full h-full relative">
            {/* Top Section */}
            <div className="absolute top-0 left-1/2 -translate-x-1/2 w-[70%] h-[12%] bg-gradient-to-b from-muted to-card rounded-t-full border-t border-l border-r border-primary/30" />
            
            {/* Left Grip */}
            <div className="absolute left-0 top-1/2 -translate-y-1/2 w-[15%] h-[65%] carbon-fiber rounded-l-full border border-primary/20">
              <div className="absolute inset-1 bg-gradient-to-r from-muted-foreground/20 to-transparent rounded-l-full" />
            </div>
            
            {/* Right Grip */}
            <div className="absolute right-0 top-1/2 -translate-y-1/2 w-[15%] h-[65%] carbon-fiber rounded-r-full border border-primary/20">
              <div className="absolute inset-1 bg-gradient-to-l from-muted-foreground/20 to-transparent rounded-r-full" />
            </div>
            
            {/* Bottom Section */}
            <div className="absolute bottom-0 left-1/2 -translate-x-1/2 w-[60%] h-[10%] bg-gradient-to-t from-muted to-card rounded-b-lg border-b border-l border-r border-primary/30" />
            
            {/* Center Display */}
            <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[55%] h-[55%] racing-panel flex flex-col items-center justify-center">
              {/* AMG Logo */}
              <div className="text-[5px] sm:text-[7px] text-primary racing-text">AMG</div>
              
              {/* Angle Display */}
              <div className={`text-base sm:text-xl md:text-2xl racing-number ${isDragging ? 'text-primary text-glow-teal' : 'text-foreground'}`}>
                {Math.round(angle)}°
              </div>
              
              {/* Direction Indicator */}
              <div className="flex gap-1 mt-0.5">
                <div className={`w-1 sm:w-1.5 h-0.5 rounded-full transition-colors ${angle < -10 ? 'bg-primary glow-teal' : 'bg-muted'}`} />
                <div className={`w-1 sm:w-1.5 h-0.5 rounded-full transition-colors ${Math.abs(angle) <= 10 ? 'bg-primary' : 'bg-muted'}`} />
                <div className={`w-1 sm:w-1.5 h-0.5 rounded-full transition-colors ${angle > 10 ? 'bg-primary glow-teal' : 'bg-muted'}`} />
              </div>
            </div>
          </div>
        </div>
      </div>
      
      {/* Instructions */}
      <div className="text-[6px] sm:text-[8px] text-muted-foreground mt-0.5 racing-text">
        DRAG TO STEER
      </div>
    </div>
  );
};
