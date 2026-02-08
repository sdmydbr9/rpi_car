import { useState } from "react";
import { Wrench, CheckCircle, AlertTriangle } from "lucide-react";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";

export interface SensorStatus {
  name: string;
  status: "ok" | "warning" | "error";
}

interface ServiceIndicatorProps {
  sensors: SensorStatus[];
  requiresService: boolean;
}

export const ServiceLight = ({ sensors, requiresService }: ServiceIndicatorProps) => {
  const [isOpen, setIsOpen] = useState(false);

  const getStatusIcon = (status: SensorStatus["status"]) => {
    switch (status) {
      case "ok":
        return <CheckCircle className="w-3 h-3 text-green-600" />;
      case "warning":
        return <AlertTriangle className="w-3 h-3 text-yellow-600" />;
      case "error":
        return <AlertTriangle className="w-3 h-3 text-red-600" />;
    }
  };

  const getStatusText = (status: SensorStatus["status"]) => {
    switch (status) {
      case "ok":
        return "OK";
      case "warning":
        return "CHECK";
      case "error":
        return "FAULT";
    }
  };

  const getStatusColor = (status: SensorStatus["status"]) => {
    switch (status) {
      case "ok":
        return "text-green-600";
      case "warning":
        return "text-yellow-600";
      case "error":
        return "text-red-600";
    }
  };

  return (
    <Popover open={isOpen} onOpenChange={setIsOpen}>
      <PopoverTrigger asChild>
        <button
          className={`
            relative w-6 h-6 sm:w-7 sm:h-7 rounded-full border-2 flex items-center justify-center
            transition-all duration-200 touch-feedback
            ${requiresService 
              ? 'border-yellow-500 bg-card cursor-pointer hover:bg-yellow-500/20' 
              : 'border-muted bg-card/50 cursor-default opacity-60'
            }
          `}
          disabled={!requiresService}
        >
          <Wrench 
            className={`
              w-3 h-3 sm:w-3.5 sm:h-3.5
              ${requiresService 
                ? 'text-yellow-500 animate-pulse' 
                : 'text-muted-foreground'
              }
            `}
            style={{
              filter: requiresService ? 'drop-shadow(0 0 4px hsl(var(--warning)))' : 'none'
            }}
          />
          {/* Pulsating ring effect when service required */}
          {requiresService && (
            <span className="absolute inset-0 rounded-full border-2 border-yellow-500 animate-ping opacity-40" />
          )}
        </button>
      </PopoverTrigger>
      
      <PopoverContent 
        className="w-48 p-0 bg-card/95 backdrop-blur-md border-primary/30"
        side="right"
        sideOffset={8}
      >
        {/* Header */}
        <div className="px-3 py-2 border-b border-border/50 bg-secondary/50">
          <div className="flex items-center gap-2">
            <Wrench className="w-3.5 h-3.5 text-yellow-600" />
            <span className="text-[10px] sm:text-xs racing-text text-foreground font-semibold">
              SYSTEM DIAGNOSTICS
            </span>
          </div>
        </div>
        
        {/* Sensor List */}
        <div className="p-2 space-y-1">
          {sensors.map((sensor, index) => (
            <div 
              key={index}
              className="flex items-center justify-between px-2 py-1.5 rounded bg-secondary/30 border border-border/30"
            >
              <span className="text-[8px] sm:text-[10px] racing-text text-muted-foreground uppercase">
                {sensor.name}
              </span>
              <div className="flex items-center gap-1.5">
                {getStatusIcon(sensor.status)}
                <span className={`text-[8px] sm:text-[10px] racing-text font-bold ${getStatusColor(sensor.status)}`}>
                  {getStatusText(sensor.status)}
                </span>
              </div>
            </div>
          ))}
        </div>
        
        {/* Footer */}
        <div className="px-3 py-2 border-t border-border/50 bg-secondary/30">
          <div className="flex items-center justify-between">
            <span className="text-[7px] sm:text-[9px] text-muted-foreground racing-text">
              LAST CHECK
            </span>
            <span className="text-[7px] sm:text-[9px] text-primary racing-text">
              {new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </span>
          </div>
        </div>
      </PopoverContent>
    </Popover>
  );
};
