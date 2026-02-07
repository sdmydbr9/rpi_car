import { OctagonX, Zap, Power, PowerOff, Radio, Radar, X } from "lucide-react";

interface GearShifterProps {
  currentGear: string;
  onGearChange: (gear: string) => void;
  isEmergencyStop: boolean;
  isAutoMode: boolean;
  isIREnabled: boolean;
  isSonarEnabled: boolean;
  isAutopilotEnabled: boolean;
  onEmergencyStop: () => void;
  onAutoMode: () => void;
  onIRToggle: () => void;
  onSonarToggle: () => void;
  onAutopilotToggle: () => void;
  isEnabled?: boolean;
  isEngineRunning?: boolean;
  onEngineStart?: () => void;
  onEngineStop?: () => void;
}

const GEARS = ["S", "3", "2", "1", "N", "R"];

export const GearShifter = ({ 
  currentGear, 
  onGearChange,
  isEmergencyStop,
  isAutoMode,
  isIREnabled,
  isSonarEnabled,
  isAutopilotEnabled,
  onEmergencyStop,
  onAutoMode,
  onIRToggle,
  onSonarToggle,
  onAutopilotToggle,
  isEnabled = true,
  isEngineRunning = false,
  onEngineStart,
  onEngineStop
}: GearShifterProps) => {
  // Gearbox layout: 2 rows x 3 columns
  const gearLayout = [
    ["S", "3", "2"],
    ["1", "N", "R"]
  ];

  return (
    <div className="flex flex-col items-center h-full pt-1 pb-1 px-1.5 overflow-hidden bg-gradient-to-b from-background to-background/80">
      {/* LIVE TELEMETRY Header */}
      <div className="text-[7px] sm:text-[8px] font-bold racing-text text-muted-foreground tracking-wider mb-1.5">
        LIVE TELEMETRY
      </div>

      {/* E-STOP and AUTO OFF - Side by Side Pill Shaped */}
      <div className="flex w-full gap-1 mb-1.5">
        {/* E-STOP Button - Pill Shaped */}
        <button
          onClick={onEmergencyStop}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-1.5 px-2
            transition-all duration-100 touch-feedback font-bold racing-text
            ${isEmergencyStop
              ? 'bg-destructive border-destructive text-destructive-foreground shadow-lg shadow-destructive/50'
              : 'bg-card border-destructive/60 text-destructive hover:bg-destructive/20 hover:border-destructive'
            }
          `}
        >
          <X className="w-3.5 h-3.5 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">E-STOP</span>
        </button>

        {/* AUTO OFF Button - Pill Shaped */}
        <button
          onClick={onAutoMode}
          disabled={isEmergencyStop}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-1.5 px-2
            transition-all duration-100 touch-feedback font-bold racing-text disabled:opacity-50
            ${isAutoMode
              ? 'bg-primary border-primary text-primary-foreground shadow-lg shadow-primary/50'
              : 'bg-card border-primary/60 text-primary hover:bg-primary/20 hover:border-primary'
            }
          `}
        >
          <Zap className="w-3.5 h-3.5 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">AUTO {isAutoMode ? "ON" : "OFF"}</span>
        </button>
      </div>

      {/* IR and SONAR and AUTOPILOT - Side by Side Round Buttons */}
      <div className="flex w-full gap-1 mb-1.5 justify-center">
        {/* IR Button - Round */}
        <button
          onClick={onIRToggle}
          disabled={!isEngineRunning}
          className={`
            w-12 h-12 rounded-full border-2 flex items-center justify-center relative
            transition-all duration-100 touch-feedback font-bold
            ${!isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : isIREnabled
              ? 'bg-green-500/20 border-green-500 shadow-lg'
              : 'bg-white/10 border-white/30 hover:bg-white/20 hover:border-white/40'
            }
          `}
          title={isIREnabled ? 'IR: ON' : 'IR: OFF'}
        >
          {/* Red diagonal line overlay when IR is off */}
          {!isIREnabled && isEngineRunning && (
            <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
              <div className="w-full h-0.5 bg-red-500 transform rotate-45 origin-center"></div>
            </div>
          )}
          <svg 
            className="w-10 h-10" 
            style={{ color: isIREnabled ? '#22c55e' : '#f3f4f6' }}
            xmlns="http://www.w3.org/2000/svg" 
            version="1.1" 
            viewBox="-5.0 -10.0 110.0 135.0"
          >
            <path d="m71.766 37.016-3.5781-1.1875v-8.1562c0-8.5625-6.9531-15.516-15.516-15.516h-6.1875c-8.5469 0-15.5 6.9531-15.5 15.516v8.1562l-3.5781 1.1875c-0.8125 0.26562-1.2656 1.1562-0.98438 1.9844 0.21875 0.65625 0.82812 1.0625 1.4844 1.0625 0.15625 0 0.32812-0.03125 0.5-0.078125l2.5938-0.85938v39.656c0 5.9844 4.8594 10.859 10.844 10.859h15.5c5.9844 0 10.859-4.875 10.859-10.859v-39.656l2.5781 0.85938c0.17188 0.0625 0.32812 0.078125 0.5 0.078125 0.65625 0 1.2656-0.42188 1.4844-1.0625 0.26562-0.8125-0.17188-1.7031-0.98438-1.9688zm-7.625 0.70312-5.8906 8.25c-5.6406-1.6875-11.688-1.6875-17.344 0l-5.8906-8.2344c9.4219-3.0781 19.672-3.0781 29.109 0zm-22.297 29.203c2.5156 0.67188 5.1094 1.0156 7.7344 1.0156s5.2188-0.34375 7.7344-1.0156v4.1094c0 0.84375-0.6875 1.5312-1.5312 1.5312h-12.391c-0.85938 0-1.5469-0.6875-1.5469-1.5312zm0-3.2344v-14.766c5.0312-1.5 10.453-1.5 15.469 0v14.766c-5.0156 1.5-10.438 1.5-15.469 0zm-7.7344-21.859 4.6094 6.4531v15.781l-4.6094 3.6094v-25.859zm26.328 22.234v-15.781l4.625-6.4688v25.875z" fill="currentColor"/>
            <path d="m23.281 15.391c-2.2344 0-4.3281 0.89062-5.9062 2.4531-1.5625 1.5625-2.4375 3.6562-2.4531 5.8906 0 0.85938 0.6875 1.5625 1.5469 1.5781 0.85938 0 1.5625-0.6875 1.5625-1.5469 0-1.4062 0.5625-2.7344 1.5312-3.7031s2.2969-1.5312 3.7031-1.5312c0.85938 0 1.5625-0.70312 1.5469-1.5781s-0.64062-1.5781-1.5781-1.5469z" fill="currentColor"/>
            <path d="m23.297 8.5938c-4.0469 0-7.8594 1.5938-10.703 4.4375s-4.4375 6.6562-4.4531 10.703c0 0.85938 0.6875 1.5625 1.5625 1.5625 0.85938 0 1.5625-0.70312 1.5625-1.5625 0-3.2188 1.2656-6.25 3.5312-8.5156s5.2812-3.5156 8.5156-3.5312c0.85938 0 1.5625-0.70312 1.5625-1.5625s-0.70312-1.5625-1.5625-1.5625z" fill="currentColor"/>
            <path d="m23.312 81.484c-1.4062 0-2.7344-0.54688-3.7031-1.5312s-1.5312-2.2969-1.5312-3.7031c0-0.85938-0.70312-1.5469-1.5625-1.5469s-1.5625 0.70312-1.5469 1.5781c0.015625 2.2344 0.89062 4.3281 2.4531 5.8906s3.6562 2.4375 5.9062 2.4531c0.85938 0 1.5625-0.6875 1.5625-1.5469s-0.6875-1.5625-1.5469-1.5781z" fill="currentColor"/>
            <path d="m23.297 88.281c-3.2188 0-6.25-1.2656-8.5156-3.5312s-3.5156-5.2812-3.5312-8.5156c0-0.85938-0.70312-1.5625-1.5625-1.5625s-1.5625 0.70312-1.5625 1.5625c0 4.0469 1.5938 7.8594 4.4531 10.703s6.6562 4.4375 10.703 4.4375c0.85938 0 1.5625-0.70312 1.5625-1.5625s-0.6875-1.5625-1.5625-1.5625z" fill="currentColor"/>
            <path d="m80.422 56.797c0.3125 0.29688 0.70312 0.45312 1.0938 0.45312s0.8125-0.15625 1.1094-0.46875c1.5781-1.5938 2.4375-3.6875 2.4375-5.9062s-0.85938-4.3125-2.4375-5.9062c-0.60938-0.60938-1.5938-0.625-2.2031 0s-0.625 1.5938 0 2.2031c0.98438 1 1.5312 2.3281 1.5312 3.7031s-0.54688 2.7031-1.5312 3.7031c-0.60938 0.60938-0.59375 1.6094 0 2.2031z" fill="currentColor"/>
            <path d="m86.328 62.062c0.40625 0 0.79688-0.15625 1.1094-0.46875 2.8594-2.875 4.4219-6.6875 4.4219-10.719s-1.5781-7.8438-4.4219-10.719c-0.60938-0.60938-1.5938-0.60938-2.2031 0s-0.60938 1.5938 0 2.2031c2.2656 2.2812 3.5156 5.3125 3.5156 8.5156s-1.25 6.2188-3.5156 8.5156c-0.60938 0.60938-0.60938 1.6094 0 2.2031 0.29688 0.29688 0.70312 0.45312 1.0938 0.45312z" fill="currentColor"/>
            <path d="m75.859 18.516c1.4062 0 2.7344 0.54688 3.7188 1.5312s1.5312 2.2969 1.5312 3.7031c0 0.85938 0.70312 1.5469 1.5625 1.5469s1.5625-0.70312 1.5469-1.5781c0-2.2344-0.89062-4.3281-2.4531-5.8906s-3.6562-2.4375-5.9062-2.4531c-0.85938 0-1.5625 0.6875-1.5625 1.5469s0.6875 1.5625 1.5469 1.5781z" fill="currentColor"/>
            <path d="m75.859 11.719c3.2188 0 6.25 1.2656 8.5156 3.5312s3.5156 5.2812 3.5312 8.5156c0 0.85938 0.70312 1.5625 1.5625 1.5625s1.5625-0.70312 1.5625-1.5625c0-4.0469-1.5938-7.8594-4.4375-10.703-2.8438-2.8594-6.6562-4.4375-10.703-4.4375-0.85938 0-1.5625 0.70312-1.5625 1.5625s0.6875 1.5625 1.5625 1.5625z" fill="currentColor"/>
            <path d="m75.859 84.609c2.2344 0 4.3281-0.89062 5.9062-2.4531s2.4375-3.6562 2.4531-5.8906c0-0.85938-0.6875-1.5625-1.5469-1.5781-0.875-0.03125-1.5625 0.6875-1.5781 1.5469 0 1.4062-0.5625 2.7344-1.5312 3.7031s-2.2969 1.5312-3.7188 1.5312c-0.85938 0-1.5625 0.70312-1.5469 1.5781 0 0.85938 0.70312 1.5469 1.5625 1.5469z" fill="currentColor"/>
            <path d="m89.469 74.688c-0.85938 0-1.5625 0.70312-1.5625 1.5625 0 3.2188-1.2656 6.25-3.5312 8.5156s-5.2812 3.5156-8.5156 3.5312c-0.85938 0-1.5625 0.70312-1.5625 1.5625s0.70312 1.5625 1.5625 1.5625c4.0469 0 7.8594-1.5938 10.703-4.4375 2.8594-2.8594 4.4375-6.6562 4.4375-10.703 0-0.85938-0.6875-1.5625-1.5625-1.5625z" fill="currentColor"/>
            <path d="m22.547 45.422c-0.60938-0.60938-1.6094-0.59375-2.2031 0-1.4219 1.4375-2.2031 3.3438-2.2031 5.3438s0.78125 3.9062 2.2031 5.3438c0.3125 0.3125 0.70312 0.46875 1.1094 0.46875s0.79688-0.15625 1.0938-0.45312c0.60938-0.60938 0.625-1.5938 0-2.2031-0.84375-0.85938-1.2969-1.9688-1.2969-3.1406s0.46875-2.2969 1.2969-3.1406c0.60938-0.60938 0.59375-1.6094 0-2.2031z" fill="currentColor"/>
            <path d="m16.094 41.203c-2.5625 2.5781-3.9688 5.9844-3.9688 9.5781s1.4062 7.0156 3.9688 9.5781c0.3125 0.3125 0.70312 0.46875 1.1094 0.46875s0.79688-0.15625 1.0938-0.45312c0.60938-0.60938 0.60938-1.5938 0-2.2031-1.9688-1.9844-3.0469-4.6094-3.0469-7.3906s1.0781-5.4062 3.0469-7.3906c0.60938-0.60938 0.60938-1.5938 0-2.2031s-1.5938-0.60938-2.2031 0z" fill="currentColor"/>
          </svg>
        </button>

        {/* SONAR Button - Round */}
        <button
          onClick={onSonarToggle}
          disabled={!isEngineRunning}
          className={`
            w-12 h-12 rounded-full border-2 flex items-center justify-center
            transition-all duration-100 touch-feedback font-bold
            ${!isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : isSonarEnabled
              ? 'bg-primary border-primary text-primary-foreground shadow-lg'
              : 'bg-card border-primary/60 text-primary hover:bg-primary/20 hover:border-primary'
            }
          `}
          title={isSonarEnabled ? 'SONAR: ON' : 'SONAR: OFF'}
        >
          <Radar className="w-5 h-5" />
        </button>

        {/* AUTOPILOT - Round Button */}
        <button
          onClick={onAutopilotToggle}
          disabled={!isEngineRunning || isEmergencyStop}
          className={`
            w-12 h-12 rounded-full border-2 flex items-center justify-center relative
            transition-all duration-100 touch-feedback font-bold
            ${!isEngineRunning || isEmergencyStop
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : isAutopilotEnabled
              ? 'bg-green-500/20 border-green-500 shadow-lg'
              : 'bg-yellow-500/10 border-yellow-500/60 hover:bg-yellow-500/20 hover:border-yellow-500'
            }
          `}
          title={isAutopilotEnabled ? 'AUTOPILOT: ON' : 'AUTOPILOT: OFF'}
        >
          {/* Red diagonal line overlay when Autopilot is off */}
          {!isAutopilotEnabled && isEngineRunning && !isEmergencyStop && (
            <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
              <div className="w-full h-0.5 bg-red-500 transform rotate-45 origin-center"></div>
            </div>
          )}
          <svg 
            className="w-10 h-10" 
            style={{ color: isAutopilotEnabled ? '#22c55e' : '#f3f4f6' }}
            xmlns="http://www.w3.org/2000/svg" 
            version="1.1" 
            viewBox="-5.0 -10.0 110.0 135.0"
          >
            <path d="m50 81.25c17.234 0 31.25-14.016 31.25-31.25s-14.016-31.25-31.25-31.25-31.25 14.016-31.25 31.25 14.016 31.25 31.25 31.25zm0-59.375c15.516 0 28.125 12.609 28.125 28.125s-12.609 28.125-28.125 28.125-28.125-12.609-28.125-28.125 12.609-28.125 28.125-28.125z" fill="currentColor"/>
            <path d="m43.672 73.828s0.03125 0.015625 0.046875 0.015625h0.03125s0.09375 0.03125 0.15625 0.03125c1.9531 0.51562 4 0.78125 6.0938 0.78125s4.1406-0.26562 6.0938-0.78125c0.0625 0 0.10938-0.015625 0.15625-0.03125h0.03125s0.03125-0.015625 0.046875-0.015625c9.4062-2.5 16.625-10.438 18.062-20.219 0-0.0625 0.015625-0.10938 0.015625-0.17188 0.17188-1.125 0.25-2.2656 0.25-3.4375 0-2.4688-0.35938-4.875-1.0625-7.125 0-0.09375-0.046875-0.1875-0.078125-0.28125-3.1406-9.9844-12.5-17.25-23.516-17.25s-20.375 7.2656-23.516 17.25c-0.03125 0.09375-0.0625 0.1875-0.078125 0.28125-0.70312 2.25-1.0625 4.6562-1.0625 7.125 0 1.1719 0.078125 2.3125 0.25 3.4375 0 0.0625 0.015625 0.10938 0.015625 0.17188 1.4375 9.7812 8.6562 17.719 18.062 20.219zm1.4219-23.438c1.3281-1.3125 3.0625-2.0312 4.9062-2.0312s3.5781 0.71875 4.9062 2.0312c0.625 0.60938 0.625 1.5938 0.015625 2.2031-0.29688 0.3125-0.70312 0.46875-1.1094 0.46875s-0.79688-0.15625-1.0938-0.45312c-1.4688-1.4531-3.9688-1.4531-5.4375 0-0.60938 0.60938-1.6094 0.60938-2.2031-0.015625-0.60938-0.60938-0.60938-1.5938 0.015625-2.2031zm-1.625-1.5938c-0.60938 0.60938-1.5938 0.59375-2.2031-0.015625s-0.60938-1.5938 0-2.2031c2.3438-2.3281 5.4531-3.6094 8.7344-3.6094s6.3906 1.2812 8.7344 3.6094c0.60938 0.60938 0.60938 1.5938 0 2.2031-0.29688 0.3125-0.70312 0.46875-1.1094 0.46875s-0.79688-0.15625-1.0938-0.45312c-1.75-1.75-4.0781-2.7031-6.5312-2.7031s-4.7812 0.95312-6.5312 2.7031zm5 6.6719c0-0.85938 0.6875-1.5625 1.5469-1.5625h0.015625c0.85938 0 1.5625 0.70312 1.5625 1.5625s-0.70312 1.5625-1.5625 1.5625-1.5625-0.70312-1.5625-1.5625zm8.6875 14.828c-0.0625-0.54688-0.09375-1.0781-0.09375-1.6094 0-7.625 6.2031-13.828 13.828-13.828h0.078125c-1.6562 7.2031-6.9375 13.016-13.812 15.438zm-7.1562-41.828c8.4688 0 15.797 4.9062 19.328 12.031-6.2344-2.0312-12.719-3.0625-19.328-3.0625s-13.094 1.0312-19.328 3.0625c3.5312-7.125 10.859-12.031 19.328-12.031zm-20.891 26.391c7.625 0 13.828 6.2031 13.828 13.828 0 0.53125-0.03125 1.0625-0.09375 1.6094-6.875-2.4219-12.156-8.2344-13.812-15.438z" fill="currentColor"/>
            <path d="m22.375 22.375c14.375-14.375 37.234-15.172 52.562-2.4375l-2.9375 0.1875c-0.85938 0.046875-1.5156 0.79688-1.4688 1.6562 0.046875 0.82812 0.73438 1.4688 1.5625 1.4688h0.09375l6.6094-0.42188c0.078125 0 0.14062-0.046875 0.21875-0.0625 0.10938-0.015625 0.21875-0.046875 0.32812-0.09375s0.1875-0.125 0.28125-0.1875c0.0625-0.046875 0.125-0.0625 0.1875-0.125 0 0 0-0.03125 0.03125-0.046875 0.078125-0.078125 0.125-0.1875 0.1875-0.28125 0.046875-0.078125 0.10938-0.14062 0.14062-0.23438 0.03125-0.078125 0.03125-0.17188 0.046875-0.26562 0.015625-0.10938 0.046875-0.21875 0.046875-0.34375v-0.046875l-0.42188-6.6094c-0.0625-0.85938-0.78125-1.5156-1.6562-1.4688-0.85938 0.046875-1.5156 0.79688-1.4688 1.6562l0.17188 2.7656c-16.531-13.703-41.203-12.828-56.719 2.6875-10.25 10.25-14.484 24.844-11.328 39.016 0.15625 0.73438 0.8125 1.2188 1.5312 1.2188 0.10938 0 0.23438 0 0.34375-0.03125 0.84375-0.1875 1.375-1.0156 1.1875-1.8594-2.9219-13.125 1-26.625 10.5-36.125z" fill="currentColor"/>
            <path d="m91.172 40.828c-0.1875-0.84375-1.0156-1.375-1.8594-1.1875s-1.375 1.0156-1.1875 1.8594c2.9219 13.125-1 26.625-10.5 36.125-14.375 14.375-37.234 15.172-52.562 2.4375l2.9375-0.1875c0.85938-0.046875 1.5156-0.79688 1.4688-1.6562s-0.78125-1.5-1.6562-1.4688l-6.6094 0.42188c-0.078125 0-0.14062 0.046875-0.21875 0.0625-0.10938 0.015625-0.21875 0.046875-0.32812 0.09375s-0.1875 0.125-0.28125 0.1875c-0.0625 0.046875-0.125 0.0625-0.1875 0.125 0 0 0 0.03125-0.03125 0.046875-0.078125 0.078125-0.125 0.1875-0.1875 0.28125-0.046875 0.078125-0.10938 0.14062-0.14062 0.23438-0.03125 0.078125-0.03125 0.17188-0.046875 0.26562-0.015625 0.10938-0.046875 0.21875-0.046875 0.34375v0.046875l0.42188 6.6094c0.046875 0.82812 0.73438 1.4688 1.5625 1.4688h0.09375c0.85938-0.046875 1.5156-0.79688 1.4688-1.6562l-0.17188-2.7656c7.7812 6.4531 17.344 9.7031 26.922 9.7031 10.797 0 21.609-4.1094 29.828-12.344 10.25-10.25 14.484-24.844 11.328-39.016z" fill="currentColor"/>
          </svg>
        </button>
      </div>

      {/* Gearbox Grid Container */}
      <div className="bg-card border-2 border-border rounded-lg p-1.5 space-y-1 mb-1.5">
        {gearLayout.map((row, rowIdx) => (
          <div key={rowIdx} className="flex gap-1 justify-center">
            {row.map((gear) => {
              const isActive = currentGear === gear;
              const isReverse = gear === "R";
              
              return (
                <button
                  key={gear}
                  onClick={() => onGearChange(gear)}
                  disabled={!isEnabled}
                  className={`
                    w-7 h-7 sm:w-8 sm:h-8 rounded-lg border-2 text-[10px] sm:text-xs font-bold racing-text
                    transition-all duration-150 touch-feedback flex items-center justify-center
                    ${!isEnabled
                      ? 'bg-muted/40 border-muted/30 text-muted-foreground opacity-50 cursor-not-allowed'
                      : isActive
                      ? isReverse
                        ? "gear-reverse-active border-destructive bg-destructive/20"
                        : "gear-active border-primary bg-primary/20"
                      : "bg-card border-border hover:border-primary/60 text-muted-foreground hover:text-foreground hover:bg-primary/5"
                    }
                  `}
                >
                  {gear}
                </button>
              );
            })}
          </div>
        ))}
      </div>

      {/* Spacer */}
      <div className="flex-1" />

      {/* START and STOP Buttons */}
      <div className="flex gap-1.5 w-full">
        {/* START Button - Pill Shaped */}
        <button
          onClick={onEngineStart}
          disabled={isEngineRunning}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-2 px-1.5
            transition-all duration-100 touch-feedback font-bold racing-text
            ${isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : 'bg-primary border-primary text-primary-foreground hover:bg-primary/90 shadow-lg'
            }
          `}
        >
          <Power className="w-4 h-4 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">START</span>
        </button>

        {/* STOP Button - Pill Shaped */}
        <button
          onClick={onEngineStop}
          disabled={!isEngineRunning}
          className={`
            flex-1 rounded-full border-2 flex flex-col items-center justify-center py-2 px-1.5
            transition-all duration-100 touch-feedback font-bold racing-text
            ${!isEngineRunning
              ? 'bg-muted border-muted text-muted-foreground cursor-not-allowed opacity-50'
              : 'bg-destructive border-destructive text-destructive-foreground hover:bg-destructive/90 shadow-lg'
            }
          `}
        >
          <PowerOff className="w-4 h-4 mb-0.5" />
          <span className="text-[7px] sm:text-[8px] leading-tight">STOP</span>
        </button>
      </div>
    </div>
  );
};