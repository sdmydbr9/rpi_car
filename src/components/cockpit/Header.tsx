import { ConnectionDialog } from "./ConnectionDialog";

interface HeaderProps {
  driverName?: string;
  position?: string;
  isConnected: boolean;
  onConnect: (ip: string) => void;
  onDisconnect: () => void;
}

export const Header = ({ 
  driverName = "HAMILTON", 
  position = "P1",
  isConnected,
  onConnect,
  onDisconnect
}: HeaderProps) => {
  return (
    <header className="flex items-center justify-between px-1.5 sm:px-4 py-0.5 sm:py-1.5 border-b border-primary/30 bg-card/30 backdrop-blur-sm h-[6dvh] min-h-[1.75rem] max-h-10 flex-shrink-0">
      {/* Left: Team Logo */}
      <div className="flex items-center gap-0.5 sm:gap-2">
        <div className="flex items-center gap-0.5">
          {/* AMG Stripes */}
          <div className="flex gap-px sm:gap-0.5">
            {[...Array(5)].map((_, i) => (
              <div 
                key={i} 
                className="w-0.5 h-2.5 sm:h-4 bg-gradient-to-b from-muted-foreground to-muted transform -skew-x-12"
              />
            ))}
          </div>
          <span className="text-foreground font-bold tracking-wider ml-0.5 text-[10px] sm:text-sm">AMG</span>
        </div>
        <span className="text-primary font-bold racing-text text-[10px] sm:text-sm">PETRONAS</span>
      </div>
      
      {/* Center: Connection Status */}
      <div className="flex items-center gap-0.5 sm:gap-2 text-[8px] sm:text-xs racing-text text-muted-foreground">
        <div className={`w-1.5 h-1.5 rounded-full ${isConnected ? 'bg-primary animate-pulse' : 'bg-destructive'}`} />
        <span className="hidden sm:inline">{isConnected ? 'CONNECTED' : 'OFFLINE'}</span>
      </div>
      
      {/* Right: Driver Status + Connection */}
      <div className="flex items-center gap-1 sm:gap-3">
        <div className="flex items-center gap-0.5 sm:gap-2">
          <span className="text-primary font-bold text-xs sm:text-base racing-text">{position}</span>
          <span className="text-foreground font-bold racing-text text-[10px] sm:text-sm hidden sm:inline">{driverName}</span>
        </div>
        {/* Driver Number Badge */}
        <div className="w-4 h-4 sm:w-6 sm:h-6 bg-primary rounded flex items-center justify-center">
          <span className="text-primary-foreground font-bold text-[8px] sm:text-xs">44</span>
        </div>
        {/* Connection Dialog */}
        <ConnectionDialog
          isConnected={isConnected}
          onConnect={onConnect}
          onDisconnect={onDisconnect}
        />
      </div>
    </header>
  );
};
