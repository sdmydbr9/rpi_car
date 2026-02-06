import { useState } from "react";
import { Satellite, X, Wifi, WifiOff } from "lucide-react";
import { z } from "zod";

interface ConnectionDialogProps {
  isConnected: boolean;
  onConnect: (ipAddress: string) => void;
  onDisconnect: () => void;
}

const ipSchema = z.string().regex(
  /^(\d{1,3}\.){3}\d{1,3}(:\d{1,5})?$/,
  "Enter valid IP (e.g., 192.168.1.100:5000)"
);

export const ConnectionDialog = ({ isConnected, onConnect, onDisconnect }: ConnectionDialogProps) => {
  const [isOpen, setIsOpen] = useState(false);
  const [ipAddress, setIpAddress] = useState("");
  const [error, setError] = useState("");

  const handleConnect = () => {
    const result = ipSchema.safeParse(ipAddress.trim());
    if (!result.success) {
      setError(result.error.errors[0].message);
      return;
    }
    setError("");
    onConnect(ipAddress.trim());
    setIsOpen(false);
  };

  const handleDisconnect = () => {
    onDisconnect();
    setIpAddress("");
  };

  return (
    <>
      {/* Satellite Icon Button */}
      <button
        onClick={() => setIsOpen(true)}
        className={`
          p-2 rounded-lg border transition-all touch-feedback
          ${isConnected 
            ? 'border-primary bg-primary/20 text-primary glow-teal' 
            : 'border-border bg-card text-muted-foreground hover:border-primary/50 hover:text-primary'
          }
        `}
      >
        <Satellite className="w-5 h-5" />
      </button>

      {/* Dialog Overlay */}
      {isOpen && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-background/80 backdrop-blur-sm">
          <div className="racing-panel bg-card p-6 w-80 max-w-[90vw] border border-primary/30">
            {/* Header */}
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-2">
                <Satellite className="w-5 h-5 text-primary" />
                <span className="racing-text text-sm text-foreground">CONNECT TO CAR</span>
              </div>
              <button
                onClick={() => setIsOpen(false)}
                className="p-1 rounded hover:bg-muted transition-colors"
              >
                <X className="w-4 h-4 text-muted-foreground" />
              </button>
            </div>

            {/* Status */}
            <div className={`flex items-center gap-2 mb-4 p-2 rounded border ${
              isConnected ? 'border-primary/50 bg-primary/10' : 'border-border bg-muted/30'
            }`}>
              {isConnected ? (
                <>
                  <Wifi className="w-4 h-4 text-primary" />
                  <span className="text-xs racing-text text-primary">CONNECTED</span>
                </>
              ) : (
                <>
                  <WifiOff className="w-4 h-4 text-muted-foreground" />
                  <span className="text-xs racing-text text-muted-foreground">DISCONNECTED</span>
                </>
              )}
            </div>

            {/* IP Input */}
            <div className="mb-4">
              <label className="text-xs racing-text text-muted-foreground mb-2 block">
                FLASK SERVER IP ADDRESS
              </label>
              <input
                type="text"
                value={ipAddress}
                onChange={(e) => {
                  setIpAddress(e.target.value);
                  setError("");
                }}
                placeholder="192.168.1.100:5000"
                maxLength={21}
                className="w-full bg-muted border border-border rounded px-3 py-2 text-foreground racing-number text-sm focus:border-primary focus:outline-none transition-colors"
                disabled={isConnected}
              />
              {error && (
                <p className="text-xs text-destructive mt-1">{error}</p>
              )}
            </div>

            {/* Action Buttons */}
            <div className="flex gap-2">
              {isConnected ? (
                <button
                  onClick={handleDisconnect}
                  className="flex-1 py-2 px-4 rounded border border-destructive bg-destructive/20 text-destructive racing-text text-sm hover:bg-destructive/30 transition-colors touch-feedback"
                >
                  DISCONNECT
                </button>
              ) : (
                <button
                  onClick={handleConnect}
                  className="flex-1 py-2 px-4 rounded border border-primary bg-primary/20 text-primary racing-text text-sm hover:bg-primary/30 transition-colors touch-feedback"
                >
                  START
                </button>
              )}
            </div>

            {/* Info */}
            <p className="text-[10px] text-muted-foreground mt-4 text-center">
              Enter the IP address of your Raspberry Pi Flask server
            </p>
          </div>
        </div>
      )}
    </>
  );
};
