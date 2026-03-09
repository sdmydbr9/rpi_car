import { useState } from "react";
import { Gamepad2 } from "lucide-react";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";

interface GamepadControlsDialogProps {
  gamepadConnected?: boolean;
  children: React.ReactNode;
}

export const GamepadControlsDialog = ({
  gamepadConnected = false,
  children,
}: GamepadControlsDialogProps) => {
  const [open, setOpen] = useState(false);

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>{children}</DialogTrigger>
      <DialogContent className="max-w-md sm:max-w-lg border-primary/30 bg-card/95 backdrop-blur-md max-h-[85dvh] overflow-y-auto p-4 sm:p-6">
        <DialogHeader>
          <DialogTitle className="racing-text text-sm sm:text-base tracking-wider flex items-center gap-2">
            <Gamepad2
              className={`w-4 h-4 sm:w-5 sm:h-5 ${
                gamepadConnected ? "text-green-400" : "text-destructive"
              }`}
            />
            GAMEPAD CONTROLS
            <span
              className={`ml-auto text-[9px] sm:text-[10px] px-2 py-0.5 rounded-full border ${
                gamepadConnected
                  ? "border-green-500/50 text-green-400 bg-green-500/10"
                  : "border-destructive/50 text-destructive bg-destructive/10"
              }`}
            >
              {gamepadConnected ? "CONNECTED" : "DISCONNECTED"}
            </span>
          </DialogTitle>
        </DialogHeader>

        <div className="space-y-4 text-xs sm:text-sm">
          {/* Sticks */}
          <Section title="STICKS">
            <Row label="Left Stick Y" desc="Throttle (forward / reverse)" />
            <Row label="Right Stick X" desc="Steering (left / right)" />
          </Section>

          {/* Gears */}
          <Section title="GEARS (Face Buttons)">
            <Row label="A" desc="Gear 1 — 35% max power" badge="1" badgeColor="text-green-400 border-green-500/40" />
            <Row label="B" desc="Gear 2 — 60% max power" badge="2" badgeColor="text-yellow-400 border-yellow-500/40" />
            <Row label="X" desc="Gear 3 — 80% max power" badge="3" badgeColor="text-orange-400 border-orange-500/40" />
            <Row label="Y" desc="SPORT — 100% max power" badge="S" badgeColor="text-red-400 border-red-500/40" />
          </Section>

          {/* Braking */}
          <Section title="BRAKING">
            <Row label="L3 / R3" desc="Hold to brake (thumb-stick press)" />
            <Row label="X (during autopilot)" desc="Emergency Brake — stops all motors" />
            <Row label="LT + RT + X" desc="Emergency Brake (manual mode)" />
          </Section>

          {/* Engine */}
          <Section title="ENGINE">
            <Row label="Start" desc="Toggle engine ON / OFF" />
            <Row label="Select × 2" desc="Kill-switch — force engine OFF" />
            <Row label="Select + Start" desc="Hot Start — instant engine + gamepad ON" />
          </Section>

          {/* Autopilot / Auto-Accel */}
          <Section title="AUTOPILOT & CRUISE">
            <Row label="LB + RB" desc="Toggle Autopilot (engine must be ON)" />
            <Row label="LT + RT" desc="Toggle Auto-Acceleration (cruise)" />
          </Section>

          {/* Sensor Combos */}
          <Section title="SENSOR TOGGLES (Select held)">
            <Row label="Select + A" desc="Toggle Sonar sensor" />
            <Row label="Select + X" desc="Toggle IR sensor" />
          </Section>
        </div>
      </DialogContent>
    </Dialog>
  );
};

/* ── tiny sub-components ─────────────────────────────────── */

function Section({
  title,
  children,
}: {
  title: string;
  children: React.ReactNode;
}) {
  return (
    <div>
      <h3 className="racing-text text-[10px] sm:text-xs text-primary/80 tracking-widest mb-1.5 border-b border-primary/20 pb-1">
        {title}
      </h3>
      <div className="space-y-1">{children}</div>
    </div>
  );
}

function Row({
  label,
  desc,
  badge,
  badgeColor,
}: {
  label: string;
  desc: string;
  badge?: string;
  badgeColor?: string;
}) {
  return (
    <div className="flex items-center gap-2 py-0.5">
      <kbd className="min-w-[5.5rem] shrink-0 text-[10px] sm:text-xs font-mono px-1.5 py-0.5 rounded bg-muted/60 border border-border/50 text-center text-foreground">
        {label}
      </kbd>
      <span className="text-muted-foreground text-[10px] sm:text-xs flex-1">
        {desc}
      </span>
      {badge && (
        <span
          className={`text-[9px] sm:text-[10px] font-bold px-1.5 py-0.5 rounded-full border ${
            badgeColor ?? "text-muted-foreground border-border/50"
          }`}
        >
          {badge}
        </span>
      )}
    </div>
  );
}
