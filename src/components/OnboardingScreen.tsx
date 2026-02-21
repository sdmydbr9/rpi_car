import { useState } from "react";

interface OnboardingScreenProps {
  onComplete: (name: string, age: number) => void;
}

export const OnboardingScreen = ({ onComplete }: OnboardingScreenProps) => {
  const [name, setName] = useState("");
  const [age, setAge] = useState("");
  const [step, setStep] = useState<"name" | "age">("name");

  const handleNameNext = () => {
    if (name.trim().length > 0) setStep("age");
  };

  const handleComplete = () => {
    const ageNum = parseInt(age, 10);
    if (ageNum > 0 && ageNum < 120 && name.trim()) {
      onComplete(name.trim().toUpperCase(), ageNum);
    }
  };

  return (
    <div className="h-[100dvh] w-full flex flex-col items-center justify-center bg-background relative overflow-hidden">
      {/* Background grid */}
      <div className="absolute inset-0 opacity-5">
        <div className="w-full h-full" style={{
          backgroundImage: 'linear-gradient(hsl(var(--primary) / 0.3) 1px, transparent 1px), linear-gradient(90deg, hsl(var(--primary) / 0.3) 1px, transparent 1px)',
          backgroundSize: '40px 40px'
        }} />
      </div>

      {/* Scanning line */}
      <div className="absolute inset-0 pointer-events-none overflow-hidden">
        <div className="w-full h-px bg-gradient-to-r from-transparent via-primary to-transparent animate-pulse opacity-40"
          style={{ position: 'absolute', top: '30%' }} />
      </div>

      {/* Content */}
      <div className="relative z-10 flex flex-col items-center gap-6 px-6">
        {/* AMG Logo */}
        <div className="flex items-center gap-2 mb-2">
          <div className="flex gap-1">
            {[...Array(5)].map((_, i) => (
              <div key={i} className="w-1 h-6 bg-gradient-to-b from-muted-foreground to-muted transform -skew-x-12" />
            ))}
          </div>
          <span className="text-foreground font-bold tracking-wider text-xl">AMG</span>
          <span className="text-primary font-bold racing-text text-xl">PETRONAS</span>
        </div>

        <div className="text-muted-foreground racing-text text-xs tracking-[0.3em] uppercase">
          Driver Registration
        </div>

        {/* Form card */}
        <div className="border border-primary/30 bg-card/60 backdrop-blur-md rounded-sm p-6 w-72 sm:w-80">
          {step === "name" ? (
            <div className="flex flex-col gap-4">
              <label className="text-[10px] sm:text-xs racing-text text-primary tracking-widest uppercase">
                Driver Name
              </label>
              <input
                type="text"
                value={name}
                onChange={e => setName(e.target.value)}
                onKeyDown={e => e.key === "Enter" && handleNameNext()}
                placeholder="Enter your callsign"
                autoFocus
                maxLength={16}
                className="bg-secondary/50 border border-border text-foreground racing-text text-lg tracking-wider px-3 py-2 rounded-sm focus:outline-none focus:border-primary placeholder:text-muted-foreground/40 placeholder:text-sm uppercase"
              />
              <button
                onClick={handleNameNext}
                disabled={!name.trim()}
                className="mt-2 bg-primary/20 border border-primary/50 text-primary racing-text text-sm tracking-widest py-2 rounded-sm hover:bg-primary/30 transition-colors disabled:opacity-30 disabled:cursor-not-allowed uppercase"
              >
                Next →
              </button>
            </div>
          ) : (
            <div className="flex flex-col gap-4">
              <label className="text-[10px] sm:text-xs racing-text text-primary tracking-widest uppercase">
                Driver Age
              </label>
              <input
                type="number"
                value={age}
                onChange={e => setAge(e.target.value)}
                onKeyDown={e => e.key === "Enter" && handleComplete()}
                placeholder="Age"
                autoFocus
                min={1}
                max={119}
                className="bg-secondary/50 border border-border text-foreground racing-text text-lg tracking-wider px-3 py-2 rounded-sm focus:outline-none focus:border-primary placeholder:text-muted-foreground/40 placeholder:text-sm"
              />
              <div className="flex gap-2">
                <button
                  onClick={() => setStep("name")}
                  className="flex-1 bg-muted/30 border border-border text-muted-foreground racing-text text-sm tracking-widest py-2 rounded-sm hover:bg-muted/50 transition-colors uppercase"
                >
                  ← Back
                </button>
                <button
                  onClick={handleComplete}
                  disabled={!age || parseInt(age) < 1 || parseInt(age) > 119}
                  className="flex-1 bg-primary/20 border border-primary/50 text-primary racing-text text-sm tracking-widest py-2 rounded-sm hover:bg-primary/30 transition-colors disabled:opacity-30 disabled:cursor-not-allowed uppercase"
                >
                  Launch ⚡
                </button>
              </div>
            </div>
          )}
        </div>

        {/* Bottom accent */}
        <div className="flex gap-1 mt-4 opacity-40">
          {[...Array(3)].map((_, i) => (
            <div key={i} className="w-8 h-0.5 bg-primary rounded-full" />
          ))}
        </div>
      </div>
    </div>
  );
};
