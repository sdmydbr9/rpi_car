export const useGameFeedback = () => {
  const triggerHaptic = (type: 'light' | 'medium' | 'heavy') => {
    if ('vibrate' in navigator) {
      switch (type) {
        case 'light':
          navigator.vibrate(10);
          break;
        case 'medium':
          navigator.vibrate(20);
          break;
        case 'heavy':
          navigator.vibrate([50, 30, 50]);
          break;
      }
    }
  };

  const playSound = (sound: string) => {
    // Placeholder for sound effects
    console.log(`Playing sound: ${sound}`);
  };

  return { triggerHaptic, playSound };
};
