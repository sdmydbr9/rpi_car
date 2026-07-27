export const ACTIVE_CONTROL_REFRESH_MS = 100;
export const SPRING_RETURN_VALUE = 0;

export function isActiveControl(input: {
  throttle: number;
  brake: boolean;
}): boolean {
  return input.throttle > 0 || input.brake;
}
