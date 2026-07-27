import { describe, expect, it } from "vitest";
import {
  ACTIVE_CONTROL_REFRESH_MS as MOBILE_REFRESH_MS,
  isActiveControl as isMobileControlActive,
  SPRING_RETURN_VALUE as MOBILE_SPRING_RETURN,
} from "../../app/src/lib/manualInput";
import {
  ACTIVE_CONTROL_REFRESH_MS as WEB_REFRESH_MS,
  isActiveControl as isWebControlActive,
  SPRING_RETURN_VALUE as WEB_SPRING_RETURN,
} from "../lib/manualInput";

describe("web and mobile manual input policy", () => {
  it("uses 100 ms refresh only for throttle or brake activity", () => {
    expect(WEB_REFRESH_MS).toBe(100);
    expect(MOBILE_REFRESH_MS).toBe(100);
    expect(isWebControlActive({ throttle: 25, brake: false })).toBe(true);
    expect(isMobileControlActive({ throttle: 0, brake: true })).toBe(true);
    expect(isWebControlActive({ throttle: 0, brake: false })).toBe(false);
    expect(isMobileControlActive({ throttle: 0, brake: false })).toBe(false);
  });

  it("uses zero as the spring-return value on both clients", () => {
    expect(WEB_SPRING_RETURN).toBe(0);
    expect(MOBILE_SPRING_RETURN).toBe(0);
  });
});
