import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { AnalogControl } from "../components/cockpit/CockpitController";


describe("manual analog controls", () => {
  it("returns throttle to zero when the pointer is released", () => {
    const onChange = vi.fn();
    render(
      <AnalogControl
        kind="throttle"
        label="Throttle"
        value={0}
        disabled={false}
        onChange={onChange}
      />,
    );
    const control = screen.getByRole("slider", { name: "Throttle" });
    vi.spyOn(control, "getBoundingClientRect").mockReturnValue({
      x: 0,
      y: 0,
      top: 0,
      left: 0,
      right: 100,
      bottom: 100,
      width: 100,
      height: 100,
      toJSON: () => ({}),
    });

    fireEvent.pointerDown(control, { pointerId: 1, clientX: 50, clientY: 0 });
    fireEvent.pointerUp(control, { pointerId: 1, clientX: 50, clientY: 0 });

    expect(onChange).toHaveBeenCalledWith(100);
    expect(onChange).toHaveBeenLastCalledWith(0);
  });

  it("centers steering after pointer cancellation", () => {
    const onChange = vi.fn();
    render(
      <AnalogControl
        kind="steering"
        label="Steering"
        value={25}
        disabled={false}
        onChange={onChange}
      />,
    );
    fireEvent.pointerCancel(screen.getByRole("slider", { name: "Steering" }));
    expect(onChange).toHaveBeenLastCalledWith(0);
  });
});
