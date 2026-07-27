import { useEffect } from "react";
import { CockpitController } from "./components/cockpit/CockpitController";

export default function App() {
  useEffect(() => {
    const preventContextMenu = (event: Event) => event.preventDefault();
    document.addEventListener("contextmenu", preventContextMenu);
    return () => document.removeEventListener("contextmenu", preventContextMenu);
  }, []);

  return <CockpitController />;
}
