import { defineConfig } from "vite";
import react from "@vitejs/plugin-react-swc";
import path from "path";
import { componentTagger } from "lovable-tagger";

// https://vitejs.dev/config/
export default defineConfig(({ mode }) => ({
  server: {
    host: "::",
    port: 8080,
    hmr: {
      overlay: false,
    },
    watch: {
      // This repo contains a self-referential symlink used by tooling.
      // Ignore it to prevent Vite watcher ELOOP crashes (blank screen in dev).
      ignored: ["**/_codeql_detected_source_root/**", "**/_codeql_detected_source_root"],
    },
  },
  build: {
    outDir: "dist",
    sourcemap: false,
  },
  plugins: [react(), mode === "development" && componentTagger()].filter(Boolean),
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
}));
