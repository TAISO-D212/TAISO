import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import svgr from "vite-plugin-svgr";

// https://vitejs.dev/config/
export default defineConfig({
  base: "/",
  server: {
    port: 5173,
    host: "0.0.0.0",
  },
  plugins: [
    react(),
    svgr({
      svgrOptions: {
        icon: true,
      },
    }),
  ],
  assetsInclude: ["**/*.jpg", "**/*.jpeg", "**/*.png", "**/*.svg", "**/*.gif"],
});
