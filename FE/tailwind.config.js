// import { colorPalette } from "./src/constants/colorPalette.ts";
import daisyui from 'daisyui';
/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{vue,js,ts,jsx,tsx}'],
  theme: {
    extend: {
      // colors: colorPalette,
      keyframes: {
        "fade-in": {
          "0%": {
            opacity: "0",
          },
          "100%": {
            opacity: "1",
          },
        },
        "fade-out": {
          "0%": {
            opacity: "1",
          },
          "100%": {
            opacity: "0",
          },
        },
        "fade-in-out": {
          "0%": {
            opacity: "0",
          },
          "50%": {
            opacity: "1",
          },
          "100%": {
            opacity: "0",
          },
        },
        "fade-out-in": {
          "0%": {
            opacity: "0",
          },
          "50%": {
            opacity: "0",
          },
          "100%": {
            opacity: "1",
          },
        },
      },
      animation: {
        fadeIn: "fade-in 0.5s ease-in",
        fadeOut: "fade-out 0.5s ease-out",
        fadeInOut: "fade-in-out 1s ease-in-out",
        fadeOutIn: "fade-out-in 1s ease-in-out",
      },
    },
    fontFamily: {
      Pretendard_Regular: ["Pretendard-Regular"],
      Pretendard_Bold: ["Prentendard-Bold"],
    },
  },
  plugins: [daisyui],
};
