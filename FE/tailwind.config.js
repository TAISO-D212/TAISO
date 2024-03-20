/** @type {import('tailwindcss').Config} */
export default {
  content: ["./src/**/*.{html,js,ts,jsx,tsx}"],
  theme: {
    extend: {
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
        "fade-in": "fade-in 0.5s ease-in",
        "fade-out": "fade-out 0.5s ease-out",
        "fade-in-out": "fade-in-out 1s ease-in-out",
        "fade-out-in": "fade-out-in 1s ease-in-out",
      },
    },
    fontFamily: {
      Pretendard_Regular: ["Pretendard-Regular"],
      Pretendard_Bold: ["Prentendard-Bold"],
    },
    plugins: [],
  },
};
