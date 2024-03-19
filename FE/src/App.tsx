import { useState } from "react";
import reactLogo from "./assets/react.svg";
import viteLogo from "/vite.svg";
import "./App.css";

export const App = () => {
  const [count, setCount] = useState(0);

  return (
    <>
      <div>
        <a href="https://vitejs.dev" target="_blank">
          <img src={viteLogo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>
      <h1>TAISO LET'S START PROJECT ! </h1>
      <div className="card">
        <button onClick={() => setCount((count) => count + 1)}>
          현재 카운트는 {count} 입니다. 클릭해서 증가시켜보세요.
        </button>
      </div>
      <p className="read-the-docs">
        더 많은 정보를 원하신다면 REACT 또는 VITE 로고를 클릭해 보세요.
      </p>
    </>
  );
};
