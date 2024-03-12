import { useState } from "react";
import reactLogo from "./assets/react.svg";
import viteLogo from "/vite.svg";
import "./App.css";

function App() {
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
      <h1>타이소</h1>
      <div className="card">
        <button onClick={() => setCount((count) => count + 1)}>
          count is {count}
        </button>
        <p>
          <code>src/App.tsx</code>를 수정하고 저장하면 페이지를 테스트 할 수
          있습니다.
        </p>
      </div>
      <p className="read-the-docs">
        Vite and React 로고를 클릭하고 더 많은 정보를 확인하세요.
      </p>
    </>
  );
}

export default App;
