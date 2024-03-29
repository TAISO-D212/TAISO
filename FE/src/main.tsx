import ReactDOM from "react-dom/client";
import { App } from "./App.tsx";
import "./index.css";
import { Provider } from "react-redux";
import LoginStore from "./store/LoginStore.ts";
import { MQTTProvider } from "./utils/mqttProvider.tsx";

ReactDOM.createRoot(document.getElementById("root")!).render(
  <Provider store={LoginStore}>
    <MQTTProvider>
      <App />
    </MQTTProvider>
  </Provider>
);
