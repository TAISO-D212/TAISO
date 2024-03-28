import { configureStore } from "@reduxjs/toolkit";
import loginSlice from "../slices/loginSlice";

// store는 금고
// reducer는 금고 안에서 파트 나누는 느낌.
// reducer는 금고를 지키는(관리하는) 애

const LoginStore = configureStore({
  reducer: {
    "loginSlice": loginSlice
  }
})

export type RootState = ReturnType<typeof LoginStore.getState>

export type AppDispatch = typeof LoginStore.dispatch

export default LoginStore