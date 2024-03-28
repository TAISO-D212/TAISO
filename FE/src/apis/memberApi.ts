import axios from "axios";
import {viteConfig} from "./viteConfig"

export type TLoginParam = {
  email: string
  pw: string
}

const host = `${viteConfig.VITE_BASE_URL}/api/members`;

// loginSlice에서 이 loginPost라는 애를 호출할 것인데 이때 사용하는 방법이 create-async-sync라는 애를 이용. 그리고 추가적으로 필요한 것이 Extra Reducer라는 애를 추가!

export const loginPost = async (loginParam:TLoginParam, ) => {

  // 일반 post 방식으로 데이터를 보내므로
  const header = { headers: { "Content-Type": "x-www-form-urlencoded" } };

  const form = new FormData();
  form.append("username", loginParam.email);
  form.append("password", loginParam.pw);

  const res = await axios.post(`${host}/login`, form, header);

  return res.data;
};