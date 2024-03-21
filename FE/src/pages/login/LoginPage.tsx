import { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import Logo192 from "../../assets/icon/icon_192.png"
import Logo72 from "../../assets/icon/icon_72.png"
import LogoImage from "../../assets/image/Loding_Image.png"
import { SignUpPage } from "../signup/SignUpPage";

export const LoginPage = () => {
  const navigate = useNavigate();
  const [loading, setLoading] = useState(true);
  const [modalIsOpen, setModalIsOpen] = useState(false);

  useEffect(() => {
    setTimeout(() => {
      setLoading(false);
    }, 2000)
  }, []);

  const handleSignUpModal = () => {
    setModalIsOpen(modalIsOpen => !modalIsOpen)
  }

  const onClickLogInHandler = () => {
    navigate('/main')
  }

  if (loading) return (<>
  <div className="flex flex-col justify-center overflow-hidden items-center animate-fadeIn">
    <img src={Logo192} alt="TAISO LOGO 192SIZE" />
    <div className="mt-[15px] font-['Pretendard-Bold'] text-[60px]">TAISO</div>
    <div className="mt-[15px] w-[90%] h-100">
      <img src={LogoImage} alt="TAISO SERVICE IMAGE" />
    </div>
  </div>
  </>)
  return (
    <>
      <div className="w-[90%] h-[90%] flex flex-col justify-center items-center">
        <div className="w-[90%] h-24 flex justify-evenly items-center my-[5%]">
          <div>
            <img src={Logo72} alt="TAISO LOGO 72SIZE" />
          </div>
          <div className="font-['Pretendard-Bold'] text-[50px]">타이소</div>
        </div>
        <div className="h-24 flex flex-col justify-center items-center my-[5%]">
          <div className="font-['Pretendard-Bold'] text-[36px] my-[5%]">Login</div>
          <div className="text-disabled">아이디 비밀번호를 입력해 주세요.</div>
        </div>
        <div className="w-[90%] h-[50%] flex flex-col justify-center items-center">
          <input className="h-14 w-[90%] placeholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]" placeholder="이메일"></input>
          <input className="h-14 w-[90%] laceholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]" placeholder="비밀번호"></input>
          <div className="h-14 w-[90%] bg-blue rounded-full text-white font-['Pretendard-Bold'] text-[20px] my-[5%] px-[5%] flex justify-center items-center" onClick={onClickLogInHandler}>로그인</div>
          <div className="w-[100%] font-['Pretendard-Bold'] mt-[10%] px-[5%] text-center">
            처음 이용하시는 유저신가요?
            <span>&nbsp;&nbsp;&nbsp;&nbsp;</span>
            <span className="text-blue hover:underline" onClick={handleSignUpModal}>회원가입</span>
          </div>  
        </div>
        </div>
        {modalIsOpen && <SignUpPage setModalIsOpen={setModalIsOpen}/>}
    </>
  );
};