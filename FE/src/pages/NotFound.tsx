import { useNavigate } from "react-router-dom";

export const NotFound = () => {

  const navigate = useNavigate()

  const onClickHandler = () => {
    // TODO : 로그인한 사용자와 로그인하지 않은 사용자의 경로를 나눠서 처리!
    navigate('/')
  }

  return (
    <div className="gird grid-cols-1 gap-6 flex flex-col justify-center items-center animate-fadeIn" onClick={onClickHandler}>
      <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/Smilies/Face%20with%20Crossed-Out%20Eyes.png" alt="Face with Crossed-Out Eyes" width="200" height="200" />
      <div className="font-['Pretendard-Bold'] text-[36px]">잘못된 경로입니다!</div>
      <div className="text-subText text-center text-[20px]">404 Not Found</div>
    </div>
  );
}