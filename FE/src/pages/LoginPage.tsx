import { useState, useEffect } from "react";

export const LoginPage = () => {
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    setTimeout(() => {
      setLoading(false);
    }, 2000)
  }, []);

  if (loading) return (<>
  <div className="w-full flex flex-col justify-center items-center animate-fade-in text-center">Loading...</div>
  </>)
  return (
    <>
      <div className="w-full h-screen flex flex-col justify-center items-center">
        <div>
          <p className="text-center">로그인페이지가 들어갈 예정입니다.</p>
        </div>
      </div>
    </>
  );
};

export default LoginPage;