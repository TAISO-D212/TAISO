import { motion, AnimatePresence } from "framer-motion";

interface ISignUpPageProps {
  setModalIsOpen: React.Dispatch<React.SetStateAction<boolean>>;
}
export const SignUpPage = ({ setModalIsOpen }: ISignUpPageProps) => {

  const overlayVariants = {
    visible: {
      opacity: 1,
      transition: {
        when: "beforeChildren",
        duration: 0.3,
        delayChildren: 0.4
      }
    },
    hidden: {
      opacity: 0,
      transition: {
        when: "afterChildren",
        duration: 0.3,
        delay: 0.4
      }
    },
  };

  return (<>
    <AnimatePresence>
      <motion.div
        initial="hidden"
        animate="visible"
        exit="hidden"
        variants={overlayVariants}
        className="modal-outer"
      >
        <motion.div
          className="modal-inner"
          initial={{ y: "100vh" }}
          animate={{ y: 0 }}
          exit={{ y: "100vh" }}
          transition={{ duration: 0.5 }}
        >
            <div className="modal-header">
              <h5 className="modal-title">회원가입</h5>
            </div>
            <div className="modal-content">
              <div className="h-10 w-[90%] flex justify-end items-center my-[5%]">
                <input className="h-10 w-[65%] placeholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%] mr-[5%]" placeholder="이메일"></input>
                <div className="h-10 w-[30%] bg-blue rounded-full text-white my-[5%] px-[5%] flex justify-center items-center">중복확인</div>
              </div>
              <input className="h-10 w-[90%] laceholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]" placeholder="비밀번호"></input>
              <input className="h-10 w-[90%] laceholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]" placeholder="비밀번호 확인"></input>
            </div>
          <div className="modal-footer">
            <button
              className="modal-button"
              onClick={() => setModalIsOpen(prev => !prev)}
            >
              회원가입
            </button>
          </div>
        </motion.div>
      </motion.div>
    </AnimatePresence>
  </>)
}