interface ISignUpPageProps {
  setShowModal: React.Dispatch<React.SetStateAction<boolean>>;
}
export const SignUpPage = ({ setShowModal }: ISignUpPageProps) => {

  const closeModal = () => {
    setShowModal(false);
  }

  return (<>
    <div className="fixed w-full h-[70%] flex flex-col justify-center items-center">
      <div onClick={closeModal}>X</div>
      <div>모달창입니다!</div>
    </div>
  </>)
}