import { create } from "zustand";

type State = {
  modalOpen: {
    isOpen: boolean;
    status: string;
  };
  setIsModalOpen: (isOpen: boolean, status: string) => void;
};

export const ModalStore = create<State>((set) => ({
  modalOpen: { isOpen: false, status: "" },
  setIsModalOpen: (isOpen, status) =>
    set(() => ({
      modalOpen: { isOpen, status },
    })),
}));
