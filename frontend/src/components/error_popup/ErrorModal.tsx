import { createContext, useContext, useEffect, useState } from "react";
import "./ErrorModal.css";
import Modal from "../Modal/Modal";

enum ErrorType {
  ERROR,
  WARNING,
}

const ErrorContext = createContext<ErrorData>({
  isOpen: false,
  msg: "",
  type: ErrorType.ERROR,
  error: (msg: string) => {},
  warning: (msg: string) => {},
  close: () => {},
});

interface ErrorData {
  isOpen: boolean;
  msg: string;
  type: ErrorType;
  error: (msg: string) => void;
  warning: (msg: string) => void;
  close: () => void;
}

const ErrorModal = () => {
  const { isOpen, msg, type, close } = useError();

  return (
    <Modal id={`${type === ErrorType.ERROR ? "error" : "warning"}-modal`} hasCloseBtn={true} isOpen={isOpen} onClose={close}>
      <div className="bt-modal-titlebar">
        <label
          className="bt-modal-titlebar-title"
          htmlFor="actionName"
          style={{ textAlign: "center" }}
        >
          {type === ErrorType.ERROR ? "Error" : "Warning"}
        </label>
      </div>
      <div className="bt-form-row">
        <div className="bt-error-modal-buttons-container">
          <label
            className={`bt-modal-${type === ErrorType.ERROR ? "error" : "warning"}-label`}
            id="errorMsg"
          >
            {msg}
          </label>
        </div>
      </div>
      <div className="bt-form-row">
        <div className="bt-error-modal-buttons-container">
          <div className={`bt-${type === ErrorType.ERROR ? "error" : "warning"}-modal-button`} onClick={() => close()}>
            Close
          </div>
        </div>
      </div>
    </Modal>
  );
};

export const ErrorProvider = ({ children }: { children: any }) => {
  const [isOpen, open] = useState<boolean>(false);
  const [msg, setMsg] = useState<string>("");
  const [type, setType] = useState<ErrorType>(ErrorType.ERROR);

  const showError = (msg: string) => {
    setMsg(msg);
    setType(ErrorType.ERROR);
    open(true);
  };

  const showWarning = (msg: string) => {
    setMsg(msg);
    setType(ErrorType.WARNING);
    open(true);
  };

  const close = () => {
    open(false);
  };

  const context: ErrorData = {
    isOpen: isOpen,
    msg: msg,
    type: type,
    error: (msg: string) => showError(msg),
    warning: (msg: string) => showWarning(msg),
    close: () => close(),
  };

  return (
    <ErrorContext.Provider value={context}>{children}</ErrorContext.Provider>
  );
};

export default ErrorModal;
export const useError = () => useContext(ErrorContext);
