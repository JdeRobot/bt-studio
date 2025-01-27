import { createContext, useContext, useState } from "react";
import "./ErrorModal.css";
import Modal from "../Modal/Modal";

enum ErrorType {
  ERROR,
  ERROR_CRITICAL,
  WARNING,
  INFO,
}

const ErrorContext = createContext<ErrorData>({
  isOpen: false,
  msg: "",
  type: ErrorType.ERROR,
  error: (msg: string) => {},
  error_critical: (msg: string) => {},
  warning: (msg: string) => {},
  info: (msg: string) => {},
  close: () => {},
});

interface ErrorData {
  isOpen: boolean;
  msg: string;
  type: ErrorType;
  error: (msg: string) => void;
  error_critical: (msg: string) => void;
  warning: (msg: string) => void;
  info: (msg: string) => void;
  close: () => void;
}

const ErrorModal = () => {
  const { isOpen, msg, type, close } = useError();

  var type_str = "error";
  var type_header = "Error";

  var onClose = () => close();

  switch (type) {
    case ErrorType.ERROR:
      type_str = "error";
      type_header = "Error";
      break;
    case ErrorType.ERROR_CRITICAL:
      type_str = "error";
      type_header = "Error";
      onClose = () => {
        document.location.href = "/apps";
      };
      break;
    case ErrorType.WARNING:
      type_str = "warning";
      type_header = "Warning";
      break;
    case ErrorType.INFO:
      type_str = "info";
      type_header = "Info";
      break;

    default:
      break;
  }

  return (
    <Modal
      id={`${type_str}-modal`}
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <div className="bt-modal-titlebar">
        <label
          className="bt-modal-titlebar-title"
          htmlFor="actionName"
          style={{ textAlign: "center" }}
        >
          {type_header}
        </label>
      </div>
      <div className="bt-form-row">
        <div className="bt-error-modal-buttons-container">
          <label className={`bt-modal-${type_str}-label`} id="errorMsg">
            {msg}
          </label>
        </div>
      </div>
      <div className="bt-form-row">
        <div className="bt-error-modal-buttons-container">
          <div
            className={`bt-${type_str}-modal-button`}
            onClick={() => onClose()}
          >
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

  const showErrorCritical = (msg: string) => {
    setMsg(msg);
    setType(ErrorType.ERROR_CRITICAL);
    open(true);
  };

  const showWarning = (msg: string) => {
    setMsg(msg);
    setType(ErrorType.WARNING);
    open(true);
  };

  const showInfo = (msg: string) => {
    setMsg(msg);
    setType(ErrorType.INFO);
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
    error_critical: (msg: string) => showErrorCritical(msg),
    warning: (msg: string) => showWarning(msg),
    info: (msg: string) => showInfo(msg),
    close: () => close(),
  };

  return (
    <ErrorContext.Provider value={context}>{children}</ErrorContext.Provider>
  );
};

export default ErrorModal;
export const useError = () => useContext(ErrorContext);
