import { Button, TextField } from "@mui/material";
import ToggleButtonGroup, {
  toggleButtonGroupClasses,
} from "@mui/material/ToggleButtonGroup";
import { styled } from "@mui/material/styles";

export const StyledToggleButtonGroup = styled(ToggleButtonGroup)(
  ({
    bg,
    selBg,
    text,
    roundness,
  }: {
    bg: string;
    selBg: string;
    text: string;
    roundness: number;
  }) => ({
    width: "100%",
    height: "15vh",
    justifyContent: "center",
    gap: "2vh",
    [`& .${toggleButtonGroupClasses.grouped}`]: {
      display: "flex",
      flexDirection: "column",
      aspectRatio: 9 / 13,
      border: 0,
      margin: 0,
      borderRadius: `${roundness}px`,
      backgroundColor: bg,
      color: text,
      boxShadow: "var(--shadow)",
      [`& label`]: {
        margin: 10,
      },
      [`& svg`]: {
        flexGrow: 1,
        width: "50%",
      },
      [`&.${toggleButtonGroupClasses.disabled}`]: {
        display: "flex",
        flexDirection: "column",
        justifyContent: "space-around",
        aspectRatio: 9 / 13,
        opacity: "40%",
        borderRadius: `${roundness}px`,
        backgroundColor: bg,
        color: text,
        border: 0,
      },
      [`&.${toggleButtonGroupClasses.selected}`]: {
        display: "flex",
        flexDirection: "column",
        justifyContent: "space-around",
        aspectRatio: 9 / 13,
        border: 0,
        margin: 0,
        borderRadius: `${roundness}px`,
        backgroundColor: selBg,
        color: text,
        [`&:hover`]: {
          backgroundColor: selBg,
          filter: "var(--hover-strong)",
        },
      },
      [`&:hover`]: {
        backgroundColor: bg,
        filter: "var(--hover-strong)",
      },
    },
  })
);

export const StyledTextField = styled(TextField)(
  ({
    bg,
    selBg,
    errBg,
    text,
    roundness,
  }: {
    bg: string;
    selBg: string;
    errBg: string;
    text: string;
    roundness: number;
  }) => ({
    width: "50%",
    backgroundColor: bg,
    borderRadius: `${roundness}px`,
    boxShadow: "var(--shadow)",
    "&:hover": {
      filter: "var(--hover-strong)",
    },
    "& label": {
      color: text,
      "&.Mui-focused": {
        color: text,
      },
      "&.Mui-error": {
        color: errBg,
      },
      "& span": {
        "&.Mui-focused": {
          color: text,
        },
        "&.Mui-error": {
          color: errBg,
        },
      },
    },
    "& input": {
      color: text,
    },
    "& .MuiFilledInput-underline:after": {
      borderBottomColor: `${selBg}`,
    },
    "& .MuiFilledInput-underline.Mui-error:before": {
      borderBottomColor: `${errBg} !important`,
    },
    "& .MuiFilledInput-underline.Mui-error:after": {
      borderBottomColor: `${errBg} !important`,
    },
  })
);

export const StyledButton = styled(Button)(({
    bg,
    text,
    roundness,
  }: {
    bg: string;
    text: string;
    roundness: number;
  }) => ({
    margin: "3vh",
    height: "5vh",
    width: "10vh",
    backgroundColor: `${bg} !important`,
    borderRadius: `${roundness}px`,
    boxShadow: "var(--shadow) !important",
    color: `${text} !important`,
    "&:hover": {
      filter: "var(--hover-strong)",
    },
}));