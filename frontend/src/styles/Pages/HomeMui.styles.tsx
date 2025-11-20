import ToggleButtonGroup, {
  toggleButtonGroupClasses,
} from "@mui/material/ToggleButtonGroup";
import { styled } from "@mui/material/styles";

export const StyledVertToggleGroup = styled(ToggleButtonGroup)(
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
    display: "flex",
    flexDirection: "column",
    width: "90%",
    justifyContent: "center",
    gap: "1vh",
    alignItems: "start",
    [`& .${toggleButtonGroupClasses.grouped}`]: {
      border: 0,
      margin: 0,
      borderRadius: `${roundness}px`,
      width: "100%",
      height: "3vh",
      fontSize: "1.2rem",
      padding: "10px",
      backgroundColor: bg,
      color: text,
      fontWeight: "unset",
      textTransform: "none",
      justifyContent: "start",
      [`&.${toggleButtonGroupClasses.disabled}`]: {
        opacity: "40%",
        borderRadius: `${roundness}px`,
        backgroundColor: bg,
        color: text,
        border: 0,
      },
      [`&.${toggleButtonGroupClasses.selected}`]: {
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
  }),
);
