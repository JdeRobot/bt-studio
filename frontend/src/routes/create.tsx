import React from "react";
import { HomeHeader } from "BtComponents/HeaderMenu";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { StyledAppContainer } from "BtStyles/App.styles";

const App = () => {
  const theme = useBtTheme();
  
  return (
    <StyledAppContainer bg={theme.palette.background}>
      <HomeHeader />
      <h1>Home Page</h1>
    </StyledAppContainer>
  );
};

export default App;
