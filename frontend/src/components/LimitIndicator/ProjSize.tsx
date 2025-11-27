import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledActionsSection,
  StyledActionsSubSection,
} from "BtStyles/Pages/Home.styles";
import React, { useEffect, useState } from "react";
import { LimitIndicator } from "BtComponents/LimitIndicator";
import { StyledSpacer } from "BtStyles/ProjectMenu/ProjectEntry.styles";
import { getUserInfo } from "BtApi/TreeWrapper";

interface UserData {
  size: { curr: number; max: number };
  projects: { curr: number; max: number };
}

const Indicator = () => {
  const theme = useBtTheme();
  const [data, setData] = useState<UserData | undefined>(undefined);

  const getUserData = async () => {
    const data = await getUserInfo();
    console.log(data);
    setData(data)
    return data;
  };

  useEffect(() => {
    getUserData();
  }, []);

  return (
    <StyledActionsSection style={{ flexGrow: 1 }}>
      {data && (
        <StyledActionsSubSection style={{ marginTop: "auto" }}>
          <LimitIndicator size={data.projects.curr} max={data.projects.max} units="projects" />
          <StyledSpacer bg={theme.palette.bgLight} />
          <LimitIndicator size={data.size.curr} max={data.projects.max} units="size" />
        </StyledActionsSubSection>
      )}
    </StyledActionsSection>
  );
};

export default Indicator;
