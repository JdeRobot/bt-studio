import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledActionsSection,
  StyledActionsSubSection,
} from "BtStyles/Pages/Home.styles";
import React, { useEffect, useState } from "react";
import {
  LimitIndicator,
  LimitIndicatorSkeleton,
} from "BtComponents/LimitIndicator";
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
    setData(data);
    return data;
  };

  useEffect(() => {
    getUserData();
  }, []);

  return (
    <StyledActionsSection style={{ flexGrow: 1 }}>
      <StyledActionsSubSection style={{ marginTop: "auto" }}>
        {data ? (
          <LimitIndicator
            size={data.projects.curr}
            max={data.projects.max}
            units="projects"
          />
        ) : (
          <LimitIndicatorSkeleton />
        )}
        <StyledSpacer bg={theme.palette.bgLight} />
        {data ? (
          <LimitIndicator
            size={data.size.curr}
            max={data.size.max}
            units="size"
          />
        ) : (
          <LimitIndicatorSkeleton />
        )}
      </StyledActionsSubSection>
    </StyledActionsSection>
  );
};

export default Indicator;
