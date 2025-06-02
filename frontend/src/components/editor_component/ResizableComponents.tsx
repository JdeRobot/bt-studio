import { Resizable } from "re-resizable";

import "./EditorComponent.css";

export const ResizableHoriz = ({
  width,
  min,
  max,
  snap,
  children,
}: {
  width: number;
  min: number;
  max: number;
  snap: number[];
  children: any;
}) => {
  return (
    <Resizable
      defaultSize={{
        width: `${width}%`,
      }}
      enable={{
        top: false,
        right: true,
        bottom: false,
        left: false,
        topRight: false,
        bottomRight: false,
        bottomLeft: false,
        topLeft: false,
      }}
      bounds="parent"
      handleClasses={{
        right: "hresize-handle",
      }}
      maxWidth={`${max}%`}
      minWidth={`${min}%`}
      snap={{ x: snap }}
      snapGap={100}
    >
      {children}
    </Resizable>
  );
};

ResizableHoriz.defaultProps = {
  min: 0,
  max: 100,
  snap: [],
};

export const ResizableVert = ({
  height,
  min,
  max,
  snap,
  children,
}: {
  height: number;
  min: number;
  max: number;
  snap: number[];
  children: any;
}) => {
  return (
    <Resizable
      defaultSize={{
        height: `${height}%`,
      }}
      enable={{
        top: false,
        right: false,
        bottom: true,
        left: false,
        topRight: false,
        bottomRight: false,
        bottomLeft: false,
        topLeft: false,
      }}
      bounds="parent"
      handleClasses={{
        bottom: "vresize-handle",
      }}
      maxHeight={`${max}%`}
      minHeight={`${min}%`}
      snap={{ y: snap }}
      snapGap={100}
    >
      {children}
    </Resizable>
  );
};

ResizableVert.defaultProps = {
  min: 0,
  max: 100,
  snap: [],
};

export const ResizableColumn = ({ children }: { children: any[] }) => {
  if (children.length === 1) {
    return <div className="ide-column-container">{children[0]}</div>;
  }

  if (children.length === 2) {
    return (
      <div className="ide-column-container">
        <ResizableVert height={100 / children.length} max={100} snap={[0]}>
          {children[0]}
        </ResizableVert>
        <div className="ide-column-filler-container">{children[1]}</div>
      </div>
    );
  }

  return <div className="ide-column-container"></div>;
};

export const ResizableRow = ({
  baseWidth,
  maxWidth,
  showExplorer,
  layout,
  children,
}: {
  baseWidth: number[];
  maxWidth: number[];
  showExplorer: boolean;
  layout: string;
  children: any[];
}) => {

  if (layout === "only-editor") {
    // Remove viewers == Remove last element
    children.pop()
    baseWidth.pop()
    maxWidth.pop()
  }

  if (layout === "only-viewers") {
    // Remove editors == Remove middle element
    children.splice(1,1)
    baseWidth.splice(1,1)
    maxWidth.splice(1,1)
  }

  if (!showExplorer) {
    // Remove explorers == Remove first element
    children.shift()
    baseWidth.shift()
    maxWidth.shift()
  }

  if (children.length === 1) {
    return <div className="ide-container">{children[0]}</div>;
  }

  return (
    <div className="ide-container">
      {children.slice(0, children.length - 1).map((comp, i) => (
        <ResizableHoriz width={baseWidth[i]} max={maxWidth[i]} snap={[0]}>
          {comp}
        </ResizableHoriz>
      ))}
      <div className="ide-filler-container">
        {children[children.length - 1]}
      </div>
    </div>
  );
};
