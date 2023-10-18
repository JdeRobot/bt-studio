import { Point } from './Point';
export var BoundsCorner;
(function (BoundsCorner) {
    BoundsCorner["TOP_LEFT"] = "TL";
    BoundsCorner["TOP_RIGHT"] = "TR";
    BoundsCorner["BOTTOM_RIGHT"] = "BR";
    BoundsCorner["BOTTOM_LEFT"] = "BL";
})(BoundsCorner || (BoundsCorner = {}));
export const boundsFromPositionAndSize = (x, y, width, height) => {
    return {
        [BoundsCorner.TOP_LEFT]: new Point(x, y),
        [BoundsCorner.TOP_RIGHT]: new Point(x + width, y),
        [BoundsCorner.BOTTOM_RIGHT]: new Point(x + width, y + height),
        [BoundsCorner.BOTTOM_LEFT]: new Point(x, y + height)
    };
};
export const createEmptyBounds = () => {
    return {
        [BoundsCorner.TOP_LEFT]: new Point(),
        [BoundsCorner.TOP_RIGHT]: new Point(),
        [BoundsCorner.BOTTOM_RIGHT]: new Point(),
        [BoundsCorner.BOTTOM_LEFT]: new Point()
    };
};
//# sourceMappingURL=Bounds.js.map