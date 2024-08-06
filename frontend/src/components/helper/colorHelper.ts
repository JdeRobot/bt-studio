export const strRGBToLuminance = (str: string) => {
  /* calculates perceived lightness using the sRGB Luma method 
  Luma = (red * 0.2126 + green * 0.7152 + blue * 0.0722) / 255 */
  let rgb: any = str.split("(")[1].split(")")[0].split(",");
  let luma = (rgb[0] * 0.2126 + rgb[1] * 0.7152 + rgb[2] * 0.0722) / 255;
  return luma;
};

export const rgbToLuminance = (r: number, g: number, b: number) => {
  /* calculates perceived lightness using the sRGB Luma method 
  Luma = (red * 0.2126 + green * 0.7152 + blue * 0.0722) / 255 */
  let luma = (r * 0.2126 + g * 0.7152 + b * 0.0722) / 255;
  return luma;
};
