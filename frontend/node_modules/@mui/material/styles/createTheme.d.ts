import { CssVarsThemeOptions } from "./createThemeWithVars.js";
import { Theme, ThemeOptions } from "./createThemeNoVars.js";
export type { ThemeOptions, Theme, CssThemeVariables } from "./createThemeNoVars.js";
/**
 * Generate a theme base on the options received.
 * @param options Takes an incomplete theme object and adds the missing parts.
 * @param args Deep merge the arguments with the about to be returned theme.
 * @returns A complete, ready-to-use theme object.
 */
export default function createTheme(options?: Omit<ThemeOptions, 'components'> & Pick<CssVarsThemeOptions, 'defaultColorScheme' | 'colorSchemes' | 'components'> & {
  cssVariables?: boolean | Pick<CssVarsThemeOptions, 'colorSchemeSelector' | 'rootSelector' | 'disableCssColorScheme' | 'cssVarPrefix' | 'shouldSkipGeneratingVar' | 'nativeColor'>;
},
// cast type to skip module augmentation test
...args: object[]): Theme;