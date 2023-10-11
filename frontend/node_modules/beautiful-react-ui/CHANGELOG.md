# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2019-06-10

### Added

- Create package.json
- Setup .gitignore
- Add a CHANGELOG.md
- Add a Readme.md and a Contributing.md
- Add Styleguidist
- Add ESLint
- Add Stylelint
- Add Jest test framework + Enzyme test utility
- Build System
- Button Component
- Spinner Component
- Icon Component

## [0.2.0] - 2019-06-10

### Added

- ToggleSwitch component
- Label private component, to be used to render labels in form components
- HelpText component, to be used to render help texts in form components
- ToggleSwitch component tests
- Label component tests
- HelpText component tests


## [0.3.0] - 2019-06-14

### Added

- Accessibility tools (AXE and jsx-a11y)


## [0.4.0] - 2019-06-13

### Added

- ButtonGroup component
- ButtonGroup tests
- `lodash` as project dependency

## [0.5.0] - 2019-06-14

### Added

- Breadcrumb component
- Breadcrumb component tests 

## [0.6.0] - 2019-06-18

### Added

- Pill component
- Pill component tests 
- Breadcrumb `renderer` prop renamed as `render`


## [0.6.1] - 2019-06-19

### Fixed

- The rounded prop now defines the shape of the Pill component.

## [0.7.0] - 2019-06-20

### Added

- Alert component
- Alert component tests 

## [0.7.1] - 2019-06-20

### Fixes

- Colors description now correctly defines `danger` color.


## [0.8.0] - 2019-06-20

### Added

- Accordion component
- Accordion component tests

## [0.9.0] - 2019-07-01

### Added

- Tab component
- Tab component tests

## [0.9.1] - 2019-07-01

### Fixes

- BaseProps added to the TabContent component
- TabContent component tests added to test the BaseProps

## [0.9.2] - 2019-07-01

### Fixes

- Accordion color behaviour changed
- Accordion tests changed to test the new color prop

## [0.9.3] - 2019-07-01

### Fixes

- ToggleSwitch component focus state defined by CSS
- Accessibility issues on ToggleSwitch component fixed: added `aria-label`

## [0.9.4] - 2019-07-02

### Fixes

- Accessibility issues on Button component fixed: added `aria-label`


## [0.9.5] - 2019-07-02

### Fixes

- Button component focus state defined by CSS

## [0.9.6] - 2019-07-02

### Fixes

- Tab component is now exported from index.js

## [0.10.0] - 2019-07-02

### Added

- Test coverage report with NYC
- Test coverage should now be >= 80%

## [0.11.0] - 2019-07-02

### Added

- Tests on shared functions

## [0.12.0] - 2019-07-02

### Added

- Placeholder component
- Placeholder component tests

## [0.13.0] - 2019-07-02

### Added

- Image component
- Image component tests

## [0.14.0] - 2019-07-03

### Added

- `rounded` prop to the Image component
- Avatar component
- Avatar component tests

## [0.15.0] - 2019-07-04

### Added

- `warn` utility function has been added to have a more readable codebase

## [0.16.0] - 2019-07-04

### Added

- Close Icon
- Close Icon tests

## [0.16.1] - 2019-07-04

### Fixed

- CloseIcon component was not showing in Alert
- Improved Alert documentation
- `nyc` added to `devDependencies`
- Avatar sizes bug fixed

## [0.17.0] - 2019-07-05

### Added

- Portal component
- Portal component tests
- Modal component
- Modal component tests

### Fixed

- Few PropTypes validation errors

## [0.18.0] - 2019-07-14

### Added

- Props are spread down to the component's first element (when possible)

### Fixed

- Removed `BaseProps` shared prop type

## [0.19.0] - 2019-07-19

### Added

- Popup component
- Popup tests
- getFloaterAbsolutePosition function
- getFloaterAbsolutePosition tests

## [0.20.0] - 2019-07-18

### Added

- Input component
- Icon component

## [0.20.1] - 2019-08-04

### Added

- Input component tests


## [0.21.0] - 2019-08-07

### Added

- FloatingContent component
- FloatingContent component tests
- Renaming Popup component as Popover
- Rewriting Popover on top of FloatingContent
- Dependencies updated

## [0.22.0] - 2019-08-07

### Added

- Select component
- Select component tests

### Fixed
- Input component: removed className from <input> tag


## [0.23.0] - 2019-09-20

### Added

- Layout folder
- Grid component
- GridColumn component
- Grid component tests
- GridColumn component tests

## [0.24.0] - 2019-09-20

### Added

- Dependencies updated

## [0.25.0] - 2019-09-20

### Added

- Card component
- CartTitle component
- CardImage component
- CardContent component
- CardFooter component
- Title component
- Paragraph component

- Card component tests
- CartTitle component tests
- CardImage component tests
- CardContent component tests
- CardFooter component tests
- Title component tests
- Paragraph component tests

## [0.25.1] - 2019-09-27

### Fixed

- Rewriting getFloaterAbsolutePosition function to return the right element position and get rid of the previous bug.

## [0.25.2] - 2019-09-30

### Fixed

- className prop support added to CardTitle, CardContent and CardFooter.

## [0.25.3] - 2019-10-03

### Fixed

<<<<<<< HEAD
- margin-left layout glitch fixed on Input component.

## [0.25.4] - 2019-10-04

### Fixed

- Adding distribution ready files that were supposed to be in the previous release

## [0.26.0] - 2019-10-05

### Added

- Adding Bit-bucket pipelines support

## [0.26.1] - 2019-10-05

### Fixed

- Bit-bucket pipeline now runs only on branch `master`

## [0.26.2] - 2019-10-05

### Fixed

- Bit-bucket pipeline configuration fixed

## [0.26.3] - 2019-10-05

### Fixed

- Some other minor fixes on the bit-bucket pipeline configuration

## [0.27.0] - 2019-10-06

### Added

- Avatar supports further information props such as `displayName` and `furtherInfo`

## [0.27.1] - 2019-10-06

### Fixed

- Pipeline script fix: push is not part of the script, not the after-script and removed personal git information

## [0.27.2] - 2019-10-06

### Fixed

- Webpack `Hot module reloading` is not perfectly working with styleguidist, so I've changed its config to
disable `Hot` and enable `live reloading` (it refresh when something changes)

## [0.27.3] - 2019-10-06

### Fixed

- Card component layout glitches

## [0.27.4] - 2019-10-06

### Fixed

- Card component background is now set to white
- Input font-size and color set as the others
- Removed pipelines
- Outlined Alert has transparent background

## [0.27.5] - 2019-10-06

### Fixed

- Grid.Column accepts className

## [0.27.6] - 2019-10-10

### Added

- Additional information on how to contribute to this project and how to 
  compact the history of a specific branch


## [0.27.7] - 2019-10-10

### Fixed

- Modal footer: replace row-reverse with justify-content to avoid objects reversing into the footer.
- Modal body: replace padding-y with padding to better show elements into it.
- Modal component: changed max-width in width

## [0.27.8] - 2019-10-10

### Fixed

- Added min-width to the Card component image when horizontally reversed
- Removed margins and add padding to all side of card's content whene there's not image and title.

## [0.27.9] - 2019-10-06

### Fixed

- Select Trigger element does not contain help-text anymore so that the dropdown will not go over the it.

## [0.27.10] - 2019-10-10

### Fixed

- Adding built files 


## [0.27.11] - 2019-10-19

### Fixed

- Card component hover effect is now actionable by the `float` prop
- Card component useless border-radius style on images
- Card component `horizontal` prop changed to `orientation`

### Added

- More renderer to the Card component 

## [0.28.0] - 2019-10-19

### Added

- Title Component
- Title Component tests

## [0.29.0] - 2019-10-19

### Added

- Checkbox component
- Checkbox component tests

## [0.29.1] - 2019-10-20

### Added

- Adding `React.memo` where missing 

### Fixed

- Missing build
- Card's icon propType warning

## [0.29.2] - 2019-10-20

### Added

- Title `size` prop
- Paragraph `fontFamily` prop

## [0.29.3] - 2019-10-20

### Added

- Typescript support, first setup, first types

## [0.29.4] - 2019-10-21

### Fixed

- Breadcrumb renamed to Breadcrumbs (with a final 's')

### Added

- Breadcrumb types
- Title types
- Paragraph types
- Spinner types

## [0.29.5] - 2019-10-21

### Fixed

- Pill defaultProps 

### Added

- Pill types

## [0.30.0] - 2019-11-01

### Added

- DisplayField component 
- DisplayField tests 
- DisplayField types 

## [0.31.0] - 2019-11-01

### Added

- Label component has been made a public component
- Label types
- Label documentation

## [0.32.0] - 2019-11-01

### Added

- FormGroup component
- FormGroup component tests
- FormGroup component types

## [0.33.0] - 2019-11-01

### Added

- Introducing node scripts: `postinstall.js`

## [0.33.1] - 2019-11-01

### Fixed

- `postinstall.js` should've been part of the distribution package 

## [0.33.2] - 2019-11-02

### Fixed

- Missing component types `Grid`, `Select`, `Checkbox`, `Popover` and `Placeholder` have been added
- Shared type `DefaultProps` has been added
- `Button` prop `pill` has been fixed

## [0.33.3] - 2019-11-02

### Fixed

- Adding missing types from the previous version to the `index.d.ts` file
- Adding missing type `Input`

## [0.33.4] - 2019-11-02

### Fixed

- Grid props issue with Column sub-component fixed

## [0.33.5] - 2019-11-02

### Fixed

- DisplayField style fix

## [0.33.6] - 2019-11-03

### Fixed

- DisplayField supports bold style for value and makes it optional for labels
- DisplayField supports React node values, not only strings
- Removing the postinstall.js script as the NodesCanvas community strongly discourage having ads in the console
- Fixed `tdd` script

## [0.33.7] - 2019-11-03

### Fixed

- Input css fixes such as icon position, max-width, fluid
- Select css fixes
- Button component's `block` prop renamed to `fluid`

### Added

- Support to `fluid` layout for Select component.

## [0.34.0] - 2019-11-04

### Added

- Divider component
- Divider component types
- Divider component tests

## [0.34.1] - 2019-11-04

### Fixed

- Divider component required types

## [0.34.2] - 2019-11-06

### Fixed

- Accordion missing types

## [0.34.3] - 2019-11-07

### Fixed

- CSS build optimization
- Tailwind fonts

## [0.34.4] - 2019-11-07

### Fixed

- Icon now uses a FontAwesome fixed width component
- Button icon only size
- DisplayField default label style is normal
- Paragraph breakStyle prop
- Title breakStyle prop

## [0.34.5] - 2019-11-07

### Fixed

- Accordion general glitches
- Removed Accodion icons, added caret

## [0.35.0] - 2019-11-08

### Added

- List component
- List component tests
- List component types
- `tiny` prop to Paragraph component
- `light` prop to Paragraph component


## [0.35.1] - 2019-11-15

### Added

- Input component box-sizing changed from 'border-box' to 'content-box'   

## [0.36.0] - 2019-11-19

### Added

- Breadcrumbs maxDisplayedItems prop

## [0.37.0] - 2019-11-19

### Added

- Link component
- Link tests
- Link types

### Fixed

- `types` directory renamed to `@types` as per the TS convention

## [0.38.0] - 2019-11-19

### Added

- Export for `useWindowResize` hook
- Hooks documentation page

## [0.39.0] - 2019-11-20

### Added

- DropDown component
- DropDown tests
- DropDown types

## [0.40.0] - 2019-11-25

### Added

- New build system based on `gulp`

## [0.41.0] - 2019-11-27

### Added

- TextArea component
- TextArea tests
- TextArea types

## [0.41.1] - 2019-11-28

### Fixed

- Input component minor style glitches
- TextArea component fluid style glitch

## [0.41.2] - 2019-11-28

### Fixed

- Divider line prop documentation

## [0.42.0] - 2019-12-02

### Added

- new `clearable` prop to the Select component
- new `clearable` prop in Select types
- Test for the `clearable` prop

## [0.42.1] - 2019-12-02

### Fixed

- Modal body padding

## [0.43.0] - 2019-11-29

### Added

- Diagram component
- useDrag hook
- useWindowScroll hook

## [0.43.1] - 2019-12-12

### Fixed

- Accordion now checks for null children before warning about unsupported children type
- Better Diagram types definition

## [0.43.2] - 2019-12-12

### Fixed

- It turned out that types from the previous commit should've been exported and not only defined, what a world we are living in.

## [0.43.3] - 2019-12-12

### Fixed

- Port checks if the target is not itself before dispatching a new link creation


## [0.43.4] - 2019-12-12

### Fixed

- Fixing Diagram canvas absolute position to avoid positioning glitches

## [0.44.0] - 2019-12-12

### Add

- A `data` property support to Diagram nodes to possibly keep data between renders


## [0.44.1] - 2019-12-12

### Fixed

- Render function working again in Diagram nodes

## [0.44.2] - 2019-12-23

### Fixed

- Button doesn't ignore anymore pills dimension

## [0.45.0] - 2020-01-10

### Added

- Better documentation
- Preparing the repository for migrating to github
- Adding CI

## [0.45.1] - 2020-01-13

### Fixed

- Paragraph and Title component now have tests for id and style props

## [0.45.2] - 2020-01-14

### Fixed

- glitch on outlined buttons on Safari

## [0.45.3] - 2020-01-14

### Fixed

- ButtonGroup glitch on Safari

## [0.45.4] - 2020-01-14

### Fixed

- issue relative to Input and Textarea overflow component

## [0.46.0] - 2020-01-14

### Added

- Improving documentation

## [0.47.0] - 2020-01-14

### Added

- Renaming library
- Preparing for NPM
- Removing husky since the lib will be built by the CI

## [0.47.1] - 2020-01-14

### Fixed

- Fixing build system
- Peer dependencies

## [0.48.0] - 2020-01-18

### Added

- Sidebar component
- Tooltip component
- Improving documentation (styleguidist hacks)
- test utility: checkColorProp
- test utility: hasDefaultClassNames
- test utility: performStandardTests

### Fixed

- Link component font fix
- Added react and react-dom to the dev dependencies 
- improving tests
- removed the old typescript codebase (FINALLY!)

## [0.48.1] - 2020-01-24

### Fixed

- Sidebar scrolling issue

## [0.48.2] - 2020-01-25

### Fixed

- Sidebar header image issue

## [0.48.3] - 2020-01-30

### Fixed

- Improving documentation for Alert, Icon, Popover, Tooltip and FloatingContent

## [0.48.4] - 2020-01-31

### Fixed

- improved getFloaterAbsolutePosition and its tests 
- Label component text style

## [0.48.5] - 2020-01-31

### Fixed

- Adding src directory to the distribution package
- Removing lodash/noop from dependencies

## [0.48.6] - 2020-02-04

### Fixed

- Adding `transitionType` to Sidebar component
- Improving Sidebar component documentation & types

## [0.48.7] - 2020-02-04

### Fixed

- Fixing types of Sidebar.Item

## [0.48.8] - 2020-02-04

### Fixed

- Sidebar.Collapsible initial open status fix

## [0.48.9] - 2020-02-04

### Fixed

- Button and Avatar alignment changed from `bottom` to `middle`

## [0.49.0] - 2020-02-06

### Added

- FormPanel component
- Sidebar.Group component

### Fixed

- Input component onChange prop shan't be required
- Checkbox component onChange prop shan't be required
- React.memo added where were missing

## [0.50.0] - 2020-02-06

### Added

- ToggleSwitch component
- Dependencies updated

## [0.51.0] - 2020-02-07

### Added

- Improved Checkbox
- Improved SwitchToggle

## [0.52.0] - 2020-02-06

### Added

- More renderers and test coverage

## [0.53.0] - 2020-02-06

### Added

- Renderers to the Accordion component
- Types for the Avatar component
- Renderers to the Paragraph component
- Renderers to the Link component

## [0.53.1] - 2020-02-12

### Fixing

- Sidebar types
- Adding more renders to Sidebar component
- Improving documentation

## [0.53.2] - 2020-02-12

### Fixing

- Avoiding helpText css override

## [0.53.3] - 2020-02-20

### Fixing

- adding babel lodash plugin to improve build size

## [0.54.0] - 2020-02-21

### Adding

- first attempt of lazy exporting

### Fixing

- Icon component asynchronously load the font-awesome library (first try)
- Sidebar full height glitch

## [0.54.1] - 2020-02-26

### Fixing

- Added ie support to the FloatingComponent by editing its util functions

## [0.55.0] - 2020-03-19

### Added

- NotificationsStack component
- NotificationsStack tests

## [0.56.0] - 2020-03-22

### Added

- FileUploader component
- FileUploader tests
- ProgressBar component
- ProgressBar tests

### Fixed

- Input fields value and onChange are not required anymore

## [0.56.1] - 2020-03-23

### Fixed

- Card component types
- Alert component type
- Avatar component type
- Image component type

## [0.56.2] - 2020-03-24

### Fixed

- Alert component type required onClose prop
- Missing FileUploader types in index.d.ts
- Missing ButtonGroup types
- TextArea null default value

## [0.56.3] - 2020-03-24

### Fixed

- FileUploader icon required value

## [0.56.4] - 2020-03-24

### Fixed

- ToggleSwitch conditional useCallback

## [0.56.5] - 2020-03-27

### Fixed

- Added a Element.remove polyfill to make _Portal component work on IE10+ (stupid browser btw)

## [0.56.6] - 2020-03-27

### Fixed

- ToggleSwitch CSS fix (top: 0 was missing)

## [0.56.7] - 2020-03-27

### Fixed

- Adding z-index:9999 to the #bi-floats element in order show floaters upon modals

## [0.56.8] - 2020-03-27

### Fixed

- Import Element.remove polyfill into _Portal component

## [0.56.9] - 2020-03-27

### Fixed

- Improve Element.remove polyfill

## [0.56.10] - 2020-03-26

### Fixed

- FloatingContent change placement if there's no enough space to show.
- Adding FloatingContent types

## [0.56.11] - 2020-03-30

### Fixed

- Icon component now has box-sizing:content-box

## [0.56.12] - 2020-03-30

### Fixed

- Improving `checkAvailableSpace` function 

## [0.56.13] - 2020-03-30

### Fixed

- Better calculation for `checkAvailableSpace` function 

## [0.56.14] - 2020-05-15

### Fixed

- Render loop in FloatingContent caused by a recursive position calculation
- drop-down/select component's pointer weird CSS glitch

## [0.57.0] - 2020-12-03

### Added

- Option to keep multiple accordion panels open at the same time

### Fixed

- Moved `react` and `react-dom` from `dependencies` to `peerDependencies`

## [0.57.1] - 2020-12-10


### Fixed

- Dropdown position calculations
