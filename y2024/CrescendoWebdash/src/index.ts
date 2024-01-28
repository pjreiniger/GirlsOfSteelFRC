import './super-structure';
import { addElements, addThemeRules } from '@frc-web-components/app';
import {elementConfig as SuperStructureElements} from "./super-structure-config";

addElements({
  'super-structure': SuperStructureElements
  }, 'Girls of Steel');

addThemeRules('dark', {
  '--super-structure-background': 'cadetblue',
  '--super-structure-color': 'black',
});

addThemeRules('light', {
  '--super-structure-background': 'cornflowerblue',
  '--super-structure-color': 'white',
});