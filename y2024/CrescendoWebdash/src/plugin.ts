import { FrcDashboard } from '@frc-web-components/fwc';
import './super-structure';
import {elementConfig as SuperStructureElements} from "./super-structure";

export default function addPlugin(dashboard: FrcDashboard) {
  dashboard.addElements({
    "super-structure": SuperStructureElements,
  } as any, "Girls of Steel");
}
