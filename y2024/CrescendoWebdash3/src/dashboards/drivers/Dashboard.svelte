<script lang="ts">
  import "@frc-web-components/fwc/components";
  import { getEntry } from "@frc-web-components/svelte";

import DriverTab from '../../components/DriverTab.svelte';

import PitTab from '../../components/PitTab.svelte';
import ArmPivotTab from '../../components/ArmPivotTab.svelte';

import Tabs from '../../components/Tabs.svelte';


// List of tab items with labels, values and assigned components
let items = [
  { label: "Drivers",
		 value: 1,
		 component: DriverTab
		},
  { label: "Drivers2",
		 value: 2,
		 component: PitTab
		},
  { label: "Driver3",
		 value: 3,
		 component: ArmPivotTab
		},
];

</script>


<Tabs {items} />