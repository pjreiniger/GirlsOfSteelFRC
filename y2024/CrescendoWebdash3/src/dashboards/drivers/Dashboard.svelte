<script lang="ts">
  import "@frc-web-components/fwc/components";

  import DriverTab from '../../components/DriverTab.svelte';

  import PitTab from '../../components/PitTab.svelte';
  import ArmPivotTab from '../../components/subsystems/ArmPivotTab.svelte';

  import Tabs from '../../components/Tabs.svelte';


  // List of tab items with labels, values and assigned components
  let items = [
    { label: "Drivers",
      value: 1,
      component: DriverTab
    },
    { label: "Pit",
      value: 2,
      component: PitTab
    },
  ];

</script>


<Tabs {items} />