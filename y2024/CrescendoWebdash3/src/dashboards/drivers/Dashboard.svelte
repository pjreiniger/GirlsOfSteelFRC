<script lang="ts">
  import "@frc-web-components/fwc/components";

  import Tabs from '../../components/Tabs.svelte';

  import DriverTab   from '../../tabs/DriverTab.svelte';
  import PitTab      from '../../tabs/PitTab.svelte';
  import ArmPivotTab from '../../tabs/subsystems/ArmPivotSubsystemTab.svelte';

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