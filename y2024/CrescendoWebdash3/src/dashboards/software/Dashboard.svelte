<script lang="ts">
  import "@frc-web-components/fwc/components";

  import DriverTab from '../../components/DriverTab.svelte';

  import PitTab from '../../components/PitTab.svelte';
  import ArmPivotTab from '../../components/subsystems/ArmPivotSubsystemTab.svelte';
  import ChassisTab from '../../components/subsystems/ChassisSubsystemTab.svelte';
  import IntakeTab from '../../components/subsystems/IntakeSubsystemTab.svelte';
  import ShooterTab from '../../components/subsystems/ShooterSubsystemTab.svelte';

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
    { label: "Chassis",
      value: 3,
      component: ChassisTab
    },
    { label: "Arm Pivot",
      value: 4,
      component: ArmPivotTab
    },
    { label: "Intake",
      value: 5,
      component: IntakeTab
    },
    { label: "Shooter",
      value: 6,
      component: ShooterTab
    },
  ];

</script>


<Tabs {items} />