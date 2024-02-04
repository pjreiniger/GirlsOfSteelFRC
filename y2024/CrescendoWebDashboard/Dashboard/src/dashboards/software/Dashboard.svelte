<script lang="ts">
  import "@frc-web-components/fwc/components";

  import Tabs from '../../components/Tabs.svelte';

  import DriverTab      from '../../tabs/DriverTab.svelte';
  import PitTab         from '../../tabs/PitTab.svelte';
  import ProgrammersTab from '../../tabs/ProgrammersTab.svelte';
  import ArmPivotTab    from '../../tabs/subsystems/ArmPivotSubsystemTab.svelte';
  import ChassisTab     from '../../tabs/subsystems/ChassisSubsystemTab.svelte';
  import IntakeTab      from '../../tabs/subsystems/IntakeSubsystemTab.svelte';
  import ShooterTab     from '../../tabs/subsystems/ShooterSubsystemTab.svelte';


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
    { label: "Programmers",
      value: 3,
      component: ProgrammersTab
    },
    { label: "Chassis",
      value: 4,
      component: ChassisTab
    },
    { label: "Arm Pivot",
      value: 5,
      component: ArmPivotTab
    },
    { label: "Intake",
      value: 6,
      component: IntakeTab
    },
    { label: "Shooter",
      value: 7,
      component: ShooterTab
    },
  ];

</script>


<Tabs {items} />