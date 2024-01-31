<script lang="ts">
  import { setNt4Context, getNt4Context } from "@frc-web-components/svelte";
  import ThemeChooser from "../../components/ThemeChooser.svelte";
  import Dashboard from "./Dashboard.svelte";

  // Set robot IP address here
  setNt4Context('localhost');
  let pair = getNt4Context();
  console.log(pair);
  let provider = pair.provider;
  let client = provider.clients.localhost;
  //client.clearAllSubscriptions();

  client.subscribeAll(['/SmartDashboard/Field/'], true);
</script>

<div class="App">
  <ThemeChooser />
  <Dashboard />
</div>
