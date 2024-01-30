<script lang="ts">
  import { setNt4Context, getNt4Context } from "@frc-web-components/svelte";
  import Dashboard from "./Dashboard.svelte";

  // Set robot IP address here
  setNt4Context('localhost');
</script>

<div class="App">
  <Dashboard />
</div>
