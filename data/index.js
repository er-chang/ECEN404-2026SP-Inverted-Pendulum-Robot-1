document.addEventListener("DOMContentLoaded", () => {
  // --- 1. DOM Elements ---
  const armBtn = document.getElementById("armBtn");
  const rebootBtn = document.getElementById("rebootBtn");
  const estopBtn = document.getElementById("estopBtn");
  const statusIndicator = document.getElementById("statusIndicator");

  // Data Elements
  const pitchVal = document.getElementById("pitchVal");
  const velVal = document.getElementById("velVal");
  const errorVal = document.getElementById("errorVal");
  const fDistVal = document.getElementById("fDistVal");
  const latencyVal = document.getElementById("latencyVal");

  // --- 2. WebSocket Setup ---
  // Connects to the ESP32 IP address automatically
  let gateway = `ws://${window.location.hostname}/ws`;
  let websocket;
  let lastPingTime = 0;

  function initWebSocket() {
    console.log("Attempting WebSocket Connection...");
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
  }

  function onOpen(event) {
    console.log("Connection opened");
    statusIndicator.className = "indicator --online";
  }

  function onClose(event) {
    console.log("Connection closed");
    statusIndicator.className = "indicator --error";
    setTimeout(initWebSocket, 2000); // Try to reconnect every 2 seconds
  }

  // --- 3. Handle Incoming Data ---
  function onMessage(event) {
    // Calculate Wi-Fi Latency based on message arrival
    const now = performance.now();
    if (lastPingTime !== 0) {
      latencyVal.innerText = (now - lastPingTime).toFixed(1) + " ms";
    }
    lastPingTime = now;

    try {
      // Parse the incoming JSON from the ESP32
      const data = JSON.parse(event.data);

      // Update the specific requested values
      if (data.pitch !== undefined)
        pitchVal.innerHTML = ` ${data.pitch.toFixed(2)}&deg;`;
      if (data.angular_velocity !== undefined)
        velVal.innerText = ` ${data.angular_velocity.toFixed(2)} rad/s`;
      if (data.setpoint_error !== undefined)
        errorVal.innerHTML = ` ${data.setpoint_error.toFixed(2)}&deg;`;
      if (data.front_distance !== undefined)
        fDistVal.innerText = ` ${data.front_distance.toFixed(2)} m`;
    } catch (e) {
      console.log("Error parsing data: ", e);
    }
  }

  // --- 4. Button Logic ---

  // Arm / Disarm Toggle
  armBtn.addEventListener("click", () => {
    if (armBtn.classList.contains("is-armed")) {
      armBtn.classList.remove("is-armed");
      armBtn.innerText = "ARM MOTORS";
      websocket.send(JSON.stringify({ command: "disarm" }));
    } else {
      armBtn.classList.add("is-armed");
      armBtn.innerText = "DISARM SYSTEM";
      websocket.send(JSON.stringify({ command: "arm" }));
    }
  });

  // Emergency Stop
  estopBtn.addEventListener("click", () => {
    websocket.send(JSON.stringify({ command: "estop" }));
    // Force disarm state on the UI
    armBtn.classList.remove("is-armed");
    armBtn.innerText = "ARM MOTORS";
    alert("!! EMERGENCY STOP TRIGGERED !!");
  });

  // Reboot STM32
  rebootBtn.addEventListener("click", () => {
    websocket.send(JSON.stringify({ command: "reboot" }));
    rebootBtn.innerText = "INITIALIZING...";
    rebootBtn.style.pointerEvents = "none"; // Disable clicks

    // Reset button after 5 seconds
    setTimeout(() => {
      rebootBtn.innerText = "REBOOT STM32";
      rebootBtn.style.pointerEvents = "auto";
    }, 5000);
  });

  // Start the connection
  initWebSocket();
});
