document.addEventListener("DOMContentLoaded", () => {
  // --- 1. DOM Elements ---
  const armBtn = document.getElementById("armBtn");
  const rebootBtn = document.getElementById("rebootBtn");
  const estopBtn = document.getElementById("estopBtn");
  const statusIndicator = document.getElementById("statusIndicator");

  const pitchVal = document.getElementById("pitchVal");
  const velVal = document.getElementById("velVal");
  const errorVal = document.getElementById("errorVal");
  const fDistVal = document.getElementById("fDistVal");
  const latencyVal = document.getElementById("latencyVal");

  const playStopBtn = document.getElementById("playStopBtn");
  const playStopText = document.getElementById("playStopText");

  // --- 2. WebSocket Setup ---
  const gateway = `ws://${window.location.hostname}/ws`;
  let websocket = null;
  let reconnectTimer = null;
  let lastMessageTime = 0;

  function setStatus(state) {
    if (!statusIndicator) return;

    if (state === "online") {
      statusIndicator.className = "indicator --online";
    } else if (state === "connecting") {
      statusIndicator.className = "indicator --warning";
    } else {
      statusIndicator.className = "indicator --error";
    }
  }

  function safeSetText(element, text) {
    if (element) element.textContent = text;
  }

  function sendCommand(command) {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) {
      console.warn(`WebSocket not open. Could not send command: ${command}`);
      return;
    }

    websocket.send(JSON.stringify({ command }));
  }

  function initWebSocket() {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
      return;
    }

    console.log("Attempting WebSocket connection...");
    setStatus("connecting");

    websocket = new WebSocket(gateway);

    websocket.onopen = () => {
      console.log("WebSocket connected");
      setStatus("online");
    };

    websocket.onclose = () => {
      console.log("WebSocket disconnected");
      setStatus("offline");

      if (!reconnectTimer) {
        reconnectTimer = setTimeout(() => {
          reconnectTimer = null;
          initWebSocket();
        }, 2000);
      }
    };

    websocket.onerror = (err) => {
      console.error("WebSocket error:", err);
    };

    websocket.onmessage = (event) => {
      const now = performance.now();

      // This is message interval, not true network latency
      if (latencyVal && lastMessageTime !== 0) {
        latencyVal.textContent = `${(now - lastMessageTime).toFixed(1)} ms`;
      }
      lastMessageTime = now;

      try {
        const data = JSON.parse(event.data);

        if (typeof data.pitch === "number") {
          pitchVal.innerHTML = `${data.pitch.toFixed(2)}&deg;`;
        }

        if (typeof data.angular_velocity === "number") {
          // STM32 sends dps_y, so unit should be deg/s
          safeSetText(velVal, `${data.angular_velocity.toFixed(2)} deg/s`);
        }

        if (typeof data.setpoint_error === "number") {
          errorVal.innerHTML = `${data.setpoint_error.toFixed(2)}&deg;`;
        }

        if (typeof data.front_distance === "number") {
          safeSetText(fDistVal, `${data.front_distance.toFixed(2)} m`);
        }
      } catch (e) {
        console.error("Error parsing WebSocket JSON:", e, event.data);
      }
    };
  }

  // --- 3. Button Logic ---
  if (armBtn) {
    armBtn.addEventListener("click", () => {
      const isArmed = armBtn.classList.contains("is-armed");

      if (isArmed) {
        armBtn.classList.remove("is-armed");
        armBtn.innerText = "ARM MOTORS";
        sendCommand("disarm");
      } else {
        armBtn.classList.add("is-armed");
        armBtn.innerText = "DISARM SYSTEM";
        sendCommand("arm");
      }
    });
  }

  if (estopBtn) {
    estopBtn.addEventListener("click", () => {
      sendCommand("estop");

      if (armBtn) {
        armBtn.classList.remove("is-armed");
        armBtn.innerText = "ARM MOTORS";
      }

      alert("!! EMERGENCY STOP TRIGGERED !!");
    });
  }

  if (rebootBtn) {
    rebootBtn.addEventListener("click", () => {
      sendCommand("reboot");
      rebootBtn.innerText = "INITIALIZING...";
      rebootBtn.style.pointerEvents = "none";

      setTimeout(() => {
        rebootBtn.innerText = "REBOOT SYSTEM";
        rebootBtn.style.pointerEvents = "auto";
      }, 5000);
    });
  }

  if (playStopBtn && playStopText) {
    playStopBtn.addEventListener("click", () => {
      const isRecording = playStopBtn.classList.toggle("is-recording");

      if (isRecording) {
        playStopText.innerText = "STOP RECORDING";
        console.log("Recording started...");
      } else {
        playStopText.innerText = "START RECORDING";
        console.log("Recording stopped and saved.");
      }
    });
  }

  // --- 4. Start connection ---
  initWebSocket();
});
