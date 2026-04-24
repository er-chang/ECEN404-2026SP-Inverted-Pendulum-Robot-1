document.addEventListener("DOMContentLoaded", () => {
  const trimFwdBtn = document.getElementById("trimFwdBtn");
  const trimBwdBtn = document.getElementById("trimBwdBtn");
  const estopBtn = document.getElementById("estopBtn");
  const statusIndicator = document.getElementById("statusIndicator");

  const pitchVal = document.getElementById("pitchVal");
  const velVal = document.getElementById("velVal");
  const errorVal = document.getElementById("errorVal");
  const fDistVal = document.getElementById("fDistVal");
  const latencyVal = document.getElementById("latencyVal");

  const playStopBtn = document.getElementById("playStopBtn");
  const playStopText = document.getElementById("playStopText");
  const webcamFeed = document.getElementById("webcamFeed");

  const sendTuneBtn = document.getElementById("sendTuneBtn");
  const kpInput = document.getElementById("kpInput");
  const kiInput = document.getElementById("kiInput");
  const kdInput = document.getElementById("kdInput");

  const armMotorsBtn = document.getElementById("armMotorsBtn");

  const pitchChart = document.getElementById("pitchChart");
  const chartCtx = pitchChart ? pitchChart.getContext("2d") : null;
  const maxDataPoints = 150;
  let pitchHistory = new Array(maxDataPoints).fill(0);

  let isLoggingData = false;
  let telemetryLog = [];
  let recordingStartTime = 0;

  const rmsWindowSize = 10;
  let squaredErrors = [];
  let initialPitchOffset = null;

  const gateway = `ws://${window.location.hostname}/ws`;
  let websocket = null;
  let reconnectTimer = null;
  let lastMessageTime = 0;

  const consoleOutput = document.getElementById("consoleOutput");

  function logToConsole(msg, type = "info") {
    if (!consoleOutput) return;

    const div = document.createElement("div");
    div.className = `console-msg console-${type}`;

    // Create a timestamp (e.g., 14:05:22.123)
    const now = new Date();
    const timeStr =
      now.toTimeString().split(" ")[0] +
      "." +
      String(now.getMilliseconds()).padStart(3, "0");

    div.innerHTML = `<span class="console-time">[${timeStr}]</span> ${msg}`;
    consoleOutput.appendChild(div);

    // Auto-scroll to the bottom
    consoleOutput.scrollTop = consoleOutput.scrollHeight;
  }

  function setStatus(state) {
    if (!statusIndicator) return;
    if (state === "online") statusIndicator.className = "indicator --online";
    else if (state === "connecting")
      statusIndicator.className = "indicator --warning";
    else statusIndicator.className = "indicator --error";
  }

  function safeSetText(element, text) {
    if (element) element.textContent = text;
  }

  function sendCommand(command) {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) return;
    websocket.send(JSON.stringify({ command }));
  }

  // --- Mode Switching Function ---
  window.setMode = function (modeValue) {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
      websocket.send(
        JSON.stringify({
          command: "set_mode",
          mode: modeValue,
        }),
      );
      logToConsole("Switched mode: " + modeValue, "sys");
    }
  };

  function initWebSocket() {
    if (websocket && websocket.readyState === WebSocket.OPEN) return;
    setStatus("connecting");
    websocket = new WebSocket(gateway);

    websocket.onopen = () => {
      logToConsole(
        "[NETWORK] Connection established. Telemetry stream started.",
      );

      setStatus("online");
      initialPitchOffset = null;
    };

    websocket.onclose = () => {
      logToConsole("[NETWORK] Connection lost. Attempting to reconnect...");
      setStatus("offline");
      if (!reconnectTimer) {
        reconnectTimer = setTimeout(() => {
          reconnectTimer = null;
          initWebSocket();
        }, 2000);
      }
    };

    websocket.onmessage = (event) => {
      const now = performance.now();
      if (latencyVal && lastMessageTime !== 0) {
        latencyVal.textContent = `${(now - lastMessageTime).toFixed(1)} ms`;
      }
      lastMessageTime = now;

      try {
        const data = JSON.parse(event.data);

        initialPitchOffset = 0;

        const displayPitch = data.pitch - (initialPitchOffset || 0);
        const displayRawPitch =
          (data.pitch_raw || 0) - (initialPitchOffset || 0);

        let currentRmsError = 0.0;
        if (
          typeof data.pitch === "number" &&
          typeof data.pitch_raw === "number"
        ) {
          let instError = data.pitch_raw - data.pitch;
          squaredErrors.push(instError * instError);
          if (squaredErrors.length > rmsWindowSize) squaredErrors.shift();
          let sumSq = squaredErrors.reduce((a, b) => a + b, 0);
          currentRmsError = Math.sqrt(sumSq / squaredErrors.length);

          if (pitchVal) pitchVal.innerHTML = `${displayPitch.toFixed(2)}&deg;`;
          pitchHistory.push(displayPitch);
          pitchHistory.shift();
          requestAnimationFrame(drawSparkline);
        }

        if (typeof data.angular_velocity === "number")
          safeSetText(velVal, `${data.angular_velocity.toFixed(2)} deg/s`);
        if (errorVal) errorVal.innerHTML = `${currentRmsError.toFixed(3)}&deg;`;
        if (document.getElementById("cpuVal")) {
          document.getElementById("cpuVal").innerText =
            typeof data.cpu === "number"
              ? " " + data.cpu.toFixed(1) + "%"
              : " --%";
        }
        if (document.getElementById("val-dist-f"))
          document.getElementById("val-dist-f").innerText =
            typeof data.dist_f === "number"
              ? data.dist_f.toFixed(1) + " cm"
              : "0.0 cm";
        if (document.getElementById("val-dist-b"))
          document.getElementById("val-dist-b").innerText =
            typeof data.dist_b === "number"
              ? data.dist_b.toFixed(1) + " cm"
              : "0.0 cm";
        if (document.getElementById("val-dist-l"))
          document.getElementById("val-dist-l").innerText =
            typeof data.dist_l === "number"
              ? data.dist_l.toFixed(1) + " cm"
              : "0.0 cm";
        if (document.getElementById("val-dist-r"))
          document.getElementById("val-dist-r").innerText =
            typeof data.dist_r === "number"
              ? data.dist_r.toFixed(1) + " cm"
              : "0.0 cm";
        // if (document.getElementById("cpuVal")) {
        //   document.getElementById("cpuVal").innerText =
        //     typeof data.cpu === "number"
        //       ? " " + data.cpu.toFixed(1) + "%"
        //       : " --%";
        // }
        if (isLoggingData && typeof data.pitch === "number") {
          const elapsedMs = (performance.now() - recordingStartTime).toFixed(0);
          telemetryLog.push([
            elapsedMs,
            displayRawPitch.toFixed(2),
            displayPitch.toFixed(2),
            typeof data.angular_velocity === "number"
              ? data.angular_velocity.toFixed(2)
              : "0.00",
            typeof data.dist_f === "number" ? data.dist_f.toFixed(1) : "0.0",
            typeof data.dist_b === "number" ? data.dist_b.toFixed(1) : "0.0",
            typeof data.dist_l === "number" ? data.dist_l.toFixed(1) : "0.0",
            typeof data.dist_r === "number" ? data.dist_r.toFixed(1) : "0.0",
            typeof data.raw_pot === "number" ? data.raw_pot.toFixed(0) : "0",
            typeof data.pot_ohms === "number"
              ? data.pot_ohms.toFixed(1)
              : "0.0",
            typeof data.pwm_fl === "number" ? data.pwm_fl.toFixed(0) : "0",
            typeof data.pwm_fr === "number" ? data.pwm_fr.toFixed(0) : "0",
            typeof data.pwm_bl === "number" ? data.pwm_bl.toFixed(0) : "0",
            typeof data.pwm_br === "number" ? data.pwm_br.toFixed(0) : "0",
          ]);
        }
        // accounts for all wheels
        if (document.getElementById("val-pwm-fl")) {
          document.getElementById("val-pwm-fl").innerText =
            typeof data.pwm_fl === "number" ? data.pwm_fl.toFixed(0) : "0";
        }
        if (document.getElementById("val-pwm-fr")) {
          document.getElementById("val-pwm-fr").innerText =
            typeof data.pwm_fr === "number" ? data.pwm_fr.toFixed(0) : "0";
        }
        if (document.getElementById("val-pwm-bl")) {
          document.getElementById("val-pwm-bl").innerText =
            typeof data.pwm_bl === "number" ? data.pwm_bl.toFixed(0) : "0";
        }
        if (document.getElementById("val-pwm-br")) {
          document.getElementById("val-pwm-br").innerText =
            typeof data.pwm_br === "number" ? data.pwm_br.toFixed(0) : "0";
        }
      } catch (e) {
        console.error("Error parsing WebSocket JSON:", e);
      }
    };
  }

  function drawSparkline() {
    if (!chartCtx || !pitchChart) return;
    pitchChart.width = pitchChart.clientWidth;
    pitchChart.height = pitchChart.clientHeight;
    const width = pitchChart.width,
      height = pitchChart.height;
    chartCtx.clearRect(0, 0, width, height);
    const maxPitch = 90,
      minPitch = -90,
      range = maxPitch - minPitch;
    const zeroY = height - ((0 - minPitch) / range) * height;

    chartCtx.strokeStyle = "rgba(255, 255, 255, 0.15)";
    chartCtx.lineWidth = 1;
    chartCtx.beginPath();
    chartCtx.moveTo(0, zeroY);
    chartCtx.lineTo(width, zeroY);
    chartCtx.stroke();

    chartCtx.strokeStyle = "#f6f09c";
    chartCtx.lineWidth = 2;
    chartCtx.beginPath();
    for (let i = 0; i < maxDataPoints; i++) {
      const x = (i / (maxDataPoints - 1)) * width;
      let val = pitchHistory[i];
      const y = height - ((val - minPitch) / range) * height;
      if (i === 0) chartCtx.moveTo(x, y);
      else chartCtx.lineTo(x, y);
    }
    chartCtx.stroke();
  }

  function saveTelemetryCSV() {
    if (telemetryLog.length <= 1) return;
    let csvContent = telemetryLog.map((e) => e.join(",")).join("\n");
    const blob = new Blob([csvContent], { type: "text/csv;charset=utf-8;" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `pendulum_data_${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  // --- UNIFIED ARM & RECORD LOGIC ---
  function toggleSystem(event) {
    // Sync both buttons visually
    const isRecording = playStopBtn.classList.toggle("is-recording");

    if (armMotorsBtn) {
      if (isRecording) {
        armMotorsBtn.classList.add("is-armed");
        armMotorsBtn.textContent = "DISARM MOTORS";
      } else {
        armMotorsBtn.classList.remove("is-armed");
        armMotorsBtn.textContent = "ARM MOTORS";
      }
    }

    if (isRecording) {
      // 1. ARM THE ROBOT
      sendCommand("arm");
      logToConsole("SYSTEM ARMED: begin data logging", "warn");
      // 2. START CSV LOGGING
      isLoggingData = true;
      recordingStartTime = performance.now();
      squaredErrors = [];
      telemetryLog = [
        [
          "Time (ms)",
          "Raw Pitch (deg)",
          "Filtered Pitch (deg)",
          "Velocity (deg/s)",
          "Front Dist (cm)",
          "Back Dist (cm)",
          "Left Dist (cm)",
          "Right Dist (cm)",
          "Raw Pot (ADC)",
          "Pot Resistance (Ohms)",
          "FL PWM",
          "FR PWM",
          "BL PWM",
          "BR PWM",
        ],
      ];
    } else {
      // 1. E-STOP THE ROBOT
      sendCommand("estop");
      logToConsole("SYSTEM DISARMED: data saved", "info");

      // 2. SAVE THE CSV FILE
      isLoggingData = false;
      if (event && event.currentTarget.id === "playStopBtn") {
        saveTelemetryCSV();
      }
    }
  }

  // Attach the same function to BOTH buttons
  if (playStopBtn) {
    playStopBtn.addEventListener("click", toggleSystem);
  }
  if (armMotorsBtn) {
    armMotorsBtn.addEventListener("click", toggleSystem);
  }

  // --- SEND TUNING LOGIC ---
  if (sendTuneBtn) {
    sendTuneBtn.addEventListener("click", () => {
      const kp_val = parseFloat(kpInput.value);
      const ki_val = parseFloat(kiInput.value);
      const kd_val = parseFloat(kdInput.value);
      if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(
          JSON.stringify({
            command: "tune",
            kp: kp_val,
            ki: ki_val,
            kd: kd_val,
          }),
        );
        logToConsole(
          `Tuning Sent - Kp: ${kp_val}, Ki: ${ki_val}, Kd: ${kd_val}`,
          "sys",
        );
      }
    });
  }

  // --- E-STOP BUTTON LOGIC ---
  if (estopBtn) {
    estopBtn.addEventListener("click", () => {
      sendCommand("estop");
      logToConsole("EMERGENCY STOP TRIGGERED", "err");

      // Ensure UI resets visually if E-Stop is hit
      if (playStopBtn) playStopBtn.classList.remove("is-recording");
      if (armMotorsBtn) {
        armMotorsBtn.classList.remove("is-armed");
        armMotorsBtn.textContent = "ARM MOTORS";
      }

      // Stop logging if active
      if (isLoggingData) {
        isLoggingData = false;
        // saveTelemetryCSV();
      }
    });
  }

  initWebSocket();
});
