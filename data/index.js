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

  const webcamFeed = document.getElementById("webcamFeed");

  // NEW: PID Tuning Elements
  const sendTuneBtn = document.getElementById("sendTuneBtn");
  const kpInput = document.getElementById("kpInput");
  const kiInput = document.getElementById("kiInput");
  const kdInput = document.getElementById("kdInput");

  // --- Sparkline Setup ---
  const pitchChart = document.getElementById("pitchChart");
  const chartCtx = pitchChart ? pitchChart.getContext("2d") : null;
  const maxDataPoints = 150;
  let pitchHistory = new Array(maxDataPoints).fill(0);

  // --- 2. Recording & Logging Variables ---
  let mediaRecorder;
  let recordedChunks = [];
  let isLoggingData = false;
  let telemetryLog = [];
  let recordingStartTime = 0;

  // --- 3. WebSocket Setup ---
  const gateway = `ws://192.168.4.1/ws`;
  let websocket = null;
  let reconnectTimer = null;
  let lastMessageTime = 0;

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

  function initWebSocket() {
    if (websocket && websocket.readyState === WebSocket.OPEN) return;

    setStatus("connecting");
    websocket = new WebSocket(gateway);

    websocket.onopen = () => setStatus("online");

    websocket.onclose = () => {
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

        // Update UI using PITCH_FILTERED
        if (typeof data.pitch_filtered === "number") {
          pitchVal.innerHTML = `${data.pitch_filtered.toFixed(2)}&deg;`;
          pitchHistory.push(data.pitch_filtered);
          pitchHistory.shift();
          requestAnimationFrame(drawSparkline);
        }
        if (typeof data.angular_velocity === "number") {
          safeSetText(velVal, `${data.angular_velocity.toFixed(2)} deg/s`);
        }
        if (typeof data.setpoint_error === "number") {
          errorVal.innerHTML = `${data.setpoint_error.toFixed(2)}&deg;`;
        }
        if (typeof data.front_distance === "number") {
          safeSetText(fDistVal, `${data.front_distance.toFixed(2)} m`);
        }

        // --- DATA LOGGING ---
        if (isLoggingData && typeof data.pitch_filtered === "number") {
          const elapsedMs = (performance.now() - recordingStartTime).toFixed(0);
          const vRaw =
            typeof data.pitch_raw === "number"
              ? data.pitch_raw.toFixed(2)
              : "0.00";
          const vFilt = data.pitch_filtered.toFixed(2);
          const vVel =
            typeof data.angular_velocity === "number"
              ? data.angular_velocity.toFixed(2)
              : "0.00";
          const vErr =
            typeof data.setpoint_error === "number"
              ? data.setpoint_error.toFixed(2)
              : "0.00";

          telemetryLog.push([elapsedMs, vRaw, vFilt, vVel, vErr]);
        }
      } catch (e) {
        console.error("Error parsing WebSocket JSON:", e, event.data);
      }
    };
  }

  // --- Canvas Drawing Function ---
  function drawSparkline() {
    if (!chartCtx || !pitchChart) return;

    pitchChart.width = pitchChart.clientWidth;
    pitchChart.height = pitchChart.clientHeight;
    const width = pitchChart.width;
    const height = pitchChart.height;

    chartCtx.clearRect(0, 0, width, height);

    const maxPitch = 20;
    const minPitch = -20;
    const range = maxPitch - minPitch;

    const zeroY = height - ((0 - minPitch) / range) * height;
    chartCtx.strokeStyle = "rgba(255, 255, 255, 0.15)";
    chartCtx.lineWidth = 1;
    chartCtx.beginPath();
    chartCtx.moveTo(0, zeroY);
    chartCtx.lineTo(width, zeroY);
    chartCtx.stroke();

    chartCtx.strokeStyle = "#f6f09c";
    chartCtx.lineWidth = 2;
    chartCtx.lineJoin = "round";
    chartCtx.beginPath();

    for (let i = 0; i < maxDataPoints; i++) {
      const x = (i / (maxDataPoints - 1)) * width;
      let val = pitchHistory[i];

      if (val > maxPitch) val = maxPitch;
      if (val < minPitch) val = minPitch;

      const y = height - ((val - minPitch) / range) * height;
      if (i === 0) chartCtx.moveTo(x, y);
      else chartCtx.lineTo(x, y);
    }
    chartCtx.stroke();
  }

  // --- 4. Webcam Setup ---
  async function initCamera() {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true });
      if (webcamFeed) webcamFeed.srcObject = stream;

      mediaRecorder = new MediaRecorder(stream, { mimeType: "video/webm" });

      mediaRecorder.ondataavailable = (event) => {
        if (event.data.size > 0) recordedChunks.push(event.data);
      };

      mediaRecorder.onstop = () => {
        const blob = new Blob(recordedChunks, { type: "video/webm" });
        recordedChunks = [];
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        document.body.appendChild(a);
        a.style = "display: none";
        a.href = url;
        const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
        a.download = `pendulum_video_${timestamp}.webm`;
        a.click();
        window.URL.revokeObjectURL(url);
        a.remove();
      };
    } catch (err) {
      console.error("Error accessing webcam:", err);
    }
  }

  function saveTelemetryCSV() {
    if (telemetryLog.length <= 1) return;

    let csvContent =
      "data:text/csv;charset=utf-8," +
      telemetryLog.map((row) => row.join(",")).join("\n");
    const encodedUri = encodeURI(csvContent);
    const a = document.createElement("a");
    a.setAttribute("href", encodedUri);
    const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
    a.setAttribute("download", `pendulum_data_${timestamp}.csv`);
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  }

  initCamera();

  // --- 6. Button Logic ---
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

  if (playStopBtn) {
    playStopBtn.addEventListener("click", () => {
      const isRecording = playStopBtn.classList.toggle("is-recording");
      if (playStopText) {
        playStopText.innerText = isRecording
          ? "STOP RECORDING"
          : "START RECORDING";
      }

      if (isRecording) {
        if (mediaRecorder && mediaRecorder.state === "inactive") {
          recordedChunks = [];
          mediaRecorder.start();
        }
        isLoggingData = true;
        recordingStartTime = performance.now();
        telemetryLog = [
          [
            "Time (ms)",
            "Raw Pitch (deg)",
            "Filtered Pitch (deg)",
            "Velocity (deg/s)",
            "Error (deg)",
          ],
        ];
      } else {
        if (mediaRecorder && mediaRecorder.state === "recording") {
          mediaRecorder.stop();
        }
        isLoggingData = false;
        saveTelemetryCSV();
      }
    });
  }

  // NEW: Tuning Button Logic
  if (sendTuneBtn) {
    sendTuneBtn.addEventListener("click", () => {
      // 1. Grab the values as floats
      const kp_val = parseFloat(kpInput.value);
      const ki_val = parseFloat(kiInput.value);
      const kd_val = parseFloat(kdInput.value);

      // 2. Validate the data (prevent sending bad data if a box is empty)
      if (isNaN(kp_val) || isNaN(ki_val) || isNaN(kd_val)) {
        alert("Please enter valid numbers for all PID values.");
        return;
      }

      // 3. Send the custom JSON payload
      if (websocket && websocket.readyState === WebSocket.OPEN) {
        const payload = {
          command: "tune",
          kp: kp_val,
          ki: ki_val,
          kd: kd_val,
        };
        websocket.send(JSON.stringify(payload));

        // Visual feedback to let you know it sent successfully
        const originalText = sendTuneBtn.innerText;
        sendTuneBtn.innerText = "SENT!";
        sendTuneBtn.style.backgroundColor = "rgba(0, 213, 99, 0.2)";
        sendTuneBtn.style.borderColor = "#00d563";
        sendTuneBtn.style.color = "white";

        setTimeout(() => {
          sendTuneBtn.innerText = originalText;
          sendTuneBtn.style.backgroundColor = "";
          sendTuneBtn.style.borderColor = "";
          sendTuneBtn.style.color = "";
        }, 1000);
      } else {
        console.warn("WebSocket not connected. Cannot send tuning.");
      }
    });
  }

  initWebSocket();
});
