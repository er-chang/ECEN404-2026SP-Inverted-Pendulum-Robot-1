document.addEventListener("DOMContentLoaded", () => {
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

  const sendTuneBtn = document.getElementById("sendTuneBtn");
  const kpInput = document.getElementById("kpInput");
  const kiInput = document.getElementById("kiInput");
  const kdInput = document.getElementById("kdInput");

  const pitchChart = document.getElementById("pitchChart");
  const chartCtx = pitchChart ? pitchChart.getContext("2d") : null;
  const maxDataPoints = 150;
  let pitchHistory = new Array(maxDataPoints).fill(0);

  let mediaRecorder;
  let recordedChunks = [];
  let isLoggingData = false;
  let telemetryLog = [];
  let recordingStartTime = 0;

  // --- RMS & TARE VARIABLES ---
  const rmsWindowSize = 10;
  let squaredErrors = [];
  let initialPitchOffset = null; // Stores the "zero" point when the site loads

  const gateway = `ws://${window.location.hostname}/ws`;
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

    websocket.onopen = () => {
      setStatus("online");
      initialPitchOffset = null; // Reset tare so it recalibrates on fresh connection
    };

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

        // 1. AUTOMATIC STARTUP TARE
        // Captures the first angle received and treats it as 0.00°
        if (initialPitchOffset === null && typeof data.pitch === "number") {
          initialPitchOffset = data.pitch;
        }

        const displayPitch = data.pitch - (initialPitchOffset || 0);
        const displayRawPitch =
          (data.pitch_raw || 0) - (initialPitchOffset || 0);

        // 2. RMS ERROR CALCULATION
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
        if (typeof data.front_distance === "number")
          safeSetText(fDistVal, `${data.front_distance.toFixed(2)} m`);

        // 3. TELEMETRY LOGGING (Using Zeroed Values)
        if (isLoggingData && typeof data.pitch === "number") {
          const elapsedMs = (performance.now() - recordingStartTime).toFixed(0);
          telemetryLog.push([
            elapsedMs,
            displayRawPitch.toFixed(2),
            displayPitch.toFixed(2),
            data.angular_velocity ? data.angular_velocity.toFixed(2) : "0.00",
            currentRmsError.toFixed(3),
          ]);
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

  async function initCamera() {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true });
      if (webcamFeed) webcamFeed.srcObject = stream;
      mediaRecorder = new MediaRecorder(stream, { mimeType: "video/webm" });
      mediaRecorder.ondataavailable = (e) => {
        if (e.data.size > 0) recordedChunks.push(e.data);
      };
      mediaRecorder.onstop = () => {
        const blob = new Blob(recordedChunks, { type: "video/webm" });
        recordedChunks = [];
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `pendulum_video_${new Date().toISOString().replace(/[:.]/g, "-")}.webm`;
        a.click();
      };
    } catch (err) {
      console.error("Camera error:", err);
    }
  }

  function saveTelemetryCSV() {
    if (telemetryLog.length <= 1) return;
    let csvContent =
      "data:text/csv;charset=utf-8," +
      telemetryLog.map((e) => e.join(",")).join("\n");
    const a = document.createElement("a");
    a.href = encodeURI(csvContent);
    a.download = `pendulum_data_${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  }

  initCamera();

  if (playStopBtn) {
    playStopBtn.addEventListener("click", () => {
      const isRecording = playStopBtn.classList.toggle("is-recording");
      if (isRecording) {
        if (mediaRecorder && mediaRecorder.state === "inactive") {
          recordedChunks = [];
          mediaRecorder.start();
        }
        isLoggingData = true;
        recordingStartTime = performance.now();
        squaredErrors = [];
        telemetryLog = [
          [
            "Time (ms)",
            "Raw Pitch (deg)",
            "Filtered Pitch (deg)",
            "Velocity (deg/s)",
            "RMS Filter Error (deg)",
          ],
        ];
      } else {
        if (mediaRecorder && mediaRecorder.state === "recording")
          mediaRecorder.stop();
        isLoggingData = false;
        saveTelemetryCSV();
      }
    });
  }

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
      }
    });
  }

  if (armBtn) {
    armBtn.addEventListener("click", () => {
      if (armBtn.classList.contains("is-armed")) {
        armBtn.classList.remove("is-armed");
        sendCommand("disarm");
      } else {
        armBtn.classList.add("is-armed");
        sendCommand("arm");
      }
    });
  }

  if (estopBtn) {
    estopBtn.addEventListener("click", () => {
      sendCommand("estop");
      if (armBtn) armBtn.classList.remove("is-armed");
    });
  }

  if (rebootBtn) {
    rebootBtn.addEventListener("click", () => {
      sendCommand("reboot");
    });
  }

  initWebSocket();
});
