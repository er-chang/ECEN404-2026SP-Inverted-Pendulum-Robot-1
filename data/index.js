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

  // --- 2. Recording & Logging Variables ---
  let mediaRecorder;
  let recordedChunks = [];
  let isLoggingData = false;
  let telemetryLog = [];
  let recordingStartTime = 0;

  // --- 3. WebSocket Setup ---
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

      // Track rough message interval
      if (latencyVal && lastMessageTime !== 0) {
        latencyVal.textContent = `${(now - lastMessageTime).toFixed(1)} ms`;
      }
      lastMessageTime = now;

      try {
        const data = JSON.parse(event.data);

        // Update UI
        if (typeof data.pitch === "number") {
          pitchVal.innerHTML = `${data.pitch.toFixed(2)}&deg;`;
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
        // If recording is active and we have data, push it to the spreadsheet array
        if (isLoggingData && typeof data.pitch === "number") {
          const elapsedMs = (performance.now() - recordingStartTime).toFixed(0);

          // Safe fallbacks in case a specific value is missing from the JSON
          const vPitch = data.pitch.toFixed(2);
          const vVel =
            typeof data.angular_velocity === "number"
              ? data.angular_velocity.toFixed(2)
              : "0.00";
          const vErr =
            typeof data.setpoint_error === "number"
              ? data.setpoint_error.toFixed(2)
              : "0.00";

          telemetryLog.push([elapsedMs, vPitch, vVel, vErr]);
        }
      } catch (e) {
        console.error("Error parsing WebSocket JSON:", e, event.data);
      }
    };
  }

  // --- 4. Webcam Setup ---
  async function initCamera() {
    try {
      // Request access to the user's webcam
      const stream = await navigator.mediaDevices.getUserMedia({ video: true });
      if (webcamFeed) {
        webcamFeed.srcObject = stream;
      }

      // Set up the MediaRecorder to capture the stream
      mediaRecorder = new MediaRecorder(stream, { mimeType: "video/webm" });

      // Collect data chunks as they are recorded
      mediaRecorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          recordedChunks.push(event.data);
        }
      };

      // When recording stops, compile the chunks and download the file
      mediaRecorder.onstop = () => {
        const blob = new Blob(recordedChunks, { type: "video/webm" });
        recordedChunks = []; // Clear memory for the next recording

        // Create a temporary hidden link to trigger the download
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        document.body.appendChild(a);
        a.style = "display: none";
        a.href = url;

        // Generate a filename with a timestamp
        const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
        a.download = `pendulum_video_${timestamp}.webm`;

        a.click();
        window.URL.revokeObjectURL(url);
        a.remove();
        console.log("Video saved to disk.");
      };

      console.log("Webcam initialized successfully.");
    } catch (err) {
      console.error(
        "Error accessing webcam. Ensure you are on localhost or HTTPS/allowed IP:",
        err,
      );
    }
  }

  // --- 5. CSV Export Helper ---
  function saveTelemetryCSV() {
    if (telemetryLog.length <= 1) return; // Don't save if there's no data besides headers

    // Convert the 2D array into a CSV string
    let csvContent =
      "data:text/csv;charset=utf-8," +
      telemetryLog.map((row) => row.join(",")).join("\n");

    // Create a hidden download link
    const encodedUri = encodeURI(csvContent);
    const a = document.createElement("a");
    a.setAttribute("href", encodedUri);

    // Name the file with a timestamp to match the video
    const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
    a.setAttribute("download", `pendulum_data_${timestamp}.csv`);

    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    console.log("Telemetry CSV saved to disk.");
  }

  // Initialize camera when page loads
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
        // --- START EVERYTHING ---
        // 1. Start Video
        if (mediaRecorder && mediaRecorder.state === "inactive") {
          recordedChunks = [];
          mediaRecorder.start();
          console.log("Video recording started...");
        }

        // 2. Start Data Logging
        isLoggingData = true;
        recordingStartTime = performance.now();
        // Setup the CSV headers
        telemetryLog = [
          ["Time (ms)", "Pitch (deg)", "Velocity (deg/s)", "Error (deg)"],
        ];
        console.log("Data logging started...");
      } else {
        // --- STOP EVERYTHING ---
        // 1. Stop Video (this triggers the download automatically via onstop)
        if (mediaRecorder && mediaRecorder.state === "recording") {
          mediaRecorder.stop();
        }

        // 2. Stop Data Logging & Download CSV
        isLoggingData = false;
        saveTelemetryCSV();
      }
    });
  }

  // --- 7. Start connection ---
  initWebSocket();
});
