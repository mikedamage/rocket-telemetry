console.log("hello main.js");

let statusTimeout;
const buttons = document.querySelectorAll("#control-messages button");
const settingsForm = document.querySelector("#settings-form");
const formStatus = settingsForm.querySelector(".status");
const restartButton = document.querySelector("#restart");

const controlCodes = {
  startTelemetry: 1,
  stopTelemetry: 2,
  startBleBeacon: 3,
  stopBleBeacon: 4,
};

async function loadConfig() {
  const res = await fetch("/config");

  if (!res.ok) {
    console.error("error loading telemetry settings");
    formStatus.innerHTML = "Error loading telemetry settings";
  }

  const configJson = await res.json();
  const telemetryHostField = settingsForm.querySelector("#telemetry-server-host");
  const telemetryPortField = settingsForm.querySelector("#telemetry-server-port");
  const wifiSSIDField = settingsForm.querySelector("#wifi-ssid");
  const wifiPSKField = settingsForm.querySelector("#wifi-psk");
  telemetryHostField.value = configJson?.telemetryHost;
  telemetryPortField.value = configJson?.telemetryPort;
  wifiSSIDField.value = configJson?.wifiSSID;
  wifiPSKField.value = configJson?.wifiPSK;
}

function flashStatus(message, timeoutSeconds = 10) {
  if (statusTimeout) {
    clearTimeout(statusTimeout);
  }

  const statusMessage = settingsForm.querySelector(".status");
  statusMessage.innerHTML = message;
  statusTimeout = setTimeout(() => {
    statusMessage.innerHTML = "";
  }, timeoutSeconds * 1000);
}

for (const b of buttons) {
  b.addEventListener(
    "click",
    async (evt) => {
      evt.preventDefault();
      evt.stopPropagation();
      const { controlMessage } = evt.target.dataset;
      const messageCode = controlCodes[controlMessage];

      if (!messageCode) {
        console.warn("invalid message code %s", controlMessage);
        return;
      }

      try {
        const res = await fetch(`/control_messages?message=${messageCode}`, { method: "post" });
        console.debug("received response: %O", res);
      } catch (err) {
        console.error("control message send failure: %O", err);
      }
    },
    false
  );
}

settingsForm.querySelector(".save").addEventListener(
  "click",
  async (evt) => {
    evt.preventDefault();
    evt.stopPropagation();

    const telemetryHost = settingsForm.querySelector("#telemetry-server-host")?.value;
    const telemetryPort = settingsForm.querySelector("#telemetry-server-port")?.value;
    const wifiSSID = settingsForm.querySelector("#wifi-ssid")?.value;
    const wifiPSK = settingsForm.querySelector("#wifi-psk")?.value;
    const res = await fetch("/config", {
      method: "PUT",
      headers: {
        "Content-Type": "application/json",
        Accept: "application/json",
      },
      body: JSON.stringify({
        telemetryHost,
        telemetryPort,
        wifiSSID,
        wifiPSK,
      }),
    });

    if (res.ok) {
      console.log("Telemetry settings updated");
      await loadConfig();
      flashStatus("Telemetry settings updated. Reconnecting...");
    }
  },
  false
);

restartButton.addEventListener(
  "click",
  async (evt) => {
    evt.preventDefault();
    evt.stopPropagation();

    const res = await fetch("/restart", { method: "post" });

    if (!res.ok) {
      console.error("Received error response: %O", res);
    }
  },
  false
);

loadConfig();
