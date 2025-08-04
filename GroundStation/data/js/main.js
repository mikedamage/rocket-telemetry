console.log("hello main.js");

const buttons = document.querySelectorAll("#control-messages button");
const telemetryForm = document.querySelector("#telemetry-settings-form");

const controlCodes = {
  startTelemetry: 1,
  stopTelemetry: 2,
  startBleBeacon: 3,
  stopBleBeacon: 4,
};

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

telemetryForm.querySelector(".save").addEventListener(
  "click",
  async (evt) => {
    evt.preventDefault();
    evt.stopPropagation();

    const telemetryHost = telemetryForm.querySelector("#telemetry-server-host")?.value;
    const telemetryPort = telemetryForm.querySelector("#telemetry-server-port")?.value;
    const res = await fetch("/config", {
      method: "PUT",
      headers: {
        "Content-Type": "application/json",
        Accept: "application/json",
      },
      body: JSON.stringify({
        telemetryHost,
        telemetryPort,
      }),
    });

    if (res.ok) {
      console.log("Telemetry settings updated");
      telemetryForm.querySelector(".status").innerHTML = "Settings updated. Reconnecting...";
    }
  },
  false
);
