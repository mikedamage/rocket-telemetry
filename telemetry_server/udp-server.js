import "dotenv/config";
import dgram from "node:dgram";
import { log } from "./lib/logger.js";
import path from "node:path";
import { createWriteStream } from "node:fs";

const port = process.env.PORT || 5150;
const fileName = process.env.FILE || "output.csv";
const server = dgram.createSocket("udp4");
const csvFile = path.resolve(path.join(import.meta.dirname, fileName));
const telemetryStream = createWriteStream(csvFile, { flags: "a" });

server.on("listening", () => {
  log(`UDP server listening on port ${port}`);
});

server.on("error", (err) => {
  console.error(`UDP server error:\n${err}`);
  console.error(err.stack);
});

server.on("message", (msg, rinfo) => {
  log(`Received ${msg.length} bytes from ${rinfo.address}:${rinfo.port}`);
  log(msg.toString().trim());
  telemetryStream.write(msg.toString().trim() + "\n");
});

server.on("close", () => {
  log("UDP server closed");
  telemetryStream.end(() => {
    log("Telemetry stream closed");
    process.exit();
  });
});

process.on("SIGINT", () => {
  server.close();
});

server.bind(port);
