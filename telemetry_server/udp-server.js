import "dotenv/config";
import dgram from "node:dgram";
import { log } from "./lib/logger.js";
import path from "node:path";
import { createWriteStream } from "node:fs";
import { decode } from "@msgpack/msgpack";
import { format as formatCsv } from "@fast-csv/format";

const port = process.env.PORT || 5150;
const fileName = process.env.FILE || "output.csv";
const server = dgram.createSocket("udp4");
const csvFile = path.resolve(path.join(import.meta.dirname, fileName));
const telemetryStream = createWriteStream(csvFile, { flags: "w" });
const formatter = formatCsv({
  headers: true,
});

formatter.pipe(telemetryStream);

server.on("listening", () => {
  log(`UDP server listening on port ${port}`);
});

server.on("error", (err) => {
  console.error(`UDP server error:\n${err}`);
  console.error(err.stack);
});

server.on("message", (msg, rinfo) => {
  log(`Received ${rinfo.size} bytes from ${rinfo.address}:${rinfo.port}`);
  // console.log(msg.toString());
  try {
    // log(msg.toString().trim());
    // telemetryStream.write(msg.toString().trim() + "\n");
    const unpacked = decode(msg);

    for (const row of unpacked.readings) {
      formatter.write(row);
    }

    log(`${telemetryStream.bytesWritten} bytes written so far`);
  } catch (err) {
    log("error decoding msgpack:");
    console.error(err);
  }
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
