import "dotenv/config";
import { createWriteStream } from "node:fs";
import path from "node:path";
import { log } from "./lib/logger.js";
import { createWriteStream } from "node:fs";
import { decode } from "@msgpack/msgpack";
import { format as formatCsv } from "@fast-csv/format";
import serialport from 'serialport';

const fileName = process.env.FILE || "output.csv";
const csvFile = path.resolve(path.join(import.meta.dirname, fileName));
const telemetryStream = createWriteStream(csvFile, { flags: "w" });
const formatter = formatCsv({
  headers: true,
});

let bytesReceived = 0;

formatter.pipe(telemetryStream);

// todo - setup and read from serial port
