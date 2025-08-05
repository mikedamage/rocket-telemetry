// import csv from "csv";
import yargs from "yargs";
import { hideBin } from "yargs/helpers";
import net from "node:net";
import { createWriteStream } from "node:fs";
import path from "node:path";

const argv = yargs(hideBin(process.argv))
  .option("verbose", {
    alias: "v",
    type: "boolean",
    description: "Run with verbose log output",
  })
  .option("port", {
    alias: "p",
    type: "number",
    description: "TCP port to bind to",
  })
  .option("file", {
    alias: "f",
    type: "string",
    description: "Output CSV file path",
  })
  .parse();

function log(msg) {
  if (!argv.verbose) return;
  const now = new Date();
  const timestamp = now.toISOString();
  console.log(`${timestamp} - ${msg}`);
}

const csvFile = path.resolve(path.join(import.meta.dirname, argv.file));

const server = net.createServer((conn) => {
  log("client connected");

  const outputStream = createWriteStream(csvFile, { flags: "a" });

  conn.pipe(outputStream);

  conn.on("end", () => {
    log("client disconnected");
    outputStream.end();
  });

  conn.on("data", (data) => {
    log(`writing ${data.length} bytes to output file`);
    log(data.toString().trim());
  });

  conn.on("error", (err) => {
    log("connection error:");
    console.error(err);
    outputStream.end();
  });

  outputStream.on("error", (err) => {
    log("output stream error:");
    console.error(err);
    outputStream.end();
  });
});

server.on("error", (err) => {
  console.error("TCP error:");
  console.error(err);
});

process.on("SIGINT", () => {
  console.log("Waiting for active connections to close");

  server.close(() => {
    console.log("Stopping server");
    process.exit();
  });
});

server.listen(argv.port, "0.0.0.0", () => {
  log(`starting server on port ${argv.port}`);
  log(`output will be written to ${argv.file}`);
});
