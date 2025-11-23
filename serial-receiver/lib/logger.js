export function log(msg) {
  const now = new Date();
  const timestamp = now.toISOString();
  console.log(`${timestamp} - ${msg}`);
}
