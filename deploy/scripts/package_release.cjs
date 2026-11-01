#!/usr/bin/env node
// Build the ZIP/TAR.GZ deployment format used by xr_teleoperate_v1.0.0_arm64.
const fs = require('node:fs');
const path = require('node:path');
const cp = require('node:child_process');
const zlib = require('node:zlib');

const root = path.resolve(__dirname, '../..');
const output = path.resolve(process.argv[2] || path.join(root, 'dist'));
const git = args => cp.execFileSync('git', args, {cwd: root, encoding: 'utf8'}).trim();
git(['diff', '--quiet']);
git(['diff', '--cached', '--quiet']);
const metadata = JSON.parse(fs.readFileSync(path.join(root, 'card-module.json'), 'utf8'));
const version = fs.readFileSync(path.join(root, 'VERSION'), 'utf8').trim();
if (metadata.version !== version) throw new Error('VERSION differs from card-module.json');
const arch = metadata.buildArchitecture;
for (const value of [metadata.name, version, arch]) {
  if (!/^[a-zA-Z0-9_.-]+$/.test(value)) throw new Error('Invalid package name/version/architecture');
}
const stem = `${metadata.name}_v${version}_${arch}`;
const buildTime = new Date();
metadata.gitSha = git(['rev-parse', 'HEAD']);
metadata.buildDate = metadata.timestamp = buildTime.toISOString().replace(/\.\d{3}Z$/, 'Z');
metadata.packageType = 'zip'; // Deployer manifest schema uses zip for both archive formats.
const files = new Map();
const tracked = cp.execFileSync('git', ['ls-files', '-s', '-z'], {cwd: root}).toString('utf8').split('\0').filter(Boolean);
for (const entry of tracked) {
  const split = entry.indexOf('\t');
  const name = entry.slice(split + 1);
  if (name === '.deb-build/card-module.json') continue; // Build input; payload has one manifest.
  if (/^(\.git\/|dist\/|temp\/|\.workbuddy-ai\/)|(__pycache__|\.egg-info)|\.(pyc|log|pid)$/.test(name)) {
    throw new Error('Runtime/build output is tracked: ' + name);
  }
  const executable = entry.startsWith('100755 ') || name.endsWith('.sh') ||
    /^\.deb-build\/debian\/(postinst|prerm|postrm)$/.test(name) || name === '.deb-build/usr/bin/xr-teleop-control' ||
    name.startsWith('third_party/dex1_1_service/bin/');
  files.set(name, {name, mode: executable ? 0o755 : 0o644, bytes: fs.readFileSync(path.join(root, name))});
}
// The reference deployment archive bundles model caches and local WebXR certificates.
// These remain ignored by Git; logs, PID files and captured camera probes are omitted.
for (const name of ['g1_29_model_cache.pkl', 'teleop/g1_29_model_cache.pkl', 'teleop/televuer/cert.pem', 'teleop/televuer/key.pem']) {
  const full = path.join(root, name);
  if (fs.existsSync(full)) files.set(name, {name, mode: name.endsWith('/key.pem') ? 0o600 : 0o644, bytes: fs.readFileSync(full)});
}
for (const name of ['install_service.sh', 'VERSION', '.deb-build/VERSION', 'scripts/start_camera.sh',
  'scripts/start_teleop.sh', 'teleop/robot_control/arm_trajectory.py',
  'teleop/teleimager/vendor/camera_sdk_test/build/libscam.so']) {
  if (!files.has(name)) throw new Error('Required deployment file missing: ' + name);
}
const payloadSize = [...files.values()].reduce((sum, file) => sum + file.bytes.length, 0) - files.get('card-module.json').bytes.length;
for (let i = 0; i < 5; i++) {
  files.get('card-module.json').bytes = Buffer.from(JSON.stringify(metadata, null, 2) + '\n');
  const size = payloadSize + files.get('card-module.json').bytes.length;
  if (metadata.applicationInfo.sizeBytes === size) break;
  metadata.applicationInfo.sizeBytes = size;
}
files.get('card-module.json').bytes = Buffer.from(JSON.stringify(metadata, null, 2) + '\n');
const dirs = new Set(['']);
for (const name of files.keys()) {
  let dir = path.posix.dirname(name);
  while (dir !== '.') {dirs.add(dir); dir = path.posix.dirname(dir);}
}
const entries = [...dirs].map(name => ({name: name ? name + '/' : '', mode: 0o755, directory: true, bytes: Buffer.alloc(0)}));
entries.push(...files.values());
entries.sort((a, b) => a.name.localeCompare(b.name, 'en'));

function tarHeader(entry) {
  const header = Buffer.alloc(512);
  let name = './' + entry.name;
  let prefix = '';
  if (Buffer.byteLength(name) > 100) {
    const candidates = [...name.matchAll(/\//g)].map(match => match.index).reverse();
    const split = candidates.find(i => Buffer.byteLength(name.slice(0, i)) <= 155 && Buffer.byteLength(name.slice(i + 1)) <= 100);
    if (split === undefined) throw new Error('TAR path is too long: ' + name);
    prefix = name.slice(0, split); name = name.slice(split + 1);
  }
  const field = (value, offset, length) => header.write(value, offset, length, 'utf8');
  const octal = (value, offset, length) => field(value.toString(8).padStart(length - 1, '0') + '\0', offset, length);
  field(name, 0, 100); octal(entry.mode, 100, 8); octal(0, 108, 8); octal(0, 116, 8);
  octal(entry.bytes.length, 124, 12); octal(Math.floor(buildTime.getTime() / 1000), 136, 12);
  header.fill(32, 148, 156); field(entry.directory ? '5' : '0', 156, 1);
  field('ustar\0', 257, 6); field('00', 263, 2); field('illusory', 265, 32); field('illusory', 297, 32);
  field(prefix, 345, 155);
  const checksum = header.reduce((sum, byte) => sum + byte, 0);
  field(checksum.toString(8).padStart(6, '0') + '\0 ', 148, 8);
  return header;
}
const tarParts = [];
for (const entry of entries) {
  tarParts.push(tarHeader(entry), entry.bytes);
  const pad = (512 - entry.bytes.length % 512) % 512;
  if (pad) tarParts.push(Buffer.alloc(pad));
}
tarParts.push(Buffer.alloc(1024));

const crcTable = Array.from({length: 256}, (_, n) => {
  for (let i = 0; i < 8; i++) n = n & 1 ? 0xedb88320 ^ n >>> 1 : n >>> 1;
  return n >>> 0;
});
function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) crc = crcTable[(crc ^ byte) & 255] ^ crc >>> 8;
  return (crc ^ 0xffffffff) >>> 0;
}
const dosTime = buildTime.getUTCHours() << 11 | buildTime.getUTCMinutes() << 5 | buildTime.getUTCSeconds() >> 1;
const dosDate = (buildTime.getUTCFullYear() - 1980) << 9 | (buildTime.getUTCMonth() + 1) << 5 | buildTime.getUTCDate();
const zipParts = [], directoryParts = [];
let position = 0;
for (const entry of entries.filter(e => e.name)) {
  const name = Buffer.from(entry.name);
  const compressed = entry.directory ? entry.bytes : zlib.deflateRawSync(entry.bytes, {level: 6});
  const crc = crc32(entry.bytes);
  const local = Buffer.alloc(30);
  local.writeUInt32LE(0x04034b50); local.writeUInt16LE(20, 4); local.writeUInt16LE(0x800, 6);
  local.writeUInt16LE(entry.directory ? 0 : 8, 8); local.writeUInt16LE(dosTime, 10); local.writeUInt16LE(dosDate, 12);
  local.writeUInt32LE(crc, 14); local.writeUInt32LE(compressed.length, 18); local.writeUInt32LE(entry.bytes.length, 22);
  local.writeUInt16LE(name.length, 26);
  const central = Buffer.alloc(46);
  central.writeUInt32LE(0x02014b50); central.writeUInt16LE(3 << 8 | 20, 4); central.writeUInt16LE(20, 6);
  central.writeUInt16LE(0x800, 8); central.writeUInt16LE(entry.directory ? 0 : 8, 10);
  central.writeUInt16LE(dosTime, 12); central.writeUInt16LE(dosDate, 14); central.writeUInt32LE(crc, 16);
  central.writeUInt32LE(compressed.length, 20); central.writeUInt32LE(entry.bytes.length, 24); central.writeUInt16LE(name.length, 28);
  central.writeUInt32LE((((entry.directory ? 0o040000 : 0o100000) | entry.mode) << 16) >>> 0, 38);
  central.writeUInt32LE(position, 42);
  zipParts.push(local, name, compressed); directoryParts.push(central, name);
  position += local.length + name.length + compressed.length;
}
const central = Buffer.concat(directoryParts);
const end = Buffer.alloc(22);
const entryCount = entries.filter(e => e.name).length;
if (entryCount >= 65535 || position + central.length >= 0xffffffff) throw new Error('ZIP64 required');
end.writeUInt32LE(0x06054b50); end.writeUInt16LE(entryCount, 8); end.writeUInt16LE(entryCount, 10);
end.writeUInt32LE(central.length, 12); end.writeUInt32LE(position, 16);
fs.mkdirSync(output, {recursive: true});
const products = [
  [stem + '.tar.gz', zlib.gzipSync(Buffer.concat(tarParts), {level: 6})],
  [stem + '.zip', Buffer.concat([...zipParts, central, end])],
];
for (const [name, bytes] of products) {
  const destination = path.join(output, name);
  fs.writeFileSync(destination + '.pending', bytes);
  fs.renameSync(destination + '.pending', destination);
  console.log(`${destination} (${bytes.length} bytes)`);
}
console.log(`Release payload: ${files.size} files; version=${version}; arch=${arch}; gitSha=${metadata.gitSha}`);
