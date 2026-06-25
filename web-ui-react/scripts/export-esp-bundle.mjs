import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const projectRoot = path.resolve(__dirname, '..');
const distDir = path.join(projectRoot, 'dist');
const assetsDir = path.join(distDir, 'assets');
const firmwareGeneratedDir = path.resolve(projectRoot, '..', 'robot-code', 'src', 'generated');
const outputFile = path.join(firmwareGeneratedDir, 'WebUiBundle.h');

fs.mkdirSync(firmwareGeneratedDir, { recursive: true });

const indexHtmlPath = path.join(distDir, 'index.html');
if (!fs.existsSync(indexHtmlPath)) {
  throw new Error('dist/index.html not found. Run vite build first.');
}

const indexHtml = fs.readFileSync(indexHtmlPath, 'utf8');
const bundleMatch = indexHtml.match(/assets\/(index-[^"']+\.js)/i);
if (!bundleMatch) {
  throw new Error('Could not locate built JS bundle from dist/index.html');
}

const bundleFile = bundleMatch[1];
const bundlePath = path.join(assetsDir, bundleFile);
if (!fs.existsSync(bundlePath)) {
  throw new Error(`Resolved bundle does not exist: ${bundleFile}`);
}

const raw = fs.readFileSync(bundlePath);
const gz = zlib.gzipSync(raw, { level: 9 });

const lines = [];
lines.push('#pragma once');
lines.push('#include <Arduino.h>');
lines.push('');
lines.push('const uint8_t WEB_UI_APP_JS_GZ[] PROGMEM = {');
for (let index = 0; index < gz.length; index += 20) {
  const line = Array.from(gz.slice(index, index + 20))
    .map((byte) => `0x${byte.toString(16).padStart(2, '0')}`)
    .join(', ');
  lines.push(`  ${line},`);
}
lines.push('};');
lines.push('');
lines.push(`const size_t WEB_UI_APP_JS_GZ_LEN = ${gz.length};`);
lines.push(`const size_t WEB_UI_APP_JS_RAW_LEN = ${raw.length};`);
lines.push('');

fs.writeFileSync(outputFile, lines.join('\n'));
console.log(
  `Generated ${path.relative(projectRoot, outputFile)} from ${bundleFile} (${raw.length} raw / ${gz.length} gz)`,
);
