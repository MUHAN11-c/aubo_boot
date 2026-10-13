'use strict';
// Execute node:test files directly: preserve assertion TAP, avoiding opaque file wrappers.
const fs = require('node:fs');
const path = require('node:path');
const {spawnSync} = require('node:child_process');
const testDirectory = path.resolve(__dirname, '../test');
const files = process.argv.length > 2 ? process.argv.slice(2).map(file => path.resolve(file)) :
  fs.readdirSync(testDirectory).filter(file => file.endsWith('.test.cjs')).sort().map(file => path.join(testDirectory, file));
const environment = {...process.env};
delete environment.NODE_TEST_CONTEXT;
let failures = 0;
for (const file of files) {
  console.log(`\n运行 ${path.basename(file)}`);
  const result = spawnSync(process.execPath, [file], {stdio: 'inherit', env: environment});
  if (result.error || result.signal || result.status !== 0) {
    failures += 1;
    if (result.error) console.error(result.error.message);
    if (result.signal) console.error(`测试进程被 ${result.signal} 终止`);
  }
}
console.log(`\n测试文件：${files.length}，通过：${files.length - failures}，失败：${failures}`);
process.exitCode = failures || !files.length ? 1 : 0;
