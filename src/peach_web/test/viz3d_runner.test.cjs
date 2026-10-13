const test = require('node:test');
const assert = require('node:assert/strict');
const {spawnSync} = require('node:child_process');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const runner = path.resolve(__dirname, '../scripts/run-tests.cjs');
function runFixture(body) {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'peach-web-test-'));
  const fixture = path.join(directory,'fixture.cjs');
  try {
    fs.writeFileSync(fixture,body);
    const outputFile=path.join(directory,'output.txt');const fd=fs.openSync(outputFile,'w');
    let result;
    try {result=spawnSync(process.execPath,[runner,fixture],{stdio:['ignore',fd,fd],env:{...process.env,NODE_TEST_CONTEXT:'child-v8'}});}
    finally {fs.closeSync(fd);}
    assert.equal(result.error,undefined,'runner process must actually spawn');
    return {...result,stdout:fs.readFileSync(outputFile,'utf8'),stderr:''};
  }
  finally {fs.rmSync(directory,{recursive:true,force:true});}
}
test('direct runner exposes actual passing subtests',()=> {
  const result=runFixture("const test=require('node:test');const assert=require('node:assert/strict');test('fixture real assertion',()=>assert.equal(process.env.NODE_TEST_CONTEXT,undefined));");
  assert.equal(result.status,0,result.stderr);assert.match(result.stdout,/fixture real assertion/);assert.match(result.stdout,/# tests 1/);assert.match(result.stdout,/测试文件：1，通过：1，失败：0/);
});
test('direct runner preserves failed assertion output and returns failure',()=> {
  const result=runFixture("const test=require('node:test');const assert=require('node:assert/strict');test('fixture intentional failure',()=>assert.equal(1,2));");
  assert.equal(result.status,1);assert.match(result.stdout,/fixture intentional failure/);assert.match(result.stdout,/ERR_ASSERTION/);assert.match(result.stdout,/测试文件：1，通过：0，失败：1/);
});
