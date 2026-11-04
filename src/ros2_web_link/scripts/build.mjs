// Explicit local frontend build. colcon never downloads npm packages implicitly.
import {build} from 'esbuild';
import {cpSync, mkdirSync, rmSync, existsSync, readFileSync, readdirSync, writeFileSync, statSync} from 'node:fs';
import path from 'node:path';
import {execFileSync} from 'node:child_process';

// npm intentionally omits root package-lock.json from a published tarball.
// Keep a byte-identical build snapshot under another name so a local source
// recipient can restore the canonical lock and run npm ci before rebuilding.
const buildLock=readFileSync('package-lock.json','utf8');
rmSync('dist', {recursive:true,force:true});
mkdirSync('dist/web', {recursive:true});
writeFileSync('dist/build-package-lock.json',buildLock);
cpSync('web','dist/web',{recursive:true});
cpSync('node_modules/uplot/dist/uPlot.min.css','dist/web/uplot.css');
execFileSync('node',['node_modules/typescript/bin/tsc','--emitDeclarationOnly'],{stdio:'inherit'});
await build({entryPoints:['sdk/index.ts'],bundle:true,format:'esm',platform:'neutral',
  external:['roslib'],target:'es2022',sourcemap:true,outfile:'dist/sdk/index.js'});
const browser=await build({entryPoints:['sdk/index.ts'],bundle:true,format:'iife',globalName:'Ros2WebLink',
  platform:'browser',target:'es2022',sourcemap:true,metafile:true,legalComments:'linked',outfile:'dist/web/ros2-web-link.js'});
const optional=[];
for(const [folder,globalName] of [['media','Ros2WebMedia'],['scene','Ros2WebScene'],['audit','Ros2WebAudit'],['graph','Ros2WebGraph'],['workbench','Ros2WebWorkbench']]) {
  await build({entryPoints:[`${folder}/index.ts`],bundle:true,format:'esm',platform:'neutral',packages:'external',
    target:'es2022',sourcemap:true,outfile:`dist/${folder}/index.js`});
  optional.push(await build({entryPoints:[`${folder}/index.ts`],bundle:true,format:'iife',globalName,
    platform:'browser',target:'es2022',sourcemap:true,metafile:true,legalComments:'linked',outfile:`dist/web/ros2-web-${folder}.js`}));
}
// Pure decoders and the observer boundary are executable without a DOM/WebGL.
await build({entryPoints:['media/image.ts','scene/pointcloud.ts','scene/tf.ts','scene/shaders.ts','scene/event-timing.ts'],bundle:true,format:'esm',platform:'neutral',
  packages:'external',outbase:'.',outdir:'dist',target:'es2022',sourcemap:true});
await build({entryPoints:['graph/model.ts'],bundle:true,format:'esm',platform:'neutral',packages:'external',outbase:'.',outdir:'dist',target:'es2022',sourcemap:true});
// npm tarballs may strip leading legal comments. Preserve the actual upstream
// LICENSE/NOTICE texts for every runtime package included in the browser bundle.
const packages=new Set();
for(const input of [browser,...optional].flatMap(bundle=>Object.keys(bundle.metafile.inputs))) {
  if(!input.includes('node_modules/')) continue;
  let directory=path.dirname(path.resolve(input));
  while(directory!==path.dirname(directory)) {
    if(existsSync(path.join(directory,'package.json'))) {packages.add(directory);break;}
    directory=path.dirname(directory);
  }
}
const notices=['Third-party software included in the ROS Web SDK / optional media / scene bundles\n'];
for(const directory of [...packages].sort()) {
  const pkg=JSON.parse(readFileSync(path.join(directory,'package.json'),'utf8'));
  notices.push(`\n=== ${pkg.name} ${pkg.version} (${pkg.license || 'see upstream license'}) ===\n`);
  for(const filename of readdirSync(directory).filter(f=>/^(licen[cs]e|notice|copying)/i.test(f))) {
    const file=path.join(directory,filename);
    if(statSync(file).isFile()) notices.push(readFileSync(file,'utf8'));
  }
}
notices.push(readFileSync('third_party/lichtblick_transforms/LICENSE','utf8'));
writeFileSync('dist/web/THIRD_PARTY_NOTICES.txt',notices.join('\n'));
console.log('Built core SDK + optional media/scene modules, declarations, source maps and full license notices');
