import { readFile, writeFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { spawn } from 'node:child_process';

if(Number(process.versions.node.split('.')[0])<24)throw new Error('Project Graph requires Node.js 24 or newer.');
const root=fileURLToPath(new URL('../',import.meta.url));
const raw=await readFile(resolve(root,'package-lock.json'),'utf8'),lock=JSON.parse(raw);
const manifest=JSON.parse(await readFile(resolve(root,'package.json'),'utf8'));
const digest=createHash('sha256').update(raw).digest('hex'),marker=resolve(root,'node_modules/.project-graph-lock');
let previous=null;
try{previous=(await readFile(marker,'utf8')).trim();}catch{}
let matches=true;
for(const name of Object.keys({...manifest.dependencies,...manifest.devDependencies})){
  try{const installed=JSON.parse(await readFile(resolve(root,'node_modules',name,'package.json'),'utf8'));if(installed.version!==lock.packages[`node_modules/${name}`]?.version)matches=false;}catch{matches=false;}
}
if(!matches||(previous&&previous!==digest)){
  if(!process.env.npm_execpath)throw new Error('Run this command through npm run setup.');
  console.log('Installing dependencies from package-lock.json...');
  await new Promise((success,failure)=>{
    const child=spawn(process.execPath,[process.env.npm_execpath,'ci'],{cwd:root,stdio:'inherit',windowsHide:true});
    child.on('error',failure);child.on('exit',code=>code===0?success():failure(new Error(`npm ci exited ${code}`)));
  });
}else console.log('Dependencies already match the lockfile.');
await writeFile(marker,digest+'\n');
