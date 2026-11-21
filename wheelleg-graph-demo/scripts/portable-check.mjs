import assert from 'node:assert/strict';
import { cp, mkdtemp, mkdir, rm, readFile, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { resolve, join, dirname, basename } from 'node:path';
import { pathToFileURL } from 'node:url';
import { spawn } from 'node:child_process';

const root=resolve(import.meta.dirname,'..');const work=await mkdtemp(join(tmpdir(),'project-graph-portable space-'));let app;
const run=(args)=>new Promise((success,failure)=>{
  const child=spawn(process.execPath,[process.env.npm_execpath,...args],{cwd:work,stdio:'inherit',windowsHide:true,env:process.env});child.on('error',failure);child.on('exit',code=>code===0?success():failure(new Error(`npm ${args.join(' ')} exited ${code}`)));
});
try{
  assert.ok(process.env.npm_execpath,'Run this check with npm run test:portable');
  for(const name of ['package.json','package-lock.json','tsconfig.json','vite.config.ts','index.html','src','shared','server','schema','scripts','examples'])await cp(join(root,name),join(work,name),{recursive:true});
  await run(['run','setup']);await run(['run','setup']);await run(['run','build']);
  const {startServer}=await import(pathToFileURL(join(work,'server','index.ts')).href);
  app=await startServer({directory:join(work,'isolated data'),host:'127.0.0.1',port:0,token:''});
  assert.equal((await fetch(app.url)).status,200);
  const sample=JSON.parse(await readFile(join(work,'examples','lqr.graph.json'),'utf8'));
  const result=await fetch(app.url+'/api/projects',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({project:sample})});assert.equal(result.status,201);
  const {project}=await result.json();assert.equal(Object.values(project.model.nodes).filter(node=>node.sceneId===project.rootSceneId&&node.kind!=='annotation').length,9);assert.equal(Object.values(project.model.nodes).filter(node=>node.kind==='annotation').length,6);assert.equal(Object.keys(project.model.scenes).length,3);
  const exported=await(await fetch(app.url+`/api/projects/${project.projectId}/export`)).json();assert.deepEqual(exported.model,sample.model);assert.deepEqual(exported.view,sample.view);
  const report={date:'2026-11-17',node:process.version,checks:['fresh directory containing spaces','npm setup installs from lockfile without copied node_modules and skips a second install','independent production build','server and UI HTTP response','LQR import/export without firmware checkout']};
  await mkdir(join(root,'test-results'),{recursive:true});await writeFile(join(root,'test-results','portable-report.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
}finally{
  await app?.close();assert.equal(dirname(resolve(work)),resolve(tmpdir()));assert.ok(basename(work).startsWith('project-graph-portable space-'));await rm(work,{recursive:true,force:true,maxRetries:3,retryDelay:300});
}
