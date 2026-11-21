import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, rm, readFile, writeFile, mkdir } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join, dirname, basename, resolve } from 'node:path';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { createServer } from 'node:http';
import { createProject, newNode, newPort, newBoard, newEdge, applyOperations, validateProject, boundaryPortId, scenePath, sourceLabel, explanationOf } from '../shared/model.ts';
import { ProjectStore } from '../server/store.ts';
import { startServer } from '../server/index.ts';

const exec = promisify(execFile);
const patch = (p, operations, actor = 'test') => ({ requestId: crypto.randomUUID(), baseRevision: p.revision, actor, summary: 'test', operations });
function graph() {
  let p = createProject('test', 'test-project');
  p = applyOperations(p, patch(p, [{ op: 'node.add', value: newNode('A','a') },{ op: 'node.add', value: newNode('B','b') }, { op: 'port.add', value: newPort('a','output','out','out') }, { op: 'port.add', value: newPort('b','input','in','in') }])).project;
  return p;
}
async function temporary(t) {
  const path = await mkdtemp(join(tmpdir(), 'project-graph-test space-'));
  return path;
}
async function dispose(path) { assert.equal(dirname(resolve(path)), resolve(tmpdir())); assert.ok(basename(path).startsWith('project-graph-test space-')); await rm(path, { recursive: true, force: true }); }
test('domain: metadata edits preserve stable IDs, positions and existing edges', () => {
  let p = graph(); p.view.nodes.a.position = { x: 417, y: -80 };
  p = applyOperations(p, patch(p, [{ op:'edge.add', value: newEdge('out','in','e') }])).project;
  const next = applyOperations(p, patch(p,[{op:'node.update',id:'a',changes:{label:'renamed',metadata:{gain:3}}}])).project;
  assert.deepEqual(next.view, p.view); assert.deepEqual(next.model.edges, p.model.edges); assert.equal(next.model.nodes.a.id, 'a');
});
test('domain: feedback cycles are valid; invalid direction and group cycles are rejected', () => {
  let p = graph();
  p = applyOperations(p, patch(p,[{op:'port.add',value:newPort('b','output','back','back')},{op:'port.add',value:newPort('a','input','feedback','feedback')},{op:'edge.add',value:newEdge('out','in','e1')},{op:'edge.add',value:newEdge('back','feedback','e2')}])).project;
  assert.equal(Object.keys(validateProject(p).project.model.edges).length,2);
  assert.throws(()=>applyOperations(p,patch(p,[{op:'edge.add',value:newEdge('in','out','wrong')}])));
  assert.throws(()=>applyOperations(p,patch(p,[{op:'group.add',value:{id:'g1',label:'1',description:'',parentGroupId:'g2'}},{op:'group.add',value:{id:'g2',label:'2',description:'',parentGroupId:'g1'}}])));
});
test('domain: removal requires explicit cascade and atomically removes related layout',()=>{
  let p=graph();p=applyOperations(p,patch(p,[{op:'edge.add',value:newEdge('out','in','e')}])).project;
  assert.throws(()=>applyOperations(p,patch(p,[{op:'node.remove',id:'a'}])));
  const next=applyOperations(p,patch(p,[{op:'node.remove',id:'a',cascade:true}])).project;
  assert.equal(next.model.ports.out,undefined);assert.equal(next.model.edges.e,undefined);assert.equal(next.view.nodes.a,undefined);assert.ok(p.model.nodes.a);
});
test('domain: portable refs, future versions, unsafe keys and incompatible dimensions',()=>{
  const p=graph();p.sourceRoots.firmware={label:'source'};p.model.nodes.a.sourceRefs=[{rootId:'firmware',path:'application/LQR.c'}];validateProject(p);
  for(const path of ['D:/private/code.c','/private/code.c','../code.c','dir\\code.c']) { const bad=structuredClone(p);bad.model.nodes.a.sourceRefs[0].path=path;assert.throws(()=>validateProject(bad)); }
  assert.throws(()=>validateProject({...p,schemaVersion:999}));
  assert.throws(()=>validateProject({...p,schemaVersion:1}),error=>error.code==='SCHEMA_UNSUPPORTED');
  assert.throws(()=>validateProject(JSON.parse('{"__proto__":{}}')));
  p.model.ports.out.shape='10';p.model.ports.in.shape='4';assert.throws(()=>applyOperations(p,patch(p,[{op:'edge.add',value:newEdge('out','in','e')}])));
});
test('store: conflict, invalid atomic transaction, retry, dry-run, restart and backup',async t=>{
  const dir=await temporary(t);let store=new ProjectStore(dir);await store.init();t.after(async()=>{await store.close();await dispose(dir);});
  let current=await store.create('persist',graph());const first=patch(current,[{op:'node.update',id:'a',changes:{label:'saved'}}]);
  await store.apply(current.projectId,first,true);assert.equal((await store.get(current.projectId)).revision,0);
  const a=store.apply(current.projectId,first);const b=store.apply(current.projectId,patch(current,[{op:'node.update',id:'b',changes:{label:'race'}}]));
  const outcomes=await Promise.allSettled([a,b]);assert.equal(outcomes[0].status,'fulfilled');assert.equal(outcomes[1].reason.code,'REVISION_CONFLICT');
  current=await store.get(current.projectId);assert.equal(current.revision,1);
  const retried=await store.apply(current.projectId,first);assert.equal(retried.duplicate,true);assert.equal(retried.project.revision,1);
  await assert.rejects(store.apply(current.projectId,patch(current,[{op:'node.update',id:'a',changes:{label:'should-not-save'}},{op:'edge.add',value:newEdge('missing','in','bad')}])));
  assert.equal((await store.get(current.projectId)).model.nodes.a.label,'saved');
  assert.equal(JSON.parse(await readFile(join(dir,'backups',`${current.projectId}.0.json`),'utf8')).model.nodes.a.label,'A');
  await store.close();store=new ProjectStore(dir);await store.init();assert.equal((await store.get(current.projectId)).model.nodes.a.label,'saved');await store.close();
});
test('store: refuses another writer, external edits, and preserves data after persist failure',async t=>{
  const dir=await temporary(t);const store=new ProjectStore(dir);await store.init();t.after(async()=>{await store.close();await dispose(dir);});
  const second=new ProjectStore(dir);await assert.rejects(second.init(),e=>e.code==='ALREADY_RUNNING');
  let p=await store.create('test',graph());const file=join(dir,`${p.projectId}.graph.json`);await writeFile(file,JSON.stringify({...p,title:'external'}));await assert.rejects(store.get(p.projectId),e=>e.code==='EXTERNAL_CHANGE');
  // A backup path replaced by a file makes persistence fail before the authoritative write.
  p=await store.create('failure',graph());await writeFile(join(dir,'backups'),'not a directory');
  await assert.rejects(store.apply(p.projectId,patch(p,[{op:'node.update',id:'a',changes:{label:'not saved'}}])));
  assert.equal((await store.get(p.projectId)).revision,0);assert.equal((await store.get(p.projectId)).model.nodes.a.label,'A');
});
test('store: undo/redo protect other actors, support multiple own changes, persist monotonic revisions',async t=>{
  const dir=await temporary(t);const store=new ProjectStore(dir);await store.init();t.after(async()=>{await store.close();await dispose(dir);});
  let p=await store.create('history',graph());p=(await store.apply(p.projectId,patch(p,[{op:'node.update',id:'a',changes:{label:'first'}}],'A'))).project;
  p=(await store.apply(p.projectId,patch(p,[{op:'node.update',id:'a',changes:{label:'second'}}],'A'))).project;
  p=(await store.undoRedo(p.projectId,'A',p.revision,'undo')).project;assert.equal(p.model.nodes.a.label,'first');
  p=(await store.undoRedo(p.projectId,'A',p.revision,'undo')).project;assert.equal(p.model.nodes.a.label,'A');
  p=(await store.undoRedo(p.projectId,'A',p.revision,'redo')).project;assert.equal(p.model.nodes.a.label,'first');
  p=(await store.apply(p.projectId,patch(p,[{op:'node.update',id:'b',changes:{label:'by B'}}],'B'))).project;
  await assert.rejects(store.undoRedo(p.projectId,'A',p.revision,'undo'),e=>e.code==='HISTORY_CONFLICT');assert.equal(p.model.nodes.b.label,'by B');
});
test('HTTP + CLI: auth, portable export/import, update notifications and bounded API',async t=>{
  const dir=await temporary(t);const token='example-test-token-12345';const app=await startServer({directory:dir,host:'127.0.0.1',port:0,token});t.after(async()=>{await app.close();await dispose(dir);});
  const call=async(path,method='GET',data)=>{
    const r=await fetch(app.url+path,{method,headers:{Authorization:`Bearer ${token}`,...(data?{'Content-Type':'application/json'}:{})},body:data?JSON.stringify(data):undefined});return{status:r.status,body:await r.json()};
  };
  assert.equal((await fetch(app.url+'/api/projects')).status,401);
  assert.equal((await fetch(app.url+'/api/projects',{headers:{Authorization:'Bearer definitely-wrong-token'}})).status,401);
  assert.equal((await fetch(app.url+'/api/events')).status,401);
  const created=await call('/api/projects','POST',{title:'shared',project:graph()});const p=created.body.project;assert.equal(created.status,201);
  const controller=new AbortController();const stream=await fetch(app.url+'/api/events',{headers:{Authorization:`Bearer ${token}`},signal:controller.signal});const reader=stream.body.getReader();await reader.read();
  const changed=await call(`/api/projects/${p.projectId}/transactions`,'POST',patch(p,[{op:'node.update',id:'a',changes:{label:'shared edit'}}],'agent'));assert.equal(changed.status,200);
  const notification=new TextDecoder().decode((await reader.read()).value);assert.match(notification,/"actor":"agent"/);controller.abort();
  const exported=await call(`/api/projects/${p.projectId}/export`);const imported=await call('/api/projects','POST',{project:exported.body});assert.notEqual(imported.body.project.projectId,p.projectId);assert.deepEqual(imported.body.project.model,exported.body.model);assert.deepEqual(imported.body.project.view,exported.body.view);
  assert.equal((await call(`/api/projects/${p.projectId}/transactions`,'POST',patch(p,[]))).status,422);
  const {stdout}=await exec(process.execPath,['scripts/graph-cli.mjs','inspect','--project',p.projectId],{cwd:resolve(import.meta.dirname,'..'),env:{...process.env,GRAPH_URL:app.url,GRAPH_ACCESS_TOKEN:token}});assert.equal(JSON.parse(stdout).project.model.nodes.a.label,'shared edit');
  const cli=async(...args)=>JSON.parse((await exec(process.execPath,['scripts/graph-cli.mjs',...args],{cwd:resolve(import.meta.dirname,'..'),env:{...process.env,GRAPH_URL:app.url,GRAPH_ACCESS_TOKEN:token}})).stdout);
  const sample=await cli('create','--title','CLI sample','--lqr');assert.equal(Object.values(sample.project.model.nodes).filter(node=>node.sceneId===sample.project.rootSceneId&&node.kind!=='annotation').length,9);assert.equal(Object.values(sample.project.model.nodes).filter(node=>node.kind==='annotation').length,6);assert.equal(Object.keys(sample.project.model.scenes).length,3);
  const scene=await cli('inspect','--project',sample.project.projectId,'--scene','mode-detail');assert.equal(scene.scene.ownerNodeId,'mode');assert.ok(Object.keys(scene.nodes).length>2);assert.ok(scene.interfaces.length>0);assert.equal(scene.nodes.lqr,undefined);
  assert.ok((await cli('list')).projects.some(item=>item.projectId===sample.project.projectId));
  assert.equal((await cli('validate','--file','examples/lqr.graph.json')).valid,true);
  const shared=join(dir,'export with spaces.json');await cli('export','--project',sample.project.projectId,'--out',shared);
  const copied=await cli('import','--file',shared);assert.notEqual(copied.project.projectId,sample.project.projectId);assert.deepEqual(copied.project.model,sample.project.model);
  const origin=await fetch(app.url+'/api/projects',{headers:{Authorization:`Bearer ${token}`,Origin:'http://different-origin'}});assert.equal(origin.status,403);
});
test('team listener requires an explicit sufficiently long token',async()=>{
  await assert.rejects(startServer({host:'0.0.0.0',port:0,token:''}),e=>e.code==='TOKEN_REQUIRED');
});

test('CLI: an old service is rejected before creating or editing any project',async t=>{
  const requests=[];const server=createServer((req,res)=>{requests.push(req.method+' '+req.url);res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify({version:'0.1.0',authRequired:false}));});
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));t.after(async()=>{server.closeAllConnections();await new Promise(resolve=>server.close(resolve));});
  const url=`http://127.0.0.1:${server.address().port}`;
  await assert.rejects(exec(process.execPath,['scripts/graph-cli.mjs','create','--title','should-not-be-created'],{cwd:resolve(import.meta.dirname,'..'),env:{...process.env,GRAPH_URL:url,GRAPH_ACCESS_TOKEN:''}}),error=>JSON.parse(error.stderr).error.code==='SERVICE_VERSION');
  assert.deepEqual(requests,['GET /api/status']);
});

function withScene(p=graph(),ownerNodeId='a',sceneId='detail') {
  return applyOperations(p,patch(p,[{op:'scene.add',value:{id:sceneId,label:'内部逻辑',description:'conditions',ownerNodeId}}])).project;
}
test('scenes: creation maps parent ports with opposite direction and synchronizes contract changes',()=>{
  let p=withScene();const mappedId=boundaryPortId('out'),mapped=p.model.ports[mappedId];
  assert.equal(p.model.nodes.a.detailSceneId,'detail');assert.equal(p.model.scenes.detail.ownerNodeId,'a');assert.equal(mapped.direction,'input');assert.equal(mapped.parentPortId,'out');assert.equal(p.model.nodes[mapped.nodeId].sceneId,'detail');
  p=applyOperations(p,patch(p,[{op:'port.update',id:'out',changes:{label:'轮端力矩',shape:'2',dataType:'float',unit:'N·m',coordinateFrame:'motor'}},{op:'port.add',value:newPort('a','input','air','air')}])).project;
  assert.equal(p.model.ports[mappedId].label,'轮端力矩');assert.equal(p.model.ports[mappedId].unit,'N·m');assert.equal(p.model.ports[mappedId].shape,'2');assert.equal(p.model.ports[boundaryPortId('air')].direction,'output');
  assert.throws(()=>applyOperations(p,patch(p,[{op:'port.update',id:mappedId,changes:{unit:'N'}}])));
  assert.throws(()=>applyOperations(p,patch(p,[{op:'port.remove',id:mappedId,cascade:true}])));
});
test('scenes: independent viewpoints, nested navigation and no parent layout changes',()=>{
  let p=withScene();const before=structuredClone(p.view.nodes.a);const rootViewport=structuredClone(p.view.scenes.root.viewport);
  p=applyOperations(p,patch(p,[{op:'node.add',value:newNode('判定','condition','detail')},{op:'view.node.update',id:'condition',changes:{position:{x:741,y:296}}},{op:'scene.add',value:{id:'deeper',label:'详细条件',description:'',ownerNodeId:'condition'}},{op:'viewport.update',id:'detail',changes:{x:124,y:-48,zoom:0.61}}])).project;
  assert.deepEqual(scenePath(p,'deeper').map(scene=>scene.id),['root','detail','deeper']);assert.deepEqual(p.view.nodes.a,before);assert.deepEqual(p.view.scenes.root.viewport,rootViewport);assert.deepEqual(p.view.scenes.detail.viewport,{x:124,y:-48,zoom:0.61});
  assert.throws(()=>applyOperations(p,patch(p,[{op:'node.update',id:'a',changes:{sceneId:'detail'}}])));
  const exported=JSON.parse(JSON.stringify(p));validateProject(exported);assert.deepEqual(exported.model.scenes,p.model.scenes);assert.deepEqual(exported.view.scenes,p.view.scenes);
});
test('scenes: connections stay local; mapped boundary ports are the parent/child contract',()=>{
  let p=withScene();p=applyOperations(p,patch(p,[{op:'node.add',value:newNode('计算','calc','detail')},{op:'port.add',value:newPort('calc','input','in','calc_in')},{op:'port.add',value:newPort('calc','output','out','calc_out')},{op:'edge.add',value:newEdge('calc_out',boundaryPortId('out'),'local')}])).project;
  assert.throws(()=>applyOperations(p,patch(p,[{op:'edge.add',value:newEdge('out','calc_in','cross-scene')}])));
  assert.throws(()=>applyOperations(p,patch(p,[{op:'group.add',value:{id:'root-group',label:'group',description:'',sceneId:'root'}},{op:'node.update',id:'calc',changes:{groupId:'root-group'}}])));
  assert.throws(()=>applyOperations(p,patch(p,[{op:'node.add',value:{...newNode('fake','fake','root'),kind:'boundary-input'}}])));
});
test('scenes: explicit cascade removes descendants and mappings but preserves parent and unrelated edges',()=>{
  let p=withScene();p=applyOperations(p,patch(p,[{op:'edge.add',value:newEdge('out','in','root-edge')},{op:'node.add',value:newNode('内层','inside','detail')},{op:'scene.add',value:{id:'nested',label:'nested',description:'',ownerNodeId:'inside'}}])).project;
  assert.throws(()=>applyOperations(p,patch(p,[{op:'scene.remove',id:'detail'}])));
  assert.throws(()=>applyOperations(p,patch(p,[{op:'node.remove',id:'a'}])));
  const next=applyOperations(p,patch(p,[{op:'scene.remove',id:'detail',cascade:true}])).project;
  assert.ok(next.model.nodes.a);assert.ok(next.model.edges['root-edge']);assert.equal(next.model.nodes.a.detailSceneId,undefined);assert.equal(next.model.scenes.nested,undefined);assert.equal(next.model.ports[boundaryPortId('out')],undefined);assert.deepEqual(Object.keys(next.model.scenes),['root']);
  const removed=applyOperations(p,patch(p,[{op:'node.remove',id:'a',cascade:true}])).project;assert.equal(removed.model.edges['root-edge'],undefined);assert.ok(removed.model.nodes.b);assert.equal(removed.model.scenes.nested,undefined);
});
test('scenes: parent port removal cascades into child connections; source ranges are checked',()=>{
  let p=withScene();p=applyOperations(p,patch(p,[{op:'node.add',value:newNode('内层','inside','detail')},{op:'port.add',value:newPort('inside','output','result','result')},{op:'edge.add',value:newEdge('result',boundaryPortId('out'),'inner-edge')}])).project;
  assert.throws(()=>applyOperations(p,patch(p,[{op:'port.remove',id:'out'}])));
  const next=applyOperations(p,patch(p,[{op:'port.remove',id:'out',cascade:true}])).project;assert.equal(next.model.edges['inner-edge'],undefined);assert.equal(next.model.ports[boundaryPortId('out')],undefined);assert.ok(next.model.scenes.detail);
  p.sourceRoots.firmware={label:'source'};p.model.nodes.a.sourceRefs=[{rootId:'firmware',path:'application/chassis/chassis.c',line:1201,endLine:1289,symbol:'ChassisTask'}];validateProject(p);assert.equal(sourceLabel(p.model.nodes.a.sourceRefs[0]),'chassis.c:1201–1289');
  p.model.nodes.a.sourceRefs[0].endLine=1199;assert.throws(()=>validateProject(p));
});
test('scenes: persisted hierarchy survives restart; undo and redo restore a whole subview transaction',async t=>{
  const dir=await temporary(t);let store=new ProjectStore(dir);await store.init();t.after(async()=>{await store.close();await dispose(dir);});
  let p=await store.create('hierarchy',graph());const tx=patch(p,[{op:'scene.add',value:{id:'detail',label:'内部',description:'',ownerNodeId:'a'}}]);
  const dry=await store.apply(p.projectId,tx,true);p=(await store.apply(p.projectId,tx)).project;assert.deepEqual(Object.keys(p.model.nodes),Object.keys(dry.project.model.nodes));
  p=(await store.undoRedo(p.projectId,'test',p.revision,'undo')).project;assert.equal(p.model.scenes.detail,undefined);p=(await store.undoRedo(p.projectId,'test',p.revision,'redo')).project;assert.ok(p.model.scenes.detail);
  await store.close();store=new ProjectStore(dir);await store.init();const loaded=await store.get(p.projectId);assert.deepEqual(loaded,p);
});

test('boards: a board has no ports or owned scene, and remains editable without changing functional nodes',()=>{
  let p=graph();const before=structuredClone(p.model.nodes.a);
  p=applyOperations(p,patch(p,[{op:'node.add',value:newBoard('阶段说明','board','root')},{op:'view.node.update',id:'board',changes:{position:{x:-600,y:200},width:640,height:720}}])).project;
  assert.deepEqual(p.model.nodes.a,before);assert.equal(Object.values(p.model.ports).some(port=>port.nodeId==='board'),false);
  assert.throws(()=>applyOperations(p,patch(p,[{op:'port.add',value:newPort('board','output','fake','fake')}])));
  assert.throws(()=>applyOperations(p,patch(p,[{op:'view.node.update',id:'board',changes:{width:200}}])));
  assert.throws(()=>applyOperations(p,patch(p,[{op:'scene.add',value:{id:'bad-scene',ownerNodeId:'board',label:'bad',description:''}}])));
  p=applyOperations(p,patch(p,[{op:'node.update',id:'board',changes:{description:'## 状态\n\n|编号|含义|\n|---|---|\n|S0|初始化|',metadata:{boardTone:'teal'}}},{op:'view.node.update',id:'board',changes:{height:900}}])).project;
  const exported=JSON.parse(JSON.stringify(p));validateProject(exported);assert.equal(exported.view.nodes.board.height,900);assert.equal(exported.model.nodes.board.metadata.boardTone,'teal');
});
test('explanations: typed explanation survives transactions and rejects malformed content without losing layout',()=>{
  let p=graph();const view=structuredClone(p.view),explanation={receives:'左右腿目标和反馈',process:'比较角度并连续计时',produces:'下一阶段',decisions:['超时→等待','两腿到位并驻留→交接']};
  p=applyOperations(p,patch(p,[{op:'node.update',id:'a',changes:{metadata:{gain:50,explanation}}}])).project;
  assert.deepEqual(explanationOf(p.model.nodes.a),explanation);assert.equal(p.model.nodes.a.metadata.gain,50);assert.deepEqual(p.view,view);
  assert.throws(()=>applyOperations(p,patch(p,[{op:'node.update',id:'a',changes:{metadata:{explanation:{receives:'only one field'}}}}])));
  assert.deepEqual(explanationOf(p.model.nodes.a),explanation);
});
test('example: every function is explained, boards are disconnected, and PD uses real leg feedback rather than raw LQR torque',async()=>{
  const p=validateProject(JSON.parse(await readFile(resolve(import.meta.dirname,'../examples/lqr.graph.json'),'utf8'))).project;
  const boards=Object.values(p.model.nodes).filter(node=>node.kind==='annotation');assert.equal(boards.length,6);
  for(const node of Object.values(p.model.nodes))if(node.kind!=='annotation'){const e=explanationOf(node);assert.ok(e?.receives&&e.process&&e.produces,node.id);}
  for(const board of boards)assert.equal(Object.values(p.model.ports).some(port=>port.nodeId===board.id),false);
  const incoming=Object.values(p.model.edges).filter(edge=>p.model.ports[edge.targetPortId].nodeId==='mode_pd');
  assert.ok(incoming.some(edge=>p.model.ports[edge.sourcePortId].dataType==='leg-feedback'));
  assert.ok(incoming.some(edge=>p.model.ports[edge.sourcePortId].dataType==='leg-goals'));
  assert.equal(incoming.some(edge=>edge.sourcePortId==='mode_pack_out'),false);
  assert.ok(Object.values(p.model.edges).some(edge=>edge.kind==='transition'));
});
test('transition lines cannot silently stand for numeric data',()=>{
  const p=graph();p.model.ports.out.dataType='float';p.model.ports.in.dataType='float';
  assert.throws(()=>applyOperations(p,patch(p,[{op:'edge.add',value:{...newEdge('out','in','numeric-transition'),kind:'transition'}}])));
});

test('appearance patches compose atomically and preserve the model and layout across export', () => {
  const base = graph();
  const original = applyOperations(base, patch(base, [
    { op:'group.add', value:{ id:'group', label:'分组', description:'', sceneId:'root' } },
  ])).project;
  const changed = applyOperations(original, patch(original, [
    { op:'view.node.update', id:'a', changes:{ appearance:{ background:'#FFEEDD', header:'#223344' } } },
    { op:'view.node.update', id:'a', changes:{ appearance:{ border:'#133557' } } },
    { op:'view.group.update', id:'group', changes:{ appearance:{ border:'#005577' } } },
  ])).project;
  assert.deepEqual(changed.model, original.model);
  assert.deepEqual(changed.view.nodes.a.position, original.view.nodes.a.position);
  assert.deepEqual(changed.view.nodes.a.appearance, { background:'#ffeedd', header:'#223344', border:'#133557' });
  assert.equal(validateProject(JSON.parse(JSON.stringify(changed))).project.view.groups.group.appearance.border,'#005577');
  assert.throws(() => applyOperations(changed, patch(changed, [
    { op:'view.node.update', id:'a', changes:{ appearance:{ border:null } } },
    { op:'view.group.update', id:'group', changes:{ appearance:{ border:'invalid' } } },
  ])));
  assert.equal(changed.view.nodes.a.appearance.border,'#133557');
  const cleared = applyOperations(changed, patch(changed, [
    { op:'view.node.update', id:'a', changes:{ appearance:{ header:null } } },
    { op:'view.group.update', id:'group', changes:{ appearance:null } },
  ])).project;
  assert.deepEqual(cleared.view.nodes.a.appearance,{ background:'#ffeedd', border:'#133557' });
  assert.equal(cleared.view.groups.group.appearance,undefined);
});
