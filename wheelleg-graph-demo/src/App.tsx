import { useCallback, useEffect, useRef, useState } from 'react';
import { Activity, ArrowDownToLine, ArrowUpFromLine, NotebookPen, Check, ChevronRight, Circle, FolderOpen, FolderPlus, GitBranch, Grid2X2, Layers, Loader2, Palette, Plus, Redo2, RefreshCw, Undo2, Users, X } from 'lucide-react';
import GraphCanvas from './GraphCanvas.tsx';
import type { Selection } from './GraphCanvas.tsx';
import Inspector from './Inspector.tsx';
import { api, eventStream } from './api.ts';
import type { Commit, History, ProjectSummary } from './api.ts';
import { id, newNode, newPort, newBoard, scenePath, EDITOR_CAPABILITIES } from '../shared/model.ts';
import type { Project, Operation, AppearancePatch, BlockAppearance } from '../shared/model.ts';
import AppearanceDialog from './AppearanceDialog.tsx';
import { defaultAppearance } from './blockAppearance.ts';

type AppearanceTarget = { id: string; kind: 'node' | 'group'; label: string; appearance: BlockAppearance; defaults: Required<BlockAppearance> };
type AppearanceEdit = { projectId: string; revision: number; targets: AppearanceTarget[] };

function getActor(){const old=sessionStorage.getItem('graph-actor');if(old)return old;const value=id('browser').slice(0,22);sessionStorage.setItem('graph-actor',value);return value;}
function ensureV2Service(status:{schemaVersion?:number;capabilities?:string[]}){if(status.schemaVersion!==2||!EDITOR_CAPABILITIES.every(capability=>status.capabilities?.includes(capability)))throw new Error('服务版本过旧：请关闭原启动窗口，重新运行 scripts/start.cmd 后刷新页面。');}
export default function App(){
  const [projects,setProjects]=useState<ProjectSummary[]>([]),[project,setProject]=useState<Project|null>(null),[selection,setSelection]=useState<Selection>(null);
  const [history,setHistory]=useState<History>({undoActor:null,redoActor:null});const [busy,setBusy]=useState(false),[error,setError]=useState(''),[notice,setNotice]=useState('');
  const [online,setOnline]=useState(false),[auth,setAuth]=useState(false),[token,setToken]=useState(''),[ready,setReady]=useState(false),[snap,setSnap]=useState(false),[selectedIds,setSelectedIds]=useState<string[]>([]);
  const [actor]=useState(getActor),[newDialog,setNewDialog]=useState(false),[newTitle,setNewTitle]=useState('我的工程地图');
  const [appearanceEdit,setAppearanceEdit]=useState<AppearanceEdit|null>(null);
  const [activeScene,setActiveScene]=useState('root');const sceneRef=useRef(activeScene);sceneRef.current=activeScene;
  const projectRef=useRef(project),pending=useRef(false),remotePending=useRef(false),fileInput=useRef<HTMLInputElement>(null),alive=useRef(true);
  projectRef.current=project;
  const enterScene=useCallback((sceneId:string)=>{
    const p=projectRef.current;if(!p?.model.scenes[sceneId])return;
    sceneRef.current=sceneId;setActiveScene(sceneId);setSelection(null);setSelectedIds([]);localStorage.setItem(`graph-scene-${p.projectId}`,sceneId);
  },[]);
  const selectAt=useCallback((target:Selection)=>{
    const p=projectRef.current;
    if(target&&p){
      let sceneId:string|undefined;
      if(target.kind==='node')sceneId=p.model.nodes[target.id]?.sceneId;
      if(target.kind==='group')sceneId=p.model.groups[target.id]?.sceneId;
      if(target.kind==='port')sceneId=p.model.nodes[p.model.ports[target.id]?.nodeId]?.sceneId;
      if(target.kind==='edge'){const port=p.model.ports[p.model.edges[target.id]?.sourcePortId];sceneId=port&&p.model.nodes[port.nodeId].sceneId;}
      if(sceneId&&sceneId!==sceneRef.current)enterScene(sceneId);
    }
    setSelection(target);
  },[enterScene]);
  const list=useCallback(async()=>{const data=await api('/projects');setProjects(data.projects);return data.projects as ProjectSummary[];},[]);
  const load=useCallback(async(projectId:string)=>{
    const result=await api(`/projects/${projectId}`);
    if(projectRef.current?.projectId!==projectId){const remembered=localStorage.getItem(`graph-scene-${projectId}`);const target=remembered&&result.project.model.scenes[remembered]?remembered:result.project.rootSceneId;sceneRef.current=target;setActiveScene(target);setSelection(null);setSelectedIds([]);}
    if(!projectRef.current||projectRef.current.projectId!==projectId||result.project.revision>=projectRef.current.revision){projectRef.current=result.project;setProject(result.project);setHistory(result.history||{undoActor:null,redoActor:null});}
    return result.project as Project;
  },[]);
  useEffect(()=>{if(project&&!project.model.scenes[activeScene])enterScene(project.rootSceneId);},[project,activeScene,enterScene]);
  const create=useCallback(async(title:string,template?:string,imported?:unknown)=>{
    if(pending.current)return;pending.current=true;setError('');setBusy(true);
    try{const data=await api('/projects','POST',{title,template,project:imported});await list();await load(data.project.projectId);localStorage.setItem('graph-last-project',data.project.projectId);}
    catch(e:any){setError(e.message);}finally{pending.current=false;setBusy(false);}
  },[list,load]);
  const initialize=useCallback(async()=>{
    const status=await api('/status');
    ensureV2Service(status);
    if(status.authRequired&&!sessionStorage.getItem('graph-token')){setAuth(true);return;}
    const items=await list();setAuth(false);setReady(true);
    const last=localStorage.getItem('graph-last-project');
    if(items.length)await load(items.some(p=>p.projectId===last)?last!:items[0].projectId);
    else await create('空白工程');
  },[list,load,create]);
  useEffect(()=>{alive.current=true;let active=true;void (async()=>{try{const status=await api('/status');if(!active)return;ensureV2Service(status);if(status.authRequired&&!sessionStorage.getItem('graph-token')){setAuth(true);return;}const items=await list();if(!active)return;setReady(true);const last=localStorage.getItem('graph-last-project');if(items.length)await load(items.some(p=>p.projectId===last)?last!:items[0].projectId);else await create('空白工程');}catch(e:any){if(active){if(e.code==='UNAUTHORIZED')setAuth(true);setError(e.message);}}})();return()=>{active=false;alive.current=false;};},[list,load,create]);
  useEffect(()=>{
    if(!ready||auth)return;
    const controller=new AbortController();let wasOnline=false;
    void eventStream(controller.signal,event=>{
      const current=projectRef.current;
      if(current?.projectId===event.projectId&&event.revision>current.revision){
        if(pending.current){remotePending.current=true;return;}
        void load(current.projectId).then(()=>{if(event.actor!==actor)setNotice(`已同步 ${event.actor} 的修改：${event.summary}`);}).catch((e:any)=>setError(e.message));
      }
    },connected=>{setOnline(connected);if(connected&&!wasOnline&&projectRef.current)void load(projectRef.current.projectId).catch((e:any)=>setError(e.message));wasOnline=connected;});
    return()=>controller.abort();
  },[ready,auth,load,actor]);
  const commit=useCallback<Commit>(async(operations,summary,baseRevision)=>{
    const current=projectRef.current;if(!current)return false;
    if(pending.current){setError('上一笔修改仍在保存，请稍后重试；正在编辑的属性草稿会保留。');return false;}
    pending.current=true;setBusy(true);setError('');
    try{
      const result=await api(`/projects/${current.projectId}/transactions`,'POST',{requestId:id('tx'),baseRevision:baseRevision??current.revision,actor,summary,operations});
      if(projectRef.current?.projectId===current.projectId){projectRef.current=result.project;setProject(result.project);setHistory(result.history);setNotice(result.warnings?.join('；')||summary+' · 已保存');}
      setProjects(prev=>prev.map(item=>item.projectId===current.projectId?{...item,title:result.project.title,revision:result.project.revision,nodes:Object.keys(result.project.model.nodes).length}:item));
      return true;
    }catch(e:any){setError((e.code?`${e.code} · `:'')+e.message);if(e.code==='REVISION_CONFLICT')await load(current.projectId).catch(()=>{});else setProject(p=>p?structuredClone(p):p);return false;}
    finally{pending.current=false;if(alive.current)setBusy(false);if(remotePending.current){remotePending.current=false;void load(current.projectId).catch((e:any)=>setError(e.message));}}
  },[actor,load]);
  const addNode=async()=>{
    if(!project)return;const n=newNode('新功能块',id('node'),activeScene);const pIn=newPort(n.id,'input'),pOut=newPort(n.id,'output');
    const count=Object.values(project.model.nodes).filter(node=>node.sceneId===activeScene).length;const viewport=project.view.scenes[activeScene].viewport;
    const position={x:Math.round((70-viewport.x)/viewport.zoom)+(count%2)*310,y:Math.round((90-viewport.y)/viewport.zoom)+Math.floor((count%4)/2)*250};
    if(await commit([{op:'node.add',value:n},{op:'view.node.update',id:n.id,changes:{position}},{op:'port.add',value:pIn},{op:'port.add',value:pOut}],'新增功能块'))setSelection({kind:'node',id:n.id});
  };
  const addBoard=async()=>{
    if(!project)return;const board=newBoard('说明板',id('board'),activeScene),viewport=project.view.scenes[activeScene].viewport;
    if(await commit([{op:'node.add',value:board},{op:'view.node.update',id:board.id,changes:{position:{x:Math.round((60-viewport.x)/viewport.zoom),y:Math.round((340-viewport.y)/viewport.zoom)},width:560,height:420}}],'新增说明板'))setSelection({kind:'node',id:board.id});
  };
  const copyNode=async(nodeId:string)=>{
    const p=projectRef.current;if(!p)return;const old=p.model.nodes[nodeId];const n={...structuredClone(old),id:id('node'),label:old.label+' 副本'};delete n.detailSceneId;const pos=p.view.nodes[nodeId].position;
    const operations:Operation[]=[{op:'node.add',value:n},{op:'view.node.update',id:n.id,changes:{...p.view.nodes[nodeId],position:{x:pos.x+50,y:pos.y+50}}}];
    for(const port of Object.values(p.model.ports).filter(port=>port.nodeId===nodeId))operations.push({op:'port.add',value:{...port,id:id('port'),nodeId:n.id}});
    if(await commit(operations,'复制功能块'))setSelection({kind:'node',id:n.id});
  };
  const groupSelected=async()=>{
    if(!project)return;const members=selectedIds.filter(key=>project.model.nodes[key]?.sceneId===activeScene);if(!members.length){setError('请 Ctrl 多选或 Shift 框选需要分组的功能块。');return;}
    const positions=members.map(key=>project.view.nodes[key].position),x=Math.min(...positions.map(p=>p.x))-35,y=Math.min(...positions.map(p=>p.y))-60;
    const group={id:id('group'),label:'功能分组',description:'',sceneId:activeScene};const width=Math.max(360,...members.map(key=>project.view.nodes[key].position.x+(project.view.nodes[key].width||250)-x+35));
    const height=Math.max(300,...members.map(key=>project.view.nodes[key].position.y-y+220+Math.max(...['input','output'].map(direction=>Object.values(project.model.ports).filter(port=>port.nodeId===key&&port.direction===direction).length))*32));
    if(await commit([{op:'group.add',value:group},{op:'view.group.update',id:group.id,changes:{position:{x,y},width,height}},...members.map(key=>({op:'node.update',id:key,changes:{groupId:group.id}}))],'封装功能分组'))setSelection({kind:'group',id:group.id});
  };
  const exportProject=async()=>{
    if(!project)return;
    try{const data=await api(`/projects/${project.projectId}/export`);const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)+'\n'],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=data.title.replace(/[<>:"/\\|?*]/g,'_')+'.graph.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);setNotice('已导出项目 JSON，可发送给队友导入。');}catch(e:any){setError(e.message);}
  };
  const importProject=async(file?:File)=>{if(!file)return;try{const data=JSON.parse(await file.text());await create(data.title||'导入工程',undefined,data);}catch(e:any){setError('导入失败：'+e.message);}finally{if(fileInput.current)fileInput.current.value='';}};
  const undoRedo=async(action:'undo'|'redo')=>{
    if(!project||pending.current)return;pending.current=true;setBusy(true);setError('');
    try{const result=await api(`/projects/${project.projectId}/${action}`,'POST',{actor,baseRevision:project.revision});projectRef.current=result.project;setProject(result.project);setHistory(result.history);setNotice(action==='undo'?'已撤销':'已重做');}catch(e:any){setError(e.message);}finally{pending.current=false;setBusy(false);if(remotePending.current){remotePending.current=false;void load(project.projectId).catch((e:any)=>setError(e.message));}}
  };
  const createDetail=async(nodeId:string)=>{
    const node=projectRef.current?.model.nodes[nodeId];if(!node)return;
    if(node.detailSceneId){enterScene(node.detailSceneId);return;}
    const sceneId=id('scene');
    if(await commit([{op:'scene.add',value:{id:sceneId,label:node.label,description:'',ownerNodeId:nodeId}}],`创建 ${node.label} 的子视图`))enterScene(sceneId);
  };
  const currentSceneId=project?.model.scenes[activeScene]?activeScene:project?.rootSceneId||'root';
  const activeNodes=project?Object.values(project.model.nodes).filter(node=>node.sceneId===currentSceneId):[];
  const activeGroups=project?Object.values(project.model.groups).filter(group=>group.sceneId===currentSceneId):[];
  const breadcrumbs=project?scenePath(project,currentSceneId):[];
  const focusedBlock=selection?.kind==='node'||selection?.kind==='group'?selection.id:undefined;
  const canEditAppearance=!!focusedBlock||selectedIds.length>0;

  function openAppearance(preferred = focusedBlock) {
    if (!project || busy) return;
    const ids = preferred && !selectedIds.includes(preferred) ? [preferred] : selectedIds;
    const targets: AppearanceTarget[] = [];
    for (const id of ids) {
      const node = project.model.nodes[id];
      const group = project.model.groups[id];
      if (node?.sceneId === currentSceneId) {
        targets.push({id,kind:'node',label:node.label,appearance:project.view.nodes[id].appearance || {},
          defaults:defaultAppearance(node.kind,String(node.metadata.boardTone || 'amber'))});
      } else if (group?.sceneId === currentSceneId) {
        targets.push({id,kind:'group',label:group.label,appearance:project.view.groups[id].appearance || {},
          defaults:defaultAppearance('group')});
      }
    }
    if (!targets.length) return;
    setError('');
    setAppearanceEdit({projectId:project.projectId,revision:project.revision,targets});
  }

  async function applyAppearance(appearance: AppearancePatch) {
    if (!appearanceEdit || projectRef.current?.projectId !== appearanceEdit.projectId) return;
    const operations = appearanceEdit.targets.map(target => ({
      op:`view.${target.kind}.update`,id:target.id,changes:{appearance},
    }));
    const description = appearance === null ? '恢复默认配色' : '调整块外观';
    if (await commit(operations,description,appearanceEdit.revision)) setAppearanceEdit(null);
  }
  if(auth)return <div className="access-screen"><div className="access-card"><Layers size={30}/><span className="eyebrow">PROJECT GRAPH</span><h1>连接团队工程地图</h1><p>输入服务管理员提供的共享访问令牌。</p><form onSubmit={async e=>{e.preventDefault();sessionStorage.setItem('graph-token',token);try{await initialize();setError('');}catch(e:any){sessionStorage.removeItem('graph-token');setError(e.message);}}}><input type="password" aria-label="共享访问令牌" value={token} onChange={e=>setToken(e.target.value)} placeholder="共享访问令牌" autoComplete="current-password"/><button className="button primary" type="submit">连接服务</button></form>{error&&<div className="form-error" role="alert">{error}</div>}</div></div>;
  return <div className="app-shell">
    <header className="app-header"><div className="brand"><span className="brand-symbol"><Layers size={24}/></span><div><strong>工程地图</strong><span>PROJECT GRAPH</span></div></div><span className="header-divider"/><div className="project-crumb"><FolderOpen size={15}/><span>{project?.title||'正在连接服务'}</span><ChevronRight size={14}/><span className="muted">{project?.model.scenes[currentSceneId]?.label||'信号与功能'}</span></div><div className="header-right"><span className={`connection-badge ${online?'online':''}`}><Circle size={7} fill="currentColor"/>{online?'服务已连接':'连接中'}</span><span className="version-pill">DEMO 0.4</span></div></header>
    <div className="app-body">
      <aside className="sidebar"><div className="sidebar-section-title">工作空间<button className="icon-button" aria-label="新建工程" onClick={()=>setNewDialog(true)}><FolderPlus size={15}/></button></div>
        <select className="project-select" aria-label="选择工程" value={project?.projectId||''} disabled={busy} onChange={e=>{localStorage.setItem('graph-last-project',e.target.value);void load(e.target.value).catch((e:any)=>setError(e.message));}}>{projects.map(item=><option key={item.projectId} value={item.projectId}>{item.title}</option>)}</select>
        <div className="project-actions"><button className="button" onClick={()=>fileInput.current?.click()} disabled={busy}><ArrowUpFromLine size={14}/>导入 JSON</button><button className="button" onClick={()=>void exportProject()} disabled={!project||busy}><ArrowDownToLine size={14}/>导出共享</button></div><input ref={fileInput} type="file" accept=".json,application/json" hidden aria-label="导入项目文件" onChange={e=>void importProject(e.target.files?.[0])}/>
        <div className="sidebar-section-title top-gap">当前视图<span>{activeNodes.length}</span></div>
        <nav className="node-navigation">{activeNodes.filter(node=>node.kind==='annotation').map(node=><button key={node.id} className={`board-nav ${selection?.id===node.id?'active':''}`} onClick={()=>setSelection({kind:'node',id:node.id})}><NotebookPen size={14}/><span>{node.label}</span></button>)}{activeGroups.map(group=><button key={group.id} className={selection?.id===group.id?'active':''} onClick={()=>setSelection({kind:'group',id:group.id})}><Layers size={14}/><span>{group.label}</span></button>)}{activeNodes.filter(node=>node.kind!=='annotation').map(node=><button key={node.id} className={selection?.id===node.id?'active':''} onClick={()=>setSelection({kind:'node',id:node.id})}><GitBranch size={14}/><span>{node.label}</span>{node.detailSceneId&&<ChevronRight size={12}/>}</button>)}{project&&!activeNodes.length&&<p className="nav-empty">从一个功能块开始<br/>逐步整理你的工程</p>}</nav>
        <div className="sidebar-bottom"><button className="example-button" disabled={busy} onClick={()=>void create('十维 LQR · 输入输出示例','lqr')}><Activity size={17}/><span>打开 LQR 示例<small>按实际源码手工梳理</small></span><ChevronRight size={14}/></button><div className="local-note"><Users size={13}/><span>人和 Agent 共用同一份图</span></div></div>
      </aside>
      <main className="workspace"><div className="toolbar"><div className="view-title"><GitBranch size={16}/><strong>信号流</strong><span className="toolbar-tag">可编辑</span></div><div className="toolbar-actions"><button className="icon-button" title="撤销自己的最新操作" aria-label="撤销" disabled={busy||history.undoActor!==actor} onClick={()=>void undoRedo('undo')}><Undo2 size={16}/></button><button className="icon-button" title="重做" aria-label="重做" disabled={busy||history.redoActor!==actor} onClick={()=>void undoRedo('redo')}><Redo2 size={16}/></button><span className="toolbar-separator"/><button className={`button compact ${snap?'toggled':''}`} onClick={()=>setSnap(!snap)} title="网格吸附"><Grid2X2 size={14}/><span>吸附</span></button><button className="button compact" onClick={()=>void groupSelected()} disabled={!project||busy}><Layers size={14}/><span>分组</span></button><button className="button compact appearance-button" aria-label="外观" title="设置所选块的颜色" onClick={()=>openAppearance()} disabled={!canEditAppearance||busy}><Palette size={15}/><span>外观</span></button><button className="button compact board-add" aria-label="添加说明板" onClick={()=>void addBoard()} disabled={!project||busy}><NotebookPen size={15}/><span>说明板</span></button><button className="button primary compact" onClick={()=>void addNode()} disabled={!project||busy}><Plus size={15}/>添加节点</button></div></div>
        {project&&<nav className="scene-breadcrumbs" aria-label="视图导航">{breadcrumbs.map((scene,index)=><span key={scene.id}>{index>0&&<ChevronRight size={12}/>}<button aria-label={`进入视图 ${scene.label}`} aria-current={scene.id===currentSceneId?'page':undefined} onClick={()=>enterScene(scene.id)}>{scene.label}</button></span>)}<small>{currentSceneId===project.rootSceneId?'总览':'独立子视图 · 输入输出映射到上层'}</small></nav>}
        {error&&<div className="message-bar error" role="alert"><span>{error}</span><button onClick={()=>{void (project?load(project.projectId):initialize()).then(()=>setError('')).catch((e:any)=>setError(e.message));}}><RefreshCw size={13}/>重新读取</button><button aria-label="关闭错误提示" onClick={()=>setError('')}><X size={14}/></button></div>}
        <div className="canvas-area" data-scene-id={currentSceneId}>{project&&<GraphCanvas key={`${project.projectId}:${currentSceneId}`} project={project} sceneId={currentSceneId} enterScene={enterScene} selection={selection} select={setSelection} commit={commit} onSelectedNodes={setSelectedIds} onAppearance={openAppearance} snap={snap}/>} {!project&&<div className="loading-state"><Loader2 className="spin"/>连接工程服务…</div>}
          {project&&!activeNodes.length&&<div className="blank-state"><div className="blank-mark"><GitBranch size={34}/></div><span className="eyebrow">YOUR ENGINEERING MAP</span><h1>把工程，从这里连接起来</h1><p>用功能块描述模块，用端口和连线记录信号关系。<br/>你和 agent 可以一起维护这张图。</p><button className="button primary" onClick={()=>void addNode()} disabled={busy}><Plus size={16}/>创建第一个功能块</button><button className="text-button" onClick={()=>void create('十维 LQR · 输入输出示例','lqr')}>先看看 LQR 示例 →</button></div>}
        </div><div className="canvas-footer"><span className="legend-item"><i className="legend-line data"/>数据</span><span className="legend-item"><i className="legend-line control"/>判定</span><span className="legend-item"><i className="legend-line transition"/>状态转移</span><span className="legend-item"><i className="legend-line force"/>力/力矩</span><span className="footer-hint">说明板不连线 · 双击无子视图节点可放大阅读</span></div>
      </main>{project&&<Inspector project={project} selection={selection} select={selectAt} commit={commit} copyNode={copyNode} createDetail={createDetail} enterScene={enterScene}/>}
    </div>
    <footer className="status-bar"><span className="save-status">{busy?<Loader2 size={13} className="spin"/>:<Check size={13}/>} {busy?'正在保存…':project?'已保存到服务数据目录':'等待连接'}</span><span className="status-notice" aria-live="polite">{notice||'图中内容由维护者编辑；源码关联为人工记录。'}</span><span className="status-actor">{actor}</span><code>rev {project?.revision??'—'}</code></footer>
    {appearanceEdit && appearanceEdit.projectId === project?.projectId &&
      <AppearanceDialog count={appearanceEdit.targets.length} label={appearanceEdit.targets[0].label}
        initial={appearanceEdit.targets.map(target=>target.appearance)} defaults={appearanceEdit.targets[0].defaults}
        error={error} busy={busy} onClose={()=>setAppearanceEdit(null)} onApply={appearance=>void applyAppearance(appearance)}/>}
    {newDialog&&<div className="dialog-backdrop" onClick={()=>setNewDialog(false)}><form className="dialog" onClick={e=>e.stopPropagation()} onSubmit={e=>{e.preventDefault();setNewDialog(false);void create(newTitle);}}><FolderPlus size={24}/><h2>新建空白工程</h2><FieldLabel/><input aria-label="工程名称" value={newTitle} onChange={e=>setNewTitle(e.target.value)} required autoFocus/><div className="button-row"><button type="button" className="button" onClick={()=>setNewDialog(false)}>取消</button><button type="submit" className="button primary">创建工程</button></div></form></div>}
  </div>;
}
function FieldLabel(){return <label className="muted">工程名称</label>;}
