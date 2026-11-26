import { useEffect, useState } from 'react';
import { ArrowDownToLine, ArrowUpFromLine, ArrowUpRight, Check, Code2, Copy, Info, Plus, Trash2, X } from 'lucide-react';
import type { Project, GraphNode, Port, GraphEdge, Group, Operation } from '../shared/model.ts';
import { newPort, sourceLabel, explanationOf } from '../shared/model.ts';
import type { Selection } from './GraphCanvas.tsx';
import type { Commit } from './api.ts';
import { BoardMarkdown } from './AnnotationCard.tsx';

type Props = { project:Project;selection:Selection;select:(s:Selection)=>void;commit:Commit;copyNode:(id:string)=>void;createDetail:(id:string)=>void;enterScene:(id:string)=>void };
function Field({label,children,hint}:{label:string;children:React.ReactNode;hint?:string}){return <label className="field"><span>{label}</span>{children}{hint&&<small>{hint}</small>}</label>;}
function SourceCopy({text}:{text:string}){
  const [label,setLabel]=useState('复制定位');
  const copy=async()=>{try{if(navigator.clipboard?.writeText)await navigator.clipboard.writeText(text);else{const field=document.createElement('textarea');field.value=text;field.style.position='fixed';field.style.opacity='0';document.body.append(field);field.select();const ok=document.execCommand('copy');field.remove();if(!ok)throw new Error('unavailable');}setLabel('已复制');}catch{setLabel('请手动复制上方路径');}};
  return <button type="button" className="button" onClick={()=>void copy()}><Copy size={13}/>{label}</button>;
}
function useDraft<T>(value:T,revision:number){
  const [draft,setDraft]=useState(value),[dirty,setDirty]=useState(false),[base,setBase]=useState(revision);
  useEffect(()=>{if(!dirty){setDraft(value);setBase(revision);}},[revision,dirty]);
  return {draft,base,dirty,edit:(next:T)=>{setDraft(next);setDirty(true);},reset:()=>{setDraft(value);setBase(revision);setDirty(false);},saved:()=>setDirty(false)};
}
function DraftBar({dirty,base,revision,reset}:{dirty:boolean;base:number;revision:number;reset:()=>void}){
  return <>{dirty&&<div className={`draft-notice ${base!==revision?'stale':''}`}>{base!==revision?'图已更新；草稿仍保留，提交会检查冲突':'有未提交的修改'}<button onClick={reset} type="button">丢弃草稿并重读</button></div>}</>;
}
function NodeEditor({project,node,select,commit,copyNode,createDetail,enterScene,sourceRequest}:Omit<Props,'selection'> & {node:GraphNode;sourceRequest?:Selection}){
  const explanation=explanationOf(node)||{receives:'',process:'',produces:'',decisions:[]};
  const extraMetadata=Object.fromEntries(Object.entries(node.metadata).filter(([key])=>key!=='explanation'));
  const [tab,setTab]=useState(sourceRequest?'source':'properties');const state=useDraft({label:node.label,description:node.description,kind:node.kind,status:node.status,groupId:node.groupId||'',receives:explanation.receives,process:explanation.process,produces:explanation.produces,decisions:explanation.decisions.join('\n'),metadata:JSON.stringify(extraMetadata,null,2),sources:JSON.stringify(node.sourceRefs,null,2),width:project.view.nodes[node.id].width||300},project.revision);
  useEffect(()=>{if(sourceRequest)setTab('source');},[sourceRequest]);
  const boundary=node.kind.startsWith('boundary-');
  const {draft,edit}=state;const [error,setError]=useState('');
  const ports=Object.values(project.model.ports).filter(p=>p.nodeId===node.id).sort((a,b)=>a.order-b.order);
  const save=async(e:React.FormEvent)=>{
    e.preventDefault();setError('');
    try{
      const metadata=JSON.parse(draft.metadata),sourceRefs=JSON.parse(draft.sources);
      if(!metadata||typeof metadata!=='object'||Array.isArray(metadata))throw new Error('进阶参数须为 JSON 对象');
      if(Object.hasOwn(metadata,'explanation'))throw new Error('请通过接收/处理/输出/判定表单编辑说明，勿在进阶 JSON 重复填写 explanation。');
      if(draft.receives||draft.process||draft.produces||draft.decisions)metadata.explanation={receives:draft.receives,process:draft.process,produces:draft.produces,decisions:draft.decisions.split('\n').map(s=>s.trim()).filter(Boolean)};
      if(!Array.isArray(sourceRefs))throw new Error('源码关联须为 JSON 数组');
      const sourceRoots={...project.sourceRoots};for(const source of sourceRefs)if(typeof source.rootId==='string'&&!sourceRoots[source.rootId])sourceRoots[source.rootId]={label:source.rootId};
      const operations:Operation[]=[{op:'project.update',changes:{sourceRoots}},{op:'node.update',id:node.id,changes:{label:draft.label,description:draft.description,kind:draft.kind,status:draft.status,groupId:draft.groupId||null,metadata,sourceRefs}},{op:'view.node.update',id:node.id,changes:{width:Number(draft.width)}}];
      if(await commit(operations,`编辑功能块：${draft.label}`,state.base))state.saved();
    }catch(e:any){setError(e.message);}
  };
  const addPort=async(direction:'input'|'output')=>{
    const port={...newPort(node.id,direction),order:ports.filter(p=>p.direction===direction).length};
    if(await commit([{op:'port.add',value:port}],'新增信号端口'))select({kind:'port',id:port.id});
  };
  return <>
    <div className="inspector-title"><span className="eyebrow">功能节点</span><h2>{node.label}</h2><code>{node.id}</code></div>
    {!boundary&&<div className="subview-actions"><button className="button subview-button" disabled={state.dirty} onClick={()=>node.detailSceneId?enterScene(node.detailSceneId):createDetail(node.id)}><ArrowUpRight size={15}/>{node.detailSceneId?'进入子视图':'创建子视图'}</button>{node.detailSceneId&&<small>内部节点、接口映射和布局一起保存、导出。</small>}</div>}
    <div className="detail-tabs">{[['properties','属性'],['ports','接口'],['source','关联资料']].map(([key,label])=><button key={key} className={tab===key?'active':''} onClick={()=>setTab(key)}>{label}</button>)}</div>
    <form onSubmit={save} className="detail-form">
      <DraftBar dirty={state.dirty} base={state.base} revision={project.revision} reset={state.reset}/>
      {tab==='properties'&&<>
        <Field label="节点名称"><input aria-label="节点名称" value={draft.label} onChange={e=>edit({...draft,label:e.target.value})} required/></Field>
        <Field label="一句话概述"><textarea aria-label="功能说明" rows={2} value={draft.description} onChange={e=>edit({...draft,description:e.target.value})}/></Field>
        <div className="section-heading"><h3>把功能讲清楚</h3></div>
        <Field label="接收什么" hint="写清来源、物理含义与必要单位。"><textarea aria-label="接收什么" rows={3} value={draft.receives} onChange={e=>edit({...draft,receives:e.target.value})}/></Field>
        <Field label="怎么处理" hint="解释处理步骤和作用，避免只复述函数或参数名称。"><textarea aria-label="怎么处理" rows={4} value={draft.process} onChange={e=>edit({...draft,process:e.target.value})}/></Field>
        <Field label="输出什么"><textarea aria-label="输出什么" rows={3} value={draft.produces} onChange={e=>edit({...draft,produces:e.target.value})}/></Field>
        <Field label="判定与结果" hint="每行一条，按优先级写：什么条件 → 什么结果；无分支可留空。"><textarea aria-label="判定与结果" rows={5} value={draft.decisions} onChange={e=>edit({...draft,decisions:e.target.value})}/></Field>
        <details className="advanced-fields"><summary>进阶类型、实现状态和参数</summary>
        <div className="field-pair"><Field label="节点类型"><select disabled={boundary} value={draft.kind} onChange={e=>edit({...draft,kind:e.target.value})}>{['algorithm','input','control','output','note','decision',...(boundary?[node.kind]:[])].map(k=><option key={k}>{k}</option>)}</select></Field><Field label="实现状态"><select value={draft.status} onChange={e=>edit({...draft,status:e.target.value as GraphNode['status']})}><option value="design">设计中</option><option value="implemented">源码已实现</option><option value="unverified">待核实</option></select></Field></div>
        <Field label="参数 / 公式 / 验证记录（JSON）" hint="结构化说明通过上方表单编辑。"><textarea className="code-input" aria-label="节点参数 JSON" rows={7} value={draft.metadata} onChange={e=>edit({...draft,metadata:e.target.value})}/></Field>
        </details>
        <Field label="所属分组"><select value={draft.groupId} onChange={e=>edit({...draft,groupId:e.target.value})}><option value="">未分组</option>{Object.values(project.model.groups).filter(g=>g.sceneId===node.sceneId).map(g=><option key={g.id} value={g.id}>{g.label}</option>)}</select></Field>
        <Field label="节点宽度"><input type="number" min={180} max={2000} value={draft.width} onChange={e=>edit({...draft,width:Number(e.target.value)})}/></Field>
      </>}
      {tab==='ports'&&<>
        <div className="section-heading"><h3>输入与输出</h3><span>{ports.length} 个端口</span></div>
        {ports.map(port=><button type="button" className="port-list-item" key={port.id} onClick={()=>select({kind:'port',id:port.id})}>{port.direction==='input'?<ArrowDownToLine size={14}/>:<ArrowUpFromLine size={14}/>}<span>{port.label}<small>{[port.dataType,port.shape,port.unit].filter(Boolean).join(' · ')||'接口信息待填写'}</small></span><code>{port.direction==='input'?'IN':'OUT'}</code></button>)}
        {!boundary&&<div className="button-row"><button type="button" className="button" onClick={()=>void addPort('input')}><Plus size={14}/>输入端口</button><button type="button" className="button" onClick={()=>void addPort('output')}><Plus size={14}/>输出端口</button></div>}
      </>}
      {tab==='source'&&<>
        <div className="inline-note"><Code2 size={16}/><span>代码、原理图和接口手册都可用逻辑根与项目相对路径记录，队友无需相同磁盘目录。</span></div>
        {node.sourceRefs.map((ref,i)=><div className="source-card" key={i}><code>{sourceLabel(ref,true)}</code><span>{ref.symbol}</span><small>{ref.rootId}{ref.revision?` · ${ref.revision}`:''}</small><SourceCopy text={`${sourceLabel(ref,true)}${ref.symbol?' → '+ref.symbol:''}`}/></div>)}
        <Field label="文件 / 资料关联（JSON 数组）" hint='示例：[{"rootId":"firmware","path":"src/control.c","symbol":"calc"}]'><textarea className="code-input" rows={10} aria-label="源码关联 JSON" value={draft.sources} onChange={e=>edit({...draft,sources:e.target.value})}/></Field>
      </>}
      {error&&<div className="form-error" role="alert">{error}</div>}
      {tab!=='ports'&&<button type="submit" className="button primary" disabled={!state.dirty}><Check size={15}/>保存节点修改</button>}
    </form>
    {!boundary&&<><div className="detail-actions"><button className="button" title="复制当前节点和端口，不复制内部子视图" onClick={()=>copyNode(node.id)}><Copy size={14}/>复制功能块</button><button className="button danger" onClick={()=>{if(window.confirm('删除此功能块及其所有端口、关联连线和内部子视图？'))void commit([{op:'node.remove',id:node.id,cascade:true}],'删除功能块').then(ok=>ok&&select(null));}}><Trash2 size={14}/>删除</button></div>{node.detailSceneId&&<div className="detail-actions"><button className="button danger" onClick={()=>{if(window.confirm('删除整个内部子视图及其节点和连接？上层功能块保留。'))void commit([{op:'scene.remove',id:node.detailSceneId!,cascade:true}],'删除内部子视图');}}>删除子视图</button></div>}</>}
  </>;
}
function BoardEditor({project,node,commit,select,copyNode}:Omit<Props,'selection'> & {node:GraphNode}){
  const view=project.view.nodes[node.id];const state=useDraft({label:node.label,text:node.description,tone:String(node.metadata.boardTone||'amber'),width:view.width||560,height:view.height||420,groupId:node.groupId||''},project.revision);const{draft,edit}=state;
  return <><div className="inspector-title"><span className="eyebrow">说明板 · 不接收或输出信号</span><h2>{node.label}</h2><code>{node.id}</code></div>
    <form className="detail-form" onSubmit={async e=>{e.preventDefault();if(await commit([{op:'node.update',id:node.id,changes:{label:draft.label,description:draft.text,groupId:draft.groupId||null,metadata:{...node.metadata,boardTone:draft.tone}}},{op:'view.node.update',id:node.id,changes:{width:Number(draft.width),height:Number(draft.height)}}],'编辑说明板',state.base))state.saved();}}>
      <DraftBar {...state} revision={project.revision}/>
      <Field label="说明板标题"><input aria-label="说明板标题" value={draft.label} onChange={e=>edit({...draft,label:e.target.value})} required/></Field>
      <Field label="说明内容（Markdown）" hint="支持标题、段落、列表、表格和行内代码。"><textarea aria-label="说明内容" className="code-input" rows={14} value={draft.text} onChange={e=>edit({...draft,text:e.target.value})}/></Field>
      <Field label="默认展板颜色" hint="使用工具栏“外观”可覆盖背景、标题栏和边框。"><select aria-label="展板颜色" value={draft.tone} onChange={e=>edit({...draft,tone:e.target.value})}><option value="amber">暖黄</option><option value="teal">青绿</option><option value="slate">灰蓝</option></select></Field>
      <div className="field-pair"><Field label="宽度"><input type="number" min={280} max={2000} aria-label="说明板宽度" value={draft.width} onChange={e=>edit({...draft,width:Number(e.target.value)})}/></Field><Field label="高度"><input type="number" min={180} max={4000} aria-label="说明板高度" value={draft.height} onChange={e=>edit({...draft,height:Number(e.target.value)})}/></Field></div>
      <Field label="所属分组"><select value={draft.groupId} onChange={e=>edit({...draft,groupId:e.target.value})}><option value="">未分组</option>{Object.values(project.model.groups).filter(g=>g.sceneId===node.sceneId).map(g=><option key={g.id} value={g.id}>{g.label}</option>)}</select></Field>
      <button className="button primary" disabled={!state.dirty} type="submit"><Check size={14}/>保存说明板</button>
      <details className="board-preview advanced-fields"><summary>预览正文</summary><BoardMarkdown text={draft.text}/></details>
    </form><div className="detail-actions"><button className="button" onClick={()=>copyNode(node.id)}><Copy size={14}/>复制说明板</button><button className="button danger" onClick={()=>{if(window.confirm('删除这块说明板？'))void commit([{op:'node.remove',id:node.id}],'删除说明板').then(ok=>ok&&select(null));}}><Trash2 size={14}/>删除说明板</button></div></>;
}
function PortEditor({project,port,commit,select}:Omit<Props,'selection'> & {port:Port}){
  const state=useDraft({...port},project.revision);const {draft,edit}=state;
  return <><div className="inspector-title"><span className="eyebrow">{port.direction==='input'?'输入':'输出'}端口</span><h2>{port.label}</h2><code>{port.id}</code></div>
    <button className="back-link" onClick={()=>select({kind:'node',id:port.nodeId})}>← {project.model.nodes[port.nodeId].label}</button>
    {port.parentPortId&&<div className="boundary-notice"><span>名称、方向和类型跟随上层接口。</span><button className="button" onClick={()=>select({kind:'port',id:port.parentPortId!})}>转到上层端口 ↗</button></div>}
    <form className="detail-form" onSubmit={async e=>{e.preventDefault();const{id,...changes}=draft;if(await commit([{op:'port.update',id:port.id,changes}],`编辑端口：${draft.label}`,state.base))state.saved();}}>
      <DraftBar {...state} revision={project.revision}/>
      <fieldset className="mapped-fields" disabled={!!port.parentPortId}>
      <Field label="端口名称"><input aria-label="端口名称" required value={draft.label} onChange={e=>edit({...draft,label:e.target.value})}/></Field>
      <Field label="方向"><select value={draft.direction} onChange={e=>edit({...draft,direction:e.target.value as Port['direction']})}><option value="input">输入</option><option value="output">输出</option></select></Field>
      <div className="field-pair"><Field label="数据类型"><input value={draft.dataType} onChange={e=>edit({...draft,dataType:e.target.value})} placeholder="float / bool"/></Field><Field label="维度"><input value={draft.shape} onChange={e=>edit({...draft,shape:e.target.value})} placeholder="10 / 4x10"/></Field></div>
      <div className="field-pair"><Field label="单位"><input value={draft.unit} onChange={e=>edit({...draft,unit:e.target.value})} placeholder="N·m"/></Field><Field label="坐标系"><input value={draft.coordinateFrame} onChange={e=>edit({...draft,coordinateFrame:e.target.value})} placeholder="机体系"/></Field></div>
      </fieldset>
      <div className="field-pair"><Field label="显示顺序"><input type="number" value={draft.order} onChange={e=>edit({...draft,order:Number(e.target.value)})}/></Field><Field label="输入连接数"><select value={draft.multiplicity} onChange={e=>edit({...draft,multiplicity:e.target.value as Port['multiplicity']})}><option value="one">一条</option><option value="many">多条</option></select></Field></div>
      <Field label="端口说明"><textarea rows={4} value={draft.description} onChange={e=>edit({...draft,description:e.target.value})}/></Field>
      <button className="button primary" type="submit" disabled={!state.dirty}><Check size={14}/>保存端口修改</button>
    </form>{!port.parentPortId&&<div className="detail-actions"><button className="button danger" onClick={()=>{if(window.confirm('删除该端口及其关联连线和边界映射？'))void commit([{op:'port.remove',id:port.id,cascade:true}],'删除端口').then(ok=>ok&&select({kind:'node',id:port.nodeId}));}}><Trash2 size={14}/>删除端口</button></div>}</>;
}
function EdgeEditor({project,edge,commit,select}:Omit<Props,'selection'> & {edge:GraphEdge}){
  const state=useDraft({...edge,style:project.view.edges[edge.id]?.style||'bezier'},project.revision);const {draft,edit}=state;
  const source=project.model.ports[edge.sourcePortId],target=project.model.ports[edge.targetPortId];
  return <><div className="inspector-title"><span className="eyebrow">信号连接</span><h2>{edge.label||'未命名连线'}</h2><code>{edge.id}</code></div>
    <div className="connection-detail"><span>{project.model.nodes[source.nodeId].label} · {source.label}</span><span>↓</span><span>{project.model.nodes[target.nodeId].label} · {target.label}</span></div>
    <form className="detail-form" onSubmit={async e=>{e.preventDefault();const{style,id,...changes}=draft;if(await commit([{op:'edge.update',id:edge.id,changes},{op:'view.edge.update',id:edge.id,changes:{style}}],'编辑信号连接',state.base))state.saved();}}>
      <DraftBar {...state} revision={project.revision}/>
      <Field label="信号名称"><input aria-label="信号名称" value={draft.label} onChange={e=>edit({...draft,label:e.target.value})}/></Field>
      <Field label="连接类别"><select value={draft.kind} onChange={e=>edit({...draft,kind:e.target.value as GraphEdge['kind']})}><option value="data">数据传递</option><option value="control">条件判定结果</option><option value="transition">状态转移（虚线）</option><option value="force">力 / 力矩通道</option></select></Field>
      <Field label="连线样式"><select aria-label="连线样式" value={draft.style} onChange={e=>edit({...draft,style:e.target.value as typeof draft.style})}><option value="bezier">贝塞尔曲线</option><option value="smoothstep">圆角折线</option><option value="straight">直线</option></select></Field>
      <Field label="生效条件"><input value={draft.condition} onChange={e=>edit({...draft,condition:e.target.value})} placeholder="例如：正常平衡阶段"/></Field>
      <Field label="连接说明"><textarea rows={4} value={draft.description} onChange={e=>edit({...draft,description:e.target.value})}/></Field>
      <button className="button primary" type="submit" disabled={!state.dirty}><Check size={14}/>保存连线修改</button>
    </form><div className="detail-actions vertical"><button className="button" onClick={()=>void commit([{op:'view.edge.update',id:edge.id,changes:{waypoints:[]}}],'恢复默认走线')}>恢复默认走线</button><button className="button danger" onClick={()=>void commit([{op:'edge.remove',id:edge.id}],'删除连线').then(ok=>ok&&select(null))}><Trash2 size={14}/>删除连线</button></div></>;
}
function GroupEditor({project,group,commit,select}:Omit<Props,'selection'> & {group:Group}){
  const state=useDraft({...group,...project.view.groups[group.id]},project.revision);const {draft,edit}=state;
  return <><div className="inspector-title"><span className="eyebrow">功能分组</span><h2>{group.label}</h2><code>{group.id}</code></div>
    <form className="detail-form" onSubmit={async e=>{e.preventDefault();if(await commit([{op:'group.update',id:group.id,changes:{label:draft.label,description:draft.description}},{op:'view.group.update',id:group.id,changes:{width:draft.width,height:draft.height,collapsed:draft.collapsed}}],'编辑分组',state.base))state.saved();}}>
      <DraftBar {...state} revision={project.revision}/>
      <Field label="分组名称"><input aria-label="分组名称" value={draft.label} onChange={e=>edit({...draft,label:e.target.value})}/></Field>
      <Field label="分组说明"><textarea rows={4} value={draft.description} onChange={e=>edit({...draft,description:e.target.value})}/></Field>
      <div className="field-pair"><Field label="宽度"><input type="number" min={240} value={draft.width} onChange={e=>edit({...draft,width:Number(e.target.value)})}/></Field><Field label="高度"><input type="number" min={120} value={draft.height} onChange={e=>edit({...draft,height:Number(e.target.value)})}/></Field></div>
      <label className="check-label"><input type="checkbox" checked={draft.collapsed} onChange={e=>edit({...draft,collapsed:e.target.checked})}/>收起分组</label>
      <button type="submit" className="button primary" disabled={!state.dirty}><Check size={14}/>保存分组修改</button>
    </form><div className="detail-actions"><button className="button danger" onClick={()=>void commit([{op:'group.remove',id:group.id}],'取消分组').then(ok=>ok&&select(null))}>取消分组，保留节点</button></div></>;
}
export default function Inspector(props:Props){
  const {project,selection}=props;
  let content;
  if(selection?.kind==='node'&&project.model.nodes[selection.id])content=project.model.nodes[selection.id].kind==='annotation'?<BoardEditor key={selection.id} {...props} node={project.model.nodes[selection.id]}/>:<NodeEditor key={selection.id} {...props} node={project.model.nodes[selection.id]} sourceRequest={selection.panel==='source'?selection:undefined}/>;
  else if(selection?.kind==='port'&&project.model.ports[selection.id])content=<PortEditor key={selection.id} {...props} port={project.model.ports[selection.id]}/>;
  else if(selection?.kind==='edge'&&project.model.edges[selection.id])content=<EdgeEditor key={selection.id} {...props} edge={project.model.edges[selection.id]}/>;
  else if(selection?.kind==='group'&&project.model.groups[selection.id])content=<GroupEditor key={selection.id} {...props} group={project.model.groups[selection.id]}/>;
  else content=<div className="inspector-empty"><Info size={26}/><h2>让关系清晰可见</h2><p>选择一个功能块、端口或连线，查看并编辑详细信息。</p><div className="empty-shortcuts"><span>拖动功能块</span><span>输出 → 输入连线</span><span>Ctrl 多选 · Shift 框选</span></div></div>;
  return <aside className="inspector"><div className="inspector-bar"><Info size={16}/>详情面板<button className="icon-button" aria-label="取消选择" onClick={()=>props.select(null)}><X size={15}/></button></div><div className="inspector-scroll">{content}</div></aside>;
}
