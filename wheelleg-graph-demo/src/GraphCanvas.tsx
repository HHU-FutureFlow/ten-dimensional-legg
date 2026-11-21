import { memo, useCallback, useEffect, useMemo, useRef, useState, createContext, useContext } from 'react';
import { ReactFlow, ReactFlowProvider, Background, Controls, MiniMap, Handle, Position, BaseEdge, EdgeLabelRenderer, getBezierPath, getSmoothStepPath, getStraightPath, useNodesState, useEdgesState, useReactFlow, useUpdateNodeInternals, Panel, SelectionMode } from '@xyflow/react';
import type { Node, NodeProps, Edge, EdgeProps, Connection, Viewport } from '@xyflow/react';
import { Activity, SlidersHorizontal, Cpu, ArrowUpRight, ChevronDown, ChevronRight, Plus, Maximize, Layers, Code2, GitFork } from 'lucide-react';
import { groupContains, newEdge, sourceLabel, explanationOf } from '../shared/model.ts';
import type { Project, Port, Operation, Point, EdgeView, SourceRef, NodeExplanation, BlockAppearance } from '../shared/model.ts';
import type { Commit } from './api.ts';
import AnnotationCard from './AnnotationCard.tsx';
import ExplanationCard from './ExplanationCard.tsx';
import { appearanceAttributes } from './blockAppearance.ts';

export type Selection = { kind: 'node' | 'port' | 'edge' | 'group'; id: string; panel?: 'source' } | null;
type Actions = { select: (selection: Selection) => void; toggle: (id: string) => void; route: (id: string, points: Point[]) => void; enter: (sceneId: string) => void; resize:(id:string,position:Point,width:number,height:number,revision:number)=>void };
const ActionsContext = createContext<Actions>({ select: () => {}, toggle: () => {}, route: () => {}, enter: () => {}, resize:()=>{} });
type NodeData = { label: string; description: string; kind: string; status: string; ports: Port[]; sourceRefs?: SourceRef[]; detailSceneId?: string; explanation?:NodeExplanation; metadata?:Record<string,unknown>; revision?:number; group?: boolean; collapsed?: boolean; count?: number; inspected?: boolean; appearance?: BlockAppearance };
type FlowNode = Node<NodeData>;
type FlowEdge = Edge<{ view: EdgeView; kind: string }>;

const FunctionNode = memo(function FunctionNode({ id, data, selected }: NodeProps<FlowNode>) {
  const actions = useContext(ActionsContext);
  const updateInternals=useUpdateNodeInternals();
  useEffect(()=>{const frame=requestAnimationFrame(()=>updateInternals(id));return()=>cancelAnimationFrame(frame);},[id,data.ports,data.collapsed,updateInternals]);
  const appearance = appearanceAttributes(data.appearance);
  if(data.kind==='annotation')return <AnnotationCard id={id} label={data.label} text={data.description} tone={String(data.metadata?.boardTone||'amber')} appearance={data.appearance} selected={!!(selected||data.inspected)} revision={data.revision||0} resize={actions.resize}/>;
  if (data.group && !data.collapsed) return <div className={`group-frame ${selected || data.inspected ? 'is-selected' : ''}`} {...appearance}>
    <div className="group-header"><Layers size={15}/><span>{data.label}</span><button className="icon-button nodrag" title="收起分组" aria-label={`收起 ${data.label}`} onClick={e => { e.stopPropagation(); actions.toggle(id); }}><ChevronDown size={16}/></button></div>
  </div>;
  const inputs = data.ports.filter(p => p.direction === 'input').sort((a,b) => a.order-b.order);
  const outputs = data.ports.filter(p => p.direction === 'output').sort((a,b) => a.order-b.order);
  const Icon = data.group ? Layers : data.kind === 'decision' ? GitFork : data.kind === 'input' || data.kind === 'boundary-input' ? Activity : data.kind === 'control' ? SlidersHorizontal : data.kind === 'output' || data.kind === 'boundary-output' ? ArrowUpRight : Cpu;
  return <div className={`function-node kind-${data.kind} ${selected || data.inspected ? 'is-selected' : ''}`} {...appearance} data-testid={`graph-node-${id}`}>
    <div className="node-header"><Icon size={17}/><strong>{data.label}</strong><span className={`node-dot status-${data.status}`} title={data.status}/>{data.group && <button className="icon-button nodrag" aria-label={`展开 ${data.label}`} onClick={e => {e.stopPropagation();actions.toggle(id);}}><ChevronRight size={16}/></button>}</div>
    <div className="node-description">{data.group ? `${data.count} 个功能块 · 分组已收起` : data.description || '填写功能说明，连接输入与输出'}</div>
    {data.explanation&&<ExplanationCard value={data.explanation}/>}
    {!data.group && (data.sourceRefs?.length ? <button className="node-source nodrag" title={data.sourceRefs.map(ref=>`${ref.rootId}/${sourceLabel(ref,true)}${ref.symbol?' → '+ref.symbol:''}`).join('\n')} aria-label={`关联资料 ${data.label}`} onClick={e=>{e.stopPropagation();actions.select({kind:'node',id,panel:'source'});}}><Code2 size={12}/><span><code>{sourceLabel(data.sourceRefs[0])}{data.sourceRefs.length>1?` +${data.sourceRefs.length-1}`:''}</code>{data.sourceRefs[0].symbol&&<small>{data.sourceRefs[0].symbol}</small>}</span><ChevronRight size={11}/></button> : <div className="node-source missing"><Code2 size={12}/>{data.kind.startsWith('boundary-')?'上层接口映射':'关联资料（可选）'}</div>)}
    <div className="node-ports">{Array.from({length:Math.max(inputs.length,outputs.length,1)},(_,index)=><div className="port-row" key={index}>
      {(['input','output'] as const).map(direction=>{
        const port = (direction === 'input' ? inputs : outputs)[index];
        return <div key={direction} className={`port-slot ${direction}`}>{port && <>
          <Handle type={direction==='input'?'target':'source'} position={direction==='input'?Position.Left:Position.Right} id={port.id} data-testid={`handle-${port.id}`}/>
          <button className="port-name nodrag" title={`${port.label} ${port.unit} ${port.shape}`} onClick={e=>{e.stopPropagation();actions.select({kind:'port',id:port.id});}}>{port.label}<small>{port.unit}</small></button>
        </>}</div>;
      })}
    </div>)}</div>
    <div className="node-footer">{data.group?'GROUP':data.kind.startsWith('boundary-')?'上下层边界接口':data.status==='implemented'?'源码已实现':data.status==='design'?'设计中':'待核实'}<span>{inputs.length} 入 / {outputs.length} 出</span></div>
    {data.detailSceneId&&<button className="node-enter nodrag" aria-label={`进入子视图 ${data.label}`} onClick={e=>{e.stopPropagation();actions.enter(data.detailSceneId!);}}>进入子视图<ArrowUpRight size={13}/></button>}
  </div>;
});

function RoutedEdge(props: EdgeProps<FlowEdge>) {
  const { id, sourceX, sourceY, targetX, targetY, sourcePosition, targetPosition, selected, data, markerEnd, label } = props;
  const actions = useContext(ActionsContext); const { screenToFlowPosition } = useReactFlow();
  const [points,setPoints] = useState<Point[]>(data?.view.waypoints || []);
  const drag = useRef<{ index:number; start:Point } | null>(null);
  useEffect(()=>{setPoints(data?.view.waypoints || []);},[data?.view]);
  const positions = [{x:sourceX,y:sourceY},...points,{x:targetX,y:targetY}];
  let path=''; let center:Point={x:(sourceX+targetX)/2,y:(sourceY+targetY)/2};
  for(let i=0;i<positions.length-1;i++){
    const a=positions[i],b=positions[i+1];
    const args={sourceX:a.x,sourceY:a.y,targetX:b.x,targetY:b.y,sourcePosition:i===0?sourcePosition:Position.Right,targetPosition:i===positions.length-2?targetPosition:Position.Left};
    const [segment,cx,cy]=(data?.view.style==='straight'?getStraightPath(args):data?.view.style==='smoothstep'?getSmoothStepPath({...args,borderRadius:12}):getBezierPath({...args,curvature:0.35}));
    path+=segment+' '; if(i===Math.floor((positions.length-2)/2))center={x:cx,y:cy};
  }
  const color=data?.kind==='control'?'#b69ce7':data?.kind==='transition'?'#d7a3ba':data?.kind==='force'?'#e7b667':'#58c9c3';
  return <>
    <BaseEdge id={id} path={path} markerEnd={markerEnd} style={{stroke:color,strokeWidth:selected?2.8:1.8,strokeDasharray:data?.kind==='transition'?'7 4':undefined}} interactionWidth={22}/>
    <EdgeLabelRenderer>
      {(label || selected) && <div className={`edge-label nodrag nopan ${selected?'selected':''}`} style={{transform:`translate(-50%,-50%) translate(${center.x}px,${center.y}px)`}}>
        {label && <span>{String(label)}</span>}{selected&&<button title="添加走线点" aria-label="添加走线点" data-testid={`route-add-${id}`} onClick={()=>{const next=[...points,{x:center.x,y:center.y}];setPoints(next);actions.route(id,next);}}><Plus size={12}/></button>}
      </div>}
      {selected&&points.map((point,index)=><button key={index} className="waypoint nodrag nopan" aria-label={`走线点 ${index+1}`} title="拖动调整；右键删除" style={{transform:`translate(-50%,-50%) translate(${point.x}px,${point.y}px)`}}
        onPointerDown={e=>{e.stopPropagation();e.currentTarget.setPointerCapture(e.pointerId);drag.current={index,start:point};}}
        onPointerMove={e=>{if(drag.current?.index!==index)return;const next=screenToFlowPosition({x:e.clientX,y:e.clientY});setPoints(prev=>prev.map((p,i)=>i===index?next:p));}}
        onPointerUp={e=>{if(!drag.current)return;e.currentTarget.releasePointerCapture(e.pointerId);const start=drag.current.start;drag.current=null;if(start.x!==points[index].x||start.y!==points[index].y)actions.route(id,points);}}
        onContextMenu={e=>{e.preventDefault();const next=points.filter((_,i)=>i!==index);setPoints(next);actions.route(id,next);}}/>) }
    </EdgeLabelRenderer>
  </>;
}
const nodeTypes={function:FunctionNode}; const edgeTypes={routed:RoutedEdge};
type Props={project:Project;sceneId:string;enterScene:(sceneId:string)=>void;selection:Selection;select:(s:Selection)=>void;commit:Commit;onSelectedNodes:(ids:string[])=>void;onAppearance:(preferred?:string)=>void;snap:boolean};

function Canvas({project,sceneId,enterScene,selection,select,commit,onSelectedNodes,onAppearance,snap}:Props){
  const [nodes,setNodes,onNodesChange]=useNodesState<FlowNode>([]);const [edges,setEdges,onEdgesChange]=useEdgesState<FlowEdge>([]);
  const flow=useReactFlow<FlowNode,FlowEdge>();const current=useRef(project);current.current=project;
  const baseDrag=useRef(project.revision);
  const visibleGroup=(nodeId:string)=>{
    let group=project.model.nodes[nodeId]?.groupId;let visible:string|undefined;
    while(group){if(project.view.groups[group]?.collapsed)visible=group;group=project.model.groups[group]?.parentGroupId;}return visible;
  };
  useEffect(()=>{
    const previous=new Map(nodes.map(n=>[n.id,n]));
    const newNodes:FlowNode[]=[];
    for(const group of Object.values(project.model.groups).filter(group=>group.sceneId===sceneId)){
      let parent=group.parentGroupId,hidden=false;
      while(parent){if(project.view.groups[parent].collapsed)hidden=true;parent=project.model.groups[parent].parentGroupId;}
      if(hidden)continue;
      const view=project.view.groups[group.id];const members=Object.values(project.model.nodes).filter(n=>groupContains(project,group.id,n.id));
      const ports=Object.values(project.model.ports).filter(p=>members.some(n=>n.id===p.nodeId));
      newNodes.push({id:group.id,type:'function',position:view.position,selected:previous.get(group.id)?.selected,measured:previous.get(group.id)?.measured,zIndex:view.collapsed?0:-1,style:{width:view.collapsed?280:view.width,height:view.collapsed?undefined:view.height},data:{label:group.label,description:group.description,kind:'group',status:'design',ports,group:true,collapsed:view.collapsed,count:members.length}});
    }
    for(const node of Object.values(project.model.nodes).filter(node=>node.sceneId===sceneId))if(!visibleGroup(node.id))newNodes.push({id:node.id,type:'function',position:project.view.nodes[node.id].position,selected:previous.get(node.id)?.selected,measured:previous.get(node.id)?.measured,zIndex:node.kind==='annotation'?-1:0,style:{width:project.view.nodes[node.id].width||(node.kind==='annotation'?560:300),height:node.kind==='annotation'?(project.view.nodes[node.id].height||420):undefined},data:{...node,explanation:explanationOf(node),revision:project.revision,ports:Object.values(project.model.ports).filter(p=>p.nodeId===node.id)}});
    const newEdges:FlowEdge[]=[];
    for(const edge of Object.values(project.model.edges)){
      const sourceNode=project.model.ports[edge.sourcePortId].nodeId,targetNode=project.model.ports[edge.targetPortId].nodeId;
      if(project.model.nodes[sourceNode].sceneId!==sceneId)continue;
      const source=visibleGroup(sourceNode)||sourceNode,target=visibleGroup(targetNode)||targetNode;
      if(source===target&&source!==sourceNode)continue;
      newEdges.push({id:edge.id,source,target,sourceHandle:edge.sourcePortId,targetHandle:edge.targetPortId,type:'routed',label:edge.label,selected:edges.find(e=>e.id===edge.id)?.selected,data:{kind:edge.kind,view:project.view.edges[edge.id]||{}},markerEnd:{type:'arrowclosed' as any,color:edge.kind==='control'?'#b69ce7':edge.kind==='transition'?'#d7a3ba':edge.kind==='force'?'#e7b667':'#58c9c3'}});
    }
    setNodes(newNodes);setEdges(newEdges);
  // Rebuild only from committed state; transient drags stay local until pointer-up.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  },[project,sceneId]);
  useEffect(()=>{const timer=setTimeout(()=>flow.setViewport(project.view.scenes[sceneId].viewport),30);return()=>clearTimeout(timer);},[project.projectId,sceneId]);
  const actions=useMemo<Actions>(()=>({select,enter:enterScene,resize:(nodeId,position,width,height,revision)=>{void commit([{op:'view.node.update',id:nodeId,changes:{position,width,height}}],'调整说明板尺寸',revision);},toggle:groupId=>{void commit([{op:'view.group.update',id:groupId,changes:{collapsed:!current.current.view.groups[groupId].collapsed}}],'切换分组展开');},route:(edgeId,points)=>{void commit([{op:'view.edge.update',id:edgeId,changes:{waypoints:points}}],'调整连线路径');}}),[commit,select,enterScene]);
  const connect=useCallback((connection:Connection)=>{if(connection.sourceHandle&&connection.targetHandle)void commit([{op:'edge.add',value:newEdge(connection.sourceHandle,connection.targetHandle)}],'连接端口');},[commit]);
  const selectionChanged=useCallback(({nodes:selected}:{nodes:FlowNode[]})=>onSelectedNodes(selected.map(n=>n.id)),[onSelectedNodes]);
  const saveDrag=(_event:any,node:FlowNode,dragged:FlowNode[])=>{
    const p=current.current;const operations:Operation[]=[];const changed=new Set<string>();
    for(const moving of dragged.length?dragged:[node]){
      if(p.model.nodes[moving.id]){operations.push({op:'view.node.update',id:moving.id,changes:{position:moving.position}});changed.add(moving.id);}
      else if(p.model.groups[moving.id]){
        const old=p.view.groups[moving.id].position;const delta={x:moving.position.x-old.x,y:moving.position.y-old.y};
        operations.push({op:'view.group.update',id:moving.id,changes:{position:moving.position}});
        for(const child of Object.values(p.model.nodes))if(groupContains(p,moving.id,child.id)&&!changed.has(child.id)){
          const pos=p.view.nodes[child.id].position;operations.push({op:'view.node.update',id:child.id,changes:{position:{x:pos.x+delta.x,y:pos.y+delta.y}}});changed.add(child.id);
        }
        for(const child of Object.values(p.model.groups)){
          let parent=child.parentGroupId,inside=false;while(parent){if(parent===moving.id)inside=true;parent=p.model.groups[parent]?.parentGroupId;}
          if(inside){const pos=p.view.groups[child.id].position;operations.push({op:'view.group.update',id:child.id,changes:{position:{x:pos.x+delta.x,y:pos.y+delta.y}}});}
        }
        for(const edge of Object.values(p.model.edges))if(groupContains(p,moving.id,p.model.ports[edge.sourcePortId].nodeId)&&groupContains(p,moving.id,p.model.ports[edge.targetPortId].nodeId)){
          const points=p.view.edges[edge.id]?.waypoints;if(points?.length)operations.push({op:'view.edge.update',id:edge.id,changes:{waypoints:points.map(pt=>({x:pt.x+delta.x,y:pt.y+delta.y}))}});
        }
      }
    }
    if(operations.length)void commit(operations,'移动功能块/分组',baseDrag.current);
  };
  const moveEnd=(_event:any,viewport:Viewport)=>{
    const old=current.current.view.scenes[sceneId]?.viewport;
    if(old&&(Math.abs(old.x-viewport.x)>1||Math.abs(old.y-viewport.y)>1||Math.abs(old.zoom-viewport.zoom)>0.005))void commit([{op:'viewport.update',id:sceneId,changes:viewport}],'保存当前视图视角');
  };
  const inspectedId=selection?.kind==='port'?project.model.ports[selection.id]?.nodeId:selection?.kind==='node'||selection?.kind==='group'?selection.id:undefined;
  const renderedNodes=useMemo(()=>nodes.map(node=>({...node,
    zIndex:node.data.kind==='annotation'&&node.id===inspectedId?2:node.zIndex,
    data:{...node.data,inspected:node.id===inspectedId,
      appearance:node.data.group?project.view.groups[node.id]?.appearance:project.view.nodes[node.id]?.appearance},
  })),[nodes,inspectedId,project.view]);
  return <ActionsContext.Provider value={actions}><ReactFlow<FlowNode,FlowEdge> nodes={renderedNodes} edges={edges} nodeTypes={nodeTypes} edgeTypes={edgeTypes}
    onNodesChange={onNodesChange} onEdgesChange={onEdgesChange} onConnect={connect}
    onNodeClick={(_,node)=>select({kind:project.model.groups[node.id]?'group':'node',id:node.id})} onEdgeClick={(_,edge)=>select({kind:'edge',id:edge.id})} onPaneClick={()=>select(null)}
    onNodeContextMenu={(event,node)=>{event.preventDefault();onAppearance(node.id);}}
    onSelectionContextMenu={(event,nodes)=>{event.preventDefault();onAppearance(nodes[0]?.id);}}
    onNodeDoubleClick={(_,node)=>{if(node.data.detailSceneId)enterScene(node.data.detailSceneId);else void flow.fitView({nodes:[{id:node.id}],minZoom:0.85,maxZoom:1,padding:0.5,duration:200});}}
    onNodeDragStart={()=>{baseDrag.current=current.current.revision;}} onNodeDragStop={saveDrag} onMoveEnd={moveEnd}
    onSelectionChange={selectionChanged}
    onNodesDelete={()=>{}} deleteKeyCode={null} zoomOnDoubleClick={false} selectionMode={SelectionMode.Partial} selectionKeyCode="Shift" multiSelectionKeyCode="Control"
    snapToGrid={snap} snapGrid={[20,20]} minZoom={0.15} maxZoom={2} onlyRenderVisibleElements={false} proOptions={{hideAttribution:true}}>
    <Background color="#303a43" gap={22} size={1}/><Controls showInteractive={false}/><MiniMap<FlowNode> nodeColor={n=>n.data.appearance?.header||n.data.appearance?.background||(n.data.kind==='control'?'#766386':'#477775')} maskColor="rgba(13,18,23,.65)" pannable zoomable/>
    <Panel position="top-right"><button className="canvas-fit" onClick={()=>void flow.fitView({padding:0.12,duration:250})}><Maximize size={14}/>适应画布</button></Panel>
  </ReactFlow></ActionsContext.Provider>;
}
export default function GraphCanvas(props:Props){return <ReactFlowProvider><Canvas {...props}/></ReactFlowProvider>;}
