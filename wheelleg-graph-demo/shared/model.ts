import Ajv from 'ajv';
import projectSchema from '../schema/project.schema.json' with { type: 'json' };
import patchSchema from '../schema/patch.schema.json' with { type: 'json' };

export type Point = { x: number; y: number };
export type SourceRef = { rootId: string; path: string; symbol?: string; line?: number; endLine?: number; revision?: string };
export type NodeExplanation = { receives: string; process: string; produces: string; decisions: string[] };
export const EDITOR_CAPABILITIES = ['nested-scenes', 'annotation-boards', 'node-explanations', 'state-transitions', 'block-appearance'];
export type GraphNode = { id: string; label: string; description: string; kind: string; status: 'implemented' | 'design' | 'unverified'; sourceRefs: SourceRef[]; metadata: Record<string, unknown>; sceneId: string; detailSceneId?: string; groupId?: string };
export type Port = { id: string; nodeId: string; direction: 'input' | 'output'; label: string; dataType: string; unit: string; shape: string; coordinateFrame: string; order: number; description: string; multiplicity: 'one' | 'many'; parentPortId?: string };
export type GraphEdge = { id: string; sourcePortId: string; targetPortId: string; label: string; kind: 'data' | 'control' | 'transition' | 'force'; condition: string; description: string };
export type Group = { id: string; label: string; description: string; sceneId: string; parentGroupId?: string };
export type Scene = { id: string; label: string; description: string; ownerNodeId?: string };
export type SceneView = { viewport: Point & { zoom: number } };
export const APPEARANCE_FIELDS = ['background', 'header', 'border'] as const;
export type BlockAppearance = Partial<Record<(typeof APPEARANCE_FIELDS)[number], string>>;
export type AppearancePatch = Partial<Record<(typeof APPEARANCE_FIELDS)[number], string | null>> | null;
export const HEX_COLOR = /^#[0-9a-f]{6}$/i;
export type NodeView = { position: Point; width?: number; height?: number; appearance?: BlockAppearance };
export type EdgeView = { style?: 'bezier' | 'smoothstep' | 'straight'; waypoints?: Point[] };
export type GroupView = { position: Point; width: number; height: number; collapsed: boolean; appearance?: BlockAppearance };
export type Project = { schemaVersion: 2; projectId: string; title: string; revision: number; rootSceneId: string; sourceRoots: Record<string, { label: string }>; model: { nodes: Record<string, GraphNode>; ports: Record<string, Port>; edges: Record<string, GraphEdge>; groups: Record<string, Group>; scenes: Record<string, Scene> }; view: { nodes: Record<string, NodeView>; edges: Record<string, EdgeView>; groups: Record<string, GroupView>; scenes: Record<string, SceneView> } };
export type Operation = { op: string; id?: string; value?: Record<string, unknown>; changes?: Record<string, unknown>; cascade?: boolean };
export type Patch = { requestId: string; baseRevision: number; actor: string; summary: string; operations: Operation[] };
export class GraphError extends Error {
  code: string; status: number; details: unknown;
  constructor(code: string, message: string, status = 400, details?: unknown) { super(message); this.code = code; this.status = status; this.details = details; }
}
const ajv = new Ajv({ allErrors: true, strict: false });
const checkProject = ajv.compile(projectSchema);
const checkPatch = ajv.compile(patchSchema);
export const id = (prefix = 'n') => {
  if (globalThis.crypto.randomUUID) return `${prefix}_${globalThis.crypto.randomUUID()}`;
  // randomUUID needs a secure browser context; LAN HTTP can still use getRandomValues.
  const bytes = globalThis.crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 15) | 64; bytes[8] = (bytes[8] & 63) | 128;
  const hex = Array.from(bytes, byte => byte.toString(16).padStart(2, '0')).join('');
  return `${prefix}_${hex.slice(0,8)}-${hex.slice(8,12)}-${hex.slice(12,16)}-${hex.slice(16,20)}-${hex.slice(20)}`;
};
function fail(message: string): never { throw new GraphError('INVALID_GRAPH', message, 422); }

function updateAppearance(current: BlockAppearance | undefined, patch: unknown): BlockAppearance {
  if (patch === null) return {};
  if (!patch || typeof patch !== 'object' || Array.isArray(patch)) fail('外观修改必须是对象或 null');
  const next = { ...current };
  for (const [key, value] of Object.entries(patch)) {
    const field = APPEARANCE_FIELDS.find(field => field === key);
    if (!field) fail(`未知外观字段：${key}`);
    if (value === null) delete next[field];
    else if (typeof value === 'string' && HEX_COLOR.test(value)) next[field] = value.toLowerCase();
    else fail(`颜色须使用 #RRGGBB：${key}`);
  }
  return next;
}

export function createProject(title = '未命名工程', projectId = id('project')): Project {
  return { schemaVersion: 2, projectId, title, revision: 0, rootSceneId: 'root', sourceRoots: {}, model: { nodes: {}, ports: {}, edges: {}, groups: {}, scenes: { root: { id: 'root', label: '工程总览', description: '' } } }, view: { nodes: {}, edges: {}, groups: {}, scenes: { root: { viewport: { x: 0, y: 0, zoom: 1 } } } } };
}
export function newNode(label = '新功能块', nodeId = id('node'), sceneId = 'root'): GraphNode {
  return { id: nodeId, label, description: '', kind: 'algorithm', status: 'design', sourceRefs: [], metadata: {}, sceneId };
}
export function newBoard(label = '说明板', nodeId = id('board'), sceneId = 'root'): GraphNode {
  return { ...newNode(label, nodeId, sceneId), kind: 'annotation', description: '## 在这里解释这张图\n\n- 名词和符号是什么意思？\n- 从哪个节点开始阅读？\n- 哪些条件会改变处理结果？', metadata: { boardTone: 'amber' } };
}
export function explanationOf(node: GraphNode): NodeExplanation | undefined {
  const value = node.metadata.explanation as NodeExplanation | undefined;
  return value && typeof value.receives === 'string' && typeof value.process === 'string' && typeof value.produces === 'string' && Array.isArray(value.decisions) ? value : undefined;
}
export function newPort(nodeId: string, direction: 'input' | 'output', label = direction === 'input' ? 'input' : 'output', portId = id('port')): Port {
  return { id: portId, nodeId, direction, label, dataType: '', unit: '', shape: '', coordinateFrame: '', order: 0, description: '', multiplicity: direction === 'input' ? 'one' : 'many' };
}
export function newEdge(sourcePortId: string, targetPortId: string, edgeId = id('edge')): GraphEdge {
  return { id: edgeId, sourcePortId, targetPortId, label: '', kind: 'data', condition: '', description: '' };
}
export function isSafePath(path: string) {
  return !!path && !path.includes('\\') && !path.startsWith('/') && !/^[a-z]+:/i.test(path) && !path.split('/').some(p => p === '..' || p === '.') && !path.includes('\0');
}
function rejectUnsafeKeys(value: unknown): void {
  if (!value || typeof value !== 'object') return;
  for (const [k, v] of Object.entries(value)) {
    if (['__proto__', 'constructor', 'prototype'].includes(k)) fail(`不允许的字段：${k}`);
    rejectUnsafeKeys(v);
  }
}
export function validateProject(value: unknown): { project: Project; warnings: string[] } {
  rejectUnsafeKeys(value);
  if ((value as any)?.schemaVersion !== 2) throw new GraphError('SCHEMA_UNSUPPORTED', '本版只接受 v2 工程图；请打开新版示例或使用旧版本工具', 422);
  if (!checkProject(value)) throw new GraphError('SCHEMA_INVALID', '项目格式不符合 v2 Schema', 422, checkProject.errors);
  const p = value as Project;
  const warnings: string[] = [];
  const ids = new Set<string>();
  for (const collection of Object.values(p.model)) for (const [key, entity] of Object.entries(collection)) {
    if (key !== entity.id || ids.has(key)) fail(`ID 不一致或重复：${key}`);
    ids.add(key);
  }
  if (!p.model.scenes[p.rootSceneId] || p.model.scenes[p.rootSceneId].ownerNodeId) fail('工程必须有一个不属于节点的根视图');
  for (const scene of Object.values(p.model.scenes)) {
    if (!p.view.scenes[scene.id]) fail(`缺少视图布局：${scene.id}`);
    if (scene.id === p.rootSceneId) continue;
    const owner = scene.ownerNodeId && p.model.nodes[scene.ownerNodeId];
    if (!owner || owner.detailSceneId !== scene.id) fail(`子视图必须与所属节点相互关联：${scene.id}`);
    const seen = new Set([scene.id]); let ancestor: string = owner.sceneId;
    while (ancestor !== p.rootSceneId) {
      if (seen.has(ancestor)) fail(`子视图层级形成循环：${scene.id}`);
      seen.add(ancestor);
      const parent = p.model.scenes[ancestor];
      if (!parent?.ownerNodeId || !p.model.nodes[parent.ownerNodeId]) fail(`视图不连接到工程总览：${scene.id}`);
      ancestor = p.model.nodes[parent.ownerNodeId].sceneId;
    }
  }
  for (const node of Object.values(p.model.nodes)) {
    if (!p.view.nodes[node.id]) fail(`缺少节点布局：${node.id}`);
    if (!p.model.scenes[node.sceneId]) fail(`节点视图不存在：${node.id}`);
    if (node.kind === 'annotation' && node.detailSceneId) fail('说明板不包含功能子视图');
    if (node.kind === 'annotation' && ((p.view.nodes[node.id].width ?? 560) < 280 || (p.view.nodes[node.id].height ?? 420) < 180)) fail('说明板最小尺寸为 280×180');
    if (node.kind.startsWith('boundary-') && (!p.model.scenes[node.sceneId].ownerNodeId || node.detailSceneId || !['boundary-input', 'boundary-output'].includes(node.kind))) fail(`边界节点必须是子视图内的接口节点：${node.id}`);
    if (node.detailSceneId && p.model.scenes[node.detailSceneId]?.ownerNodeId !== node.id) fail(`节点详情视图关联无效：${node.id}`);
    if (node.groupId && !p.model.groups[node.groupId]) fail(`不存在的节点分组：${node.groupId}`);
    if (node.groupId && p.model.groups[node.groupId].sceneId !== node.sceneId) fail(`节点与分组必须位于同一视图：${node.id}`);
    for (const source of node.sourceRefs) {
      if (!p.sourceRoots[source.rootId] || !isSafePath(source.path)) fail(`源码引用须使用已声明的 rootId 和相对路径：${node.id}`);
      if (source.endLine && source.line && source.endLine < source.line) fail(`源码行号范围倒置：${node.id}`);
    }
  }
  for (const group of Object.values(p.model.groups)) {
    if (!p.view.groups[group.id]) fail(`缺少分组布局：${group.id}`);
    if (!p.model.scenes[group.sceneId]) fail(`分组视图不存在：${group.id}`);
    const seen = new Set([group.id]); let parent = group.parentGroupId;
    while (parent) {
      if (!p.model.groups[parent] || seen.has(parent) || p.model.groups[parent].sceneId !== group.sceneId) fail(`分组引用不存在、跨视图或形成循环：${group.id}`);
      seen.add(parent); parent = p.model.groups[parent].parentGroupId;
    }
  }
  const mapped = new Set<string>();
  for (const port of Object.values(p.model.ports)) {
    const node = p.model.nodes[port.nodeId];
    if (!node) fail(`端口节点不存在：${port.id}`);
    if (node.kind === 'annotation') fail(`说明板没有输入输出端口：${node.id}`);
    if (node.kind.startsWith('boundary-') && !port.parentPortId) fail(`边界端口必须映射上层接口：${port.id}`);
    if (port.parentPortId) {
      const parent = p.model.ports[port.parentPortId];
      const scene = p.model.scenes[node.sceneId];
      if (!parent || parent.nodeId !== scene.ownerNodeId || mapped.has(parent.id)) fail(`边界接口映射无效或重复：${port.id}`);
      const expectedKind = parent.direction === 'input' ? 'boundary-input' : 'boundary-output';
      if (node.kind !== expectedKind || port.direction === parent.direction) fail(`边界接口方向必须与上层端口相反：${port.id}`);
      for (const key of ['label', 'dataType', 'unit', 'shape', 'coordinateFrame'] as const) if (port[key] !== parent[key]) fail(`边界接口应跟随上层 ${key}：${port.id}`);
      mapped.add(parent.id);
    }
  }
  for (const node of Object.values(p.model.nodes)) if (node.detailSceneId) for (const port of Object.values(p.model.ports).filter(v => v.nodeId === node.id)) if (!mapped.has(port.id)) fail(`上层端口缺少子视图边界映射：${port.id}`);
  const inputs = new Map<string, number>(); const connections = new Set<string>();
  for (const edge of Object.values(p.model.edges)) {
    const source = p.model.ports[edge.sourcePortId]; const target = p.model.ports[edge.targetPortId];
    if (!source || !target || source.direction !== 'output' || target.direction !== 'input') fail(`连线必须从有效输出连接到输入：${edge.id}`);
    if (edge.kind === 'transition' && [source, target].some(port => port.dataType && port.dataType !== 'bool')) fail(`状态转移应使用判定/事件端口，不是数值数据通道：${edge.id}`);
    if (p.model.nodes[source.nodeId].sceneId !== p.model.nodes[target.nodeId].sceneId) fail(`跨视图连接必须经过边界映射：${edge.id}`);
    const pair = `${source.id}:${target.id}`;
    if (connections.has(pair)) fail(`重复连接：${edge.id}`);
    connections.add(pair);
    const count = (inputs.get(target.id) || 0) + 1; inputs.set(target.id, count);
    if (count > 1 && target.multiplicity === 'one') fail(`输入端口只允许一条连接：${target.label}`);
    for (const key of ['dataType', 'shape'] as const) if (source[key] && target[key] && source[key] !== target[key]) fail(`${edge.id} 的 ${key} 不兼容：${source[key]} → ${target[key]}`);
    for (const key of ['unit', 'coordinateFrame'] as const) if (source[key] && target[key] && source[key] !== target[key]) warnings.push(`${edge.id} 的 ${key} 不一致，请人工确认`);
  }
  for (const key of Object.keys(p.view.nodes)) if (!p.model.nodes[key]) fail(`布局引用不存在的节点：${key}`);
  for (const key of Object.keys(p.view.groups)) if (!p.model.groups[key]) fail(`布局引用不存在的分组：${key}`);
  for (const key of Object.keys(p.view.edges)) if (!p.model.edges[key]) fail(`布局引用不存在的连线：${key}`);
  for (const key of Object.keys(p.view.scenes)) if (!p.model.scenes[key]) fail(`布局引用不存在的视图：${key}`);
  return { project: p, warnings };
}
export function validatePatch(value: unknown): Patch {
  rejectUnsafeKeys(value);
  if (!checkPatch(value)) throw new GraphError('INVALID_PATCH', '事务格式错误', 422, checkPatch.errors);
  return value as Patch;
}

function derivedId(prefix: string, key: string): string {
  const value = `${prefix}_${key}`;
  if (value.length <= 100) return value;
  let hash = 2166136261;
  for (const char of key) hash = Math.imul(hash ^ char.charCodeAt(0), 16777619);
  return `${value.slice(0, 85)}_${(hash >>> 0).toString(16)}`;
}
export const boundaryPortId = (parentPortId: string) => derivedId('binding', parentPortId);
export function scenePath(p: Project, sceneId: string): Scene[] {
  const path: Scene[] = []; let scene: Scene | undefined = p.model.scenes[sceneId];
  while (scene) { path.unshift(scene); scene = scene.ownerNodeId ? p.model.scenes[p.model.nodes[scene.ownerNodeId].sceneId] : undefined; }
  return path;
}
export function sourceLabel(ref: SourceRef, full = false): string {
  const path = full ? ref.path : ref.path.split('/').at(-1);
  return `${path}${ref.line ? ':' + ref.line : ''}${ref.endLine && ref.endLine !== ref.line ? '–' + ref.endLine : ''}`;
}

function syncSceneInterfaces(p: Project) {
  for (const scene of Object.values(p.model.scenes)) {
    if (!scene.ownerNodeId || !p.model.nodes[scene.ownerNodeId]) continue;
    const boundaries: Record<string, GraphNode> = {};
    for (const direction of ['input', 'output'] as const) {
      const kind = `boundary-${direction}`;
      let node = Object.values(p.model.nodes).find(n => n.sceneId === scene.id && n.kind === kind);
      if (!node) {
        const nodeId = derivedId(`boundary_${direction}`, scene.id);
        if (Object.values(p.model).some(collection => Object.hasOwn(collection, nodeId))) fail(`边界节点 ID 冲突：${nodeId}`);
        node = { ...newNode(direction === 'input' ? '来自上层的输入' : '返回上层的输出', nodeId, scene.id), kind, description: '接口映射到上层功能块；在上层维护接口定义。' };
        p.model.nodes[nodeId] = node;
        p.view.nodes[nodeId] = { position: { x: direction === 'input' ? 0 : 1600, y: 220 }, width: 250 };
      }
      boundaries[direction] = node;
    }
    for (const parent of Object.values(p.model.ports).filter(port => port.nodeId === scene.ownerNodeId)) {
      const existing = Object.values(p.model.ports).find(port => port.parentPortId === parent.id);
      const portId = existing?.id || boundaryPortId(parent.id);
      if (!existing && Object.values(p.model).some(collection => Object.hasOwn(collection, portId))) fail(`边界端口 ID 冲突：${portId}`);
      p.model.ports[portId] = {
        ...(existing || newPort(boundaries[parent.direction].id, parent.direction === 'input' ? 'output' : 'input', parent.label, portId)),
        nodeId: boundaries[parent.direction].id, direction: parent.direction === 'input' ? 'output' : 'input',
        parentPortId: parent.id, label: parent.label, dataType: parent.dataType, shape: parent.shape, unit: parent.unit, coordinateFrame: parent.coordinateFrame,
      };
    }
  }
}

export function applyOperations(project: Project, patch: Patch): { project: Project; warnings: string[] } {
  validatePatch(patch);
  if (patch.baseRevision !== project.revision) throw new GraphError('REVISION_CONFLICT', '项目已被其他编辑更新，请重新读取后提交', 409, { currentRevision: project.revision });
  const p = structuredClone(project);
  const removeEdge = (edgeId: string) => { delete p.model.edges[edgeId]; delete p.view.edges[edgeId]; };
  const removePort = (portId: string, cascade = false, internal = false) => {
    if (p.model.ports[portId]?.parentPortId && !internal) fail('边界端口由上层维护，请删除或修改上层接口');
    const edges = Object.values(p.model.edges).filter(e => e.sourcePortId === portId || e.targetPortId === portId);
    const bindings = Object.values(p.model.ports).filter(port => port.parentPortId === portId);
    if ((edges.length || bindings.length) && !cascade) fail(`端口仍有关联连线或子视图映射，需显式 cascade：${portId}`);
    bindings.forEach(port => removePort(port.id, true, true));
    edges.forEach(e => removeEdge(e.id)); delete p.model.ports[portId];
  };
  const deletingScenes = new Set<string>();
  const removeScene = (sceneId: string, cascade = false) => {
    if (sceneId === p.rootSceneId) fail('不能删除工程总览');
    if (!cascade) fail('删除子视图需显式 cascade，以确认移除其全部内容');
    if (deletingScenes.has(sceneId)) fail('删除路径中的视图层级形成循环');
    deletingScenes.add(sceneId);
    const scene = p.model.scenes[sceneId];
    for (const node of Object.values(p.model.nodes).filter(n => n.sceneId === sceneId)) removeNode(node.id, true, true);
    for (const group of Object.values(p.model.groups).filter(g => g.sceneId === sceneId)) { delete p.model.groups[group.id]; delete p.view.groups[group.id]; }
    if (scene?.ownerNodeId && p.model.nodes[scene.ownerNodeId]) delete p.model.nodes[scene.ownerNodeId].detailSceneId;
    delete p.model.scenes[sceneId]; delete p.view.scenes[sceneId];
    deletingScenes.delete(sceneId);
  };
  const removeNode = (nodeId: string, cascade = false, internal = false) => {
    const node = p.model.nodes[nodeId];
    if (node.kind.startsWith('boundary-') && !internal) fail('边界节点由子视图接口维护，不能单独删除');
    if (node.detailSceneId) {
      if (!cascade) fail('该节点包含子视图，删除需显式 cascade');
      removeScene(node.detailSceneId, true);
    }
    Object.values(p.model.ports).filter(port => port.nodeId === nodeId).forEach(port => removePort(port.id, cascade, internal));
    delete p.model.nodes[nodeId]; delete p.view.nodes[nodeId];
  };
  for (const op of patch.operations) {
    const changes = op.changes || {};
    if (op.op === 'project.update') {
      if (Object.keys(changes).some(k => !['title', 'sourceRoots'].includes(k))) fail('只能修改项目标题和源码根标签');
      Object.assign(p, changes); continue;
    }
    if (op.op === 'viewport.update') {
      const sceneId = op.id || p.rootSceneId;
      if (!p.view.scenes[sceneId]) fail(`视图不存在：${sceneId}`);
      Object.assign(p.view.scenes[sceneId].viewport, changes); continue;
    }
    const parts = op.op.split('.'); const isView = parts[0] === 'view';
    const kind = isView ? parts[1] : parts[0]; const action = isView ? parts[2] : parts[1];
    const collectionKey = `${kind}s` as keyof Project['model'];
    const collection = (isView ? p.view[collectionKey as keyof typeof p.view] : p.model[collectionKey]) as unknown as Record<string, Record<string, unknown>>;
    if (!collection) fail(`未知对象：${op.op}`);
    if (action === 'add') {
      const value = op.value; if (!value || typeof value.id !== 'string') fail(`新增对象需要 id：${op.op}`);
      const key = value!.id as string;
      if (Object.values(p.model).some(c => Object.hasOwn(c, key))) fail(`ID 已存在：${key}`);
      collection[key] = { ...(['node', 'group'].includes(kind) ? { sceneId: p.rootSceneId } : {}), ...structuredClone(value!) };
      if (kind === 'node') p.view.nodes[key] = { position: { x: 100, y: 100 }, width: value!.kind === 'annotation' ? 560 : 250, ...(value!.kind === 'annotation' ? { height: 420 } : {}) };
      if (kind === 'group') p.view.groups[key] = { position: { x: 50, y: 50 }, width: 500, height: 400, collapsed: false };
      if (kind === 'scene') {
        const scene = p.model.scenes[key], owner = scene.ownerNodeId && p.model.nodes[scene.ownerNodeId];
        if (!owner || owner.detailSceneId || owner.kind === 'annotation' || owner.kind.startsWith('boundary-')) fail('创建子视图需要一个尚无子视图的普通功能节点');
        owner.detailSceneId = key; p.view.scenes[key] = { viewport: { x: 50, y: 60, zoom: 0.65 } };
      }
      continue;
    }
    const key = op.id; if (!key || (!Object.hasOwn(collection, key) && !(isView && kind === 'edge' && p.model.edges[key]))) fail(`对象不存在：${op.id}`);
    if (action === 'update') {
      if (Object.hasOwn(changes, 'id')) fail('对象 ID 不可修改');
      if (!isView && kind === 'node' && Object.hasOwn(changes, 'detailSceneId')) fail('通过 scene.add/remove 管理详情视图关联');
      if (!isView && kind === 'scene' && Object.hasOwn(changes, 'ownerNodeId')) fail('子视图所属节点不可直接修改');
      if (!isView && kind === 'port') {
        const port = p.model.ports[key!];
        if (Object.hasOwn(changes, 'parentPortId') && changes.parentPortId !== port.parentPortId) fail('接口映射由子视图自动维护，不能直接修改 parentPortId');
        if (port.parentPortId && Object.entries(changes).some(([field, value]) => !['description', 'order', 'multiplicity'].includes(field) && (port as any)[field] !== value)) fail('边界接口定义跟随上层，请在上层编辑名称、方向或类型');
      }
      const previous = collection[key!];
      collection[key!] = { ...previous, ...structuredClone(changes) };
      if (isView && (kind === 'node' || kind === 'group') && Object.hasOwn(changes, 'appearance')) {
        const appearance = updateAppearance(previous?.appearance as BlockAppearance | undefined, changes.appearance);
        if (Object.keys(appearance).length) collection[key!].appearance = appearance;
        else delete collection[key!].appearance;
      }
      if (!isView && kind === 'node' && changes.groupId === null) delete collection[key!].groupId;
      if (!isView && kind === 'group' && changes.parentGroupId === null) delete collection[key!].parentGroupId;
    } else if (action === 'remove') {
      if (kind === 'edge') removeEdge(key!);
      if (kind === 'port') removePort(key!, op.cascade);
      if (kind === 'node') removeNode(key!, op.cascade);
      if (kind === 'scene') removeScene(key!, op.cascade);
      if (kind === 'group') {
        for (const node of Object.values(p.model.nodes)) if (node.groupId === key) delete node.groupId;
        for (const group of Object.values(p.model.groups)) if (group.parentGroupId === key) delete group.parentGroupId;
        delete p.model.groups[key!]; delete p.view.groups[key!];
      }
    }
  }
  syncSceneInterfaces(p);
  p.revision++;
  return validateProject(p);
}

export function groupContains(p: Project, groupId: string, nodeId: string): boolean {
  let current = p.model.nodes[nodeId]?.groupId;
  while (current) { if (current === groupId) return true; current = p.model.groups[current]?.parentGroupId; }
  return false;
}
