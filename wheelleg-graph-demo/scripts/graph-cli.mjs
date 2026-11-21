import { readFile, writeFile } from 'node:fs/promises';
import { validateProject, EDITOR_CAPABILITIES } from '../shared/model.ts';

const args = process.argv.slice(2); const command = args.shift();
const option = (name, fallback) => { const at = args.indexOf(`--${name}`); return at < 0 ? fallback : args[at + 1]; };
const base = option('url', process.env.GRAPH_URL || `http://127.0.0.1:${process.env.GRAPH_PORT || 4317}`).replace(/\/$/, '');
const token = process.env.GRAPH_ACCESS_TOKEN || '';
const projectId = option('project');
const file = option('file');
let serviceVerified = false;
async function request(path, method = 'GET', value) {
  if (!serviceVerified) {
    const statusResponse = await fetch(base + '/api/status', { signal: AbortSignal.timeout(15000) });
    const status = await statusResponse.json();
    if (!statusResponse.ok || status.schemaVersion !== 2 || !EDITOR_CAPABILITIES.every(capability=>status.capabilities?.includes(capability))) throw Object.assign(new Error('服务版本过旧或不匹配，请关闭旧服务后重新启动工具。'), { code: 'SERVICE_VERSION' });
    serviceVerified = true;
  }
  const response = await fetch(base + path, { method, headers: { ...(token ? { Authorization: `Bearer ${token}` } : {}), ...(value !== undefined ? { 'Content-Type': 'application/json' } : {}) }, body: value === undefined ? undefined : JSON.stringify(value), signal: AbortSignal.timeout(15000) });
  const result = await response.json();
  if (!response.ok) { const error = new Error(result.error?.message || `HTTP ${response.status}`); Object.assign(error, result.error); throw error; }
  return result;
}
async function main() {
  let result;
  const needProject = () => { if (!projectId) throw new Error('请提供 --project <id>'); return `/api/projects/${encodeURIComponent(projectId)}`; };
  const readInput = async () => { if (!file) throw new Error('请提供 --file <path>'); return JSON.parse(await readFile(file, 'utf8')); };
  switch (command) {
    case 'list': result = await request('/api/projects'); break;
    case 'create': result = await request('/api/projects', 'POST', { title: option('title', '未命名工程'), ...(args.includes('--lqr') ? { template: 'lqr' } : {}) }); break;
    case 'inspect': {
      result = await request(needProject());
      const objectId = option('object');
      const sceneId = option('scene');
      if (sceneId) {
        const p = result.project;
        if (!p.model.scenes[sceneId]) throw new Error('视图不存在');
        const nodes = Object.fromEntries(Object.entries(p.model.nodes).filter(([,node])=>node.sceneId===sceneId));
        const ports = Object.fromEntries(Object.entries(p.model.ports).filter(([,port])=>nodes[port.nodeId]));
        const edges = Object.fromEntries(Object.entries(p.model.edges).filter(([,edge])=>ports[edge.sourcePortId]));
        const groups = Object.fromEntries(Object.entries(p.model.groups).filter(([,group])=>group.sceneId===sceneId));
        result = { projectId:p.projectId, revision:p.revision, scene:p.model.scenes[sceneId], nodes, ports, edges, groups, viewport:p.view.scenes[sceneId].viewport, interfaces:Object.values(ports).filter(port=>port.parentPortId), layout:{nodes:Object.fromEntries(Object.entries(p.view.nodes).filter(([key])=>nodes[key])),edges:Object.fromEntries(Object.entries(p.view.edges).filter(([key])=>edges[key])),groups:Object.fromEntries(Object.entries(p.view.groups).filter(([key])=>groups[key]))} };
      }
      if (objectId) {
        if (sceneId) throw new Error('--scene 与 --object 请分别查询');
        const p = result.project; let entity;
        for (const [kind, collection] of Object.entries(p.model)) if (Object.hasOwn(collection, objectId)) entity = { kind, value: collection[objectId] };
        if (!entity) throw new Error('对象不存在');
        result = { projectId: p.projectId, revision: p.revision, entity, view: { node: p.view.nodes[objectId], edge: p.view.edges[objectId], group: p.view.groups[objectId], scene:p.view.scenes[objectId] } };
      }
      break;
    }
    case 'validate': { const checked = validateProject(await readInput()); result = { valid: true, revision: checked.project.revision, warnings: checked.warnings }; break; }
    case 'apply': result = await request(needProject() + '/transactions' + (args.includes('--dry-run') ? '?dryRun=true' : ''), 'POST', await readInput()); break;
    case 'export': {
      result = await request(needProject() + '/export'); const out = option('out');
      if (out) { await writeFile(out, JSON.stringify(result, null, 2) + '\n'); result = { exported: out, projectId: result.projectId, revision: result.revision }; }
      break;
    }
    case 'import': result = await request('/api/projects', 'POST', { project: await readInput(), title: option('title') }); break;
    case 'undo': case 'redo': {
      const revision = Number(option('revision', 'NaN')); if (!Number.isInteger(revision)) throw new Error('需要 --revision <当前版本>');
      result = await request(needProject() + '/' + command, 'POST', { actor: option('actor', 'agent'), baseRevision: revision }); break;
    }
    case 'help': case '--help': case undefined:
      result = { commands: ['list', 'create [--title name] [--lqr]', 'inspect --project id [--object id | --scene id]', 'validate --file path', 'apply --project id --file patch.json [--dry-run]', 'export --project id [--out file]', 'import --file project.json [--title name]', 'undo|redo --project id --revision n --actor name'], config: ['--url or GRAPH_URL', 'GRAPH_ACCESS_TOKEN'], note: 'apply uses the patch baseRevision; export without --out writes the project JSON to stdout.' }; break;
    default: throw new Error(`未知命令：${command}`);
  }
  process.stdout.write(JSON.stringify(result, null, 2) + '\n');
}
main().catch(error => { process.stderr.write(JSON.stringify({ error: { code: error.code || 'CLI_ERROR', message: error.message, details: error.details } }) + '\n'); process.exitCode = 1; });
