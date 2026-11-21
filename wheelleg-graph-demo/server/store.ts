import { mkdir, open, readFile, readdir, rename, rm, writeFile } from 'node:fs/promises';
import { join } from 'node:path';
import { randomUUID, createHash } from 'node:crypto';
import { GraphError, applyOperations, createProject, validateProject, validatePatch } from '../shared/model.ts';
import type { Project, Patch } from '../shared/model.ts';

type HistoryEntry = { before: Project; after: Project; actor: string };
type History = { undo: HistoryEntry[]; redo: HistoryEntry[] };
export class ProjectStore {
  directory: string;
  private tail: Promise<unknown> = Promise.resolve();
  private hashes = new Map<string, string>();
  private history = new Map<string, History>();
  private requests = new Map<string, { signature: string; revision: number }>();
  private lockToken = randomUUID();
  private locked = false;
  onChange: (event: { projectId: string; revision: number; actor: string; summary: string }) => void = () => {};
  constructor(directory: string) { this.directory = directory; }
  async init() {
    await mkdir(this.directory, { recursive: true });
    const lockPath = join(this.directory, '.writer.lock');
    for (let attempt = 0; attempt < 2; attempt++) {
      try {
        const file = await open(lockPath, 'wx');
        await file.writeFile(JSON.stringify({ pid: process.pid, token: this.lockToken })); await file.close();
        this.locked = true; return;
      } catch (error: any) {
        if (error.code !== 'EEXIST') throw error;
        let owner: { pid?: number };
        try { owner = JSON.parse(await readFile(lockPath, 'utf8')); } catch { throw new GraphError('LOCK_INVALID', '数据目录锁不可读取，请确认没有运行中的服务后手动处理', 409); }
        if (!Number.isInteger(owner.pid) || Number(owner.pid) <= 0) throw new GraphError('LOCK_INVALID', '数据目录锁格式错误', 409);
        let alive = true;
        try { process.kill(owner.pid!, 0); } catch (e: any) { alive = e.code !== 'ESRCH'; }
        if (alive) throw new GraphError('ALREADY_RUNNING', '此数据目录已由一个服务使用', 409);
        await rm(lockPath); // Only a confirmed stale, exact lock file is removed.
      }
    }
    throw new GraphError('ALREADY_RUNNING', '无法获得数据目录写入锁', 409);
  }
  async close() {
    await this.tail.catch(() => {});
    if (!this.locked) return;
    const lockPath = join(this.directory, '.writer.lock');
    const owner = JSON.parse(await readFile(lockPath, 'utf8'));
    if (owner.token === this.lockToken) await rm(lockPath);
    this.locked = false;
  }
  private path(projectId: string) {
    if (!/^[A-Za-z0-9_-]{1,100}$/.test(projectId)) throw new GraphError('INVALID_ID', '项目 ID 无效');
    return join(this.directory, `${projectId}.graph.json`);
  }
  private hash(data: string) { return createHash('sha256').update(data).digest('hex'); }
  private exclusive<T>(task: () => Promise<T>): Promise<T> {
    const result = this.tail.then(task); this.tail = result.catch(() => {}); return result;
  }
  async list() {
    const result = [];
    for (const name of await readdir(this.directory)) if (name.endsWith('.graph.json')) {
      const p = await this.get(name.slice(0, -11));
      result.push({ projectId: p.projectId, title: p.title, revision: p.revision, nodes: Object.keys(p.model.nodes).length });
    }
    return result;
  }
  async get(projectId: string): Promise<Project> {
    let text: string;
    try { text = await readFile(this.path(projectId), 'utf8'); } catch (e: any) { if (e.code === 'ENOENT') throw new GraphError('NOT_FOUND', '项目不存在', 404); throw e; }
    const hash = this.hash(text);
    if (this.hashes.has(projectId) && this.hashes.get(projectId) !== hash) throw new GraphError('EXTERNAL_CHANGE', '项目文件在服务外被修改；请停止服务检查，或恢复备份后重新导入', 409);
    const p = validateProject(JSON.parse(text)).project;
    if (p.projectId !== projectId) throw new GraphError('INVALID_GRAPH', '项目文件名与 ID 不一致', 422);
    this.hashes.set(projectId, hash); return p;
  }
  historyInfo(projectId: string) {
    const h = this.history.get(projectId);
    return { undoActor: h?.undo.at(-1)?.actor || null, redoActor: h?.redo.at(-1)?.actor || null };
  }
  private async persist(p: Project, previous?: Project) {
    validateProject(p);
    const target = this.path(p.projectId);
    if (previous) {
      await this.get(previous.projectId); // Check external changes immediately before writing.
      const backups = join(this.directory, 'backups'); await mkdir(backups, { recursive: true });
      await writeFile(join(backups, `${p.projectId}.${previous.revision}.json`), JSON.stringify(previous, null, 2) + '\n', { flag: 'wx' }).catch((e: any) => { if (e.code !== 'EEXIST') throw e; });
    }
    const temp = `${target}.${randomUUID()}.tmp`;
    const text = JSON.stringify(p, null, 2) + '\n';
    try {
      const file = await open(temp, 'wx');
      try { await file.writeFile(text); await file.sync(); } finally { await file.close(); }
      await rename(temp, target);
    } finally { await rm(temp, { force: true }); }
    this.hashes.set(p.projectId, this.hash(text));
  }
  async create(title: string, imported?: unknown) {
    return this.exclusive(async () => {
      const p = imported ? structuredClone(validateProject(imported).project) : createProject(title);
      p.projectId = createProject().projectId; p.revision = 0;
      if (title) p.title = title;
      await this.persist(p); return p;
    });
  }
  async apply(projectId: string, value: unknown, dryRun = false) {
    const patch = validatePatch(value);
    return this.exclusive(async () => {
      const requestKey = `${projectId}:${patch.requestId}`; const signature = JSON.stringify(patch);
      const known = this.requests.get(requestKey);
      if (known) {
        if (known.signature !== signature) throw new GraphError('REQUEST_ID_REUSED', '同一 requestId 不能用于不同事务', 409);
        return { project: await this.get(projectId), warnings: [], history: this.historyInfo(projectId), duplicate: true, committedRevision: known.revision };
      }
      const before = await this.get(projectId);
      const { project, warnings } = applyOperations(before, patch);
      if (dryRun) return { project, warnings, history: this.historyInfo(projectId), dryRun: true };
      await this.persist(project, before);
      const h = this.history.get(projectId) || { undo: [], redo: [] };
      h.undo.push({ before, after: structuredClone(project), actor: patch.actor });
      if (h.undo.length > 30) h.undo.shift(); h.redo = []; this.history.set(projectId, h);
      this.requests.set(requestKey, { signature, revision: project.revision });
      if (this.requests.size > 300) this.requests.delete(this.requests.keys().next().value!);
      this.onChange({ projectId, revision: project.revision, actor: patch.actor, summary: patch.summary });
      return { project, warnings, history: this.historyInfo(projectId) };
    });
  }
  async undoRedo(projectId: string, actor: string, baseRevision: number, action: 'undo' | 'redo') {
    return this.exclusive(async () => {
      const current = await this.get(projectId);
      if (baseRevision !== current.revision) throw new GraphError('REVISION_CONFLICT', '撤销/重做基于过期版本，请刷新', 409);
      const h = this.history.get(projectId); const source = h?.[action]; const entry = source?.at(-1);
      if (!entry || entry.actor !== actor) throw new GraphError('HISTORY_CONFLICT', '只能撤销/重做全局栈顶且由自己提交的操作', 409);
      const next = structuredClone(action === 'undo' ? entry.before : entry.after); next.revision = current.revision + 1;
      await this.persist(next, current);
      source!.pop(); h![action === 'undo' ? 'redo' : 'undo'].push(entry);
      this.onChange({ projectId, revision: next.revision, actor, summary: action });
      return { project: next, warnings: validateProject(next).warnings, history: this.historyInfo(projectId) };
    });
  }
}
