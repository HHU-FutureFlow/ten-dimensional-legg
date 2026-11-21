import type { Project, Operation } from '../shared/model.ts';
export type History = { undoActor: string | null; redoActor: string | null };
export type ProjectResponse = { project: Project; warnings?: string[]; history?: History };
export type ProjectSummary = { projectId: string; title: string; revision: number; nodes: number };
export async function api(path: string, method = 'GET', value?: unknown): Promise<any> {
  const token = sessionStorage.getItem('graph-token');
  const res = await fetch('/api' + path, { method, headers: { ...(token ? { Authorization: `Bearer ${token}` } : {}), ...(value !== undefined ? { 'Content-Type': 'application/json' } : {}) }, body: value !== undefined ? JSON.stringify(value) : undefined });
  const data = await res.json();
  if (!res.ok) { const error = new Error(data.error?.message || '请求失败'); Object.assign(error, data.error); throw error; }
  return data;
}
export async function eventStream(signal: AbortSignal, onEvent: (event: { projectId: string; revision: number; actor: string; summary: string }) => void, onStatus: (online: boolean) => void) {
  while (!signal.aborted) {
    try {
      const token = sessionStorage.getItem('graph-token');
      const res = await fetch('/api/events', { headers: token ? { Authorization: `Bearer ${token}` } : {}, signal });
      if (!res.ok || !res.body) throw new Error('通知连接失败');
      onStatus(true);
      const reader = res.body.getReader(); const decoder = new TextDecoder(); let buffer = '';
      try {
        while (!signal.aborted) {
          const chunk = await reader.read(); if (chunk.done) break;
          buffer += decoder.decode(chunk.value, { stream: true });
          let at: number;
          while ((at = buffer.indexOf('\n\n')) >= 0) {
            const frame = buffer.slice(0, at); buffer = buffer.slice(at + 2);
            const data = frame.split('\n').find(line => line.startsWith('data: ')); if (data) onEvent(JSON.parse(data.slice(6)));
          }
        }
      } finally { await reader.cancel().catch(() => {}); }
    } catch (e) { if (signal.aborted) return; }
    if (signal.aborted) return;
    onStatus(false);
    await new Promise<void>(resolve => { const timer = setTimeout(resolve, 2000); signal.addEventListener('abort', () => { clearTimeout(timer); resolve(); }, { once: true }); });
  }
}
export type Commit = (operations: Operation[], summary: string, baseRevision?: number) => Promise<boolean>;
