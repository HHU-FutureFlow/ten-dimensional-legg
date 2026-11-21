import { createServer, type IncomingMessage, type ServerResponse } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { resolve, extname, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { timingSafeEqual } from 'node:crypto';
import { ProjectStore } from './store.ts';
import { GraphError, validateProject, EDITOR_CAPABILITIES } from '../shared/model.ts';

const appRoot = fileURLToPath(new URL('../', import.meta.url));
type Options = { host?: string; port?: number; directory?: string; token?: string };
export async function startServer(options: Options = {}) {
  const host = options.host ?? process.env.GRAPH_HOST ?? '127.0.0.1';
  const port = options.port ?? Number(process.env.GRAPH_PORT || 4317);
  const token = options.token ?? process.env.GRAPH_ACCESS_TOKEN ?? '';
  const local = ['127.0.0.1', 'localhost', '::1'].includes(host);
  if (!local && token.length < 16) throw new GraphError('TOKEN_REQUIRED', '团队监听需要至少 16 字符的 GRAPH_ACCESS_TOKEN');
  const directory = options.directory ?? resolve(appRoot, process.env.GRAPH_DATA_DIR || 'data/v2');
  const store = new ProjectStore(directory); await store.init();
  const clients = new Set<ServerResponse>();
  store.onChange = event => { for (const client of clients) client.write(`data: ${JSON.stringify(event)}\n\n`); };
  const send = (res: ServerResponse, status: number, value: unknown) => {
    res.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff' }); res.end(JSON.stringify(value));
  };
  const body = async (req: IncomingMessage) => {
    if (!req.headers['content-type']?.startsWith('application/json')) throw new GraphError('JSON_REQUIRED', '请求必须使用 application/json', 415);
    let length = 0; const chunks = [];
    for await (const chunk of req) { length += chunk.length; if (length > 8 * 1024 * 1024) throw new GraphError('TOO_LARGE', '项目/事务超过 8 MB', 413); chunks.push(chunk); }
    try { return JSON.parse(Buffer.concat(chunks).toString('utf8')); } catch { throw new GraphError('INVALID_JSON', 'JSON 无法解析'); }
  };
  const server = createServer(async (req, res) => {
    try {
      const url = new URL(req.url || '/', 'http://service'); const path = url.pathname;
      if (path.startsWith('/api/')) {
        if (local && !['127.0.0.1', 'localhost', '[::1]'].includes(new URL(`http://${req.headers.host}`).hostname)) throw new GraphError('HOST_REJECTED', '请求 Host 不受支持', 403);
        if (req.headers.origin && new URL(req.headers.origin).host !== req.headers.host) throw new GraphError('ORIGIN_REJECTED', '拒绝跨来源 API 请求', 403);
        if (path === '/api/status' && req.method === 'GET') return send(res, 200, { name: 'Project Graph', version: '0.4.0', schemaVersion: 2, capabilities: EDITOR_CAPABILITIES, authRequired: !!token });
        if (token) {
          const provided = req.headers.authorization?.replace(/^Bearer /, '') || '';
          if (Buffer.byteLength(provided) !== Buffer.byteLength(token) || !timingSafeEqual(Buffer.from(provided), Buffer.from(token))) throw new GraphError('UNAUTHORIZED', '请提供正确的共享访问令牌', 401);
        }
        if (path === '/api/events' && req.method === 'GET') {
          res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache', Connection: 'keep-alive', 'X-Accel-Buffering': 'no' });
          res.write(': connected\n\n'); clients.add(res);
          const heartbeat = setInterval(() => res.write(': heartbeat\n\n'), 15000);
          req.on('close', () => { clients.delete(res); clearInterval(heartbeat); }); return;
        }
        if (path === '/api/projects' && req.method === 'GET') return send(res, 200, { projects: await store.list() });
        if (path === '/api/projects' && req.method === 'POST') {
          const value = await body(req); let imported = value.project;
          if (value.template === 'lqr') imported = JSON.parse(await readFile(resolve(appRoot, 'examples/lqr.graph.json'), 'utf8'));
          return send(res, 201, { project: await store.create(value.title || (imported ? imported.title : '未命名工程'), imported) });
        }
        if (path === '/api/validate' && req.method === 'POST') return send(res, 200, validateProject(await body(req)));
        const match = path.match(/^\/api\/projects\/([A-Za-z0-9_-]{1,100})(?:\/(transactions|undo|redo|export))?$/);
        if (match) {
          const [, projectId, action] = match;
          if (req.method === 'GET' && (!action || action === 'export')) {
            const p = await store.get(projectId);
            return send(res, 200, action === 'export' ? p : { ...validateProject(p), history: store.historyInfo(projectId) });
          }
          if (req.method === 'POST') {
            const value = await body(req);
            if (action === 'transactions') return send(res, 200, await store.apply(projectId, value, url.searchParams.get('dryRun') === 'true'));
            if (action === 'undo' || action === 'redo') {
              if (typeof value.actor !== 'string' || !value.actor || !Number.isInteger(value.baseRevision)) throw new GraphError('INVALID_HISTORY_REQUEST', '需要 actor 和 baseRevision');
              return send(res, 200, await store.undoRedo(projectId, value.actor, value.baseRevision, action));
            }
          }
        }
        throw new GraphError('NOT_FOUND', '接口不存在', 404);
      }
      if (req.method !== 'GET' && req.method !== 'HEAD') throw new GraphError('METHOD_NOT_ALLOWED', '不支持的请求方式', 405);
      const dist = resolve(appRoot, 'dist');
      let file = resolve(dist, '.' + decodeURIComponent(path));
      if (file !== dist && !file.startsWith(dist + sep)) throw new GraphError('NOT_FOUND', '文件不存在', 404);
      if (path === '/' || !extname(path)) file = resolve(dist, 'index.html');
      try { if (!(await stat(file)).isFile()) throw new Error('not a file'); } catch { throw new GraphError('NOT_BUILT', '页面尚未构建，请先执行 npm run build', 404); }
      const types: Record<string, string> = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.json': 'application/json', '.svg': 'image/svg+xml', '.png': 'image/png' };
      res.writeHead(200, { 'Content-Type': types[extname(file)] || 'application/octet-stream', 'X-Content-Type-Options': 'nosniff', 'Cache-Control': extname(file) === '.html' ? 'no-cache' : 'public, max-age=3600' });
      res.end(req.method === 'HEAD' ? undefined : await readFile(file));
    } catch (e: any) { if (!res.headersSent) send(res, e.status || 500, { error: { code: e.code || 'INTERNAL_ERROR', message: e.message || '请求失败', details: e.details } }); else res.end(); }
  });
  try { await new Promise<void>((success, failure) => { server.once('error', failure); server.listen(port, host, () => { server.removeListener('error', failure); success(); }); }); } catch (e) { await store.close(); throw e; }
  const address = server.address(); const actualPort = typeof address === 'object' && address ? address.port : port;
  const url = `http://${host === '::1' ? '[::1]' : host}:${actualPort}`;
  const close = async () => { for (const client of clients) client.end(); server.closeAllConnections(); await new Promise<void>((done, reject) => server.close(e => e ? reject(e) : done())); await store.close(); };
  return { server, store, url, close };
}
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  startServer().then(app => {
    console.log(`Project Graph ready: ${app.url}`);
    const stop = () => app.close().then(() => process.exit(0)).catch(e => { console.error(e.message); process.exit(1); });
    process.once('SIGINT', stop); process.once('SIGTERM', stop);
  }).catch(e => { console.error(`${e.code || 'START_FAILED'}: ${e.message}`); process.exit(1); });
}
