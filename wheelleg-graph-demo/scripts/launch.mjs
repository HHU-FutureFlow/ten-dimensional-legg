import { spawn } from 'node:child_process';
import { startServer } from '../server/index.ts';

try {
  const app=await startServer();
  const url=app.url.replace('://0.0.0.0:', '://127.0.0.1:');
  console.log(`Project Graph: ${url}\nKeep this terminal open. Press Ctrl+C to stop.`);
  const command=process.platform==='win32'?'explorer.exe':process.platform==='darwin'?'open':'xdg-open';
  const opener=spawn(command,[url],{stdio:'ignore',detached:true,windowsHide:true});opener.on('error',()=>console.log('Open the URL above in your browser.'));opener.unref();
  const stop=()=>app.close().then(()=>process.exit(0)).catch(e=>{console.error(e.message);process.exit(1);});
  process.once('SIGINT',stop);process.once('SIGTERM',stop);
}catch(error){console.error(error.code||'START_FAILED',error.message);process.exitCode=1;}
