import { mkdir, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { createProject, newNode, newPort, newEdge, validateProject } from '../shared/model.ts';
import { detailedLqrExample } from './detail-example.mjs';
import { explainLqrExample } from './explain-lqr.mjs';

const dir = fileURLToPath(new URL('../examples/', import.meta.url));
await mkdir(dir, { recursive: true });
const save = (name, p) => { validateProject(p); return writeFile(dir + name, JSON.stringify(p, null, 2) + '\n'); };
await save('blank.graph.json', createProject('空白工程', 'blank-example'));
const p = createProject('十维 LQR · 输入输出示例', 'lqr-example');
p.sourceRoots.firmware = { label: '十维轮腿固件（源码可选）' };
function node(key, label, description, x, y, source, symbol, kind = 'algorithm') {
  const n = { ...newNode(label, key), description, kind, status: 'implemented' };
  if (source) n.sourceRefs.push({ rootId: 'firmware', path: `application/chassis/${source}`, symbol, revision: 'a7d8f90' });
  n.metadata = { verification: '2026-11-17 源码梳理；非实车验证' };
  p.model.nodes[key] = n; p.view.nodes[key] = { position: { x, y }, width: 260 }; return n;
}
function port(nodeId, key, direction, label, unit = '', shape = '', dataType = 'float') {
  const v = { ...newPort(nodeId, direction, label, key), unit, shape, dataType, order: Object.values(p.model.ports).filter(v => v.nodeId === nodeId && v.direction === direction).length };
  p.model.ports[key] = v; return key;
}
function connect(from, to, label, kind = 'data') {
  const key = `e_${from}_${to}`; p.model.edges[key] = { ...newEdge(from, to, key), label, kind }; p.view.edges[key] = { style: 'bezier' };
}
node('observe', '状态观测', 'BalanceObserve()\ns / ds / yaw / dyaw\nlθ / ldθ / rθ / rdθ / φ / dφ', 40, 210, 'balance.c', 'BalanceObserve', 'input');
node('target', '控制目标', 'ten_balance_target\n由底盘模式和遥控意图生成十维参考值', 40, 530, 'chassis.c', 'ChassisTask', 'input');
node('gain', '腿长选 K', 'LQR_calc 内部子功能\n平均实测腿长 → 六组 K\n0.15～0.40 m · 1 cm 迟滞', 400, 610, 'LQR.c', 'LQR_calc');
node('air', 'GRF 与模式门控', 'chassis 根据接地观测和恢复阶段处理 air\nLQR_SetAir()；模式条件人工梳理', 400, 50, 'chassis.c', 'ChassisTask', 'control');
node('lqr', '十维状态反馈', 'u = −K · (x − x_ref)\n四路原始力矩，尚未经过 chassis 覆盖\n悬空时轮端清零，腿端降维', 790, 280, 'LQR.c', 'LQR_calc');
p.model.nodes.lqr.sourceRefs[0].line = 71;
p.model.nodes.lqr.metadata = { formula: 'u = -K(x-x_ref)', stateOrder: ['s','ds','yaw','dyaw','l_theta','l_dtheta','r_theta','r_dtheta','phi','dphi'], outputOrder: ['l_Tw','r_Tw','l_Tp','r_Tp'], verification: '源码检查，未进行本轮实测' };
node('mode', '模式处理', '起立时混合交接\n悬空分支可用 PD 覆盖 Tp\n需按当前模式阅读条件', 1190, 300, 'chassis.c', 'ChassisTask', 'control');
node('wheel', '轮端处理', '摩擦前馈 · 限幅 · 门控\n再转换为左右轮电机指令', 1570, 180, 'chassis.c', 'ChassisTask', 'output');
node('force', '腿长力控制', '腿长 PID + 重力前馈 ± roll\nF_leg 是独立通道，单位 N', 790, 820, 'chassis.c', 'ChassisTask');
node('vmc', 'VMC 与关节执行', 'Tp + F_leg → Tf / Tb\n后续还有关节摩擦补偿、限幅和门控', 1570, 580, 'VMC.c', 'VMC_Calc / l_VMC_Calc', 'output');
const locations={observe:[84],target:[547,635],gain:[77,88],air:[554,562],lqr:[71],mode:[1201,1372],wheel:[1436,1460],force:[1292,1345],vmc:[9,36]};
for(const [key,[line,endLine]] of Object.entries(locations))Object.assign(p.model.nodes[key].sourceRefs[0],{line,...(endLine?{endLine}:{})});
p.model.nodes.air.sourceRefs.push({rootId:'firmware',path:'application/chassis/grf_observer.c',line:116,symbol:'GRF_Update',revision:'a7d8f90'});
p.model.nodes.mode.description='进入详情查看条件分支与覆盖顺序\n恢复 PD → 交接混合 → 悬空覆盖 → 最终输出闸';
p.model.nodes.vmc.sourceRefs.push({rootId:'firmware',path:'application/chassis/chassis.c',line:1374,endLine:1422,symbol:'ChassisTask',revision:'a7d8f90'});
port('observe','obs_x','output','x [10]','','10'); port('observe','obs_l','output','L_l / L_r','m','2');
port('target','target_x','output','x_ref [10]','','10');
port('gain','gain_l','input','平均腿长输入','m','2'); port('gain','gain_k','output','K [4×10]','','4x10');
port('air','air_flag','output','air','','','bool'); port('air','mode_flag','output','mode','','','mode');
port('lqr','lqr_x','input','x [10]','','10'); port('lqr','lqr_ref','input','x_ref [10]','','10'); port('lqr','lqr_k','input','K','','4x10'); port('lqr','lqr_air','input','air','','','bool');
for (const key of ['l_Tw','r_Tw','l_Tp','r_Tp']) { port('lqr',`lqr_${key}`,'output',key,'N·m'); port('mode',`mode_${key}`,'input',key,'N·m'); connect(`lqr_${key}`,`mode_${key}`,key); }
port('mode','mode_in','input','mode','','','mode'); port('mode','mode_tw','output','Tw_l / Tw_r','N·m','2'); port('mode','mode_tp','output','Tp_l / Tp_r','N·m','2');
port('mode','mode_f_in','input','F_leg,l / F_leg,r','N','2');port('mode','mode_f_out','output','F_leg,l / F_leg,r','N','2');
port('wheel','wheel_tw','input','Tw_l / Tw_r','N·m','2'); port('wheel','wheel_out','output','左右轮电机指令');
port('force','force_out','output','F_leg,l / F_leg,r','N','2');
port('vmc','vmc_tp','input','Tp_l / Tp_r','N·m','2'); port('vmc','vmc_f','input','F_leg,l / F_leg,r','N','2'); port('vmc','vmc_out','output','四关节电机指令');
connect('obs_x','lqr_x','观测状态'); connect('obs_l','gain_l','实测腿长'); connect('target_x','lqr_ref','参考状态'); connect('gain_k','lqr_k','K'); connect('air_flag','lqr_air','air','control'); connect('mode_flag','mode_in','模式条件','control'); connect('mode_tw','wheel_tw','轮端力矩'); connect('mode_tp','vmc_tp','腿端力矩'); connect('force_out','mode_f_in','独立腿长力','force');connect('mode_f_out','vmc_f','经输出闸的腿长力','force');
p.view.scenes[p.rootSceneId].viewport = { x: 25, y: 25, zoom: 0.65 };
await save('lqr.graph.json', explainLqrExample(detailedLqrExample(p)));
await mkdir(dir + 'patches', { recursive: true });
await writeFile(dir + 'patches/describe-lqr.json', JSON.stringify({ requestId: 'replace-with-a-fresh-id', baseRevision: 0, actor: 'agent', summary: '补充 LQR 描述，保留画布位置', operations: [{ op: 'node.update', id: 'lqr', changes: { description: 'u = -K(x-x_ref)；四路原始反馈力矩。该说明由 agent 手工维护。' } }] }, null, 2) + '\n');
console.log('Created blank, LQR, and patch examples.');
