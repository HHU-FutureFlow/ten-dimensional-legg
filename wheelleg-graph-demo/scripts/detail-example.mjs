import { applyOperations, newNode, newPort, newEdge, boundaryPortId, validateProject } from '../shared/model.ts';

// Manually authored from firmware a7d8f90. This file does not scan or execute firmware.
export function detailedLqrExample(initial) {
  let p=initial,sceneId=p.rootSceneId;
  const detail=(ownerNodeId,id,label)=>{
    p=applyOperations(p,{requestId:`example-${id}`,baseRevision:p.revision,actor:'example-author',summary:label,operations:[{op:'scene.add',value:{id,label,description:'按 a7d8f90 源码人工整理，条件图不执行控制代码。',ownerNodeId}}]}).project;
    sceneId=id;
  };
  const block=(key,label,description,x,y,line,endLine,kind='algorithm')=>{
    const n={...newNode(label,key,sceneId),description,kind,status:'implemented',sourceRefs:[{rootId:'firmware',path:'application/chassis/chassis.c',line,...(endLine?{endLine}:{}),symbol:'ChassisTask',revision:'a7d8f90'}],metadata:{evidence:'2026-11-17 源码核对，非本轮实车验证'}};
    p.model.nodes[key]=n;p.view.nodes[key]={position:{x,y},width:300};return key;
  };
  const port=(nodeId,key,direction,label,type='float',shape='',unit='',many=false)=>{
    const value={...newPort(nodeId,direction,label,key),dataType:type,shape,unit,order:Object.values(p.model.ports).filter(v=>v.nodeId===nodeId&&v.direction===direction).length};
    if(many)value.multiplicity='many';p.model.ports[key]=value;return key;
  };
  const link=(from,to,label='',kind='data',condition='')=>{
    const id=`detail_e_${Object.keys(p.model.edges).length}`;
    p.model.edges[id]={...newEdge(from,to,id),label,kind,condition,description:condition};p.view.edges[id]={style:'bezier'};
  };
  const boolIn=(node,key='in',label='判定输入',many=false)=>port(node,`${node}_${key}`,'input',label,'bool','','',many);
  const boolOut=(node,key,label)=>port(node,`${node}_${key}`,'output',label,'bool');
  const contextIn=node=>port(node,`${node}_ctx`,'input','状态 / armed / air','mode');
  const torqueIn=(node,key='u',label='四路力矩',many=false)=>port(node,`${node}_${key}`,'input',label,'float','4','N·m',many);
  const torqueOut=(node,key='out')=>port(node,`${node}_${key}`,'output','四路力矩','float','4','N·m');

  detail('mode','mode-detail','模式处理');
  const input=p.model.nodes[p.model.ports[boundaryPortId('mode_in')].nodeId].id;
  const output=p.model.nodes[p.model.ports[boundaryPortId('mode_tw')].nodeId].id;
  p.view.nodes[input]={position:{x:0,y:300},width:280};p.view.nodes[output]={position:{x:2620,y:420},width:280};
  block('mode_classify','模式判定与状态转换','进入下一层查看 S0 位形判定、S1～S7 转移条件\n以及零力提前返回和运行中翻倒保护。',360,-120,418,1188,'control');
  contextIn('mode_classify');port('mode_classify','mode_classify_out','output','当前模式上下文','mode');link(boundaryPortId('mode_in'),'mode_classify_ctx','模式上下文','control');
  block('mode_pack','四路原始力矩','图内接口打包：l_Tw / r_Tw / l_Tp / r_Tp\n对应同一个 four_control_data，不是新增固件函数。',360,330,1188,1192,'note');
  for(const key of ['l_Tw','r_Tw','l_Tp','r_Tp']){port('mode_pack',`mode_pack_${key}`,'input',key,'float','','N·m');link(boundaryPortId(`mode_${key}`),`mode_pack_${key}`,key);}
  torqueOut('mode_pack');
  block('mode_branch','恢复 / 交接选择','① (S7 && prot_armed) 或 S1/S2/S3/S6 → PD\n② 否则 S4 → PD/LQR 混合\n③ 其余保留 LQR；稍后仍有悬空与输出闸。',740,-120,1201,1254,'decision');contextIn('mode_branch');
  for(const [key,label] of [['pd','恢复 PD'],['ramp','S4 交接'],['normal','其余']])boolOut('mode_branch',key,label);
  link('mode_classify_out','mode_branch_ctx','状态选择','control');
  for(const [key,label,description,y,line,end] of [
    ['mode_pd','恢复 PD 覆盖','Tw_l = Tw_r = 0\nTp = kp·wrap(目标−θ_body) − kd·θ̇_body\nS7 使用 prot 参数，其余使用 rec 参数。',180,1205,1234],
    ['mode_ramp','S4 连续交接','Tp = PD·(1−ramp) + LQR·ramp\nTw = LQR·ramp，腿长保持低位\n角误差 wrap，PD 与 LQR 连续混合。',490,1236,1253],
    ['mode_keep','保留原始 LQR','当前既非恢复 PD 段，也非 S4\n保留四路原始反馈输出\n这一步不是最终电机指令。',800,1188,1254],
  ]){
    block(key,label,description,740,y,line,end);torqueIn(key);boolIn(key,'select','选择');torqueOut(key);
    link('mode_pack_out',`${key}_u`,'原始反馈');
  }
  link('mode_branch_pd','mode_pd_select','成立','control','(S7 && prot_armed) || S1/S2/S3/S6');
  link('mode_branch_ramp','mode_ramp_select','否则且 S4','control','前一个分支不成立 && prone_rec_state == PRS_LQR_RAMP');
  link('mode_branch_normal','mode_keep_select','其余','control','前两个分支均不成立');
  block('mode_merge','阶段输出汇合','以上三个分支按 if / else-if 互斥选择。\n悬空覆盖是后续独立 if，不属于这一组互斥选择。',1120,420,1201,1254);torqueIn('mode_merge','u','所选分支输出',true);torqueOut('mode_merge');
  for(const key of ['mode_pd','mode_ramp','mode_keep'])link(`${key}_out`,'mode_merge_u','阶段输出');
  block('mode_air_check','悬空条件 air_state','在阶段输出处理之后检查 air_state。\n恢复阶段的 air 门控在上游处理，S5 恢复检测。\n成立与否决定下一次覆盖。',1120,-120,554,562,'decision');contextIn('mode_air_check');boolOut('mode_air_check','yes','悬空');boolOut('mode_air_check','no','接地');link('mode_classify_out','mode_air_check_ctx','air','control');
  block('mode_air_apply','悬空输出覆盖','Tw 两路清零\n非 S7：Tp 改为腿角 PD 并限幅\nS7：保留前面 Tp 语义；置 air_reanchor=1。',1500,180,1267,1283);torqueIn('mode_air_apply');boolIn('mode_air_apply','select','air 成立');torqueOut('mode_air_apply');
  block('mode_ground','接地 / 落地沿','无 air 时保留前面输出\n若 air_reanchor 已置位：重锚 s_target/yaw_target\n随后清 air_reanchor，消除空转里程漂移。',1500,600,1284,1289);torqueIn('mode_ground');boolIn('mode_ground','select','air 不成立');torqueOut('mode_ground');
  link('mode_merge_out','mode_air_apply_u');link('mode_merge_out','mode_ground_u');link('mode_air_check_yes','mode_air_apply_select','是','control','air_state != 0');link('mode_air_check_no','mode_ground_select','否','control','air_state == 0');
  block('mode_output_gate','最终输出闸','S0 尚未完成判定，或 S7 尚未武装时：\nTw / Tp / F_leg 全部置零\n其余情况保留前面结果，后续摩擦补偿也受门控。',1880,400,1355,1372,'decision');torqueIn('mode_output_gate','u','悬空处理后输出',true);contextIn('mode_output_gate');torqueOut('mode_output_gate');
  port('mode_output_gate','mode_output_gate_force_in','input','独立腿长力','float','2','N');port('mode_output_gate','mode_output_gate_force_out','output','门控后腿长力','float','2','N');
  link('mode_air_apply_out','mode_output_gate_u','悬空分支');link('mode_ground_out','mode_output_gate_u','接地分支');link('mode_classify_out','mode_output_gate_ctx','state / armed','control');link(boundaryPortId('mode_f_in'),'mode_output_gate_force_in','F_leg','force');link('mode_output_gate_force_out',boundaryPortId('mode_f_out'),'F_leg','force');
  block('mode_unpack','输出接口拆分','按固定顺序分出左右轮力矩与左右腿力矩。\nTw → 轮端补偿/限幅；Tp 与 F_leg → VMC。',2260,400,1362,1375,'note');torqueIn('mode_unpack');
  port('mode_unpack','mode_unpack_tw','output','Tw_l / Tw_r','float','2','N·m');port('mode_unpack','mode_unpack_tp','output','Tp_l / Tp_r','float','2','N·m');
  link('mode_output_gate_out','mode_unpack_u');link('mode_unpack_tw',boundaryPortId('mode_tw'),'轮端力矩');link('mode_unpack_tp',boundaryPortId('mode_tp'),'腿端力矩');
  p.view.scenes[sceneId].viewport={x:30,y:120,zoom:0.5};

  detail('mode_classify','startup-detail','模式判定与状态转换');
  const inContext=boundaryPortId('mode_classify_ctx'),outContext=boundaryPortId('mode_classify_out');
  p.view.nodes[p.model.ports[inContext].nodeId]={position:{x:0,y:60},width:270};p.view.nodes[p.model.ports[outContext].nodeId]={position:{x:2400,y:1000},width:270};
  block('startup_zero','零力急停？','chassis_mode == CHASSIS_ZERO_FORCE\n在 LQR 之前使电机停机并直接 return。\n本图只解释判定，零力时不会执行下游控制链。',350,0,418,482,'decision');contextIn('startup_zero');boolOut('startup_zero','yes','零力');boolOut('startup_zero','no','非零力');link(inContext,'startup_zero_ctx','指令与观测','control');
  block('startup_return','停机并提前返回','Stop 标志生效；任务本拍 return。\n恢复沿会清模式锁存；恢复过程被打断时重回 S0。',720,-220,418,482,'control');boolIn('startup_return');link('startup_zero_yes','startup_return_in','是','control','zero_force != 0');
  block('startup_dispatch','当前处于 S0？','仅 S0 做初始位形判定；其他状态继续既有流程。\n控制块按源码顺序运行，本图不做周期仿真。',350,320,638,691,'decision');boolIn('startup_dispatch');boolOut('startup_dispatch','detect','S0');boolOut('startup_dispatch','existing','已有状态');link('startup_zero_no','startup_dispatch_in','非零力','control');
  block('startup_valid','S0：观测有效驻留','双腿长均 > 0.05 m，连续 50 tick（约 50 ms）\n不满足则清 prone_valid 并继续等待。',720,60,638,690,'decision');boolIn('startup_valid');boolOut('startup_valid','ready','达到有效期');boolOut('startup_valid','waiting','继续等待');link('startup_dispatch_detect','startup_valid_in','S0','control');
  block('startup_wait','S0 · 等待有效观测','未达到有效驻留时保持 S0。\n最终输出闸阻断所有力矩/腿长力。',720,320,638,690,'control');boolIn('startup_wait');port('startup_wait','startup_wait_state','output','当前状态 S0','mode');link('startup_valid_waiting','startup_wait_in','无效或驻留不足','control');
  block('startup_existing','继续既有状态流程','不重新执行 S0 位形分选。\n按 S1～S7 的当前状态执行动作和转移判断。',2200,320,714,1184,'note');boolIn('startup_existing');port('startup_existing','startup_existing_state','output','既有模式上下文','mode');link('startup_dispatch_existing','startup_existing_in','非 S0','control');
  block('startup_tilt','明显翻倒位形？','|pitch| > 1.2 rad 或 |roll| > 1.2 rad\n这条优先于折叠与普通起立判定。',1090,60,661,665,'decision');boolIn('startup_tilt');boolOut('startup_tilt','yes','是 → S7');boolOut('startup_tilt','no','否');link('startup_valid_ready','startup_tilt_in','S0 有效后','control');
  block('startup_fold','折叠低位位形？','L_avg < rec_fold_l_th（当前 0.13 m）\n双腿 |θ_body| < rec_fold_th（当前 0.85 rad）\n并且 |roll| < 0.6 rad；以上全部满足。',1460,60,666,672,'decision');boolIn('startup_fold');boolOut('startup_fold','yes','是 → S6');boolOut('startup_fold','no','否');link('startup_tilt_no','startup_fold_in','否则','control');
  block('startup_prone','需要常规起立？','L_avg < 0.13 m，或任一 |θ_body| > 0.6 rad\n或 |pitch| > 0.5 rad，或 |roll| > 0.6 rad\n以上任一满足 → S1；否则 → S5。',1830,60,673,684,'decision');boolIn('startup_prone');boolOut('startup_prone','yes','是 → S1');boolOut('startup_prone','no','否 → S5');link('startup_fold_no','startup_prone_in','否则','control');
  for(const [state,label,desc,x,y,line,end] of [
    [1,'S1 · 腿长整理','腿长斜坡到 rec_l_rise，锚定摆角\n双腿长误差 < rec_eps_len，驻留 rec_hold_ticks → S2\n超 rec_stage_timeout → S7 并解除武装。',350,550,809,852],
    [2,'S2 · 双腿单向对齐','按规定正向对齐，短路程腿作锚\n双腿对 end_l/end_r 的误差 < rec_align_eps 并驻留 → S3\n超时 → S7。',720,550,854,900],
    [3,'S3 · 蹬地与收腿','双腿追赶式转到 end_l/end_r，联动收腿\n仅双腿角误差 < rec_align_eps 并驻留 → S4\n不额外等待腿长到位；超时 → S7。',1090,550,902,979],
    [4,'S4 · LQR 混合接管','进入沿重锚 s/yaw/腿角并重启 GRF 静默期\nrec2_ramp = tick/(rec_lqr_ramp_t·1000)\nramp 达 1 → S5。',1460,550,1027,1069],
    [5,'S5 · 起立与正常控制','LQR 已接管，腿长从低位斜坡升起\n腿角目标缓慢收零，到顶后交接遥控腿长挡位\n保护监控另行按开关/阈值判定。',1830,550,1071,1139],
    [6,'S6 · 折叠位形快转','允许最短/反向角路径，腿长保持 rec_l_low\n双腿角误差 < rec_align_eps 并驻留 → S4\n超时 → S7。',1460,1040,981,1025],
    [7,'S7 · 翻身等待与动作','未武装先脱力；prot_seen_zf 或 prot_force_arm 触发武装\n武装后伸腿造支点，追赶式摆角\n翻正/双腿回窗并驻留 → S1；超时回脱力等待。',350,1040,714,777],
  ]){
    const key=`startup_s${state}`;block(key,label,desc,x,y,line,end,'control');boolIn(key,'enter','进入 / 转移',true);port(key,`${key}_state`,'output',`当前状态 S${state}`,'mode');
  }
  link('startup_tilt_yes','startup_s7_enter','翻倒','control','|pitch| > 1.2 || |roll| > 1.2');link('startup_fold_yes','startup_s6_enter','折叠快转','control','三组折叠条件全部成立');link('startup_prone_yes','startup_s1_enter','常规恢复','control');link('startup_prone_no','startup_s5_enter','直立基线','control');
  for(const [from,to,label] of [[1,2,'腿长到位并驻留'],[2,3,'腿角对齐并驻留'],[3,4,'腿角到位并驻留'],[4,5,'ramp 达 1'],[6,4,'角到位并驻留'],[7,1,'已武装且翻正并驻留']]){
    const output=boolOut(`startup_s${from}`,'advance',label);link(output,`startup_s${to}_enter`,label,'control');
  }
  for(const state of [1,2,3,6])link(boolOut(`startup_s${state}`,'timeout','阶段超时'),'startup_s7_enter','超时 → 等确认','control','rec6_t > rec_stage_timeout');
  block('startup_monitor','T3：运行中翻倒保护','prot_enable 开启且处于 S4/S5，双腿观测有效\n任一 |θ_body| > prot_theta_th 持续 prot_hold\n进入 S7，清武装/确认与恢复锚定。',1830,1040,1155,1184,'decision');contextIn('startup_monitor');boolOut('startup_monitor','trigger','保护触发');link(inContext,'startup_monitor_ctx','并行保护监控','control');link('startup_monitor_trigger','startup_s7_enter','S4/S5 保护沿','control');
  block('startup_state','模式上下文输出','图中状态与转换说明对应源码，不在浏览器执行。\n非 S0 时继续既有状态流程；保护开关/驻留独立生效。\n返回 state / armed / air，供上层输出选择使用。',2200,550,638,1188,'note');port('startup_state','startup_state_in','input','当前状态','mode','','',true);port('startup_state','startup_state_out','output','模式上下文','mode');
  for(const state of [1,2,3,4,5,6,7])link(`startup_s${state}_state`,'startup_state_in',`S${state}`,'control',`prone_rec_state == ${state}`);
  link('startup_wait_state','startup_state_in','S0 等待','control','prone_rec_state == 0');link('startup_existing_state','startup_state_in','原状态 / 后续转移','control','prone_rec_state != 0');
  link('startup_state_out',outContext,'回到上层条件选择','control');
  p.view.scenes[sceneId].viewport={x:30,y:160,zoom:0.45};
  p.revision=0;validateProject(p);return p;
}
