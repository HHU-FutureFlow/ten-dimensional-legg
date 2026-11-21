import { applyOperations, newPort, newEdge, boundaryPortId, validateProject } from '../shared/model.ts';
import { addLqrBoards } from './lqr-boards.mjs';

// Human explanations of the inspected a7d8f90 firmware, not an automatic code parser.
export function explainLqrExample(initial){
  let p=structuredClone(initial);
  const port=(nodeId,key,direction,label,dataType,shape='',unit='')=>{
    p.model.ports[key]={...newPort(nodeId,direction,label,key),dataType,shape,unit,order:Object.values(p.model.ports).filter(v=>v.nodeId===nodeId&&v.direction===direction).length};
  };
  const wire=(from,to,label='',kind='data')=>{const key=`reading_${from}_${to}`;p.model.edges[key]={...newEdge(from,to,key),label,kind};p.view.edges[key]={style:'bezier'};};
  port('observe','obs_leg_feedback','output','腿角、转速与几何反馈','leg-feedback');
  port('observe','obs_aux','output','腿长、姿态与传感器观测','robot-observation');
  port('target','target_leg_goals','output','当前腿角目标','leg-goals');
  port('target','target_leg_lengths','output','左右腿目标长度','float','2','m');
  port('mode','mode_leg_feedback','input','腿角与转速反馈','leg-feedback');
  port('mode','mode_leg_goals','input','当前腿角目标','leg-goals');
  port('air','air_observations','input','腿部与惯性观测','robot-observation');
  port('force','force_observations','input','腿长与横滚观测','robot-observation');
  port('force','force_targets','input','左右腿目标长度','float','2','m');
  port('force','force_context','input','阶段与离地情况','mode');
  port('wheel','wheel_observations','input','左右轮缘速度','robot-observation');
  port('wheel','wheel_context','input','阶段与离地情况','mode');
  port('vmc','vmc_feedback','input','腿几何与关节转速','leg-feedback');
  port('vmc','vmc_context','input','阶段与离地情况','mode');
  for(const [from,to,label] of [['obs_leg_feedback','mode_leg_feedback','角度反馈'],['target_leg_goals','mode_leg_goals','腿角目标'],['obs_aux','air_observations','支撑估计所需观测'],['obs_aux','force_observations','支撑反馈'],['target_leg_lengths','force_targets','目标长度'],['mode_flag','force_context','阶段/离地'],['obs_aux','wheel_observations','轮速反馈'],['mode_flag','wheel_context','阶段/离地'],['obs_leg_feedback','vmc_feedback','关节反馈'],['mode_flag','vmc_context','阶段/离地']])wire(from,to,label);
  // Pure recovery PD does not consume the LQR torque. Show its real feedback dependencies.
  for(const edge of Object.values(p.model.edges))if(edge.targetPortId==='mode_pd_u'){delete p.model.edges[edge.id];delete p.view.edges[edge.id];}
  delete p.model.ports.mode_pd_u;
  p.model.ports.mode_in.label='恢复阶段 / 确认 / 离地';p.model.ports.mode_flag.label='恢复阶段 / 确认 / 离地';p.model.ports.air_flag.label='是否离地';p.model.ports.lqr_air.label='是否离地';
  // Synchronize boundary names and create mappings for the added parent ports.
  p=applyOperations(p,{requestId:'reading-interfaces',baseRevision:p.revision,actor:'example-author',summary:'补足目标与反馈接口',operations:[{op:'project.update',changes:{title:p.title}}]}).project;
  for(const node of ['mode_pd','mode_ramp','mode_air_apply']){
    port(node,`${node}_goals`,'input','腿角目标','leg-goals');port(node,`${node}_feedback`,'input','腿角与转速','leg-feedback');
    wire(boundaryPortId('mode_leg_goals'),`${node}_goals`,'读取目标');wire(boundaryPortId('mode_leg_feedback'),`${node}_feedback`,'读取反馈');
  }
  const explain=(key,title,summary,receives,process,produces,decisions=[])=>{
    const node=p.model.nodes[key];if(!node)throw new Error(`Missing explanation target: ${key}`);
    node.label=title;node.description=summary;node.metadata={...node.metadata,explanation:{receives,process,produces,decisions}};
    if(node.detailSceneId)p.model.scenes[node.detailSceneId].label=title;
  };
  explain('observe','状态观测','把传感器反馈整理成控制器能够使用的状态。',
    '左右轮和四个关节的编码器、反馈力矩，以及惯性测量单元的姿态、转速和加速度。',
    '由轮/腿运动学得到位移、速度、腿角和腿长；机身俯仰使用惯性姿态。底盘任务随后用轮差、腿角差重构偏航。',
    '十维实际状态、左右腿长，以及供腿部控制、接地判断和补偿使用的观测包。');
  p.model.nodes.observe.sourceRefs.push({rootId:'firmware',path:'application/chassis/chassis.c',line:482,endLine:543,symbol:'ChassisTask',revision:'a7d8f90'});
  explain('target','控制目标','把操作意图和恢复阶段要求变成具体的控制目标。',
    '遥控前进/转向意图、行驶或锁定模式、当前位置/航向，以及恢复流程要求。',
    '行驶时生成速度目标并积分转向目标；松杆或锁定时冻结相应位置/航向。恢复过程再按当前阶段覆盖腿角和腿长目标。',
    '十维参考状态、当前腿角目标和左右腿目标长度。',
    ['有前进速度命令 → 位移目标跟随当前位置，让速度控制起作用。','松杆或锁定 → 保持冻结的位置/航向目标。','恢复未完成 → 使用该阶段给定的腿角/腿长；完成起立后再接遥控腿长挡位。']);
  p.model.nodes.target.sourceRefs.push({rootId:'firmware',path:'application/chassis/chassis.c',line:638,endLine:1139,symbol:'ChassisTask',revision:'a7d8f90'});
  explain('gain','腿长选 K','选择与当前腿长匹配的一组平衡反馈系数。',
    '左右腿的实际长度，单位米；并保留上一次使用的参数组。',
    '取两腿平均长度，在 0.15～0.40 m 的六组参数间选带；使用 1 cm 迟滞，减少边界附近来回切换。',
    '一个 4×10 的 K 矩阵：规定每个状态误差分别怎样影响四路力矩。',
    ['跨过当前带的相邻边界，并满足迟滞距离 → 切到相邻参数组。','仍在当前带或迟滞范围内 → 保留原参数组。']);
  explain('air','接地观测与阶段信息','判断是否离地，并汇集后续控制需要的阶段和确认标志。',
    '腿部几何、关节反馈力矩、竖直加速度；另读取保留的恢复阶段与翻身确认标志。',
    '估计左右轮的支撑力，再用平均支撑力、连续达标时间和迟滞判断接地；恢复阶段会屏蔽不适用的离地判断。',
    '是否离地，以及“当前恢复阶段 / 翻身是否已确认 / 是否离地”的信息。',
    ['上电或重启检测后的 3 s 内 → 暂不判离地。','原来接地，平均支撑力 < 30 N 连续约 100 ms → 判离地。','原来离地，平均支撑力 > 40 N 连续约 25 ms → 判接地。','腿长整理、对齐、蹬地、交接或折叠快转阶段 → 底盘本地暂按非离地处理。']);
  explain('lqr','十维状态反馈','根据“实际状态离目标有多远”，计算四路原始纠偏力矩。',
    '十维实际状态、十维目标、当前腿长的 K 参数，以及是否离地。',
    '先逐项求实际值减目标值，再按 K 加权相加并取负号；各状态误差共同决定轮和腿需要的纠偏力矩。',
    '左/右轮力矩 Tw 和左/右腿摆动力矩 Tp，均为 N·m；后续模式处理仍可能改写。',
    ['接地时 → 使用完整的十维反馈。','离地时 → 轮力矩清零；腿反馈不再使用轮里程计相关的位移、速度、偏航通道。']);
  explain('mode','模式处理','按恢复、交接、离地和等待条件，决定真正送往执行端的输出。',
    '原始四路 LQR 力矩、独立腿长力、腿角目标/反馈，以及阶段、翻身确认、离地标志。',
    '先选择恢复 PD、平滑交接或普通 LQR；之后单独执行离地覆盖；最后检查是否允许输出。',
    '经过选择和门控的轮力矩、腿摆动力矩与腿长力。',
    ['整理/对齐/蹬地/折叠快转，或已确认的翻身 → 腿用恢复 PD，轮停止。','平衡接管阶段 → PD 逐渐退出、LQR 逐渐进入。','之后若离地 → 再按悬空规则覆盖；等待观测或等待翻身确认 → 最终输出清零。']);
  explain('wheel','轮端处理','把允许使用的轮力矩变成左右轮电机指令。',
    '模式处理后的轮力矩、左右轮缘线速度，以及当前恢复阶段和离地标志。',
    '允许驱动时按轮速方向叠加摩擦补偿、限制力矩，再换算成电流码；下发前检查数值是否有限。',
    '左右轮电机的控制参考值。',
    ['离地，或仍在观测等待/恢复 PD/翻身/折叠快转阶段 → 轮指令为零，跳过摩擦补偿。','允许驱动 → 使用补偿并限幅后的轮力矩。','结果出现 NaN 或无穷大 → 对应指令改为零。']);
  explain('force','腿长力控制','根据腿长误差和机身侧倾，为两条腿分配伸缩方向的力。',
    '左右腿目标长度、实际长度及其变化，机身横滚角/角速度，以及恢复阶段和离地标志。',
    '腿长误差产生伸缩力，腿长变化速度提供阻尼；再叠加重力支撑和左右差动的横滚补偿，并限制误差控制分量。',
    '左右腿轴向力 F_leg，单位 N；它与摆腿力矩 Tp 不同，之后还要经过输出闸。',
    ['腿长整理 → 重力前馈为零；其他恢复 PD 阶段 → 用恢复专用支撑量。','平衡接管 → 支撑前馈与横滚补偿随交接比例渐入。','离地 → 关闭重力前馈和横滚补偿，腿长误差控制继续维持长度。']);
  explain('vmc','VMC 与关节执行','把“沿腿施力”和“让腿摆动”分配给四个关节电机。',
    '左右腿摆动力矩 Tp、腿长力 F_leg、当前腿部几何和关节转速，以及输出许可。',
    '利用当前几何将虚拟力/力矩映射成前后关节力矩，处理左右坐标方向；允许输出时再叠加关节摩擦前馈。',
    '四个关节电机各自坐标系下的力矩参考值，单位 N·m。',
    ['等待观测或翻身未确认 → 上游清零且不叠加关节摩擦前馈。','离地且不是翻身动作 → 使用悬空摩擦系数；当前该系数为零。','下发结果非有限值 → 对应电机参考值改为零。']);

  explain('mode_classify','模式判定与状态转换','决定恢复流程现在处在哪一步；进入子视图可看完整判定。',
    '急停命令、机身/腿部观测、保留的阶段编号，以及是否已确认翻身动作。',
    '急停优先返回；只有初始化阶段重新按姿态选路线，其他阶段执行自己的动作和转移条件。运行中翻倒监控另受开关控制。',
    '当前恢复阶段、翻身确认和离地信息，供后面的输出选择使用。',
    ['零力急停 → 本拍任务提前结束。','初始化且观测有效 → 选择翻身等待、折叠快转、常规起立或正常控制。','已在某阶段 → 检查该阶段的到位、驻留和超时结果。']);
  explain('mode_pack','收纳四路原始力矩','这只是图内接口打包，源码里没有新增计算。',
    'LQR 刚算出的左/右轮力矩和左/右腿摆动力矩。',
    '按固定顺序收纳为同一组数，减少图上的重复连线；不改变任何值。',
    '四路原始反馈力矩，供交接混合或普通反馈路径使用。');
  explain('mode_branch','选择当前控制方式','先辨认恢复阶段，再选择三条互斥路径中的一条。',
    '当前恢复阶段，以及翻身动作是否已获操作手确认。',
    '先检查是否应由恢复 PD 接管；不满足再检查是否进入平衡交接；仍不满足则保留普通 LQR。',
    '“恢复 PD / 平滑交接 / 普通反馈”三条路径的选择结果。',
    ['腿长整理、对齐、蹬地或折叠快转；或翻身且已确认 → 选恢复 PD。','前一条件不成立，且处于平衡接管（S4）→ 选平滑交接。','其余情况 → 保留原始 LQR；后面仍要检查离地和输出闸。']);
  explain('mode_pd','恢复时用腿角 PD 接管','腿按恢复目标移动，轮子暂不参与驱动。',
    '恢复分支选择结果、两腿的目标摆角、实际摆角和摆动速度；不使用原始 LQR 力矩。',
    '按角度偏差产生纠偏力矩，用角速度抑制摆动；跨角度边界时取正确角差，并限制力矩幅值。',
    '左右腿 PD 力矩；左右轮力矩直接为零。',
    ['翻身且已确认 → 使用翻身专用目标和增益。','整理、对齐、蹬地或折叠快转 → 使用恢复目标和增益。']);
  explain('mode_ramp','平滑切换到 LQR','让原来的支撑控制逐渐退出，避免交接时突然失去支撑。',
    '原始 LQR 力矩、仍在工作的恢复 PD 目标/反馈，以及从 0 增至 1 的交接比例。',
    '腿力矩按“PD×(1−比例) + LQR×比例”混合；轮力矩按比例渐入。',
    '连续变化的左右腿力矩和逐渐启用的左右轮力矩。',
    ['进入平衡接管（S4）→ 比例从零开始。','比例达到 1 → 交给起立与正常控制阶段（S5）。']);
  explain('mode_keep','使用原始 LQR 输出','未进入恢复接管或交接混合时，先保留原始反馈。',
    '四路原始 LQR 力矩，以及普通反馈路径已被选中的结果。',
    '四路数值直接向后传递；这里不叠加新的补偿，也不保证最终一定下发。',
    '未经本阶段改写的四路力矩，仍需后续离地覆盖和门控。');
  explain('mode_merge','取被选中的阶段输出','三条路径中只有当前条件命中的结果继续向后传。',
    '恢复 PD、交接混合、普通 LQR 三种候选结果及各自生效条件。',
    '按前面互斥的选择条件取一条结果，不把三条力矩相加。',
    '当前阶段的四路力矩。',
    ['本图的多条输入表示候选路径；源码通过 if / else-if 选择其中之一。']);
  explain('mode_air_check','现在是否离地？','离地判断发生在阶段输出选择之后，可能再次覆盖刚才的结果。',
    '上游给出的是否离地标志；其支撑力阈值、驻留和屏蔽规则见说明板。',
    '检查离地标志是否为真。',
    '离地 → 悬空处理；未离地 → 接地/落地沿处理。',
    ['离地 → 轮子不驱动，并检查是否改用空中腿角控制。','未离地 → 保留阶段输出；若刚落地，重置位置与航向参考。']);
  explain('mode_air_apply','离地时停轮并保持腿角','没有地面支撑时，轮驱动清零，腿按悬空规则保持目标。',
    '阶段输出、是否离地、当前阶段、腿角目标和角度/角速度反馈。',
    '先将左右轮力矩清零。普通悬空时用腿角 PD 改写腿力矩并限幅，同时记录需要在落地沿重新锚定参考。',
    '轮力矩为零、腿力矩按悬空规则处理后的四路结果。',
    ['离地且不是翻身阶段（S7）→ 腿力矩改用悬空 PD。','离地且正在翻身 → 不覆盖翻身动作的腿力矩，轮仍为零。']);
  explain('mode_ground','接地时保留输出，落地沿重锚','把空转造成的位置/航向参考漂移隔离在落地时处理。',
    '阶段输出、未离地的结果，以及是否记录过此前离地。',
    '力矩不再改写；如果是离地后的首次接地，就把位置和航向目标设为当前估计值，然后清除落地沿标志。',
    '原阶段力矩；在落地沿另外更新位置与航向目标。',
    ['此前离地、现在接地 → 重锚一次。','一直接地 → 直接保留输出，不反复重设目标。']);
  explain('mode_output_gate','是否允许向电机输出？','最后一道等待门：条件未满足时，所有虚拟输出都清零。',
    '已完成阶段/离地处理的四路力矩、独立腿长力、当前阶段和翻身确认状态。',
    '检查是否还在等有效观测，或翻身动作尚未确认；满足等待条件就清零轮、腿摆动和腿长三个通道。',
    '允许通过的轮力矩、腿摆动力矩和腿长力；或全部为零。',
    ['初始化判定尚未结束（S0）→ 全部为零。','翻身等待（S7）但未确认 → 全部为零。','其余情况 → 保留前面结果，后续补偿也遵守该门控。']);
  explain('mode_unpack','分送轮、腿和支撑输出','把已通过门控的结果接到正确的执行支路。',
    '最终四路力矩；腿长力通过独立通道传递。',
    '按既定顺序拆成左右轮一组、左右腿摆动力矩一组；图内整理不产生额外计算。',
    '轮力矩送轮端处理；腿摆动力矩与腿长力送 VMC。');

  explain('startup_zero','是否收到零力急停？','先判断能不能继续本拍控制，避免急停时仍运行恢复动作。',
    '命令层给出的底盘工作模式。',
    '检查工作模式是否为零力；零力表示让执行端停止主动出力。',
    '是 → 停机并提前返回；否 → 继续阶段判断。',
    ['工作模式是零力 → 提前返回。','非零力 → 本拍继续。']);
  explain('startup_return','停机并结束本拍','本拍到这里就结束，不继续计算或下发后面的控制结果。',
    '零力急停成立的判定；另外保存恢复沿需要的模式锁存信息。',
    '设置电机停止标志并 return；切出零力时会清旧模式锁存，必要时重新做初始化判定。',
    '本拍不再向后续控制链提供有效输出。');
  explain('startup_dispatch','需要重新选择起立路线吗？','只有初始化阶段需要从当前位形重新决定起立路线。',
    '非零力条件和当前恢复阶段编号。',
    '检查是否处于初始化（S0）。已经在恢复或正常控制中的机器人，不每拍重做初始分选。',
    'S0 → 等待有效观测并分选；非 S0 → 继续既有阶段。',
    ['阶段为 S0 → 走初始姿态判断。','阶段不是 S0 → 走已有阶段的动作与转移。']);
  explain('startup_valid','先等观测可信（S0）','防止上电时读到尚未就绪的腿长，就立刻选择动作。',
    '左右腿的实际长度，以及连续有效观测的计数。',
    '两条腿长度都必须大于 5 cm，连续达到约 50 ms；任意一条无效就清零计数。',
    '观测已稳定的结果，或者继续等待的结果。',
    ['两腿都 > 5 cm 且连续约 50 ms → 可以判断姿态。','一腿无效或时间不足 → 继续 S0，最终输出闸保持不出力。']);
  explain('startup_wait','保持等待，不输出（S0）','观测没准备好时不抢先执行起立动作。',
    '观测无效或连续有效时间不足的结果。',
    '维持初始化等待；不把短暂的错误长度当作起立依据。',
    '当前阶段仍为 S0；最终门控清零轮、腿力矩和腿长力。');
  explain('startup_existing','继续当前恢复阶段','从保留的阶段继续执行，不把流程每拍重置到开头。',
    '已经不是 S0 的恢复阶段，以及该阶段所需的目标与反馈。',
    '进入对应阶段的动作与退出判断；这里汇总的是已有状态流程，不是额外控制器。',
    '本拍阶段处理后的阶段编号与相关标志。');
  explain('startup_tilt','机身是否明显翻倒？','初始姿态分选的第一优先级：先识别明显翻倒。',
    '机身俯仰角和横滚角，已经换成弧度。',
    '分别取两个角度的绝对值，与 1.2 rad（约 69°）比较。',
    '是 → 翻身等待（S7）；否 → 再检查折叠姿态。',
    ['俯仰或横滚任意一个绝对值 > 1.2 rad → S7。','两个都不超过阈值 → 继续下一条分选。']);
  explain('startup_fold','是否适合折叠短路起立？','腿已经收拢、轮心在机身下方时，优先走短路径转腿。',
    '两腿平均长度、两腿相对机身的摆角，以及机身横滚角。',
    '检查是否同时满足短腿、两腿都在允许摆角内、侧倾不大的三个条件。',
    '全部满足 → 折叠快转（S6）；否则 → 判断常规起立。',
    ['平均腿长 < 0.13 m、两腿摆角绝对值都 < 0.85 rad、横滚绝对值 < 0.6 rad，三项全满足 → S6。','任一不满足 → 继续检查常规起立，不能只凭腿短就走此路线。']);
  explain('startup_prone','是否需要完整起立流程？','排除翻倒和折叠短路后，再判断机器人是否已经适合正常控制。',
    '平均腿长、两腿机身系摆角、机身俯仰和横滚。',
    '检查长度过短、腿偏离竖直过大或机身倾斜过大的任一条件。',
    '需要恢复 → 腿长整理（S1）；都不需要 → 起立/正常控制（S5）。',
    ['平均腿长 < 0.13 m；或任一腿摆角绝对值 > 0.6 rad；或俯仰 > 0.5 rad；或横滚 > 0.6 rad（均取绝对值），任一成立 → S1。','以上全部不成立 → S5。此规则只在 S0 分选时执行。']);
  explain('startup_s1','整理双腿长度（S1）','先把腿长整理到站立目标，为后面的两腿对齐准备位形。',
    '左右腿实际长度、站立目标长度，以及进入本阶段时的腿摆角。',
    '目标长度从当前实际值逐步逼近 0.25 m；腿摆角锚定保持。每拍检查两腿长度是否都接近目标。',
    '连续变化的左右腿长目标、保持的摆角目标，以及下一阶段选择。',
    ['阶段用时 > 8 s → 转入翻身等待 S7 并取消武装。','未超时，两腿长度误差都 < 3 cm 并连续保持 0.5 s → 双腿对齐 S2。','其余 → 继续整理；一条腿误差出带就清零达标计时。']);
  explain('startup_s2','把两腿转到一致摆角（S2）','用规定方向对齐两腿，避免两腿从不同位置开始蹬地。',
    '两腿连续机身系摆角、各自的对齐目标，以及达标时间。',
    '比较按规定方向转到目标的路程，以路程短的一腿作锚；另一腿追赶到相同摆角。',
    '两腿对齐过程的目标角；完成后选择蹬地阶段 S3。',
    ['阶段用时 > 8 s → 翻身等待 S7。','未超时，两腿目标角误差都 < 0.10 rad（约 5.7°）并保持 0.5 s → S3。','其余 → 继续对齐；任一角误差出带就清零达标计时。']);
  explain('startup_s3','蹬地抬起机身（S3）','让两腿协同转到交接角，同时按角度联动收腿。',
    '两腿目标/实际摆角、当前长度、达标时间与阶段用时。',
    '以追赶式摆角目标推动机身，同时限速收腿；退出判断只看腿角和驻留，不额外等腿长到位。',
    '本拍腿角/腿长目标，以及是否转入平衡接管的结果。',
    ['阶段用时 > 8 s → 翻身等待 S7；这一超时处理会覆盖本拍的到位转移结果。','未超时，两腿角误差都 < 0.10 rad 并连续保持 0.5 s → 平衡接管 S4，同时把交接比例清零。','其余 → 继续蹬地；任一腿角误差出带，达标时间重新累计。']);
  explain('startup_s4','平滑进入平衡控制（S4）','用一段渐变交接，避免恢复控制突然撤掉而平衡控制尚未接上。',
    '进入时的实际位置、航向、腿角，以及交接经过的时间。',
    '先锚定参考值并重启接地检测静默期；保持低腿长，将交接比例在约 0.4 s 内从 0 增到 1。',
    '保持的参考目标、低位腿长和交接比例；比例用于外层 PD/LQR 混合。',
    ['比例达到 1 → 起立/正常控制 S5。','未达到 1 → 留在 S4，持续平滑混合。']);
  explain('startup_s5','站起并保持正常控制（S5）','平衡已经接管，再逐步升高并交给正常遥控控制。',
    '实际腿长、起立高度、锚定腿角和遥控腿长挡位。',
    '腿长从低位逐步升起，腿角目标缓慢收零；完成起立后再跟随遥控高度，减少切换阶跃。',
    '正常平衡期间持续更新的目标；没有一个“完成就结束任务”的返回动作。',
    ['起立高度尚未达到 → 继续爬升，不立刻跳到遥控高度。','到顶并满足接管逻辑 → 目标跟随遥控腿长挡位。','运行中翻倒监控只有开关开启才可能触发 S7；当前源码默认关闭。']);
  explain('startup_s6','折叠姿态下短路径转腿（S6）','已经收拢的腿直接短转到交接角，省去完整伸腿与对齐路线。',
    '当前两腿摆角、交接目标角、达标时间和阶段用时。',
    '允许最短/反向转动到目标；腿长保持低位，不先做伸腿顶升。',
    '短转过程的摆角目标与固定低腿长目标。',
    ['阶段用时 > 8 s → 翻身等待 S7。','未超时，两腿角误差都 < 0.10 rad 并保持 0.5 s → 平衡接管 S4。','其余 → 继续转到目标，达标时间按连续满足条件计算。']);
  explain('startup_s7','等待确认后翻身（S7）','翻倒后先等待确认，确认后才执行翻身动作。',
    '是否经历零力再切出、当前机身/腿部姿态，以及翻身动作的时间。',
    '未确认时不出力；确认后伸腿造支点并按追赶式角度目标摆腿，持续检查是否翻正。',
    '等待或翻身所需的目标；翻正后接回腿长整理 S1。',
    ['未确认 → 保持 S7 并由最终门控清零输出。','确认后，俯仰绝对值 < 0.3 rad、两腿摆角绝对值都 < 1.2 rad，连续保持 0.5 s → S1。','仍未满足翻正退出、动作时间又 > 8 s → 清武装并留在 S7 等待新确认，不能自动再次启动。']);
  explain('startup_monitor','运行中是否触发翻倒保护？','这是正常控制中的额外监控，不是上电 S0 的姿态分选。',
    '监控开关、当前阶段、两腿有效长度和机身系腿摆角。',
    '只有开关开启且在 S4/S5 才累计越界时间；任一腿摆角持续越界则转入翻身等待并清确认。',
    '保护触发的状态转移，或保持当前阶段。',
    ['当前源码 prot_enable=0 → 整段监控不运行。','开启后，在 S4/S5 且两腿长度有效，任一摆角绝对值 > 1.3 rad 连续约 0.2 s → S7。','不满足阶段或越界条件 → 不触发；越界计数按代码清零。']);
  explain('startup_state','记录本拍的阶段结果','把判定与阶段动作对同一组状态变量的影响交给上层使用。',
    '当前有效的阶段编号、翻身确认和离地标志；多条线表示带条件的候选来源。',
    '按真实阶段取对应结果，不把多个阶段同时执行，也不把编号相加。',
    '给上层控制选择使用的阶段/确认/离地信息。',
    ['图是代码逻辑说明，不在浏览器执行控制，也不是逐周期仿真。']);

  for(const node of Object.values(p.model.nodes).filter(node=>node.kind.startsWith('boundary-'))){
    const input=node.kind==='boundary-input';
    explain(node.id,node.label,input?'把父功能块接收的数据带入这一层。':'把这一层处理完的数据交回父功能块。',
      input?'父功能块外部接收的输入；名称、类型、单位与父端口保持一致。':'内部节点送来的处理结果。',
      '边界仅表示接口对应关系，不计算、不复制创建新的控制行为。',
      input?'作为本层的输出端口，供内部功能块连接。':'送回父功能块对应的输出端口，供上层继续连接。');
  }
  for(const node of Object.values(p.model.nodes)){
    const view=p.view.nodes[node.id];view.width=node.kind.startsWith('boundary-')?300:360;
    if(node.sceneId===p.rootSceneId)continue;
    view.position={x:view.position.x*1.35,y:view.position.y*1.8+300};
    if(node.sceneId==='startup-detail'&&view.position.y>=1200)view.position.y+=100;
  }
  const rootPositions={observe:[0,120],target:[0,780],gain:[500,780],air:[500,-220],lqr:[1000,280],mode:[1500,280],wheel:[2000,0],force:[1000,1200],vmc:[2000,1000]};
  for(const[key,[x,y]]of Object.entries(rootPositions))p.view.nodes[key].position={x,y};
  for(const scene of Object.values(p.model.scenes))p.view.scenes[scene.id].viewport={x:680,y:80,zoom:0.8};
  for(const edge of Object.values(p.model.edges)){
    const source=p.model.ports[edge.sourcePortId],target=p.model.ports[edge.targetPortId];
    if(source.unit==='N'||source.unit==='N·m')edge.kind='force';
    else if(/^startup_s[1-7]_enter$/.test(target.id))edge.kind='transition';
    else edge.kind=source.dataType==='bool'?'control':'data';
  }
  addLqrBoards(p);
  p.revision=0;validateProject(p);return p;
}
