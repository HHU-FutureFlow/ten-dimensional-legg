# 工程图数据格式 v2

权威约束为 [项目 Schema](../schema/project.schema.json)、[事务 Schema](../schema/patch.schema.json) 和 `shared/model.ts` 的关系校验。示例见 `examples/blank.graph.json`、`examples/lqr.graph.json`。

项目文件由 schemaVersion=2、projectId、title、revision、rootSceneId、sourceRoots、model、view 组成。model 和 view 位于同一 JSON 文档中，保证一个版本整体保存。按使用者要求，本版不兼容或迁移 v1 数据。

## 模型与布局

- `model.nodes`：功能名称、说明、类型、状态、源码引用、sceneId、可选 detailSceneId 和自由 JSON metadata。
- `metadata.explanation`：可选结构化说明，必须完整包含 receives/process/produces 字符串与 decisions 字符串数组。其他 metadata 字段仍可自由保存。
- `model.ports`：端口所属节点、输入/输出方向、类型、维度 shape、单位、坐标系、顺序和 multiplicity。
- `model.edges`：源端口、目标端口、信号名、数据/控制/力类别、生效条件和说明。
- `model.groups`：同一视图内的功能分组，含 sceneId 及可选 parentGroupId。节点通过可选 groupId 加入分组。
- `model.scenes`：独立视图的 id、label、description；子视图另有 ownerNodeId，与所属节点的 detailSceneId 相互对应。
- `view.nodes` / `view.groups`：画布世界坐标和尺寸；组内节点仍保存世界坐标。
- `view.edges`：bezier / smoothstep / straight 和世界坐标中的中间 waypoints。
- `view.scenes[sceneId].viewport`：该层保存的 x、y、zoom。进入/返回时恢复；队友更新视角不会强迫正在编辑的窗口跳转。

更新说明/参数不会重置布局。节点移动不改变语义连线。收起分组只改变显示，边界连接仍引用原始成员端口。分组层级禁止循环；控制系统的信号反馈回路允许存在。

## 父子视图与边界映射

rootSceneId 指向唯一没有 ownerNodeId 的工程总览。每个普通节点至多拥有一个子视图，子视图通过节点所在 sceneId 回溯上层；整个层级必须连接到总览，禁止循环。节点和分组的 sceneId 必须存在，分组和成员必须同层。

`scene.add` 自动建立两个边界节点。边界端口的 `parentPortId` 指向该视图所属父节点的一个端口，并采用相反方向：父输入 → 子边界输出；子边界输入 → 父输出。

边界名称、类型、维度、单位、坐标系由父端口派生；顺序、说明及内部输入的 multiplicity 可调整。契约字段不直接改写，新增/删除父端口会同步其映射；已有内部连线不兼容时整笔修改拒绝提交。

普通连线只能连接同一 sceneId 的节点端口。跨层传递使用映射，不通过直接连线穿透。scene.remove 需 cascade，删除内部所有后代及布局，保留上层节点；删除拥有子视图的节点同样明确级联。所有层级仍在一个事务/项目 JSON 中保存。

ID 为 1～100 个 ASCII 字母、数字、下划线或短横线。对象键与 ID 一致，同一项目的各类对象不能重名。节点状态为 implemented / design / unverified；implemented 只表示维护者记录的实现状态，不表示已经实测。

shape 使用字符串，例如 `10` 或 `4x10`。两端都明确且类型/维度不一致时拒绝连接；一端未知时允许连接。单位和坐标系不同给出警告，工具不负责物理单位换算或控制意义证明。

## 说明板和关系类别

`kind: "annotation"` 为零端口说明板。description 是 Markdown，metadata.boardTone 可为 amber/teal/slate；不允许添加端口或拥有 detailSceneId。默认 560×420，最小 280×180，尺寸仍保存在 view.nodes。

普通 kind=note 仍可用于带接口的概念整理块，和 annotation 分开。说明板参与布局、分组、保存、事务历史和导出，但没有数据连接。

edge.kind 为 data/control/transition/force，分别表示数据、条件结果、状态转移、力或力矩。transition 显示虚线，端口若声明类型须是 bool；不能拿数值力矩线冒充状态转移。

本次仍使用 schemaVersion=2；/api/status 的 capabilities 宣告支持的编辑能力。新浏览器/CLI 检查能力，避免连接缺少说明板等规则的旧服务。

## 块配色

`view.nodes[id].appearance` 和 `view.groups[id].appearance` 都是可选对象，支持 `background`、`header`、`border`，各字段使用 `#RRGGBB`。省略任意字段即可使用对应默认色。说明板原有的 `metadata.boardTone` 继续提供默认颜色。

颜色与位置、尺寸一起保存和导出。旧 v2 文件可以直接读取。

## 便携源码引用

sourceRoots 保存逻辑根 ID 和显示标签，sourceRefs 保存 rootId、相对 path、可选 symbol、line、endLine、revision。endLine 需要 line 且不能小于起始行。路径使用 `/` 分隔，不允许绝对路径、盘符、反斜线和 `..`。工具不会读取源码或自动扫描/更正引用；未绑定源码不妨碍编辑。

如果后续增加源码根目录的本机绑定，应放在独立的忽略配置中，不能改变导出格式为机器绝对路径。

## 版本与分享

- schemaVersion 为文件格式版本；当前只接受 2，v1 和未知新版明确拒绝，文件不会被自动改写。
- revision 为当前服务下项目的修改版本；由服务生成，agent 不能用 project.update 修改它。
- 导出保留模型、布局和导出时 revision；导入创建新 projectId 并从 revision 0 开始。
- 导入保持节点/端口/连接 ID，这些 ID 的唯一性只限于各自项目内。
- 未来更改文件格式时，需要明确兼容策略和版本验证，不能静默覆盖不支持的数据。

服务端写入前保存上一版本备份，临时文件写入并同步后再替换权威 JSON。当前自动测试覆盖普通失败和重启，未进行断电恢复测试。历史备份不是独立共编入口。
