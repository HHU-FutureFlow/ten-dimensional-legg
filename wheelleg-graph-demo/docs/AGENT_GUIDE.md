# Agent 使用指南

本工具记录维护者整理的工程结构。agent 负责理解目标工程、读取源码并维护图；工具提供可验证的编辑入口。修改图不产生对固件的修改授权。

内容编写先读 [说明写作指南](WRITING_GUIDE.md)：普通节点按接收/处理/输出/判定填写，术语和状态编号先在零端口说明板解释。不要只复述代码符号，也不要为了连成一条链而接上并不实际使用的数据。

## 定位工具和服务

在包含 `package.json`、`shared/` 和 `scripts/graph-cli.mjs` 的工具目录运行命令。目录可以任意迁移；不要使用示例开发者的绝对路径。先按 README 启动服务。

默认连接本机 4317。远端设置 `GRAPH_URL`，需要认证时设置 `GRAPH_ACCESS_TOKEN`。`--url` 可以覆盖地址。先执行：

```sh
npm run graph -- list
npm run graph -- inspect --project PROJECT_ID
npm run graph -- inspect --project PROJECT_ID --scene mode-detail
```

若需要纯 JSON stdout 给程序解析，直接使用 `node scripts/graph-cli.mjs ...`，避免 npm 的脚本标题混入 stdout。

## 修改流程

1. 核实用户要维护哪个项目、哪些节点、是否保留布局。需要实际源码证据时自行读取工程；不要把猜测标记为已实现。
2. 读取目标项目，记录最新 revision 和稳定 ID。可使用 `inspect --project ID --object OBJECT_ID` 精简查询。
3. 编写最小字段补丁，不整图重排、不重建未涉及的节点、不用名称猜 ID。
4. `apply --project ID --file PATCH --dry-run` 校验，然后不带 `--dry-run` 提交同一补丁。
5. 成功后核对返回的项目/revision，说明改了哪些内容，哪些关系仍需确认。

```json
{
  "requestId": "a-new-unique-request-id",
  "baseRevision": 0,
  "actor": "codex",
  "summary": "补充 LQR 功能说明",
  "operations": [
    { "op": "node.update", "id": "lqr", "changes": { "description": "十维反馈产生四路原始力矩，后续仍有模式覆盖。" } }
  ]
}
```

`baseRevision: 0` 和 `lqr` 仅为示例；需要替换为实际读取的版本和 ID。每个新事务生成新的 requestId。删除有关联节点/端口时必须明确 `cascade: true`，并说明影响。

## 冲突和错误

- `REVISION_CONFLICT`：重新 inspect 当前图，按新事实重新生成补丁。保留其他人的节点/连接/位置；不把 baseRevision 改成最新值后盲目重放旧整图。
- `SCHEMA_INVALID` / `INVALID_GRAPH`：根据具体字段修复补丁。方向、维度、端口存在性由工具验证；物理正确性仍由维护者核实。
- `EXTERNAL_CHANGE`：不要继续覆盖文件；通知使用者核对外部改动，按 README 停止服务处理或导入独立副本。
- 网络失败：先读取当前 revision 和目标对象，判断是否已提交。去重只在当前服务会话的有限窗口内有效。
- `HISTORY_CONFLICT`：不撤销其他 actor 的后续修改。根据当前内容生成显式的小范围修正事务。

## 新增节点、端口与分组

要修改配色，读取块 ID 后通过 `view.node.update` 或 `view.group.update` 修改 `changes.appearance`。可只填写 `background`、`header` 或 `border` 中需要修改的一项，多选统一配色使用一个事务。具体格式见 [API 契约](API.md)。

说明板使用 newBoard(label,id,sceneId) 或 kind=annotation 的完整节点值，正文在 description，颜色在 metadata.boardTone；不添加端口或子视图。功能说明的 metadata.explanation 包含 receives、process、produces、decisions；更新 metadata 前保留旧参数和证据。说明板、说明及布局一并随整图导出。

完整字段见 [数据格式](DATA_FORMAT.md) 和 `schema/`。领域层的 `newNode`、`newPort`、`newEdge` 可用于生成符合格式的值；它们可以由 Node.js 脚本导入 `shared/model.ts`，不要求手写所有默认字段。

当前格式为 v2，不接受旧 v1。新增内部节点必须填写所在 sceneId，或使用 `newNode(label,id,sceneId)`；不要把当前视图的新增节点误放进总览。

创建子视图使用 `scene.add`，value 含 id、label、description、ownerNodeId。接口边界自动生成，不手改 detailSceneId 或 parentPortId。先查询该层的边界端口 ID，再建立内部连线；父输入在子层为输出，父输出在子层为输入。新增/修改父端口会同步映射，内部类型不兼容时需要一笔事务一起修正。

所有节点、分组、连线与布局按当前层操作。父子导航由工具界面负责，agent 用 inspect --scene 获取范围；导出是全工程。scene.remove 或删除拥有子视图的节点需显式 cascade。源码可填写 line/endLine 范围，卡片会直接显示；不凭旧行号声称已重新核实代码。

添加节点可以在同一事务内继续新增端口和布局。图中的反馈回路允许存在。分组是抽象层级；源码文件、函数与节点不是强制一一对应。

源码引用以 rootId 和相对路径保存；需要时同一事务用 `project.update` 增补 sourceRoots 标签。不得写入自己的用户名、源码绝对路径或服务令牌。参数/公式只作为数据保存。

## 导入导出与交接

```sh
npm run graph -- export --project PROJECT_ID --out ./shared.graph.json
npm run graph -- import --file ./shared.graph.json
```

导入创建独立 projectId。队友互传文件不是实时共同编辑；需要共编就连接同一服务。

交接至少留下：项目 ID、当前 revision、修改对象 ID、源码证据/待确认点、实际提交结果和任何保留的草稿。不要求维护者具备某种 agent 全局 skill 配置。
