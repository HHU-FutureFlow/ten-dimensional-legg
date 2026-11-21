# API 与 CLI 契约 v2

Node.js 24 服务，默认地址由配置决定。API 为同源 `/api`，JSON 请求使用 `Content-Type: application/json`。启用令牌时附加 `Authorization: Bearer <token>`；不要将令牌放在 URL。

| 方法与路径 | 用途 / 主要参数 |
| --- | --- |
| GET `/api/status` | 公开的服务版本和 authRequired，不返回项目内容 |
| GET `/api/projects` | 列举项目 ID、标题、revision、节点数 |
| POST `/api/projects` | `{title}` 新建；`{template:"lqr"}` 复制示例；`{project}` 导入独立项目 |
| GET `/api/projects/:id` | `{project,warnings,history}`；history 含 undoActor / redoActor |
| GET `/api/projects/:id/export` | 原始项目 JSON，不含服务配置、访问令牌或历史栈 |
| POST `/api/validate` | 校验项目 JSON，返回结构结果和单位/坐标系提示 |
| POST `/api/projects/:id/transactions` | 校验并提交补丁；`?dryRun=true` 不落盘 |
| POST `/api/projects/:id/undo` | `{actor,baseRevision}`，只撤销自己的全局栈顶操作 |
| POST `/api/projects/:id/redo` | `{actor,baseRevision}`，检查相同的历史约束 |
| GET `/api/events` | 经过认证的 SSE；浏览器使用 fetch 流携带认证头 |

事务字段为 requestId、baseRevision、actor、summary、operations，格式由 `schema/patch.schema.json` 定义。

## 领域操作

| 操作 | 格式 |
| --- | --- |
| `node/port/edge/group.add` | `{op,value:{id,...完整字段}}`；节点/分组需指定 sceneId，省略时为工程总览 |
| `node/port/edge/group.update` | `{op,id,changes:{...字段}}`，ID 不可修改 |
| `node/port/edge/group.remove` | `{op,id,cascade?:true}`，有关联或子视图的删除须明确级联 |
| `scene.add` | `{op,value:{id,label,description,ownerNodeId}}`，自动关联父节点并生成接口边界 |
| `scene.update` | `{op,id,changes:{label?,description?}}`，ownerNodeId 不可直接更改 |
| `scene.remove` | `{op,id,cascade:true}`，删除整个后代树并清除上层节点关联；总览不可删除 |
| `view.node.update` | `{op,id,changes:{position?:{x,y},width?,height?,appearance?}}` |
| `view.group.update` | `{op,id,changes:{position?,width?,height?,collapsed?,appearance?}}` |
| `view.edge.update` | `{op,id,changes:{style?,waypoints?:[{x,y}]}}` |
| `viewport.update` | `{op,id:sceneId,changes:{x?,y?,zoom?}}`，省略 id 时为总览 |
| `project.update` | `{op,changes:{title?,sourceRoots?}}` |

节点/端口/连线/分组 ID 在同一项目中必须全局唯一；模型对象键须与其 ID 相同。添加节点/分组会生成默认布局，可在同一事务内改位置。

视图 ID 也参与同一项目的全局唯一性检查。节点的 detailSceneId 和端口的 parentPortId 通过视图/父接口操作维护，不直接改指针；边界的契约字段跟随父端口。普通连线不可跨层，分组不可跨层。

`node.update` 的 `groupId:null` 或 `group.update` 的 `parentGroupId:null` 用于清除可选分组关联。取消分组自动解除成员关联，保留世界坐标。视图字段是字段级浅合并；提供 position 时提交完整 x/y。

原子事务先在副本上应用全部操作，再校验并持久化。失败返回非 2xx，活动项目和 revision 不变。dry-run 返回预期下一版本，但不保留请求去重记录或修改历史。

配色支持逐项合并。通过 `changes.appearance={background:"#193835",header:"#23594f",border:"#5cb3a2"}` 设置所需字段。某个字段设为 `null` 恢复该项默认。`changes.appearance=null` 恢复全部默认。将多个块的更新放在同一事务中，可以统一保存和撤销。v0.4 服务通过 `block-appearance` 能力标记声明支持配色。

## 通知

事件体：`{projectId,revision,actor,summary}`。更新事件不携带完整项目；客户端重新查询。通知断线重连后查询当前状态，不假定事件完整；心跳为 SSE 注释。

客户端不应按收到事件的先后覆盖较新 revision。revision 是服务端单项目单调递增版本。undo/redo 也是新 revision。

## 错误

错误 JSON 为 `{error:{code,message,details?}}`。

| HTTP | 典型 code | 含义 |
| --- | --- | --- |
| 400 | INVALID_JSON / INVALID_PATCH / INVALID_ID | 请求无法处理 |
| 401 | UNAUTHORIZED | 共享令牌缺失或错误 |
| 403 | ORIGIN_REJECTED / HOST_REJECTED | API 请求来源被拒绝 |
| 404 | NOT_FOUND | 接口或项目不存在 |
| 409 | REVISION_CONFLICT | 基于过期版本；details 可包含 currentRevision |
| 409 | HISTORY_CONFLICT | 撤销/重做会影响其他 actor 或没有可用历史 |
| 409 | EXTERNAL_CHANGE | 项目磁盘文件被外部修改 |
| 409 | ALREADY_RUNNING | 数据目录已有写入服务 |
| 409 | REQUEST_ID_REUSED | 同一请求 ID 对应不同事务 |
| 413 | TOO_LARGE | 请求超过 8 MB |
| 415 | JSON_REQUIRED | 缺少 JSON Content-Type |
| 422 | SCHEMA_INVALID / INVALID_GRAPH | 数据结构或关系非法 |
| 422 | SCHEMA_UNSUPPORTED | 文件不是当前 v2 格式，不执行旧格式迁移 |

CLI 命令在 `node scripts/graph-cli.mjs --help` 查看。CLI 的 validate 为离线结构检查；其余图数据读写先核对服务 schemaVersion=2，不匹配时返回 SERVICE_VERSION 并停止，不向旧服务提交编辑。凭据从环境读取，地址用 GRAPH_URL 或 `--url` 配置。导出不带 `--out` 时 stdout 为项目 JSON；其他正常输出也是 JSON。使用 npm 包装时 npm 自己可能打印脚本标题。

v0.3 还核对 capabilities：nested-scenes、annotation-boards、node-explanations、state-transitions。说明板通过 node.add/update/remove 编辑，kind=annotation，不添加 port；尺寸用 view.node.update，结构化功能说明使用 metadata.explanation。node.update 的 metadata 为整字段替换，需保留未涉及的参数/证据，不能只发送 explanation 覆盖其他 metadata。

`inspect --project ID --scene SCENE_ID` 返回该层节点、端口、连线、分组、布局、视角及边界映射；`--object` 可查询 scene 对象本身，二者分别使用。导出始终包含整个工程，不仅是当前层。
