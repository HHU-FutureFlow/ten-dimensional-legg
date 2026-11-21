# 第三方依赖说明

下表从本轮安装包的 package.json 读取。完整依赖树与解析来源在 package-lock.json 中，各包的完整许可证文本随其 npm 分发包提供。

| 直接依赖 | 本轮版本 | 包声明的许可 |
| --- | --- | --- |
| @xyflow/react | 12.12.1 | MIT |
| ajv | 8.20.0 | MIT |
| lucide-react | 0.468.0 | ISC |
| react | 19.3.1 | MIT |
| react-dom | 19.3.1 | MIT |
| react-markdown | 10.1.0 | MIT |
| remark-gfm | 4.0.1 | MIT |
| @playwright/test（开发） | 1.65.1 | Apache-2.0 |
| @types/node（开发） | 24.19.2 | MIT |
| @types/react（开发） | 19.3.0 | MIT |
| @types/react-dom（开发） | 19.3.0 | MIT |
| @vitejs/plugin-react（开发） | 5.2.0 | MIT |
| typescript（开发） | 5.9.3 | Apache-2.0 |
| vite（开发） | 7.3.8 | MIT |

React Flow 官方文档：[自定义节点](https://reactflow.dev/learn/customization/custom-nodes)、[端口](https://reactflow.dev/learn/customization/handles)。构建工具：[Vite 文档](https://vite.dev/guide/)。

未修改第三方包源代码。工具自身代码的开源许可和实际公开发布由所有者后续决定。LQR 示例只记录维护者梳理的功能、接口和源码相对位置，不附带完整机器人固件。

画布说明板参考了 ComfyUI 的 [Note / Markdown Note](https://raw.githubusercontent.com/Comfy-Org/ComfyUI_frontend/main/src/extensions/core/noteNode.ts) 组织思路；本工具仍使用 React Flow，不依赖 ComfyUI 的运行环境。
