---
name: project-graph
description: Read and edit engineering node maps through the Project Graph CLI, including nodes, ports, connections, layout and JSON sharing. Use for this tool's maps; source-code changes remain a separate task.
---

# Project Graph

Use the tool's CLI to edit its engineering map. Human users edit the same map in a browser; preserve their unrelated layout and content.

Locate the tool directory supplied by the user (contains `package.json` and `scripts/graph-cli.mjs`). This skill ships inside that directory; it does not depend on an installed global agent configuration. If copied elsewhere, use the actual tool checkout as the command working directory.

Read [AGENT_GUIDE](../../docs/AGENT_GUIDE.md) for commands, conflict handling and examples. If the skill has been relocated, read `<tool-root>/docs/AGENT_GUIDE.md` instead. Consult DATA_FORMAT.md and API.md there only when their field or operation details are needed.

For explanatory content, read WRITING_GUIDE.md there. Functional metadata.explanation has receives/process/produces strings and decisions as ordered condition→result strings. Explain unfamiliar states before using their IDs. Use kind=annotation (newBoard) for Markdown exhibits with no ports/owned subviews, stored in description. Keep unrelated metadata when replacing that object. Hardware and other domains use the same generic model; robot logic belongs in examples.

- Inspect the target project to obtain its current revision and stable object IDs: `node scripts/graph-cli.mjs inspect --project ID`.
- For an inner view use `inspect --project ID --scene SCENE_ID`. New nodes/groups must use the intended sceneId; do not accidentally add them to the overview.
- Read source files yourself when the task needs engineering evidence. The map tool does not infer architecture or update source code.
- Prepare a minimal transaction with a fresh requestId, inspected baseRevision, actor, summary and operations. Edit semantic fields without resetting layout.
- Validate with `apply --project ID --file PATCH --dry-run`, then submit without `--dry-run` and inspect the result.
- On a revision conflict, reread and reconcile the intended changes against the new state. Do not replace the project with an old full snapshot.
- On ambiguous network results, inspect before retrying. Request deduplication is limited to the current service session.
- Use `export --project ID --out FILE` and `import --file FILE` for portable sharing. Import creates an independent project, not a live shared session.

The current format is v2; v1 migration is deliberately unsupported. Create an owned subview with `scene.add` (id, label, description, ownerNodeId). The tool generates boundary nodes and parentPortId mappings; child boundary directions are opposite to the corresponding parent ports. Query those generated IDs before wiring internals. Do not hand-edit detailSceneId or mapping pointers. Source references accept line/endLine ranges for visible code locations. Export includes the entire hierarchy.

Configure remote services with GRAPH_URL or `--url`; use GRAPH_ACCESS_TOKEN for credentials. Never put tokens in map metadata, exported files or URLs. Source references use logical roots and relative paths.

CLI output is JSON; failures use stderr and a nonzero exit code. Report changed IDs, resulting revision, evidence and unresolved relationships. A successful map edit is not firmware validation.
