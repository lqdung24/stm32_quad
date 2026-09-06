# Agent context contract

Before performing any task in this repository:

1. Run `tools/read_context index.md` and use `.context/index.md` to identify the task scope.
2. Read only the relevant domain documents listed by the index.
3. Follow links from those documents to the source files or the nearby `function-flow.md` when function-level detail is needed.
4. Use `tools/search_context '<keyword>'` when the domain is unclear. Do not load every Markdown file into the prompt.

The `.context` directory is a navigation and architecture layer, not a substitute for source inspection. Source code, build configuration, tests, and hardware configuration remain authoritative. If context disagrees with implementation, verify the implementation and update the relevant context document as part of the same task.

Useful commands:

```sh
tools/read_context index.md
tools/read_context domains/control-safety.md
tools/search_context failsafe
```

When working below `stm32cube/`, also follow any more-specific instructions in `stm32cube/.agents/AGENTS.md`.
