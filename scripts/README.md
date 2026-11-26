# Scripts

Git hooks for the pre-commit framework (`.pre-commit-config.yaml`) or as plain hooks (`install-hooks.sh` / `install-hooks.bat`), and CI helpers (`ci/`).

## Code graph (graphify, optional)

`graphify-out/graph.json` is a [graphify](https://docs.graphify.com/cookbooks/shared-team-graph) graph of
the code: functions, classes, files and the calls and includes between them, built locally from the
syntax tree (no LLM, no API key). The pre-commit hook keeps it up to date for those who have graphify
installed; everyone else can ignore it. Everything else in `graphify-out/` is local and ignored.

**Setup, once per clone:**

```bash
uv tool install --python 3.12 graphifyy

# the merge driver: a union merge of graph.json instead of a conflict
PY="$(sed -n '1s/^#!//p' "$(command -v graphify)")"
git config merge.graphify.name "graphify graph.json union merge"
git config merge.graphify.driver "\"$PY\" -m graphify merge-driver %O %A %B"
```

On Windows, `graphify.exe` has no shebang line: set `PY` to the `python.exe` of the graphify
environment instead (`uv tool dir` gives its folder, then `graphifyy\Scripts\python.exe`).

Do not run `graphify hook install`: it rebuilds the graph after each commit, which leaves it as an
uncommitted change. After switching branches, `graphify update .` refreshes the local report and
`graph.html`. To rebuild the graph from scratch: `PYTHONHASHSEED=0 graphify extract . --code-only`.
