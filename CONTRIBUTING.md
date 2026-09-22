# Contributing

Small team, one repo, `main` as the shared branch. This is deliberately lightweight — adjust it
once the team/process actually needs more structure.

## Daily workflow

1. **Before starting work**, sync:
   ```bash
   git pull --rebase origin main
   ```
2. Work on a **feature branch**, not directly on `main`:
   ```bash
   git checkout -b <yourname>/<short-topic>   # e.g. vlad/grpc-server
   ```
3. Commit in small, working steps. Rebuild and actually run the simulator before committing —
   see [`README.md`](README.md).
4. Push your branch and open a PR into `main`:
   ```bash
   git push -u origin <yourname>/<short-topic>
   gh pr create --fill
   ```
5. After merge, delete the branch and pull `main` again before starting the next task.

If you're working solo and PRs feel like overhead for a given change, committing straight to
`main` is fine — just still `git pull --rebase` first so you don't diverge.

## Keeping your checkout in sync

Pull periodically, not just at the start of a session — especially if more than one person (or
AI agent) is touching the project in parallel:

```bash
git pull --rebase origin main
```

If `simulator/build/` ever causes a conflict or looks tracked, that's a bug — it must stay
git-ignored (see [`AGENTS.md`](AGENTS.md)). Don't resolve that by force-adding it; fix the
`.gitignore` instead.

## Commit messages

Short, imperative, explain *why* over *what* when it's not obvious from the diff (e.g. "Add
ball rigid body" not "Update physics.cpp").

## What never gets committed

- `simulator/build/` (git-ignored — CMake build output, machine-specific)
- Anything under a personal absolute path (`/Users/...`, `/home/...`)
- `.DS_Store` / editor scratch files

## Before opening a PR

- `bash simulator/build.sh` succeeds
- `./run_simulator.sh` actually launches and the change is visible in the viewer
- If you touched `simulator/CMakeLists.txt` to add a dependency, also update the `brew install`
  line in `simulator/setup.sh` so a clean machine can still build (see `AGENTS.md`)
