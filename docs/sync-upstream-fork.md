# 同步上游仓库到 Fork 指南

本文档说明如何把上游仓库（upstream）的更新同步到你的 fork，同时保留你本地/自有提交与新增文件。

> 约定：
> - upstream：原始仓库（例如 https://github.com/isaac-sim/IsaacSim）
> - origin：你的 fork（例如 https://github.com/<you>/IsaacSim_5.0）
> - main：主分支

## 1) 备份或储藏当前改动（可选但推荐）
```bash
# 如有未提交改动，先储藏（包含未跟踪文件）
git stash -u
# 或创建备份分支
git switch -c backup/pre-sync-$(date +%Y%m%d%H%M%S)
```

## 2) 添加上游 remote（首次需要）
```bash
git remote add upstream https://github.com/isaac-sim/IsaacSim.git
# 验证
git remote -v
```

## 3) 获取最新更新
```bash
git fetch upstream
git fetch origin
```

## 4) 同步 main（二选一）

### 方式 A：rebase（推荐，历史更线性；可能需要安全强推）
```bash
git switch main
# 将你的本地提交重放到 upstream/main 之后
git pull --rebase upstream main
# 遇到冲突：编辑冲突文件 -> git add <文件> -> 继续
git rebase --continue
# 若想放弃本次 rebase：
# git rebase --abort

# 完成后推送到 fork（安全强推）
git push origin main --force-with-lease
```

### 方式 B：merge（不改写历史；无需强推）
```bash
git switch main
# 将 upstream/main 合并到本地 main
git pull upstream main
# 遇到冲突：编辑冲突文件 -> git add <文件> -> 提交
git commit

# 推送到 fork
git push origin main
```

## 5) 冲突处理要点
- 用 `git status` 查看冲突文件。
- 编辑冲突文件，删除 `<<<<<<<`, `=======`, `>>>>>>>` 标记，保留期望内容。
- 标记解决后：`git add <文件>`。
- rebase 场景继续：`git rebase --continue`；merge 场景提交：`git commit`。
- 随时放弃：`git rebase --abort` 或 `git merge --abort`。
- 若遇到“skipped previously applied commit”提示，可用：
```bash
git pull --rebase --reapply-cherry-picks upstream main
```

## 6) 常见实践建议
- 在 feature 分支开发，保持 main 专注于同步：
```bash
# 新建功能分支
git switch -c feature/my-change
# 同步上游时：
git switch main && git pull --rebase upstream main && git push origin main
# 功能分支跟进：
git switch feature/my-change && git rebase main
```
- 不要将本地缓存/生成物（如 cache、build 目录）提交到仓库；若上游删除了某些缓存而你本地修改了，冲突时一般按“删除”处理。
- 使用 `--force-with-lease` 而不是 `--force`，更安全地推送经 rebase 改写过的历史。
