# Git 日常更新操作指南

## 快速开始 - 三步推送代码

```bash
# 1. 添加所有修改的文件
git add .

# 2. 提交到本地仓库
git commit -m "描述你的修改内容"

# 3. 推送到 Gitee
git push origin master
```

## 详细的日常更新流程

### 步骤 1：查看当前状态

在推送之前，先查看哪些文件发生了变化：

```bash
git status
```

你会看到类似输出：
- **红色**：未暂存的文件（需要 `git add`）
- **绿色**：已暂存的文件（可以 `git commit`）
- **Untracked files**：新创建的文件（需要 `git add`）

### 步骤 2：添加文件到暂存区

#### 添加所有文件（推荐）

```bash
git add .
```

#### 只添加特定文件

```bash
git add src/content/posts/guide/push-to-gitee.md
git add src/components/MyComponent.svelte
```

#### 交互式添加（高级）

```bash
git add -p
```

### 步骤 3：提交到本地仓库

```bash
git commit -m "feat: 添加了 Gitee 推送教程"
```

或者使用多行提交信息：

```bash
git commit -m "docs: 更新项目文档

- 添加了 Gitee 推送教程
- 完善了 README 说明
- 修复了拼写错误"
```

### 步骤 4：推送到 Gitee

```bash
git push origin master
```

如果配置了 SSH 密钥，会自动推送成功。否则需要输入 Gitee 的账号密码。

## 完整的更新示例

假设你刚刚创建了新的博客文章 `push-to-gitee.md`：

```bash
# 1. 查看状态
git status

# 输出示例：
# Untracked files:
#   src/content/posts/guide/push-to-gitee.md

# 2. 添加新文件
git add src/content/posts/guide/push-to-gitee.md

# 或者直接添加所有文件
git add .

# 3. 提交
git commit -m "docs: 添加 Gitee 推送教程文档"

# 4. 推送
git push origin master
```

## 常用场景

### 场景 1：只修改了一个文件

```bash
# 比如你只修改了 config.ts
git add src/config.ts
git commit -m "fix: 修复配置问题"
git push origin master
```

### 场景 2：添加了多个新文件

```bash
# 添加了多篇博客文章
git add .
git commit -m "docs: 添加三篇新的博客文章"
git push origin master
```

### 场景 3：删除了文件

```bash
# 删除了某个旧文件
git rm old-post.md
git commit -m "chore: 删除过时的文章"
git push origin master
```

### 场景 4：重命名或移动了文件

```bash
# 移动了文件位置
git mv old-path/file.md new-path/file.md
git commit -m "refactor: 重构文件目录结构"
git push origin master
```

## 查看提交历史

### 查看最近的提交

```bash
git log --oneline -n 5
```

输出示例：
```
a1b2c3d docs: 添加 Gitee 推送教程文档
e4f5g6h feat: 添加新功能
i7j8k9l fix: 修复 bug
```

### 查看详细的提交记录

```bash
git log
```

按 `q` 键退出查看。

## 撤销操作

### 撤销工作区的修改

```bash
# 撤销某个文件的修改
git checkout -- src/config.ts

# 撤销所有文件的修改
git checkout .
```

### 撤销暂存区的文件

```bash
# 从暂存区移除某个文件（保留工作区修改）
git reset HEAD src/config.ts
```

### 修改最后一次提交

```bash
# 如果刚提交完发现有问题，可以修正
git add corrected-file.md
git commit --amend -m "新的提交信息"

# 强制推送（谨慎使用）
git push origin master --force
```

## 同步远程仓库

### 拉取远程最新代码

```bash
# 拉取并合并
git pull origin master

# 只下载不合并
git fetch origin
```

### 解决冲突

如果 `git pull` 时出现冲突：

1. 打开冲突文件，找到冲突标记 `<<<<<<<`、`=======`、`>>>>>>>`
2. 手动编辑解决冲突
3. 保存文件后执行：

```bash
git add resolved-file.md
git commit -m "fix: 解决合并冲突"
git push origin master
```

## 分支管理（可选）

### 创建新分支

```bash
# 基于当前分支创建新分支
git checkout -b feature/new-design

# 在新分支上工作...
git add .
git commit -m "feat: 新设计功能"

# 切换回主分支
git checkout master

# 合并新分支
git merge feature/new-design

# 推送
git push origin master
```

### 查看分支

```bash
# 查看本地分支
git branch

# 查看所有分支（包括远程）
git branch -a
```

## 实用技巧

### 1. 配置 Git 别名（提高效率）

```bash
# 添加到 ~/.gitconfig 或全局配置
git config --global alias.st 'status'
git config --global alias.co 'checkout'
git config --global alias.br 'branch'
git config --global alias.ci 'commit'
git config --global alias.lg 'log --oneline -n 10'

# 之后可以使用简写
git st      # 代替 git status
git co -b feature  # 代替 git checkout -b feature
git lg      # 代替 git log --oneline -n 10
```

### 2. 使用 .gitignore 避免上传不必要的文件

确保项目根目录有 `.gitignore` 文件，包含：

```gitignore
# 依赖
node_modules/
.pnpm-store/

# 构建输出
dist/
.build/

# 环境文件
.env
.env.local

# 系统文件
.DS_Store
Thumbs.db

# IDE
.vscode/
.idea/

# 日志
*.log
```

### 3. 提交信息规范

推荐使用约定式提交：

- `feat:` 新功能
- `fix:` 修复 bug
- `docs:` 文档更新
- `style:` 代码格式调整
- `refactor:` 代码重构
- `test:` 测试相关
- `chore:` 构建工具、依赖管理等杂项

示例：
```bash
git commit -m "feat: 添加暗色模式支持"
git commit -m "fix: 修复导航栏显示问题"
git commit -m "docs: 更新 README 文档"
```

## 常见问题

### Q1: 推送失败，提示需要拉取

```bash
# 先拉取远程代码
git pull origin master

# 解决可能的冲突后
git push origin master
```

### Q2: 忘记添加某些文件

```bash
# 补充提交
git add forgotten-file.md
git commit --amend --no-edit

# 如果已经推送，需要强制推送
git push origin master --force
```

### Q3: 想查看某次提交的详细内容

```bash
git show <commit-hash>
```

### Q4: 如何回退到之前的版本

```bash
# 软回退（保留修改）
git reset --soft HEAD~1

# 硬回退（丢弃修改，谨慎使用）
git reset --hard HEAD~1

# 回退到特定版本
git reset --hard <commit-hash>

# 强制推送
git push origin master --force
```

## 完整的工作流示例

假设你要添加一篇新的博客文章：

```bash
# 1. 创建并编辑文章
cd src/content/posts/guide
vim my-new-post.md

# 2. 预览效果（可选）
cd /home/ubuntu/Mizuki
pnpm dev

# 3. 确认修改
git status

# 4. 添加文件
git add src/content/posts/guide/my-new-post.md

# 5. 提交
git commit -m "docs: 添加新的教程文章"

# 6. 推送
git push origin master

# 7. 验证
# 访问 Gitee 仓库页面，确认代码已更新
```

## 总结

日常更新只需要记住这三个命令：

```bash
git add .          # 添加所有修改
git commit -m "描述修改"  # 提交
git push           # 推送到 Gitee
```

其他命令都是可选的，根据需要学习使用！
