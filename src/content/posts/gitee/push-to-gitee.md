---
title: 如何将项目推送到 Gitee 仓库
published: 2026-03-31
image: './1583124371_587283.jpg'
description: '详细教程：教你如何将本地项目推送到 Gitee 代码托管平台，包括 Git 配置、仓库创建和推送步骤。'
tags: [Git, Gitee, 版本控制，部署]
category: '教程，Git'
draft: false
---

# 如何将项目推送到 Gitee 仓库

本教程将详细介绍如何将你的本地项目推送到 Gitee（码云）代码托管平台。Gitee 是国内常用的代码托管服务，由 OSCHINA 推出，提供免费的 Git 仓库托管服务。

## 准备工作

### 1. 注册 Gitee 账号

如果你还没有 Gitee 账号，需要先访问 [Gitee 官网](https://gitee.com/) 注册账号。

:::note[提示信息]
本教程使用的示例账号信息：
- **用户名**: tydfgt
- **邮箱**: 997065247@qq.com

请将这些信息替换为你自己的实际账号信息。
:::

### 2. 安装 Git

确保你的计算机已安装 Git。可以通过以下命令检查：

```bash
git --version
```

如果未安装 Git，请前往 [Git 官网](https://git-scm.com/) 下载并安装。

## 步骤一：配置 Git 用户信息

在推送代码之前，需要配置 Git 的用户名和邮箱，这些信息会与你的提交记录关联。

### 全局配置（推荐）

打开终端或命令行工具，执行以下命令：

```bash
# 配置全局用户名
git config --global user.name "tydfgt"

# 配置全局邮箱
git config --global user.email "997065247@qq.com"
```

### 验证配置

查看当前配置是否正确：

```bash
git config --list
```

你应该能看到类似输出：

```text
user.name=tydfgt
user.email=997065247@qq.com
```

### 针对单个项目配置（可选）

如果只想为当前项目配置 Git 信息，可以在项目目录下执行（不需要 `--global` 参数）：

```bash
git config user.name "tydfgt"
git config user.email "997065247@qq.com"
```

## 步骤二：在 Gitee 创建远程仓库

### 1. 登录 Gitee

访问 [Gitee](https://gitee.com/) 并使用你的账号登录。

### 2. 创建新仓库

1. 点击右上角的 **+** 号
2. 选择 **新建仓库**
3. 填写仓库信息：
   - **仓库名称**: 例如 `my-blog`
   - **仓库介绍**: 可选，简单描述你的项目
   - **编程语言**: 根据项目选择（如 Astro、JavaScript 等）
   - **是否开源**: 根据需要选择公开或私有
   - **初始化选项**: 
     - ✅ 勾选 **添加 README.md**（推荐新手选择）
     - ✅ 勾选 **添加 .gitignore**（可选择对应语言的模板）
     - ✅ 勾选 **添加开源许可证**（如 MIT、Apache 等，根据需要选择）

:::tip[最佳实践]
建议勾选初始化选项，这样 Gitee 会自动创建一些必要的文件，避免后续冲突。
:::

4. 点击 **立即创建** 按钮

### 3. 获取仓库地址

创建成功后，你会被重定向到仓库页面。复制仓库的 Git URL，格式如下：

```text
https://gitee.com/tydfgt/your-repo-name.git
```

或者使用 SSH 方式（需要先配置 SSH 密钥）：

```text
git@gitee.com:tydfgt/your-repo-name.git
```

## 步骤三：初始化本地仓库并推送

### 情况 A：全新项目（从零开始）

如果你的项目还没有进行版本控制，按照以下步骤操作：

#### 1. 初始化 Git 仓库

在项目根目录执行：

```bash
cd /path/to/your/project
git init
```

你会看到提示：`Initialized empty Git repository in ...`

#### 2. 添加所有文件到暂存区

```bash
git add .
```

:::note[注意事项]
- `git add .` 会添加所有文件，包括可能不需要上传的文件
- 建议先创建 `.gitignore` 文件排除敏感文件（见下文）
:::

#### 3. 提交到本地仓库

```bash
git commit -m "Initial commit: 项目初始化"
```

#### 4. 关联远程仓库

将 `<your-repo-url>` 替换为你在步骤二中复制的仓库地址：

```bash
git remote add origin https://gitee.com/tydfgt/your-repo-name.git
```

验证远程仓库是否添加成功：

```bash
git remote -v
```

应该看到类似输出：

```text
origin  https://gitee.com/tydfgt/your-repo-name.git (fetch)
origin  https://gitee.com/tydfgt/your-repo-name.git (push)
```

#### 5. 推送到 Gitee

```bash
git push -u origin master
```

或者如果使用 main 分支：

```bash
git push -u origin main
```

:::warning[分支名称说明]
- Git 默认分支名称可能是 `master` 或 `main`
- 新版 Git 倾向于使用 `main` 作为默认分支名
- 可以通过 `git branch` 查看当前分支名
:::

#### 6. 输入认证信息

首次推送时，系统会要求你输入 Gitee的账号密码：

```text
Username for 'https://gitee.com': tydfgt
Password for 'https://gitee.com': ********
```

输入你的 Gitee 登录密码即可。

### 情况 B：已有 Git 仓库的项目

如果项目已经有 Git 历史记录，只需要关联远程仓库并推送：

```bash
# 查看当前远程仓库
git remote -v

# 如果没有远程仓库，添加 Gitee 远程
git remote add origin https://gitee.com/tydfgt/your-repo-name.git

# 如果已有其他远程仓库（如 GitHub），可以重命名
git remote rename origin github
git remote add origin https://gitee.com/tydfgt/your-repo-name.git

# 推送到 Gitee
git push -u origin main
```

## 步骤四：配置 SSH 密钥（可选但推荐）

使用 HTTPS 方式每次推送都需要输入密码，配置 SSH 后可以免密推送。

### 1. 生成 SSH 密钥

```bash
ssh-keygen -t ed25519 -C "997065247@qq.com"
```

或使用 RSA 算法（更兼容）：

```bash
ssh-keygen -t rsa -b 4096 -C "997065247@qq.com"
```

按回车接受默认路径，然后设置一个安全的密码短语（passphrase）。

### 2. 查看公钥内容

```bash
cat ~/.ssh/id_ed25519.pub
```

或

```bash
cat ~/.ssh/id_rsa.pub
```

复制显示的全部内容（以 `ssh-ed25519` 或 `ssh-rsa` 开头）。

### 3. 添加到 Gitee

1. 登录 Gitee
2. 点击右上角头像 → **设置**
3. 左侧菜单选择 **SSH 密钥**
4. 点击 **添加 SSH 密钥**
5. 粘贴刚才复制的公钥内容
6. 输入标题（如 "My Laptop"）
7. 点击 **确定**

### 4. 测试 SSH 连接

```bash
ssh -T git@gitee.com
```

如果看到欢迎信息，说明配置成功：

```text
Hi tydfgt! You've successfully authenticated, but GITEE.COM does not provide shell access.
```

### 5. 切换到 SSH 方式推送

如果之前使用 HTTPS 方式，可以切换到 SSH：

```bash
# 查看当前远程地址
git remote get-url origin

# 修改为 SSH 地址
git remote set-url origin git@gitee.com:tydfgt/your-repo-name.git

# 验证
git remote -v
```

之后推送就无需输入密码了。

## 常用 Git 操作速查

### 日常开发流程

```bash
# 1. 查看文件变更状态
git status

# 2. 添加修改的文件
git add <filename>        # 添加指定文件
git add .                 # 添加所有文件

# 3. 提交到本地仓库
git commit -m "描述你的修改"

# 4. 推送到 Gitee
git push
```

### 同步远程更新

```bash
# 拉取远程最新代码并合并
git pull origin main

# 仅下载不合并
git fetch origin
```

### 查看历史记录

```bash
# 查看提交历史
git log
git log --oneline         # 简洁模式

# 查看具体修改
git show <commit-hash>
```

### 分支管理

```bash
# 查看分支
git branch                # 本地分支
git branch -a             # 所有分支

# 创建新分支
git checkout -b feature/new-feature

# 切换分支
git checkout main

# 合并分支
git checkout main
git merge feature/new-feature

# 推送分支到远程
git push origin feature/new-feature
```

## 创建 .gitignore 文件

`.gitignore` 文件用于告诉 Git 哪些文件不应该被版本控制。

### 示例：Astro/Node.js 项目的 .gitignore

```gitignore
# 依赖目录
node_modules/
.pnpm-store/

# 构建输出
dist/
.build/

# 环境变量
.env
.env.local
.env.*.local

# 系统文件
.DS_Store
Thumbs.db

# IDE 配置
.vscode/
.idea/
*.swp
*.swo

# 日志文件
*.log
npm-debug.log*

# 临时文件
tmp/
temp/
.cache/
```

在项目根目录创建此文件，Git 就会自动忽略这些目录和文件。

## 常见问题解决

### 问题 1：推送被拒绝

**错误信息**: `rejected master -> master (non-fast-forward)`

**原因**: 远程仓库有本地没有的提交

**解决方案**:

```bash
# 先拉取远程代码并合并
git pull origin main --allow-unrelated-histories

# 解决可能的冲突后
git push -u origin main
```

### 问题 2：密码验证失败

**原因**: Gitee 账号密码错误或开启了双重认证

**解决方案**:
1. 检查密码是否正确
2. 如果开启了两步验证，需要使用 **私人令牌**（Personal Access Token）代替密码
3. 生成令牌：设置 → 私人令牌 → 生成新令牌（勾选 `project` 权限）
4. 使用令牌作为密码进行推送

### 问题 3：SSL 证书验证失败

**错误信息**: `SSL certificate problem`

**解决方案**:

```bash
# 临时方案（不推荐生产环境）
git config --global http.sslVerify false

# 推荐方案：更新 CA 证书包
# Ubuntu/Debian
sudo apt-get install --reinstall ca-certificates

# macOS
brew install ca-certificates
```

### 问题 4：大文件推送失败

**原因**: Gitee 限制单个文件大小不超过 100MB

**解决方案**:

```bash
# 撤销最近一次提交
git reset --soft HEAD~1

# 移除大文件
git reset HEAD path/to/large-file

# 重新提交（不包含大文件）
git commit -m "提交信息"

# 推送
git push
```

对于确实需要存储的大文件，考虑使用 Git LFS 或外部存储服务。

## 安全建议

:::danger[重要提醒]
以下文件和信息**绝对不要**推送到仓库：

1. **敏感信息**
   - `.env` 文件中的 API 密钥、数据库密码
   - 私钥文件（`id_rsa`、`id_ed25519` 等）
   - 配置文件中的密码

2. **个人隐私**
   - 个人联系方式
   - 身份证号、手机号
   - 家庭住址

3. **版权内容**
   - 未授权的商业软件
   - 受版权保护的图片、视频
:::

### 如果不慎推送了敏感信息

1. **立即删除远程文件**（即使本地删除，历史记录中仍存在）

```bash
# 从 Git 历史中彻底删除
git filter-branch --force --index-filter \
  'git rm --cached --ignore-unmatch path/to/secret' \
  --prune-empty --tag-name-filter cat -- --all

# 强制推送
git push origin --force --all
```

2. **更改相关密码和密钥**
3. **联系 Gitee 支持**（如果需要）

## 进阶技巧

### 1. 使用 Git Hooks

可以在推送前自动运行测试、格式化代码等。

示例：`.git/hooks/pre-commit`

```bash
#!/bin/bash
# 提交前自动格式化代码
pnpm run format

# 运行测试
pnpm test
```

### 2. 配置多个远程仓库

可以同时推送到 Gitee 和 GitHub：

```bash
# 添加 GitHub 远程
git remote add github https://github.com/username/repo.git

# 同时推送到两个平台
git push origin main
git push github main
```

### 3. 使用 Git Worktree

在不同分支间并行工作：

```bash
# 创建新的工作树
git worktree add ../feature-branch feature/new-feature

# 在另一个目录工作
cd ../feature-branch
```

## 总结

通过以上步骤，你已经成功掌握了：

✅ 配置 Git 用户信息  
✅ 在 Gitee 创建仓库  
✅ 初始化本地 Git 仓库  
✅ 关联远程仓库并推送代码  
✅ 配置 SSH 密钥实现免密推送  
✅ 日常 Git 操作流程  
✅ 常见问题解决方法  

现在你可以将你的项目代码安全地托管到 Gitee 平台，享受版本控制带来的便利！

## 参考资源

- [Gitee 官方文档](https://gitee.com/help)
- [Git 官方文档](https://git-scm.com/doc)
- [Git 中文社区](https://git-scm.com/book/zh/v2)
- [Astro 部署指南](https://docs.astro.build/zh-cn/guides/deploy/)

祝你编码愉快！🚀
