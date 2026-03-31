#!/usr/bin/env node

const https = require("https");

// Gitee API 配置
const GITEE_USERNAME = "tydfgt";
const GITEE_EMAIL = "997065247@qq.com";
const REPO_NAME = "ysdwebcedar";
const REPO_DESC = "Mizuki blog project - 静态博客网站";

// 注意：由于安全原因，这里无法直接使用用户的密码或 token
// 实际使用时需要手动提供 token

console.log("📦 准备在 Gitee 上创建仓库...");
console.log(`用户名：${GITEE_USERNAME}`);
console.log(`仓库名：${REPO_NAME}`);
console.log("");
console.log("⚠️  重要提示：");
console.log("由于安全原因，Gitee API 需要认证才能创建仓库。");
console.log("请按照以下步骤操作：");
console.log("");
console.log("1. 访问 Gitee 个人令牌页面：");
console.log("   https://gitee.com/profile/personal_access_tokens");
console.log("");
console.log('2. 点击"生成新令牌"');
console.log('   - 勾选 "projects" 权限');
console.log('   - 点击"确定"');
console.log("");
console.log("3. 复制生成的令牌（以字母开头的一串字符）");
console.log("");
console.log("4. 在终端执行以下命令（替换 YOUR_TOKEN）：");
console.log(`   curl -X POST "https://gitee.com/api/v5/user/repos" \\`);
console.log(`     -H "Content-Type: application/json" \\`);
console.log(
	`     -d "access_token=YOUR_TOKEN&name=${REPO_NAME}&description=${encodeURIComponent(REPO_DESC)}&auto_init=false"`,
);
console.log("");
console.log("或者直接在浏览器中访问：");
console.log("https://gitee.com/new");
console.log("手动创建仓库，仓库名为：ysdwebcedar");
console.log("");
console.log("✅ 创建完成后，执行以下命令推送代码：");
console.log("   cd /home/ubuntu/Mizuki");
console.log("   git push -u origin master");
console.log("");

