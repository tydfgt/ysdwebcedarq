#!/bin/bash

# 启动Mizuki博客服务器的脚本

echo "正在启动Mizuki博客服务器..."

# 进入项目目录
cd /home/ubuntu/Mizuki

# 构建项目
#echo "正在构建项目..."
#pnpm build

# 后台运行预览服务器，输出重定向到日志文件
echo "正在启动预览服务器..."
nohup pnpm preview --port 18084 --host 0.0.0.0 > server.log 2>&1 &

# 保存进程ID到文件
echo $! > server.pid

echo "服务器已在后台启动！"
echo "进程ID已保存到 server.pid 文件"
echo "服务器日志输出到 server.log 文件"
echo "可以通过 http://43.139.97.198:18084 访问网站"