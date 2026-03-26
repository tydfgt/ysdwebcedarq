#!/bin/bash

# 停止Mizuki博客服务器的脚本

echo "正在停止Mizuki博客服务器..."

# 进入项目目录
cd /home/ubuntu/Mizuki

# 检查pid文件是否存在
if [ -f "server.pid" ]; then
    # 读取进程ID
    PID=$(cat server.pid)
    
    # 检查进程是否存在
    if ps -p $PID > /dev/null; then
        # 停止进程
        kill $PID
        echo "服务器已停止（进程ID: $PID）"
        # 删除pid文件
        rm server.pid
    else
        echo "服务器进程不存在，可能已经停止"
        # 删除无效的pid文件
        rm server.pid
    fi
else
    echo "未找到服务器进程ID文件，服务器可能未启动"
fi