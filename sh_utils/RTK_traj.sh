#!/usr/bin/expect -f

# 启动一个新的终端会话
spawn bash

# 等待环境变量加载完成，这里以出现提示符为标志
expect "cat@lubancat-desktop:~/HAM_Flight/sh_utils$ "

# 发送命令到终端
send "bash -c \"sleep 2; roslaunch estimator add_bias.launch; exec bash\"\r"

# 保持终端打开
interact
