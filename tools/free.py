import os

vfs_info = os.statvfs('/')
# 提取关键参数（索引对应文档顺序）
frsize = vfs_info[1]  # 片段大小（字节）
bfree = vfs_info[3]   # 空闲块数（以 frsize 为单位）
total_blocks = vfs_info[2]  # 总块数（以 frsize 为单位）

# 计算剩余空间（字节）
free_space = frsize * bfree
total_space = frsize * total_blocks

# 输出更详细的信息
print(f"文件系统信息：")
print(f"总容量: {total_space // (1024**2)} MB")
print(f"已用容量: {(total_space - free_space) // (1024**2)} MB")
print(f"剩余容量: {free_space // (1024**2)} MB")
print(f"块大小: {frsize} 字节")
print(f"空闲块数: {bfree}（单位：{frsize} 字节）")