#!/bin/bash
set -e
set -u
version=$(git rev-parse --short HEAD)
echo "Generating distribution files for git version $version..."

rm -rf dist
mkdir dist
echo $version > dist/version.txt

mkdir dist/lib
for libpy in lib/*.py; do
    echo "Compiling $libpy..."
    mpy-cross -o dist/${libpy%.py}.mpy $libpy &
done
if [ -d usageapp ]; then
    mkdir -p dist/usageapp
    for apppy in usageapp/*.py; do
        echo "Compiling $apppy..."
        mpy-cross -o dist/${apppy%.py}.mpy $apppy &
    done
fi
wait
echo "Copying files..."
# 布局原则：用户不改的代码（lib/、usageapp/）编译为 .mpy——体积更小、
# 导入更快更省 RAM；可能被用户修改的留在根目录源码：
#   入口链 boot→main（系统按文件名找 boot.py，必须源码；板级包装
#   board 在 lib/，随 lib 编译循环出 .mpy）
#   配置    wlan_cfg / usage_cfg（模板形态，用户复制填写）
cp boot.py main.py dist/
# 配置模板批量处理：*.template 去后缀复制为可编辑配置；
# 新增配置只需在根目录加模板文件，此处零改动
for tpl in *.template; do
    [ -e "$tpl" ] || continue      # 无模板时 glob 保持字面量，跳过
    cp "$tpl" "dist/${tpl%.template}"
done
echo "Done."