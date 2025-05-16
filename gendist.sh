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
wait
echo "Copying files..."
cp -r imgs boot.py main.py utils.py dist/
cp wlan_cfg.py.template dist/wlan_cfg.py
echo "Done."