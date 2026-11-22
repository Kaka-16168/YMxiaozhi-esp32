name: Build Otto Robot Firmware

on:
  workflow_dispatch:
  push:
    branches: [ main, master ]

permissions:
  contents: read

jobs:
  build-otto-robot:
    runs-on: ubuntu-latest
    container:
      image: espressif/idf:v6.1

    defaults:
      run:
        shell: bash

    steps:
      - name: Trust workspace (fix dubious ownership in container)
        run: git config --global --add safe.directory '*'

      - name: Checkout
        uses: actions/checkout@v4
        with:
          ref: ${{ github.sha }}
          submodules: 'recursive'

      - name: Build otto-robot with custom wake word "卡卡"
        run: |
          . $IDF_PATH/export.sh

          # 第1步：用 build.py 生成基础配置（不传 --wake-word，避免覆盖）
          python scripts/build.py otto-robot --language zh-CN

          # 第2步：手动追加自定义唤醒词配置到 sdkconfig
          # build.py 会强制把 CONFIG_USE_CUSTOM_WAKE_WORD 设为 n
          # 所以需要在生成配置后手动改回来
          echo "" >> build/sdkconfig
          echo "# Custom wake word: 卡卡 (ka ka)" >> build/sdkconfig
          echo "CONFIG_USE_CUSTOM_WAKE_WORD=y" >> build/sdkconfig
          echo "CONFIG_SR_MN_CN_MULTINET7_QUANT=y" >> build/sdkconfig
          echo 'CONFIG_CUSTOM_WAKE_WORD="ka ka"' >> build/sdkconfig
          echo 'CONFIG_CUSTOM_WAKE_WORD_DISPLAY="卡卡"' >> build/sdkconfig
          echo "CONFIG_CUSTOM_WAKE_WORD_THRESHOLD=20" >> build/sdkconfig
          echo "CONFIG_USE_AFE_WAKE_WORD=n" >> build/sdkconfig
          echo "CONFIG_USE_ESP_WAKE_WORD=n" >> build/sdkconfig
          echo "CONFIG_WAKE_WORD_DISABLED=n" >> build/sdkconfig

          # 第3步：用 idf.py 重新构建（此时 sdkconfig 已包含自定义唤醒词）
          idf.py build

          # 第4步：合并二进制
          idf.py merge-bin

      - name: Rename merged binary for easy identification
        run: |
          cp build/merged-binary.bin build/otto-robot-merged.bin

      - name: Upload merged firmware
        uses: actions/upload-artifact@v4
        with:
          name: otto-robot-merged-firmware
          path: build/otto-robot-merged.bin
          if-no-files-found: error
