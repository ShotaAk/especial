# Especial

![Especial](https://github.com/ShotaAk/especial/blob/images/images/especial.jpg)

ESP32を搭載したマイクロマウスのプログラム

# Requirements

- Especial (Optional)
  - 私が作成したオリジナルマウスがあると、本プログラムの全機能を動かせます
- ESP32-WROOM-32D
- ESP-IDF v3.3.0
  - 最新バージョンのv4.0.0には対応していません
  
# Installation

## ESP-IDFのインストール

公式マニュアルを参照してください。

[ESP-IDF Get Started | v3.3.0](https://docs.espressif.com/projects/esp-idf/en/release-v3.3/get-started/index.html)

## especialのビルド

```sh
cd ~/esp
git clone https://github.com/ShotaAk/especial
cd especial

# 走行プログラムのビルド
make
## 書き込み
make flash

# サンプルプログラムのビルド
cd examples/0_hello_world 
make
## 書き込み
make flash
```

# 現在の開発工程

[ここ見て](https://github.com/ShotaAk/especial/milestones)

# 問題点一覧

[ここ見て](https://github.com/ShotaAk/especial/issues)

