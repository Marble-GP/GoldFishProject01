#!/bin/bash

# ESP32クラッシュアドレス解析スクリプト (Bash版)
# 使用法: ./decode_esp32_trace.sh [ELFファイルパス] アドレス1 アドレス2 ...

# --- 設定項目 ---
# addr2line実行ファイルのパス (環境に合わせて確認・修正)
ADDR2LINE_EXE_PATH="$HOME/.platformio/packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-addr2line"

# デフォルトのELFファイルへの相対パス (プロジェクトルートからのパス)
# <env_name> は platformio.ini のビルド環境名 (例: esp32s3dev)
DEFAULT_ELF_RELATIVE_PATH=".pio/build/esp32s3dev/firmware.elf"

# --- 処理開始 ---

# addr2line実行ファイルの存在確認
if [[ ! -f "$ADDR2LINE_EXE_PATH" ]]; then
    echo "ERROR: xtensa-esp32-elf-addr2lineが見つかりません。"
    echo "パスを確認してください: $ADDR2LINE_EXE_PATH"
    echo ""
    echo "PlatformIOの場合、以下のパスを確認してください："
    echo "  Linux/macOS: ~/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-addr2line"
    echo "  または: ~/.platformio/packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-addr2line"
    exit 1
fi

# 引数チェック
if [[ $# -eq 0 ]]; then
    echo "使用方法: $0 [ELFファイルのパス(省略可)] アドレス1 アドレス2 アドレス3 ..."
    echo "  例1 (デフォルトELF使用): $0 0x400d1234 0x400d5678"
    echo "  例2 (ELFファイル指定):   $0 .pio/build/your_env/firmware.elf 0x400d1234"
    echo "  省略時のELFファイルパス: $DEFAULT_ELF_RELATIVE_PATH"
    exit 1
fi

ELF_FILE_PATH=""
ADDRESSES_TO_DECODE=""

# 第1引数がELFファイルかどうかを判定
FIRST_ARG="$1"
if [[ "$FIRST_ARG" == *.elf ]]; then
    if [[ -f "$FIRST_ARG" ]]; then
        ELF_FILE_PATH="$FIRST_ARG"
        shift  # 第1引数を削除
    else
        echo "WARNING: 第1引数 '$FIRST_ARG' はELFファイルに見えますが見つかりません。デフォルトELFを使用します。"
        ELF_FILE_PATH="$DEFAULT_ELF_RELATIVE_PATH"
    fi
else
    ELF_FILE_PATH="$DEFAULT_ELF_RELATIVE_PATH"
fi

# ELFファイルの最終確認
if [[ ! -f "$ELF_FILE_PATH" ]]; then
    echo "ERROR: ELFファイルが見つかりません: $ELF_FILE_PATH"
    echo ""
    echo "対処方法："
    echo "1. PlatformIOでプロジェクトをビルドしてください"
    echo "2. DEFAULT_ELF_RELATIVE_PATH の設定を確認してください"
    echo "3. ELFファイルのパスを引数で指定してください"
    echo ""
    echo "ビルド環境名を確認するには: platformio.ini の [env:環境名] を確認"
    exit 1
fi

# 残りの引数をアドレスとして収集
for arg in "$@"; do
    ADDRESSES_TO_DECODE="$ADDRESSES_TO_DECODE $arg"
done

if [[ -z "$ADDRESSES_TO_DECODE" ]]; then
    echo "エラー: デコードするアドレスが指定されていません。"
    exit 1
fi

# 実行するコマンドを構築 (addr2lineのオプション -e を使用)
FINAL_COMMAND="$ADDR2LINE_EXE_PATH -e $ELF_FILE_PATH -fpCai $ADDRESSES_TO_DECODE"
# (-a オプションは -p と機能が重複気味なので、-p に集約。-iはインライン展開で有用)

echo
echo "--- デバッグ情報 ---"
echo "ADDR2LINE_EXE_PATH : $ADDR2LINE_EXE_PATH"
echo "ELF_FILE_PATH      : $ELF_FILE_PATH"
echo "Addresses to decode: $ADDRESSES_TO_DECODE"
echo "Executing command  : $FINAL_COMMAND"
echo "---"

echo
echo "--- デコード結果 ---"
eval "$FINAL_COMMAND"
echo "--------------------------------------"
echo
