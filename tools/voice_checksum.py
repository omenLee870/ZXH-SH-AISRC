#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
voice_checksum.py

语音 UART 协议校验值计算工具。

协议规则来自 APP/Src/voice_uart.c：
固定 8 字节帧中，第 8 字节为校验字节，校验值等于前 7 字节逐字节异或结果。
"""

import re
import sys


FRAME_DATA_LEN = 7
FRAME_TOTAL_LEN = 8


def parse_byte_tokens(text):
    """把用户输入的十六进制字节串解析为 0~255 的整数列表。"""
    tokens = re.split(r"[\s,;]+", text.strip())
    values = []

    for token in tokens:
        if not token:
            continue

        if token.lower().startswith("0x"):
            value = int(token, 16)
        else:
            value = int(token, 16)

        if value < 0 or value > 0xFF:
            raise ValueError(f"字节超出范围 0x00~0xFF: {token}")

        values.append(value)

    return values


def calc_voice_checksum(values):
    """按 voice uart 规则计算校验值：前 7 字节逐字节异或。"""
    checksum = 0
    for value in values[:FRAME_DATA_LEN]:
        checksum ^= value
    return checksum


def print_result(values):
    """输出校验结果；如果用户输入完整 8 字节，则额外判断第 8 字节是否正确。"""
    if len(values) not in (FRAME_DATA_LEN, FRAME_TOTAL_LEN):
        raise ValueError("请输入 7 个字节用于计算，或输入 8 个字节用于计算并校验。")

    checksum = calc_voice_checksum(values)
    print(f"输入前 7 字节: {' '.join(f'{value:02X}' for value in values[:FRAME_DATA_LEN])}")
    print(f"校验值: 0x{checksum:02X} ({checksum})")

    if len(values) == FRAME_TOTAL_LEN:
        recv_checksum = values[FRAME_TOTAL_LEN - 1]
        if checksum == recv_checksum:
            print(f"第 8 字节 0x{recv_checksum:02X} 校验通过")
        else:
            print(f"第 8 字节 0x{recv_checksum:02X} 校验失败，应为 0x{checksum:02X}")


def main():
    """命令行入口：支持参数模式和交互模式。"""
    try:
        if len(sys.argv) > 1:
            input_text = " ".join(sys.argv[1:])
        else:
            input_text = input("请输入 7 或 8 个字节，例如 A5 01 02 03 04 05 06: ")

        values = parse_byte_tokens(input_text)
        print_result(values)
        return 0
    except ValueError as exc:
        print(f"错误: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
