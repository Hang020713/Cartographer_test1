#!/usr/bin/env python3
# pwm_control_enhanced.py
"""智能PWM控制脚本 (Python 版本)"""

import os
import sys
import time
import subprocess

NODE = "/sys/class/pwm/pwmchip0"

# 引脚 -> (通道, pinctrl 功能)
PIN_MAP = {
    "18": ("2", "a3"),
    "19": ("3", "a3"),
}


def show_usage():
    """打印用法说明并退出。"""
    prog = os.path.basename(sys.argv[0])
    print("\n\033[1m智能PWM控制脚本\033[0m")
    print(f"用法: {prog} <引脚> <模式> <值>")
    print(" 引脚: 12,13,18,19")
    print(" 模式:")
    print("   freq <Hz> - 设置PWM频率")
    print("   duty <%>  - 设置占空比百分比")
    print("   off       - 关闭PWM输出")
    print("\n示例:")
    print(f" {prog} 18 freq 50   # 设置50Hz频率")
    print(f" {prog} 18 duty 7.5  # 7.5%占空比(舵机中位)")
    sys.exit(1)


def run_root(cmd):
    """以 sudo 运行命令,失败返回 False。"""
    result = subprocess.run(
        ["sudo"] + cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def sudo_write(path, value):
    """相当于 `echo value | sudo tee path`。"""
    # 使用 tee 是为了在没有 root 的 shell 里也能写入受保护的 sysfs 节点
    proc = subprocess.run(
        ["sudo", "tee", path],
        input=f"{value}\n".encode(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc.returncode == 0


def pwm_write(channel, attr, value):
    """向 pwm<channel> 的属性写入值,失败即退出。"""
    path = os.path.join(NODE, f"pwm{channel}", attr)
    if not sudo_write(path, value):
        print(f"错误: 无法写入{attr}")
        sys.exit(1)


def pwm_read(channel, attr):
    """读取 pwm<channel> 的属性,读不到返回 None。"""
    path = os.path.join(NODE, f"pwm{channel}", attr)
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except OSError:
        return None


def pwm_init(channel):
    """如通道尚未导出则导出并等待初始化。"""
    pwm_dir = os.path.join(NODE, f"pwm{channel}")
    if not os.path.isdir(pwm_dir):
        if not sudo_write(os.path.join(NODE, "export"), channel):
            print("错误: 无法导出通道")
            sys.exit(1)
        time.sleep(0.1)  # 等待设备初始化


def pinctrl_set(pin, func):
    """设置引脚复用功能。"""
    run_root(["pinctrl", "set", pin, func])


def mode_freq(pin, channel, func, value):
    """freq 模式: 设置频率。"""
    if value is None or value == "0":
        print("错误: 无效频率")
        show_usage()

    try:
        period = int(1_000_000_000 / float(value))
    except (ValueError, ZeroDivisionError):
        print("错误: 无效频率")
        show_usage()

    pwm_init(channel)

    cur_period = pwm_read(channel, "period")

    # 关键: period 为 0 时不能写 duty_cycle;有残留 duty 时又不能先缩小 period
    # 策略: 若当前已有周期,先把 duty 归零再改 period;若周期为 0,直接写 period
    if cur_period and cur_period != "0":
        pwm_write(channel, "duty_cycle", "0")
        pwm_write(channel, "period", str(period))
    else:
        pwm_write(channel, "period", str(period))
        pwm_write(channel, "duty_cycle", "0")

    pwm_write(channel, "enable", "1")
    pinctrl_set(pin, func)
    print(f"引脚{pin}: 频率={value}Hz (周期={period}ns)")


def mode_duty(pin, channel, value):
    """duty 模式: 设置占空比百分比。"""
    if value is None:
        print("错误: 无效占空比")
        show_usage()

    pwm_init(channel)

    period_str = pwm_read(channel, "period")
    if not period_str or period_str == "0":
        print("错误: 周期未设置,请先运行 freq 模式")
        sys.exit(1)

    try:
        duty = int(int(period_str) * float(value) / 100)
    except ValueError:
        print("错误: 无效占空比")
        show_usage()

    pwm_write(channel, "duty_cycle", str(duty))
    print(f"引脚{pin}: 占空比={value}% ({duty}ns)")


def mode_off(pin, channel):
    """off 模式: 关闭并注销 PWM。"""
    pwm_write(channel, "enable", "0")
    sudo_write(os.path.join(NODE, "unexport"), channel)
    pinctrl_set(pin, "no")
    print(f"引脚{pin}: PWM已禁用")


def main():
    args = sys.argv[1:]
    pin = args[0] if len(args) > 0 else None
    mode = args[1] if len(args) > 1 else None
    value = args[2] if len(args) > 2 else None

    if not pin:
        show_usage()

    if pin not in PIN_MAP:
        print("错误: 无效引脚")
        show_usage()

    channel, func = PIN_MAP[pin]

    if mode == "freq":
        mode_freq(pin, channel, func, value)
    elif mode == "duty":
        mode_duty(pin, channel, value)
    elif mode == "off":
        mode_off(pin, channel)
    else:
        show_usage()


if __name__ == "__main__":
    main()