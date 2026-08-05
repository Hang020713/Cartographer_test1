import os
import sys
import time
import subprocess

NODE = "/sys/class/pwm/pwmchip0"

def run_root(cmd):
    """Run command with sudo, return False on failure."""
    result = subprocess.run(
        ["sudo"] + cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0

def sudo_write(path, value):
    """Equivalent to `echo value | sudo tee path`."""
    # Using tee to write to protected sysfs nodes even without root shell
    proc = subprocess.run(
        ["sudo", "tee", path],
        input=f"{value}\n".encode(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc.returncode == 0

def pinctrl_set(pin, func):
    """Set pin multiplexing function."""
    run_root(["pinctrl", "set", pin, func])

def pwm_write(channel, attr, value):
    """Write value to pwm<channel> attribute, exit on failure."""
    path = os.path.join(NODE, f"pwm{channel}", attr)
    if not sudo_write(path, value):
        print(f"Error: unable to write to {attr}")
        sys.exit(1)


def pwm_read(channel, attr):
    """Read pwm<channel> attribute, return None if unreadable."""
    path = os.path.join(NODE, f"pwm{channel}", attr)
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except OSError:
        return None

def pwm_init(channel):
    """Export channel if not already exported and wait for initialization."""
    pwm_dir = os.path.join(NODE, f"pwm{channel}")
    if not os.path.isdir(pwm_dir):
        if not sudo_write(os.path.join(NODE, "export"), channel):
            print("Error: unable to export channel")
            sys.exit(1)
        time.sleep(0.1)  # Wait for device initialization


def mode_freq(pin, channel, func, value):
    """freq mode: set frequency."""
    if value is None or value == "0":
        print("Error: invalid frequency")

    try:
        period = int(1_000_000_000 / float(value))
    except (ValueError, ZeroDivisionError):
        print("Error: invalid frequency")

    pwm_init(channel)

    cur_period = pwm_read(channel, "period")

    # Critical: cannot write duty_cycle when period is 0; cannot shrink period if duty remains
    # Strategy: if period exists, zero out duty first then change period; if period is 0, write period directly
    if cur_period and cur_period != "0":
        pwm_write(channel, "duty_cycle", "0")
        pwm_write(channel, "period", str(period))
    else:
        pwm_write(channel, "period", str(period))
        pwm_write(channel, "duty_cycle", "0")

    pwm_write(channel, "enable", "1")
    pinctrl_set(pin, func)
    print(f"Pin{pin}: frequency={value}Hz (period={period}ns)")


def mode_duty(pin, channel, value):
    """duty mode: set duty cycle percentage."""
    if value is None:
        print("Error: invalid duty cycle")

    pwm_init(channel)

    period_str = pwm_read(channel, "period")
    if not period_str or period_str == "0":
        print("Error: period not set, please run freq mode first")
        sys.exit(1)

    try:
        duty = int(int(period_str) * float(value) / 100)
    except ValueError:
        print("Error: invalid duty cycle")

    pwm_write(channel, "duty_cycle", str(duty))
    print(f"Pin{pin}: duty cycle={value}% ({duty}ns)")

def mode_off(pin, channel):
    """off mode: disable and unexport PWM."""
    pwm_write(channel, "enable", "0")
    sudo_write(os.path.join(NODE, "unexport"), channel)
    pinctrl_set(pin, "no")
    print(f"Pin{pin}: PWM disabled")