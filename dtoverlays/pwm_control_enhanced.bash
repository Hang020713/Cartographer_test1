#!/bin/bash
# pwm_control_enhanced.sh

NODE="/sys/class/pwm/pwmchip0"
PIN="$1"
MODE="$2"
VALUE="$3"

declare -A PIN_MAP=(
    ["18"]="2 a3"
    ["19"]="3 a3"
)

function show_usage {
    echo -e "\n\033[1m智能PWM控制脚本\033[0m"
    echo "用法: $0 <引脚> <模式> <值>"
    echo " 引脚: 12,13,18,19"
    echo " 模式:"
    echo "   freq <Hz> - 设置PWM频率"
    echo "   duty <%>  - 设置占空比百分比"
    echo "   off       - 关闭PWM输出"
    echo -e "\n示例:"
    echo " $0 18 freq 50   # 设置50Hz频率"
    echo " $0 18 duty 7.5  # 7.5%占空比(舵机中位)"
    exit 1
}

[[ -z $PIN ]] && show_usage
[[ -z ${PIN_MAP[$PIN]} ]] && { echo "错误: 无效引脚"; show_usage; }

read CHANNEL FUNC <<< "${PIN_MAP[$PIN]}"

function pwm_write {
    echo "$2" | sudo tee "$NODE/pwm$CHANNEL/$1" >/dev/null || {
        echo "错误: 无法写入$1"; exit 1
    }
}

function pwm_init {
    if [ ! -d "$NODE/pwm$CHANNEL" ]; then
        echo "$CHANNEL" | sudo tee "$NODE/export" >/dev/null || {
            echo "错误: 无法导出通道"; exit 1
        }
        sleep 0.1  # 等待设备初始化
    fi
}

case $MODE in
    "freq")
        { [[ -z $VALUE ]] || [[ $VALUE == 0 ]]; } && { echo "错误: 无效频率"; show_usage; }
        PERIOD=$(echo "scale=0; 1000000000/$VALUE" | bc)
        pwm_init
        # 先把 duty_cycle 归零,避免新 period 小于旧 duty_cycle 时写 period 失败
        pwm_write "duty_cycle" "0"
        pwm_write "period" "$PERIOD"
        pwm_write "enable" "1"
        sudo pinctrl set $PIN $FUNC
        echo "引脚$PIN: 频率=${VALUE}Hz (周期=${PERIOD}ns)"
        ;;
    "duty")
        [[ -z $VALUE ]] && { echo "错误: 无效占空比"; show_usage; }
        pwm_init
        # 从 sysfs 读回当前周期,而不是依赖内存变量
        PERIOD=$(cat "$NODE/pwm$CHANNEL/period")
        { [[ -z $PERIOD ]] || [[ $PERIOD == 0 ]]; } && {
            echo "错误: 周期未设置,请先运行 freq 模式"; exit 1
        }
        DUTY=$(echo "scale=0; $PERIOD*$VALUE/100" | bc)
        pwm_write "duty_cycle" "$DUTY"
        echo "引脚$PIN: 占空比=${VALUE}% (${DUTY}ns)"
        ;;
    "off")
        pwm_write "enable" "0"
        echo "$CHANNEL" | sudo tee "$NODE/unexport" >/dev/null
        sudo pinctrl set $PIN no
        echo "引脚$PIN: PWM已禁用"
        ;;
    *)
        show_usage
        ;;
esac