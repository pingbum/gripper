#!/bin/bash
# Jetson Orin NX (JetPack 6) 40핀 Pin 7 (SOC_GPIO59_PAC6) 를 GPIO 출력 가능 상태로 설정
# DYNAMIXEL Shield TX_EN(A6) 방향제어용. 재부팅 시 초기화되므로 jetson-pinmux.service 로 부팅마다 실행
# 레지스터 비트: [1:0]=기능, [3:2]=풀업/다운, [4]=TRISTATE(1이면 출력 차단),
#               [6]=E_INPUT, [10]=SFIO 선택(0=GPIO)
set -e

PIN7=0x02448030

old=$(( $(busybox devmem $PIN7 32) ))
new=$(( (old & ~((1<<10) | (1<<4))) | (1<<6) ))   # GPIO 모드, TRISTATE 해제, 입력 버퍼 ON
busybox devmem $PIN7 32 $(printf "0x%08x" $new)
printf "pin7 pinmux: 0x%08x -> %s\n" $old "$(busybox devmem $PIN7 32)"
