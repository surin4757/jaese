#!/usr/bin/env python3

import serial.tools.list_ports
def PortFig() :
    # 현재 연결된 시리얼 포트 목록 가져오기
    ports = serial.tools.list_ports.comports()
    # 포트 목록 출력
    for port, desc, hwid in ports:
        print("---port imformation---")
        print(f"Port: {port}")
        print(f"Description: {desc}")
        print(f"HWID: {hwid}")

PortFig()