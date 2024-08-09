import os
import serial
import subprocess

class LiftController:
    COMMAND_STOP = bytes.fromhex('01060001000119CA')
    COMMAND_UP = bytes.fromhex('01060001000259CB')
    COMMAND_DOWN = bytes.fromhex('010600010004D9C9')
    COMMAND_RESET = bytes.fromhex('010600010008D9CC')
    Read_PositionOfLift = bytes.fromhex('01030002000125CA')

    def __init__(self, device='/dev/ttyUSB0', baudrate=9600, timeout=0.2):
        self.current_user = os.getlogin()
        self.device = device
        self.kill_existing_connections()
        self.set_device_permissions()
        self.SERIAL = serial.Serial(
            port=self.device,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
        if self.SERIAL.is_open:
            print("Serial port opened successfully.")
        else:
            print("Failed to open serial port.")
        self.signal = True
    def kill_existing_connections(self):
        try:
            # fuser 명령어로 장치를 사용하는 프로세스 찾기 및 종료
            subprocess.run(["fuser", "-k", self.device], check=True)
            print(f"Terminated existing connections on {self.device}.")
        except subprocess.CalledProcessError as e:
            print(f"No existing connections to terminate: {e}")
    def set_device_permissions(self):
        try:
            # 장치에 대해 읽기/쓰기 권한 부여
            subprocess.run(["chmod", "666", self.device], check=True)
            # 장치의 소유자 변경
            subprocess.run(["chown", f"{self.current_user}:{self.current_user}", self.device], check=True)
            print(f"Set permissions for {self.device} to user {self.current_user}.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to set permissions: {e}")

    def hex_to_float(self, hex_string):
        integer_value = int.from_bytes(hex_string, byteorder='big')
        float_value = integer_value / 10
        return float_value
    def sendcommand(self, command_type) :
        self.SERIAL.write(command_type)
        try :
            ret = self.SERIAL.readline()
            return ret
        except serial.SerialException as e:
            print(f"Serial read error: {e}")
            return None
            
    def read(self):
        if self.signal == True:
            try:
                self.SERIAL.write(self.Read_PositionOfLift)
                high = self.SERIAL.read(7)
                return high
            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                return None
        else:
            print("command__sending.....")
        
    def read_high(self):
        position = self.read()
        if position:
            position = self.hex_to_float(position[3:5])
            return position
        else:
            return -1.0

    def send(self, input_command):
        self.signal = False
        input_command = int(input_command)
        if self.signal == False:
            if input_command == 1:
                self.SERIAL.write(self.COMMAND_UP)
            elif input_command == 2:
                self.SERIAL.write(self.COMMAND_DOWN)
            elif input_command == 3:
                self.SERIAL.write(self.COMMAND_STOP)
            elif input_command == 4:
                self.SERIAL.write(self.COMMAND_RESET)
            self.SERIAL.timeout = 1
            ret = self.SERIAL.read()
            print(ret)
            self.signal = True
        else:
            print("exception_1")