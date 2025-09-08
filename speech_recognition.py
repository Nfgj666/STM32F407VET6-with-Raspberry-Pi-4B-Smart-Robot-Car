#!/usr/bin/python3
# coding=utf8
import smbus
import time

# I2C地址
I2C_ADDR = 0x34  # I2C地址
ASR_RESULT_ADDR = 100  # ASR语音识别结果寄存器地址(0x64)
ASR_SPEAK_ADDR = 110  # ASR播音设置寄存器地址(0x6E)
ASR_CMDMAND = 0x00    #播报语类型：命令词
ASR_ANNOUNCER = 0xFF  #播报语类型：普通播报语

class ASRModule:
    def __init__(self,address, bus=1):
        # 初始化 I2C 总线和设备地址
        self.bus = smbus.SMBus(bus)  # 使用 I2C 总线 1
        self.address = address  # 设备的 I2C 地址
        self.send = [0, 0]  # 初始化发送数据的列表

    def wire_write_byte(self, val):
        """
        向设备写入单个字节
        :param val: 要写入的字节值
        :return: 如果成功写入返回 True，失败返回 False
        """
        try:
            self.bus.write_byte(self.address, val) # 发送字节到设备
            return True # 写入成功
        except IOError:
            return False # 写入失败，返回 False

    def wire_write_data_array(self, reg, val, length):
        """
        向指定寄存器写入字节列表
        :param reg: 寄存器地址
        :param val: 要写入的字节列表
        :param length: 要写入的字节数
        :return: 如果成功写入返回 True，失败返回 False
        """
        try:            
            self.bus.write_i2c_block_data(self.address, reg, val[:length]) # 发送字节列表到设备的指定寄存器
            return True # 写入成功
        except IOError:
            return False # 写入失败，返回 False

    def wire_read_data_array(self, reg, length):
        """
        从指定寄存器读取字节列表
        :param reg: 寄存器地址
        :param length: 要读取的字节数
        :return: 读取到的字节列表，失败时返回空列表
        """          
        try:
            result = self.bus.read_i2c_block_data(self.address, reg, length) # 从设备读取字节列表
            return result # 返回读取结果
        except IOError:
            return [] # 读取失败，返回空列表

    def rec_recognition(self):
        """
        识别结果读取
        :return: 识别结果，如果读取失败返回 0
        """
        result = self.wire_read_data_array(ASR_RESULT_ADDR, 1) # 从结果寄存器读取一个字节
        if result:
            return result # 返回读取到的结果
        return 0  # 如果没有结果，返回 0

    def speak(self, cmd, id):
        """
        向设备发送说话命令
        :param cmd: 命令字节
        :param id: 说话的 ID
        """
        if cmd == ASR_ANNOUNCER or cmd == ASR_CMDMAND: # 检查命令是否有效
            self.send[0] = cmd # 设置发送列表的第一个元素为命令
            self.send[1] = id # 设置发送列表的第二个元素为 ID
            self.wire_write_data_array(ASR_SPEAK_ADDR, self.send, 2) # 发送命令和 ID 到指定寄存器


if __name__ == "__main__":
    asr_module = ASRModule(I2C_ADDR)    
    while True:
        recognition_result = asr_module.rec_recognition()
        if recognition_result[0] != 0:
            if recognition_result[0] == 1:
                print("go")
            elif recognition_result[0] == 2:
                print("back")
            elif recognition_result[0] == 3:
                print("left")
            elif recognition_result[0] == 4:
                print("right")
            elif recognition_result[0] == 9:
                print("stop")