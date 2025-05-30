# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 16:49:35 2020

@author: Henry + grg

open COM port
set RTC of TriOS Sensor to UTC based on system clock
start series of trigger measurements 
stop by interrupting the program with Strg+C

"""

import datetime, struct, time
import configparser
import sys

import pandas as pd

import numpy as np
#from serial import Serial
from serial import Serial

from matplotlib import pyplot as plt

#srcfolder="D:\\Science\\Projekte\\DArgo2025\\Fahrten\\MT-v141_Stand_EMB251"
#import os
#os.chdir(srcfolder)
#from CRC16 import CRC16

from ctypes import c_ushort


DOUT = "./out/"



class CRC16(object):
    crc16_tab = []
 
    # The CRC's are computed using polynomials. Here is the most used
    # coefficient for CRC16
    crc16_constant = 0xA001  # 40961
 
    def __init__(self, modbus_flag=False):
        # initialize the precalculated tables
        if not len(self.crc16_tab):
            self.init_crc16()
        self.mdflag = bool(modbus_flag)
 
    def calculate(self, input_data=None):
        try:
            is_string = isinstance(input_data, str)
            is_bytes = isinstance(input_data, bytes)
 
            if not is_string and not is_bytes:
                raise Exception("Please provide a string or a byte sequence "
                                "as argument for calculation.")
 
            crcValue = 0x0000 if not self.mdflag else 0xffff
 
            for c in input_data:
                d = ord(c) if is_string else c
                tmp = crcValue ^ d
                rotated = c_ushort(crcValue >> 8).value
                crcValue = rotated ^ int(self.crc16_tab[(tmp & 0x00ff)], 0)
 
            return crcValue
        except Exception as e:
            print("EXCEPTION(calculate): {}".format(e))
 
    def init_crc16(self):
        '''The algorithm uses tables with precalculated values'''
        for i in range(0, 256):
            crc = c_ushort(i).value
            for j in range(0, 8):
                if (crc & 0x0001):
                    crc = c_ushort(crc >> 1).value ^ self.crc16_constant
                else:
                    crc = c_ushort(crc >> 1).value
            self.crc16_tab.append(hex(crc))


############# modbus_read_Uint16
def read_Uint16(outbytes, Uint16_count):
    
    result = np.zeros(Uint16_count)
    for i in range(0, Uint16_count):
        result[i] = struct.unpack('>H', outbytes[(2 * i):(2 * i + 2)])[0]
    
    return result

############# modbus_read_float
def read_Float(outbytes, Float_count):
    
    result = np.zeros(Float_count)
    for i in range(0, Float_count):
        result[i] = struct.unpack('>f', outbytes[(4 * i):(4 * i + 4)])[0]
    
    return result

###########

############# modbus_send_cmd
def modbus_send_cmd(cmd):
    
    trios_serial_port.flushOutput() # 
    trios_serial_port.flushInput() # 
    # write to port
    trios_serial_port.write(cmd)
    #time.sleep(0.1)
    # get answer
    trios_answer = trios_serial_port.read(3)
    # print(str(trios_answer))

    # get validity and length of answer
    if len(trios_answer)==3:        
        outnum = np.zeros(3, dtype=int)
        for i in range(0, 3):
            outnum[i] = struct.unpack('>B', trios_answer[(i):(i+1)])[0]
    
    else:
        trios_serial_port.flushInput() # 
        return b'', 0
        
    #  check that there is no error (second bit '83' instead of '03')
    if outnum[1]==131:
        #print('Error: Register not accessible yet')
        print('Error: Register not accessible yet' + '. Exception code: ' + str(outnum[2]))
        trios_serial_port.read(2) # empty read cache of CRC
        trios_serial_port.flushInput() # 
        return b'', 0

    elif outnum[1]==3:
        trios_answer = trios_serial_port.read(int(outnum[2]+2)) # get data plus 2 bytes CRC to empty cache
        trios_serial_port.flushInput() # 
    
    return trios_answer, outnum

###########

############# CRC
def add_modbus_crc(cmd):
    
    CRCModbus = CRC16(modbus_flag=True)
    CRCtemp = CRCModbus.calculate(cmd)
    low = CRCtemp & 0xff
    high = CRCtemp >> 8
    
    return (cmd+bytes([low]) + bytes([high]))

###########

############# OPUS_set_RTC
def OPUS_set_RTC():
    
    posix_utc = (datetime.datetime.now(datetime.timezone.utc) - datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)).total_seconds()
    # convert to hex
    #hex(struct.unpack('>I', struct.pack('>L', int(posix_utc)))[0])
    #hex(int(posix_utc))
    #struct.pack('>L', int(posix_utc))
    #int(posix_utc).to_bytes(4, byteorder='big', signed = False)
    #posix_list=list(struct.pack('>L', int(posix_utc)))
    posix_bin = int(posix_utc).to_bytes(4, byteorder='big', signed = False)
    # add cmd together: Address + write/read + register + length + value + crc
    utccmd = b'\x01' + b'\x10' 
    utccmd = utccmd + int(237).to_bytes(2, byteorder='big') + int(2).to_bytes(2, byteorder='big') + b'\x04'
    utccmd = add_modbus_crc(utccmd + posix_bin)    
    # send command
    UTC_answer, UTC_outnum = modbus_send_cmd(utccmd)
    
    return UTC_answer, UTC_outnum

###########

############# OPUS_trigger_measurement
def OPUS_trigger_measurement(meascmd, wait):
    t0 = datetime.datetime.now()
    # meascmd = b'\x01\x06\x00\x01\x01\x01\x18\x5A'  # Absorption + Substance Analysis
    # meascmd = b'\x01\x06\x00\x01\x01\x00\xD9\x9A'  # Absorption
    # meascmd = b'\x01\x06\x00\x01\x02\x00\xD9\x6A'  # Calibrated
    # meascmd = b'\x01\x06\x00\x01\x04\x00\xDA\xCA'  # Raw Light    
    # meascmd = b'\x01\x06\x00\x01\x05\x00\xDB\x5A'  # Raw Dark 
    
    trios_serial_port.flushOutput() # 
    print(str(datetime.datetime.now()), ' Send trigger:', ''.join(format(x,'02x') for x in meascmd))
    # write to port
    trios_serial_port.write(meascmd)
    # # get answer (echo) to empty serial port
    # trios_answer = trios_serial_port.read(8)
    # print(str(datetime.datetime.now()), ': Receive_mirror:', trios_answer.hex())
    # print(str(datetime.datetime.now()), ' Receive_mirror:', ''.join(format(x,'02x') for x in trios_answer))
    
    time.sleep(wait)
    trios_serial_port.flushInput() #
    
    # get measurement timeout
    cmd =  b'\x01\x03\x00\x01\x00\x01\xD5\xCA'
    
    jj = 0
    while True:
        trios_answer, outnum = modbus_send_cmd(cmd)
        
        #time.sleep(0.05)
        
        if len(trios_answer) < 3:
            print('No answer')
            #print(str(trios_answer) )
            
        else:
            output = read_Uint16(trios_answer, int(outnum[2]/2))
            
            if( output[0] != 0):
                # print(str(datetime.datetime.now()), ': Sensor busy')
                jj +=1
                
            elif ( output[0] ==0):
                print('     Elapsed time: ', str(datetime.datetime.now()-t0))
                print(output)
                break

            else:
                print("answer unknown: ", str(trios_answer) )
                break
        
        time.sleep(0.4)
        trios_serial_port.flushInput()

###########

############# OPUS_read_wv
def OPUS_read_wv():

    ###### read wv spectrum
    cmd = add_modbus_crc(b'\x01\x03\x08\x34\x00\x7C')  # wv: 001-062
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputWV001_062 = read_Float(trios_answer[0:outnum[2]], 62)
   
    
    cmd = add_modbus_crc(b'\x01\x03\x08\xB0\x00\x7C')  # wv: 063-124 \x07\x85
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputWV063_124 = read_Float(trios_answer[0:outnum[2]], 62)
    #print(outputWV063_124)

    cmd = add_modbus_crc(b'\x01\x03\x09\x2C\x00\x7C')  # wv: 125-186
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputWV125_186 = read_Float(trios_answer[0:outnum[2]], 62)
    # print(outputWV125_186)

    cmd = add_modbus_crc(b'\x01\x03\x09\xA8\x00\x7C')  # wv: 187-248 09 A8
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputWV187_248 = read_Float(trios_answer[0:outnum[2]], 62)
    # print(outputWV187_248)
    wv = np.concatenate([outputWV001_062, outputWV063_124, outputWV125_186, outputWV187_248])

    return wv

###########

############# OPUS_read_acs
def OPUS_read_abs():

    #### get Abs spectrum
    cmd = add_modbus_crc(b'\x01\x03\x0A\x34\x00\x7C')  # wv: 001-062
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputABS001_062 = read_Float(trios_answer[0:outnum[2]], 62)
    #print(outputABS001_062)

    cmd = add_modbus_crc(b'\x01\x03\x0A\xB0\x00\x7C')  # wv: 063-124
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputABS063_124 = read_Float(trios_answer[0:outnum[2]], 62)
    #print(outputABS063_124)

    cmd = add_modbus_crc(b'\x01\x03\x0B\x2C\x00\x7C')  # wv: 125-186
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputABS125_186 = read_Float(trios_answer[0:outnum[2]], 62)
    # print(outputABS125_186)

    cmd = add_modbus_crc(b'\x01\x03\x0B\xA8\x00\x7C')  # wv: 187-248 09 A8
    trios_answer, outnum = modbus_send_cmd(cmd)
    outputABS187_248 = read_Float(trios_answer[0:outnum[2]], 62)
    # print(outputABS187_248)

    OPUS_abs = np.concatenate([outputABS001_062, outputABS063_124, outputABS125_186, outputABS187_248])

    return OPUS_abs

###########


if __name__ == "__main__":
    # config_filename = 'HydroC_CO2_OPUS_config_EMB.txt'
    if len(sys.argv) >= 2:
        #config_filename = str(sys.argv[1])
        config_filename = sys.argv[1]
    
    conf = configparser.ConfigParser()
    # conf.read(config_filename)
    
    portstring = conf.get('OPUS', 'COM-Port', fallback='COM5')


    print('Connect to Port:', portstring)
    trios_serial_port = Serial(portstring, 9600, timeout=2)
    time.sleep(0.1)
    
    # Absorption + Substance Analysis
    meascmd = b'\x01\x06\x00\x01\x01\x01\x18\x5A'
    # # Absorption
    # meascmd0 = b'\x01\x06\x00\x01\x01\x00\xD9\x9A'
    # # Calibrated
    # meascmd1 = b'\x01\x06\x00\x01\x02\x00\xD9\x6A'
    # # Raw Light
    # meascmd2 = b'\x01\x06\x00\x01\x04\x00\xDA\xCA'
    # # Raw Dark
    # meascmd3 = b'\x01\x06\x00\x01\x05\x00\xDB\x5A'

    wait = 1.5
    
    trios_serial_port.flushInput()






    try:
        print('Synchronize RTC with PC UTC')
        OPUS_set_RTC()

        # initialize time counting
        t0 = pd.Timestamp.now()


        while True:

            # sync instrument time
            if np.mod(pd.Timestamp.now().minute, 10) == 0:
                print('Synchronize RTC with PC UTC')
                OPUS_set_RTC()

            # initialize dictionary with results
            row_dict = {
                        'Timestamp': [],
                        'Abs': np.empty(200),
                        'T_spectrum': [],
                        'T_internal': [],
                        'Flash_count': [],
                        }

            print('trigger calibrated measurement')
            t0_spectrum = pd.Timestamp.now()
            OPUS_trigger_measurement(meascmd, wait)
            t_spectrum = pd.Timestamp.now() - t0_spectrum
            print(t_spectrum)

            trios_serial_port.flushInput()

            # get temperature
            cmd = b'\x01\x03\x07\xD7\x00\x02\x75\x47'
            trios_answer, outnum = modbus_send_cmd(cmd)
            outputT = read_Float(trios_answer,int(outnum[2]/4))
            # print(outputT)

            # # # get length of spectrum
            # cmd =  add_modbus_crc(b'\x01\x03\x07\xD9\x00\x01')
            # trios_answer, outnum = modbus_send_cmd(cmd)
            # outputLength = trios_answer[0:outnum[2]]
            # # print(read_Uint16(outputLength, 1))

            # # get Flash count
            cmd =  add_modbus_crc(b'\x01\x03\x07\xD4\x00\x01')
            trios_answer, outnum = modbus_send_cmd(cmd)
            outputFlashCount = read_Uint16(trios_answer[0:outnum[2]], 1)
            # print(read_Uint16(outputFlashCount, 1))



            if 'wv' not in locals():
                    print('reading wv')
                    wv = OPUS_read_wv()

                    # initialize dictionary
                    wv_str = ['{:.01f}'.format(iwv) for iwv in np.round(wv[:200], 1)]
                    # OPUS_data = pd.DataFrame(columns=wv_str)
                    row_list = []


            print('reading absorbance')
            OPUS_abs = OPUS_read_abs()
            
            
            # store data in dictionary
            row_dict.update({'Timestamp': t0_spectrum,
                             'Abs': OPUS_abs,
                            'T_spectrum': t_spectrum,
                            'T_internal': outputT,
                            'Flash_count': outputFlashCount,
                             })

            row_list.append(row_dict)


            #### check if it's the start of the new hour and, if so, save hourly file
            if pd.Timestamp.now().minute != t0_spectrum.minute: # save a file every hour
            # xMin = 2 # save a file every xMin
            # if np.mod(t0_spectrum.minute, xMin) == 0:
                df_opus = pd.DataFrame(row_list).set_index('timestamp')
                fn_out = t0_spectrum.strftime('opus_%Y%m%d_%H%M.pkl')
                print('writing file: ' + fn_out + '\n')
                df_opus.to_pickle(DOUT + fn_out)










            ## print spectra
            plt.clf()
            plt.plot(wv, OPUS_abs, 'k-', lw=1,
                     # marker='.',
                     label='T_internal = {:.02f} degC'.format(outputT[0]) + "\n" +
                           'Flash count = {:.0g}'.format(outputFlashCount[0])
                     )
            plt.plot(wv[0:62], OPUS_abs[0:62], 'r.', lw=1, ms=5,
                     # marker='.',
                     # label='T_internal = {:.02f} degC'.format(outputT[0]) + "\n" +
                           # 'Flash count = {:.0g}'.format(outputFlashCount[0])
                     )
            plt.title(t0_spectrum.strftime('%Y-%m-%d %H:%M:%S'))
            plt.xlabel('wavelength [nm]', fontweight='bold')
            plt.ylabel('absorbance [-]', fontweight='bold')
            plt.ylim([-0.5, 1])
            plt.grid('on', ls='--', color='lightgray')
            plt.legend()
            plt.pause(0.05)





        # # get serial number
        # cmd =  b'\x01\x03\x00\x0A\x00\x05\xA5\xCB'
        # trios_answer, outnum = modbus_send_cmd(cmd)
        # outputSN = trios_answer[0:outnum[2]]
        # print(outputSN)

        # # get measurement timeout
        # cmd =  b'\x01\x03\x00\x01\x00\x01\xD5\xCA'
        # trios_answer, outnum = modbus_send_cmd(cmd)
        # outputTimeout = read_Uint16(trios_answer,int(outnum[2]/2))

        # # get temperature
        # cmd = b'\x01\x03\x07\xD7\x00\x02\x75\x47'
        # trios_answer, outnum = modbus_send_cmd(cmd)
        # outputT = read_Float(trios_answer,int(outnum[2]/4))
        # print(outputT)


    except KeyboardInterrupt:
        print('Done with acquisition')

    except:
        print('Something else went wrong. Try again?')

    finally:
        trios_serial_port.close()

    plt.show()


