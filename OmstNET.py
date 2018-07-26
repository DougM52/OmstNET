#!/usr/bin/env python3.4 -i

##MIT License
##
##Copyright (c) 2018 Douglas E. Moore
##
##Permission is hereby granted, free of charge, to any person obtaining a copy
##of this software and associated documentation files (the "Software"), to deal
##in the Software without restriction, including without limitation the rights
##to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
##copies of the Software, and to permit persons to whom the Software is
##furnished to do so, subject to the following conditions:
##
##The above copyright notice and this permission notice shall be included in all
##copies or substantial portions of the Software.
##
##THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
##IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
##FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
##AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
##LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
##OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
##SOFTWARE.

import os                               # for isatty?
import re                               # needed for parsing optomux ASCII packets
from   time import perf_counter         # needed for computing turnaround timeouts
from   time import sleep                # delays (and perhaps a form of kernel yield?)
from   itertools import chain           # needed for combining ranges in arg verification
import OmstTTY as tty                   # communications via serial port
import OmstUTL as utl                   # logging and TBD
import datetime                         # for timedelta
from   collections import namedtuple    # used to return mistic data
from   collections import OrderedDict   # used to build mistic commands and parse responses
import inspect                          # used to parse mistic responses
##import binascii as ba                   # hexlify

# init the utils such as logging
utl._init()

MsgFmt = namedtuple('MsgFmt',['cmd','rsp'])
CmdFmt = namedtuple('CmdFmt',['cmd','args'])
RspMsg = namedtuple('RspMsg',['ack','data'])
PktTyp = namedtuple('PktTyp',['jumpers','options'])

class OmstNET:    

    binary_mode = False
    crc_mode = False
    
    """
    'MISTIC PROTOCOL USER’S GUIDE, Form 270-100823 — August, 2010'
    p2-5
    """
    errors = {
        1:('UNDEFINED COMMAND','The command character was not a legal command character.'),
        2:('CHECKSUM OR CRC ERROR',
           'The checksum or CRC received by the Mistic I/O unit did not match \
            the value calculated from the command message.'),
        3:('BUFFER O VERRUN ERROR',
           'The receive buffer limit of 256 characters has been exceeded. \
            The command was ignored.'),
        4:('POWER-UP CLEAR ERROR',
           'After a power failure (4.8 VDC or lower) or a reset command, \
            you must issue a Power-Up Clear Command before issuing any \
            other command. If you do not, you will receive this error code. \
            This alerts the host that a power failure has occurred.'),
        5:('DATA FIELD ERROR','Not enough characters received.'),
        6:('COMMUNICATIONS LINK WATCHDOG TIME-OUT ERROR',
           'The communications link watchdog timer has timed out and the \
            specified actions have been taken. The command that returns \
            this error is not executed but it does clear the error.'),
        7:('SPECIFIED DATA INVALID ERROR',
           'One or more data fields contains an illegal value.'),
        8:('BUSY ERROR',
           'This error is used by the LC Communicator to indicate a setup \
            mode condition.'),
        9:('INVALID MODULE TYPE ERROR',
           'This error code is returned when one or more specified modules \
            is not of the type required by the command.'),
        10:('INVALID EVENT ENTRY ERROR',
            'This error code is returned when an attempt is made to enable \
            an event interrupt on a null entry in the event reaction table \
            or an illegal reaction command is specified.'),
        11:('HIGH RESOLUTION TIME DELAY LIMIT REACHED - DIGITAL ONLY',
            'This error code is returned when an attempt is made to start a \
            square wave, generate N pulses or time-proportional output with a \
            delay time value less than 10 milliseconds on more than eight output positions.'),
    }

    """
    analog event datatypes.
    """
    analog_event_data_types = {
        0x00:'Current Counts',
        0x01:'Average Counts',
        0x02:'Peak Counts',
        0x03:'Lowest Counts',
        0x04:'Totalized Counts',
        0x10:'Current Engineering Units',
        0x11:'Average Engineering Units',
        0x12:'Peak Engineering Units',
        0x13:'Lowest Engineering Units',
        0x14:'Totalized Engineering Units',
        }
        
    """
    'MISTIC PROTOCOL USER’S GUIDE, Form 270-100823 — August, 2010'

    This dictionary is built from information in Chapters 4 - 16.
        Chapter 4 has tables of commands by category
        Chapters 5 - 16 describe how each command is built, and what
            the device response contains

    Each of these commands has a corresponding function with a similar name.
    
    As an example, for the dictionary entry:
        'READ EVENT TABLE ENTRY':
            MsgFmt('O EE','CB CC RC RCC ED RES NNNNNNNN TTTTTTTT'),

    there is a corresponding function:
        def read_event_entry_enable_disable_status(self,aa,EE)
    """
    commands = {
        # Chapter 5: Digital Setup / System Commands
        # Command Name, Command Format, Version
        'IDENTIFY TYPE':MsgFmt('F', 'DDDD'),
        'POWER UP CLEAR':MsgFmt('A',None),
        'REPEAT LAST RESPONSE':MsgFmt('^',None),
        'RESET':MsgFmt('B',None),
        'RESET ALL PARAMETERS TO DEFAULT':MsgFmt('x',None),
        'SET RESPONSE DELAY':MsgFmt('~ DD',None),
        'SET SYSTEM OPTIONS':MsgFmt('C SS CC','DDDD'),
        'SET WATCHDOG MOMO AND DELAY':MsgFmt('D MMMM NNNN TTTT',None),
        # Chapter 6 = MsgFmt Digital I/O Configuration Commands
        # Command Name, Command Format, Version
        'READ MODULE CONFIGURATION':MsgFmt('Y','TT'),
        'SET CHANNEL CONFIGURATION':MsgFmt('a CC TT',None),
        'SET I/O CONFIGURATION-GROUP':MsgFmt('G MMMM TT',None),
        'STORE SYSTEM CONFIGURATION':MsgFmt('E',None),
        # Chapter 7 = MsgFmt Digital Read/Write, Latch Commands
        # Command Name, Command Format, Version
        'CLEAR OUTPUT (DEACTIVATE OUTPUT)':MsgFmt('e CC',None),
        'READ AND OPTIONALLY CLEAR LATCHES GROUP':MsgFmt('S FF','PPPP NNNN'),
        'READ AND OPTIONALLY CLEAR LATCH':MsgFmt('w CC FF','DD'),
        'READ MODULE STATUS':MsgFmt('R','DDDD'),
        'SET OUTPUT MODULE STATE-GROUP':MsgFmt('J MMMM NNNN',None),
        'SET OUTPUT (ACTIVATE OUTPUT)':MsgFmt('d CC',None),
        # Chapter 8 = MsgFmt Digital Counter, Frequency Commands
        # Command Name, Command Format, Version
        'READ 32 BIT COUNTER':MsgFmt('l CC','DDDDDDDD'),
        'READ AND CLEAR 16 BIT COUNTER':MsgFmt('o CC','DDDD'),
        'CLEAR COUNTER':MsgFmt('c CC',None),
        'ENABLE/DISABLE COUNTER GROUP':MsgFmt('H MMMM SS',None),
        'ENABLE/DISABLE COUNTER':MsgFmt('b CC SS',None),
        'READ 16 BIT COUNTER':MsgFmt('m CC','DDDD'),
        'READ 32 BIT COUNTER GROUP':MsgFmt('T MMMM','DDDDDDDD'),
        'READ AND CLEAR 32 BIT COUNTER GROUP':MsgFmt('U MMMM','DDDDDDDD'),
        'READ AND CLEAR 32 BIT COUNTER':MsgFmt('n CC','DDDDDDDD'),
        'READ COUNTER ENABLE/DISABLE STATUS':MsgFmt('u','DDDD'),
        'READ FREQUENCY MEASUREMENT':MsgFmt('t CC','DDDD'),
        'READ FREQUENCY MEASUREMENT GROUP':MsgFmt('Z MMMM','DDDD'),
        # Chapter 9 = MsgFmt Digital Time Delay/Pulse Output Commands
        # Command Name, Command Format, Version
        'SET TIME PROPORTIONAL OUTPUT PERIOD':MsgFmt('] CC TTTTTTTT',None),
        'SET TPO PERCENTAGE':MsgFmt('j CC PPPPPPPP',None),
        'START OFF PULSE':MsgFmt('g CC TTTTTTTT',None),
        'START ON PULSE':MsgFmt('f CC TTTTTTTT',None),
        'GENERATE N PULSES':MsgFmt('i CC NNNNNNNN FFFFFFFF XXXXXXXX',None),
        'READ OUTPUT TIMER COUNTER':MsgFmt('k CC','TTTTTTTT'),
        'START CONTINUOUS SQUARE WAVE':MsgFmt('h CC NNNNNNNN FFFFFFFF',None),
        # Chapter 10: Digital Pulse/Period Measurement Commands
        # Command Name, Command Format, Version
        'READ 32 BIT PULSE/PERIOD MEASUREMENT':MsgFmt('p CC','DDDDDDDD'),
        'READ AND RESTART 16 BIT PULSE/PERIOD':MsgFmt('s CC','DDDD'),
        'READ AND RESTART 32 BIT PULSE/PERIOD':MsgFmt('r CC','DDDDDDDD'),
        'READ 16 BIT PULSE/PERIOD MEASUREMENT':MsgFmt('q CC','DDDD'),
        'READ 32 BIT PULSE/PERIOD GROUP':MsgFmt('W MMMM','DDDDDDDD'),
        'READ AND RESTART 32 BIT PULSE/PERIOD GROUP':MsgFmt('X MMMM','DDDDDDDD'),
        'READ PULSE/PERIOD COMPLETE STATUS':MsgFmt('V','DDDD'),
        # Chapter 11: Digital Event/Reaction Commands
        # Command Name, Command Format, Version
        'ENABLE/DISABLE EVENT ENTRY GROUP':MsgFmt('{ GG MMMM NNNN',None),
        'ENABLE/DISABLE EVENT TABLE ENTRY':MsgFmt('N EE SS',None),
        'READ AND CLEAR EVENT LATCHES':MsgFmt('Q EE','DDDD'),
        'READ EVENT DATA HOLDING BUFFER':MsgFmt('I EE','DDDDDDDD'),
        'READ EVENT ENTRY ENABLE/DISABLE STATUS':MsgFmt('v EE','DDDD'),
        'READ EVENT LATCHES':MsgFmt('P EE','DDDD'),
        'SET EVENT ON COUNTER/TIMER >=':MsgFmt('L EE CC NNNNNNNN',None),
        'SET EVENT ON COUNTER/TIMER <=':MsgFmt('} EE CC NNNNNNNN',None),
        'CLEAR EVENT/REACTION TABLE':MsgFmt('_',None),
        'CLEAR EVENT TABLE ENTRY':MsgFmt('\ EE',None),
        'CLEAR INTERRUPT':MsgFmt('zB',None),
        'READ AND OPTIONALLY CLEAR EVENT LATCH':MsgFmt('zA EE FF','DD'),
        'READ EVENT TABLE ENTRY':MsgFmt('O EE','CB CC RC RCC ED RES NNNNNNNN TTTTTTTT'),
        'SET EVENT INTERRUPT STATUS':MsgFmt('I EE SS',None),
        'SET EVENT ON COMM LINK WATCHDOG TIMEOUT':MsgFmt('y EE',None),
        'SET EVENT ON MOMO MATCH':MsgFmt('K EE MMMM NNNN',None),
        'SET EVENT REACTION COMMAND':MsgFmt('M EE',None),
        # Chapter 12: Analog Setup/System Commands
        # Command Name, Command Format, Version
        'SET COMM LINK WATCHDOG AND DELAY':MsgFmt('D TTTT',None),
        'SET COMM LINK WATCHDOG TIMEOUT DATA':MsgFmt('H MMMM DDDDDDDD',None),
        # Chapter 13: Analog I/O Configuration Commands
        # Command Name, Command Format, Version
        'CALCULATE AND SET ADC MODULE OFFSET':MsgFmt('d CC','OOOO'),
        'CALCULATE AND SET ADC MODULE GAIN':MsgFmt('e CC','GGGG'),     
        'SET ADC MODULE OFFSET':MsgFmt('b CC OOOO',None),
        'SET ADC MODULE GAIN':MsgFmt('c CC GGGG',None),
        'SET AVERAGING SAMPLE WEIGHT (DIG. FILTERING)':MsgFmt('h CC DDDD',None),
        'SET TOTALIZATION SAMPLE RATE':MsgFmt('g CC DDDD',None),
        'SET ENGINEERING UNIT SCALING PARAMETERS':MsgFmt('f CC HHHHHHHH LLLLLLLL',None),
        'SET TPO RESOLUTION':MsgFmt('] CC SS',None),
        # Chapter 14: Analog Read/Write/Output Commands
        # Command Name, Command Format, Version
        'RAMP DAC OUTPUT TO ENDPOINT':MsgFmt('Z CC EEEEEEEE SSSSSSSS',None),
        'READ AND CLEAR I/O MODULE 16 BIT DATA':MsgFmt('s CC TT','DDDD'),
        'READ AND CLEAR I/O MODULE 16 BIT DATA-GROUP':MsgFmt('S MMMM TT','DDDD'),
        'READ AND CLEAR I/O MODULE 32 BIT DATA':MsgFmt('s CC TT','DDDDDDDD'),
        'READ AND CLEAR I/O MODULE 32 BIT DATA-GROUP':MsgFmt('S MMMM TT','DDDDDDDD'),
        'READ I/O MODULE 16 BIT MAGNITUDE':MsgFmt('r CC TT','DDDD'),
        'READ I/O MODULE 16 BIT MAGNITUDE-GROUP':MsgFmt('R MMMM TT','DDDD'),
        'READ I/O MODULE 32 BIT MAGNITUDE':MsgFmt('r CC TT','DDDDDDDD'),
        'READ I/O MODULE 32 BIT MAGNITUDE-GROUP':MsgFmt('R MMMM TT','DDDDDDDD'),
        'SET DAC MODULE MAGNITUDE, ENG. UNITS':MsgFmt('w CC DDDDDDDD',None),
        'SET DAC MODULE MAGNITUDE, ENG. UNITS-GRP.':MsgFmt('W MMMM DDDDDDDD',None),
        'SET DAC MODULE MAGNITUDE, COUNTS':MsgFmt('x CC DDDD',None),
        'SET DAC MODULE MAGNITUDE, COUNTS-GROUP':MsgFmt('X MMMM DDDD',None),
        # Chapter 15: Analog Event/Reaction Commands
        # Command Name, Command Format, Version
        
        # Note: ***************************************************************
        # Setpoints may be 16 or 32 bits.  The command format showed the data
        # as DDDD[DDDD].  Since DDDD[DDDD] is not a valid Python variable
        # name, and the length of the variable name is used as a field width
        # indication, the commands 'SET EVENT ON I/O >= SETPOINT' and
        # 'SET EVENT ON I/O <= SETPOINT are expanded to two versions for each,
        # a 16 bit (COUNTS) and a 32 bit (ENG. UNITS) version
        # *********************************************************************
        'SET EVENT ON I/O >= SETPOINT (COUNTS)':MsgFmt('K EE CC TT DDDD',None),
        'SET EVENT ON I/O >= SETPOINT (ENG. UNITS)':MsgFmt('K EE CC TT DDDDDDDD',None),
        'SET EVENT ON I/O <= SETPOINT (COUNTS)':MsgFmt('L EE CC TT DDDD',None),
        'SET EVENT ON I/O <= SETPOINT (ENG. UNITS)':MsgFmt('L EE CC TT DDDDDDDD',None),
        
        # Note: ***************************************************************
        # 'SET EVENT REACTION COMMAND':MsgFmt('M EE [REACTION COMMAND]',None)
        #
        # The 'SET EVENT REACTION COMMAND' takes a parameter [REACTION COMMAND]
        # which is specific to each reaction type.  It seemed like the best way
        # handle this was to split the 'SET EVENT REACTION COMMAND' into
        # separated functions, pasing the [REACTION COMMAND] parameters as if
        # they were command arguments.
        #
        # A [REACTION COMMAND] is essentially a way for Mistic to call certain
        # digital or analog functions when a specific event happens, thus
        # minimizing the need for the host to poll serially before sending a
        # command to take some action.
        # *********************************************************************

        # REACTION COMMANDS (Configuration M)
        'ON EVENT NULL REACTION (DO NOTHING)':MsgFmt('M EE RC',None),
        'ON EVENT ENABLE/DISABLE EVENT TABLE ENTRY':MsgFmt('M EE RC ee SS',None),
        'ON EVENT ENABLE/DISABLE EVENT ENTRY GROUP':MsgFmt('M EE RC GG MMMM NNNN',None),
        # REACTION COMMANDS (To be used with analog command M)
        'ON EVENT SET OUTPUT MODULE STATE-GROUP':MsgFmt('M EE RC MMMM NNNN',None),
        'ON EVENT START ON PULSE':MsgFmt('M EE RC CC TTTTTTTT',None),
        'ON EVENT START OFF PULSE':MsgFmt('M EE RC CC TTTTTTTT',None),
        'ON EVENT ENABLE/DISABLE COUNTER':MsgFmt('M EE RC CC SS',None),
        'ON EVENT CLEAR COUNTER':MsgFmt('M EE RC CC',None),        
        'ON EVENT READ AND HOLD COUNTER VALUE':MsgFmt('M EE RC CC',None),       
        # REACTION COMMANDS (To be used with analog command M)
        'ON EVENT SET DAC MODULE MAGNITUDE, COUNTS':MsgFmt('M EE RC CC DDDD',None),
        'ON EVENT SET DAC MODULE MAGNITUDE, ENG. UNITS':MsgFmt('M EE RC CC DDDDDDDD',None),
        'ON EVENT RAMP DAC OUTPUT TO ENDPOINT':MsgFmt('M EE RC CC EEEEEEEE SSSSSSSS',None),
        'ON EVENT ENABLE/DISABLE PID LOOP':MsgFmt('M EE RC LL SS',None),
        'ON EVENT SET PID LOOP SETPOINT':MsgFmt('M EE RC LL SSSSSSSS',None),
        'ON EVENT READ AND HOLD I/O DATA':MsgFmt('M EE RC CC TT',None),
        'ON EVENT SET PID LOOP MIN-MAX OUTPUT LIMITS':MsgFmt('M EE RC LL HHHHHHHH LLLLLLLL',None),
        
        # Chapter 16: PID Loop Commands
        # Command Name, Command Format, Version
        'SET PID LOOP CONTROL OPTIONS':MsgFmt('j CC SSSS CCCC',None),
        'SET PID LOOP DERIVATIVE RATE':MsgFmt('n LL DDDDDDDD',None),
        'SET PID LOOP GAIN':MsgFmt('l LL GGGGGGGG',None),
        'SET PID LOOP PROCESS VARIABLE':MsgFmt('q LL SSSSSSSS',None),
        'SET PID LOOP SETPOINT':MsgFmt('k LL SSSSSSSS',None),
        'INITIALIZE PID LOOP':MsgFmt('i LL II SS OO TTTT',None),
        'READ ALL PID LOOP PARAMETERS':MsgFmt('T LL',\
            'CCCC RRRR II SS ZZ OO PPPPPPPP SSSSSSSS GGGGGGGG IIIIIIII DDDDDDDD HHHHHHHH ' + \
            'LLLLLLLL ZZZZ YYYY OOOOOOOO AAAAAAAA BBBBBBBB QQQQQQQQ FFFFFFFF KKKKKKKK'),
        
##        'READ PID LOOP PARAMETER':MsgFmt('t LL PP','DDDD'),
        # break down into sub commands because the response data field varies
        'READ PID LOOP CONTROL WORD':MsgFmt('t LL PP','DDDD'),
        'READ PID LOOP RATE WORD':MsgFmt('t LL PP','DDDD'),
        'READ PID LOOP OUTPUT COUNTS':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP INPUT, SETPOINT, OUTPUT CHANNELS':MsgFmt('t LL PP','II SS ZZ OO'),
        'READ PID LOOP INPUT VALUE':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP SETPOINT VALUE':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP OUTPUT VALUE':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP GAIN TERM':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP INTEGRAL TERM':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP DERIVATIVE TERM':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP MAXIMUM SETPOINT LIMIT':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP MINIMUM SETPOINT LIMIT':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP OUTPUT MAXIMUM LIMIT':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP OUTPUT MINIMUM LIMIT':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP OUTPUT MAXIMUM CHANGE PER SCAN':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP OUTPUT COUNTS':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP FULL SCALE IN ENG. UNITS':MsgFmt('t LL PP','DDDDDDDD'),
        'READ PID LOOP ZERO SCALE IN ENG. UNITS':MsgFmt('t LL PP','DDDDDDDD'),
        
        'SET PID LOOP INTEGRAL RESET RATE':MsgFmt('m LL IIIIIIII',None),
        'SET PID LOOP MAXIMUM RATE OF CHANGE':MsgFmt('u LL RRRRRRRR',None),
        'SET PID LOOP MIN-MAX OUTPUT LIMITS':MsgFmt('p LL HHHHHHHH LLLLLLLL',None),
        'SET PID LOOP MIN-MAX SETPOINT LIMITS':MsgFmt('o LL HHHHHHHH LLLLLLLL',None)
        }       

##        Bit 0 = Frequency resolution setting: 0 = 1 Hz, 1 = 10 Hz.
##        Bit 1 = Not used.
##        Bit 2 = Not used.
##        Bit 3 = Not used.
##        Bit 4 = CRC initialization value: 0 = 0000, 1 = FFFF.
##        Bit 5 = CRC method select: 0 = reverse, 1 = classical.
##        Bit 6 = CRC polynomial select: 0 = CRC16, 1 = CCITT.
##        Bit 7 = Global event interrupt enable: 0 = disabled, 1 = enabled.
    def __init__(self):
        """
        """
        self.tty = tty.OmstTTY()
        # array of bytes, resolution is 10ms
        self.response_delay = {}
        
        # save a copy of the 'SET SYSTEM OPTIONS" byte
        self.brain_syst_options = {}
        
        # not sure if this is needed but it might be since
        # there is a repeat last response and we don't want
        # to lose sight of what command was sent
        self.last_command = {}

        if self.crc16r == None:
            self.crc16r = self.generate_crc_table(0xa001)            

    @utl.logger
    def get_response_timeout(self,aa):
        """
        Compute a response timeout so read won't hang forever.
        It is based on the time at the current baudrate of
        sending the longest command and receiving the longest
        response, plus the 'Set Turnaround Delay' time plus
        a 10ms buffer starting at the current perf_counter
        reading.

        if the tty is invalid, return -1
        """
        if self.tty.fd and os.isatty(self.tty.fd):
            if aa not in self.response_delay:
                self.response_delay[aa] = 0
            # max ASCII response packet would likely be:
            # 'A'+ (16 32 bit analog values) + (16 bit crc) + \r
            max_opto_msg = 1 + 16 * 8 + 4 + 1
            # time required to send two max sized messages
            # 2 * secs/char * len(maxmsg)
            # + turnaround delay setting
            # + current perf_counter
            # + 10 ms fudge factor
            to = 2 * 10/self.tty.baud * max_opto_msg \
                + 0.010 * self.response_delay[aa] \
                + perf_counter() \
                + 0.010
            # some extra time for twiddling termios in binary mode
            if self.binary_mode:
                to += 0.005 * max_opto_msg//2
            return to
        return -1

    @utl.logger
    def verify_dvf(self,aa,data):
        """
        """
        if self.binary_mode:
            if self.crc_mode:
                exp = int.from_bytes(data[-2:],byteorder='big')
                act = self.crc(aa,data[0:-2])
                return(act == exp)
            else:
                exp = int.from_bytes(data[-1:],byteorder='big')
                act = self.chksum(aa,data[0:-1])
                return(act == exp)
        else:
            if self.crc_mode:
                act = int(data[-5:-1],16)
                exp = self.crc(aa,data[:-5])
                return(act == exp)
            else:            
                act = int(data[-3:-1],16)
                exp = self.chksum(aa,data[:-3])
                return(act == exp)
        return False

    @utl.logger
    def mistic_data_to_int16(self,v):
        """
        Mistic returns module data in 2's complement
        """
        if v & 0x8000:
            return 0x10000 - v
        return v

    @utl.logger
    def mistic_data_to_int32(self,v):
        """
        Mistic returns module data in 2's complement
        """
        if v & 0x80000000:
            return (0x100000000 - v)
        return v

    @utl.logger
    def mistic_data_to_engineering_units(self,v):
        if v & 0x80000000:
            v = (0x100000000 - v)
        return v / 65536
        
    @utl.logger
    def parse_mistic_response_data(self,names,rsp):
        """
        Analog data tends to come over the wire as twos complement
        """
        ntflds = namedtuple('ntflds',list(rsp.data._fields))
        ntdats = []
        for field in rsp.data._fields:
            v = getattr(rsp.data,field)
            if isinstance(v,int):
                if field in names:
                    if len(field) == 4:
                        ntdats += [self.mistic_data_to_int16(v)]
                    elif len(field) == 8:                    
                        ntdats += [self.mistic_data_to_engineering_units(v)]
                else:
                    ntdats += [v]
            elif isinstance(v,tuple):
                ll = []
                for e in v:
                    if field in names:
                        if len(field) == 4:
                            ll += [self.mistic_data_to_int16(e)]
                        elif len(field) == 8:                    
                            ll += [self.mistic_data_to_engineering_units(e)]
                    else:
                        ll += [v]
                ntdats += [tuple(ll)]
        return RspMsg(rsp.ack,ntflds(*ntdats))    

    @utl.logger
    def int16_to_mistic_data(self,v):
        """
        Mistic wants module data in 2's complement
        """
        if v < 0:
            v += 0x10000
        return v

    @utl.logger
    def int32_to_mistic_data(self,v):
        if v < 0:
            v += 0x100000000
        return v
        
    @utl.logger
    def engineering_units_to_mistic_data(self,v):
        """
        Mistic wants module data in 2's complement
        """
        v = int(v*65536)
        if v < 0:
            v += 0x100000000
        return v

    """
    responses to the identify type command
    Note:  this one is a bit confusing.
    The B3000 returns A003004\r which I presume
    is hex, but the manual lists the expected
    value as 0048 which is heximal or deximal?
    """    
    digital_brainboards = {
        0x0a:'Remote 16-Point Digital Multifunction I/O Unit',
        0x0b:'G4D32RS Digital Remote Simple I/O Unit',
        0x14:'Local 16-Point Digital Multifunction I/O Unit',
        0x15:'Local 16-Point Digital Non-multifunction I/O Unit',
        0x30:'B3000 (digital address) Multifunction I/O Unit',
        }
    
    analog_brainboards = {
        0x0c:'Remote 16-Point Analog I/O Unit',
        0x0d:'Remote 8-Point Analog I/O Unit',
        0x16:'Local 16-Point Analog I/O Unit',
        0x17:'Local 8-Point Analog I/O Unit',
        0x32:'B3000 (analog address) Multifunction I/O Unit',
        }    

    @utl.logger
    def identify_type(self,aa):
        """
        Sends an 'IDENTIFY TYPE'
            MsgFmt(cmd='F', rsp='DDDD')

        Data out:
            None
            
        Data in:
            DDDD is a 16 bit value from the following table:        
                10 = Remote 16-Point Digital Multifunction I/O Unit
                11 = G4D32RS Digital Remote Simple I/O Unit
                12 = Remote 16-Point Analog I/O Unit
                13 = Remote 8-Point Analog I/O Unit
                20 = Local 16-Point Digital Multifunction I/O Unit
                21 = Local 16-Point Digital Non-multifunction I/O Unit
                22 = Local 16-Point Analog I/O Unit
                23 = Local 8-Point Analog I/O Unit
                48 = B3000 (digital address) Multifunction I/O Unit
                50 = B3000 (analog address) Multifunction I/O Unit        
        """
        return self.send_receive_2(aa,'IDENTIFY TYPE',inspect.currentframe())

    @utl.logger
    def what_am_i(self,aa):
        """
        what type of brain or controller am I in text?
        """
        rsp = self.identify_type(aa)
        if rsp.ack == 'A':
            ntflds = namedtuple('ntflds',['DDDD','BrainBoardType'])
            devices = {}
            devices.update(self.digital_brainboards)
            devices.update(self.analog_brainboards)
            bbt =(rsp.data.DDDD,devices[rsp.data.DDDD[0]])
            return RspMsg(rsp.ack,ntflds(*bbt))
        return rsp
    
    @utl.logger
    def power_up_clear(self,aa):
        """
        Sends a 'POWER UP CLEAR'
            MsgFmt(cmd='A', rsp=None)
        
        Data out:
            None

        Data In:
            None

        Prevents the I/O unit from returning a Power-Up Clear Expected
        error message in response to instructions following application
        of power or the Reset command.
        """
        return self.send_receive_2(aa,'POWER UP CLEAR',inspect.currentframe())        

    @utl.logger
    def repeat_last_response(self,aa):
        """
        """
        return self.send_receive_2(aa,'REPEAT LAST RESPONSE',inspect.currentframe())        

    @utl.logger
    def reset(self,aa):
        """
        Send a 'REPEAT LAST RESPONSE'
            MsgFmt(cmd='^', rsp=None)
        
        Data out:
            None

        Data In:
            None

        This command causes the addressed unit to repeat the response to the
        previous command.
        """
        return self.send_receive_2(aa,'RESET',inspect.currentframe())
    
    @utl.logger
    def reset_all_parameters_to_default(self,aa):
        """
        Send a 'RESET ALL PARAMETERS TO DEFAULT'
            MsgFmt(cmd='x', rsp=None)
        
        Data out:
            None

        Data In:
            None

        This command will cause all modules and event parameters to be reset to
        factory default conditions.
        """
        return self.send_receive_2(aa,'RESET ALL PARAMETERS TO DEFAULT',inspect.currentframe())

    @utl.logger
    def set_response_delay(self,aa,DD=0):
        """
        Send a 'SET RESPONSE DELAY'
            MsgFmt(cmd='~ DD', rsp=None)
        
        Data out:
            DD - delay in 10 ms increments

        Data In:
            None

        REMARKS :
        After acknowledging the “Set Response Delay” command, for all
        subsequent commands, the I/O unit will wait DD x 10 milliseconds
        before responding. Note that any particular command is executed
        immediately and that only the acknowledge or data response is delayed.
        The default response delay setting is zero unless a different value
        has been set and saved in EEPROM with a “Store System Configuration”
        command.
        """
        rsp = self.send_receive_2(aa,'SET RESPONSE DELAY',inspect.currentframe())
        if rsp.ack == 'A':
            self.response_delay[aa] = DD
        return rsp

    """
    bits in the system options byte
    """
    system_options = {
        'Frequency resolution setting':(0,0),   # 0 = 1 Hz, 1 = 10 Hz.
        'CRC initialization value':(4,0),       # 0 = 0000, 1 = FFFF.
        'CRC method select':(5,0),              # 0 = reverse, 1 = classical.
        'CRC polynomial select':(6,0),          # 0 = CRC16, 1 = CCITT.
        'Global event interrupt enable':(7,0)   # 0 = disabled, 1 = enabled.
        }

    @utl.logger
    def set_frequency_resolution_to_10_hz(self,aa,SS=1,CC=0):
        rsp = self.send_receive_2(aa,'SET SYSTEM OPTIONS',inspect.currentframe())
        if rsp.ack == 'A':
            self.brain_syst_options[aa] = rsp.data[0]
        return rsp
    
    @utl.logger
    def set_frequency_resolution_to_1_hz(self,aa,SS=0,CC=1):
        if rsp.ack == 'A':
            self.brain_syst_options[aa] = rsp.data[0]
        return rsp

    @utl.logger
    def get_frequency_resolution(self,aa,SS=0,CC=0):
        rsp = self.send_receive_2(aa,'SET SYSTEM OPTIONS',inspect.currentframe())
        if rsp.ack == 'A':
            self.brain_syst_options[aa] = rsp.data[0]
        return rsp
    
    @utl.logger
    def enable_global_interrupt(self,aa,SS=128,CC=0):
        rsp = self.send_receive_2(aa,'SET SYSTEM OPTIONS',inspect.currentframe())
        if rsp.ack == 'A':
            self.brain_syst_options[aa] = rsp.data[0]
        return rsp
    
    @utl.logger
    def disable_global_interrupt(self,aa,SS=0,CC=128):
        rsp = self.send_receive_2(aa,'SET SYSTEM OPTIONS',inspect.currentframe())
        if rsp.ack == 'A':
            self.brain_syst_options[aa] = rsp.data[0]
        return rsp

    @utl.logger
    def get_global_interrupt_enable(self,aa,SS=0,CC=0):
        rsp = self.send_receive_2(aa,'SET SYSTEM OPTIONS',inspect.currentframe())
        if rsp.ack == 'A':
            self.brain_syst_options[aa] = rsp.data[0]
        return rsp

    @utl.logger
    def set_system_options(self,aa,SS=0,CC=0):
        """
        Sends a 'SET SYSTEM OPTIONS'
            MsgFmt(cmd='C SS CC', rsp='DDDD')

        Data out:
            SS - bits to be set
            CC - bits to be cleared

        Data In:
            DDDD - 

        This command is used to set (SS) or clear (CC) the bits in the Option
        Control Byte. The Option Control Byte is used to select certain system
        options. The system options and the controlling bits are as follows:
        
            Bit 0 = Frequency resolution setting: 0 = 1 Hz, 1 = 10 Hz.
            Bit 1 = Not used.
            Bit 2 = Not used.
            Bit 3 = Not used.
            Bit 4 = CRC initialization value: 0 = 0000, 1 = FFFF.
            Bit 5 = CRC method select: 0 = reverse, 1 = classical.
            Bit 6 = CRC polynomial select: 0 = CRC16, 1 = CCITT.
            Bit 7 = Global event interrupt enable: 0 = disabled, 1 = enabled.
            
        The factory default is 00. This byte is stored in EEPROM when command E
        is executed and is restored upon power-up or when the RESET command (B)
        is executed.

        You can use this command to set (SS), clear (CC) or do nothing to the 
        bits in the Option Control Byte.

        You can use the Store System Configuration (Command E) to save the Option
        Control Byte in EEPROM. The Option Control Byte is always restored from
        EEPROM upon power-up or when the RESET (Command B) is executed.
        
        The response from this command is a 16-bit word. The current value of the
        Option Control Byte is returned as the least significant byte of the 16-bit
        word. The most significant byte can be ignored. It is for future expansion.
        """
        return self.send_receive_2(aa,'SET SYSTEM OPTIONS',inspect.currentframe())

    @utl.logger
    def set_comm_link_watchdog_momo_and_delay(self,aa,MMMM,NNNN,TTTT):
        """
        Send a 'SET WATCHDOG MOMO AND DELAY'
            MsgFmt(cmd='D MMMM NNNN TTTT', rsp=None)

        Data out:
            MMMM - digital outputs to be turned on
            NNNN - digital outputs to be turned off
            TTTT - delay in 10ms ticks

        Data In:
            None

        This command sets the communications line watchdog parameters for the
        addressed I/O unit. When enabled, if a command (or > character for ASCII
        protocol) is not received after the specified delay, output modules may
        be instructed to turn on, turn off or do nothing. Anytime delays on
        specified output channels are canceled upon watchdog timeout. After
        the specified modules are rturned on or turned off, a complete scan of the
        event/reaction table occurs, starting from 0. A delay of zero (0) disables
        the watchdog function.

        The minimum delay for the watchdog timer is 200 milliseconds
        (TTTT = 0014 Hex). If the watchdog timer for a particular I/O unit times
        out, the next instruction sent to the I/O unit will not be executed.
        Instead an error code will be sent back to the host computer. This error
        code is sent as a warning to let the host know that a timeout occurred.
        Subsequent commands will be executed in a normal manner, provided the
        time interval between commands is shorter than the watchdog timer delay
        time.

        On power up this parameter is restored from EEPROM memory.
        """
        return self.send_receive_2(aa,'SET WATCHDOG MOMO AND DELAY',inspect.currentframe())

    @utl.logger
    def cancel_comm_link_watchdog_momo_and_delay(self,aa):
        """
        Almost impossible to recover from a short WD timer while typing at terminal,
        so this should do it
        """
        rsp = self.power_up_clear(aa)
        if rsp.ack == 'A':
            return self.set_comm_link_watchdog_momo_and_delay(aa,0,0,0)
        return rsp

    """
    Commands that set or get the digital module type at
    one or more locations need these values
    """
    digital_module_types = {
        0x00:'Counter Input',
        0x01:'Positive Pulse Measurement Input',
        0x02:'Negative Pulse Measurement Input',
        0x03:'Period Measurement Input',
        0x04:'Frequency Measurement Input',
        0x05:'Quadrature Counter Input',
        0x06:'On Time Totalizer Input',
        0x07:'Off Time Totalizer Input',
        0x80:'Standard Output'
        }

    """
    Commands that set or get the analog module type at
    one or more locations need these values
    """
    analog_module_types = {
        0x00:'Generic Input Module',
        0x01:'Reserved',
        0x02:'Reserved',
        0x03:'G4AD3 4 to 20 mA',
        0x04:'G4AD4 ICTD',
        0x05:'G4AD5 Type J Thermocouple',
        0x06:'G4AD6 0 to 5 VDC',
        0x07:'G4AD7 0 to 10 VDC',
        0x08:'G4AD8 Type K Thermocouple',
        0x09:'G4AD9 0 to 50 mV',
        0x0A:'G4AD10 100 Ohm RTD',
        0x0B:'G4AD11 -5 to +5 VDC',
        0x0C:'G4AD12 -10 to +10 VDC',
        0x0D:'G4AD13 0 to 100 mV',
        0x10:'G4AD16 0 to 5 Amperes',
        0x11:'G4AD17 Type R Thermocouple',
        0x12:'G4AD18 Type T Thermocouple',
        0x13:'G4AD19 Type E Thermocouple',
        0x14:'G4AD20 0 to 4095 Hz.',
        0x16:'G4AD22 0 to 1 VDC',
        0x17:'G4AD17 Type S Thermocouple',
        0x18:'G4AD24 Type B Thermocouple',
        0x19:'G4AD25 0 to 100 VAC/VDC',
        0x80:'Generic Output Module',
        0x81:'Reserved',
        0x82:'Reserved',
        0x83:'G4DA3 4 to 20 mA',
        0x84:'G4DA4 0 to 5 VDC',
        0x85:'G4DA5 0 to 10 VDC',
        0x86:'G4DA6 -5 to +5 VDC',
        0x87:'G4DA7 -10 to +10 VDC',
        0x88:'G4DA8 0 to 20 mA',
        0x89:'G4DA9 Time Proportional Output',
        }
    
    @utl.logger
    def read_module_configuration(self,aa):
        """
        Send a 'READ MODULE CONFIGURATION'
            MsgFmt(cmd='Y', rsp='DD')
            
        Data out:
            None

        Data In:
            DD - a tuple of module types

        This command causes the addressed unit to send back a response to the
        host that identifies the module configuration type for each of the 16
        channels. Data is returned as 2 ASCII Hex digits for each of the 16
        channels for ASCII protocol. For binary protocol, data is returned as
        1 data byte for each channel.

        For digital modules, the data is interpreted as follows:
            00 Hex = Counter Input
            01 Hex = Positive Pulse Measurement Input
            02 Hex = Negative Pulse Measurement Input
            03 Hex = Period Measurement Input
            04 Hex = Frequency Measurement Input
            05 Hex = Quadrature Counter Input
            06 Hex = On Time Totalizer Input
            07 Hex = Off Time Totalizer Input
            80 Hex = Standard Output

        For analog modules, the data is interpreted as follows:
            00 = Generic Input Module
            01 = Reserved
            02 = Reserved
            03 = G4AD3 4 to 20 mA
            04 = G4AD4 ICTD
            05 = G4AD5 Type J Thermocouple
            06 = G4AD6 0 to 5 VDC
            07 = G4AD7 0 to 10 VDC
            08 = G4AD8 Type K Thermocouple
            09 = G4AD9 0 to 50 mV
            0A = G4AD10 100 Ohm RTD
            0B = G4AD11 -5 to +5 VDC
            0C = G4AD12 -10 to +10 VDC
            0D = G4AD13 0 to 100 mV
            10 = G4AD16 0 to 5 Amperes
            11 = G4AD17 Type R Thermocouple
            12 = G4AD18 Type T Thermocouple
            13 = G4AD19 Type E Thermocouple
            14 = G4AD20 0 to 4095 Hz.
            16 = G4AD22 0 to 1 VDC
            17 = G4AD17 Type S Thermocouple
            18 = G4AD24 Type B Thermocouple
            19 = G4AD25 0 to 100 VAC/VDC
            80 = Generic Output Module
            81 = Reserved
            82 = Reserved
            83 = G4DA3 4 to 20 mA
            84 = G4DA4 0 to 5 VDC
            85 = G4DA5 0 to 10 VDC
            86 = G4DA6 -5 to +5 VDC
            87 = G4DA7 -10 to +10 VDC
            88 = G4DA8 0 to 20 mA
            89 = G4DA9 Time Proportional Output
        """
        return self.send_receive_2(aa,'READ MODULE CONFIGURATION',inspect.currentframe())

    @utl.logger
    def set_channel_configuration(self,aa,CC,TT):
        """
        Send a 'SET CHANNEL CONFIGURATION'
            MsgFmt(cmd='a CC TT', rsp=None)
        
        Data out:
            CC - channel index
            TT - module type

        Data In:
            None

        This command configures the module type on a singel channel.
        
        For digital modules, the data is interpreted as follows:
            00 Hex = Counter Input
            01 Hex = Positive Pulse Measurement Input
            02 Hex = Negative Pulse Measurement Input
            03 Hex = Period Measurement Input
            04 Hex = Frequency Measurement Input
            05 Hex = Quadrature Counter Input
            06 Hex = On Time Totalizer Input
            07 Hex = Off Time Totalizer Input
            80 Hex = Standard Output

        For analog modules, the data is interpreted as follows:
            00 = Generic Input Module
            01 = Reserved
            02 = Reserved
            03 = G4AD3 4 to 20 mA
            04 = G4AD4 ICTD
            05 = G4AD5 Type J Thermocouple
            06 = G4AD6 0 to 5 VDC
            07 = G4AD7 0 to 10 VDC
            08 = G4AD8 Type K Thermocouple
            09 = G4AD9 0 to 50 mV
            0A = G4AD10 100 Ohm RTD
            0B = G4AD11 -5 to +5 VDC
            0C = G4AD12 -10 to +10 VDC
            0D = G4AD13 0 to 100 mV
            10 = G4AD16 0 to 5 Amperes
            11 = G4AD17 Type R Thermocouple
            12 = G4AD18 Type T Thermocouple
            13 = G4AD19 Type E Thermocouple
            14 = G4AD20 0 to 4095 Hz.
            16 = G4AD22 0 to 1 VDC
            17 = G4AD17 Type S Thermocouple
            18 = G4AD24 Type B Thermocouple
            19 = G4AD25 0 to 100 VAC/VDC
            80 = Generic Output Module
            81 = Reserved
            82 = Reserved
            83 = G4DA3 4 to 20 mA
            84 = G4DA4 0 to 5 VDC
            85 = G4DA5 0 to 10 VDC
            86 = G4DA6 -5 to +5 VDC
            87 = G4DA7 -10 to +10 VDC
            88 = G4DA8 0 to 20 mA
            89 = G4DA9 Time Proportional Output

        """
        return self.send_receive_2(aa,'SET CHANNEL CONFIGURATION',inspect.currentframe())

    @utl.logger
    def set_io_configuration_group(self,aa,MMMM,TT):
        """
        Send a 'SET I/O CONFIGURATION-GROUP'
            MsgFmt(cmd='G MMMM TT', rsp=None)

        Data Out:
            MMMM - bit mask of channels to be configured
            TT   - tuple of module types starting with lowest channel

        Data In:
            None

        This command configures the modules specified by data field MMMM to
        the Configuration Type specified by data field TT. For each bit in
        data field MMMM that is set to a 1, there must be a corresponding
        data field TT.    
        """
        return self.send_receive_2(aa,'SET I/O CONFIGURATION-GROUP',inspect.currentframe())
            
    @utl.logger
    def store_system_configuration(self,aa):
        """
        Send a 'STORE SYSTEM CONFIGURATION'
            MsgFmt(cmd='E', rsp=None)

        Data Out:
            None

        Data In:
            None

        This command saves the current system parameters to EEPROM. The
        parameters saved to EEPROM are:
            (1) Module configuration.
            (2) Counter enable/disable status.
            (3) Communications link watchdog parameters.
            (4) Option Control Byte.
            (5) Response Delay Setting
            (6) Event/Reaction table entries 00 thru 20 Hex.
        This command requires one (1) second to execute. The command response
        is sent after the command has finished executing.
            
        """
        return self.send_receive_2(aa,'STORE SYSTEM CONFIGURATION',inspect.currentframe())

    @utl.logger
    def deactivate_output(self,aa,CC):
        """
        Send a 'CLEAR OUTPUT (DEACTIVATE OUTPUT)'
            MsgFmt(cmd='e CC', rsp=None)

        Data Out:
            CC - channel number

        Data In:
            None

        This command turns ON (activates) the output channel (module) specified
        by data field CC. Any time delay or TPO function on the specified chan-
        nel will be canceled.

        """
        return self.send_receive_2(aa,'CLEAR OUTPUT (DEACTIVATE OUTPUT)',inspect.currentframe())

    """
    encoding of latch data
    """
    latch_groups = {
        0x00 : 'No latches are cleared.',
        0x01 : 'All positive latches are cleared.',
        0x02 : 'All negative latches are cleared.',
        0x03 : 'Both positive and negative latches are cleared.'
        }
   
    @utl.logger
    def read_and_optionally_clear_latches_group(self,aa,FF):
        """
        Send a 'READ AND OPTIONALLY CLEAR LATCHES GROUP'
            MsgFmt(cmd='S FF', rsp='PPPP NNNN')

        Data Out:
            FF - latch group)s) to clear
                0x00 : 'No latches are cleared.',
                0x01 : 'All positive latches are cleared.',
                0x02 : 'All negative latches are cleared.',
                0x03 : 'Both positive and negative latches are cleared.'

        Data In:
            PPPP - mask of posative latches set
            NNNN = mask of negative latches set

        This command returns input latch data (32 bits) for both the positive
        and the negative input latches and then clears the group of latches
        specified by data field FF (see Remarks). Latches function independently
        and concurrently with all other input functions. The latches are always
        functional.

        Positive latches are set (latched) by an OFF to ON (0 to 1) input
        transition. Negative latches are set(latched) by an ON to OFF
        (0 to 1) transition of the input.
        The return data consists of two 16-bit data fields PPPP and NNNN.
        A ‘1’ in the data field indicates that the latch is set and a ‘0’
        in the data field indicates that the latch is cleared.
        """
        return self.send_receive_2(aa,'READ AND OPTIONALLY CLEAR LATCHES GROUP',inspect.currentframe())

    @utl.logger
    def read_and_optionally_clear_latch(self,aa,CC,FF):
        """
        Send a 'READ AND OPTIONALLY CLEAR LATCH'
            MsgFmt(cmd='w CC FF', rsp='DD')

        Data Out:
            CC - channel number
            FF - latch group)s) to clear
                0x00 : 'No latches are cleared.',
                0x01 : 'All positive latches are cleared.',
                0x02 : 'All negative latches are cleared.',
                0x03 : 'Both positive and negative latches are cleared.'

        Data In:
            DD - channel data as mask
            D0 - state of posative latch
            D1 - state of negative latch
            D2 - channel state

        This command returns input latch data and the current module status
        (ON or OFF) for the channel specified by data field CC. Data field
        FF specifies which latch will be cleared (see Remarks). Latches
        function independently and concurrently with all other input functions.
        The latches are always functional. Each channel has one positive and
        one negative latch. The return data consists of an 8-bit data field DD.
        
        The interpretation of the bits in the data field DD are as follows:
            Bit 0 is the state of the positive latch.
                0 = latch is cleared. No positive transition (OFF to ON) has occurred.
                1 = latch is set. At least one positive transition (OFF to ON) has occurred.
            Bit 1 is the state of the negative latch.
                0 = latch is cleared. No negative transition (ON to OFF) has occurred.
                1 = latch is set. At least one negative transition (ON to OFF) has occurred.
            Bit 2 is the current module status.
                0 = input module is OFF. Input voltage is below the logic dropout voltage.
                1 = input module is ON. Input voltage is above the logic pickup voltage.
            Bits 3 to 7 are reserved for future use and will read as 0 (zero).
        """
        return self.send_receive_2(aa,'READ AND OPTIONALLY CLEAR LATCH',inspect.currentframe())

    @utl.logger
    def read_module_status(self,aa):
        """
        Send a 'READ MODULE STATUS'

        Data Out:
            None

        Data In:
            DDDD - module state bit mask as unsigned 16
            bit = 0, module is off
            bit = 1, module is on

        This command reads and returns the current state of all module channels.
        """
        return self.send_receive_2(aa,'READ MODULE STATUS',inspect.currentframe())
    
    @utl.logger
    def set_output_module_state_group(self,aa,MMMM,NNNN):
        """
        Send a 'SET OUTPUT MODULE STATE-GROUP'
            MsgFmt(cmd='J MMMM NNNN', rsp=None)

        Data Out:
            MMMM - bit mask of modules to turn on
            NNNN - bit mask of modules to turn off

        Data In:
            None

        This command is used to set the state of any or all output channels.
        The specified channels must have previously been configured as standard
        outputs by using the ‘G’ or ‘a’ command. Returns an error if a channel
        is specified which is not a standard output. An error is also returned
        if the command specifies a module to be both turned ON and OFF. Any time
        delay or TPO function on a specified channel will be canceled.            
        """
        return self.send_receive_2(aa,'SET OUTPUT MODULE STATE-GROUP',inspect.currentframe())

    @utl.logger
    def activate_output(self,aa,CC):
        """
        Send a 'SET OUTPUT (ACTIVATE OUTPUT)'
        MsgFmt(cmd='d CC', rsp=None)}

        Data Out:
            CC - channel number to activate

        Data In:
            None

        This command turns ON (activates) the output channel (module) specified
        by data field CC. Any time delay or TPO function on the specified channel
        will be canceled.
        """
        return self.send_receive_2(aa,'SET OUTPUT (ACTIVATE OUTPUT)',inspect.currentframe())

    # Chapter 8: Digital Counter, Frequency Commands
    # Command Name, Command Format, Version
    @utl.logger
    def read_32_bit_counter(self,aa,CC):
        """
        Send a 'READ 32 BIT COUNTER'
            MsgFmt(cmd='l CC', rsp='DDDDDDDD')
        
        Data Out:
            CC - channel number to activate

        Data In:
            DDDDDDDD - counter value

        This command returns the current counter value (32 bits) for the
        counter input channel number specified by data field CC.

        The channel specified must be configured as a counter before issuing
        this command. All 32 bits of the current counter value are returned.
        """
        return self.send_receive_2(aa,'READ 32 BIT COUNTER',inspect.currentframe())

    @utl.logger
    def read_and_clear_16_bit_counter(self,aa,CC):
        """
        Send a 'READ AND CLEAR 16 BIT COUNTER'
            MsgFmt(cmd='o CC', rsp='DDDD')
        
        Data Out:
            CC - channel number to read/clear

        Data In:
            DDDD - counter value prior to clear

        This command is used to read the least significant 16 bits of the
        current counter value for the counter input channel number specified
        by data field CC. Counter is cleared after the count is read.

        The channel specified must be configured as a counter before issuing
        this command. Only the least significant 16 bits of the current counter
        value are returned. However, all 32 bits of the counter are cleared.
        """
        return self.send_receive_2(aa,'READ AND CLEAR 16 BIT COUNTER',inspect.currentframe())

    @utl.logger
    def clear_counter(self,aa,CC):    
        """
        Send a 'CLEAR COUNTER'
            MsgFmt(cmd='c CC', rsp=None)
            
        Data Out              
            CC - channel number to clear

        Data In:
            None

        This command clears the counter specified by data field CC. Count is reset
        to zero.

        The channel specified must be configured as a counter before issuing this
        command.
            """
        return self.send_receive_2(aa,'CLEAR COUNTER',inspect.currentframe())

    @utl.logger
    def enable_disable_counter_group(self,aa,MMMM,SS):
        """
        Send a 'ENABLE/DISABLE COUNTER GROUP'
            MsgFmt(cmd='H MMMM SS', rsp=None)
            
        Data Out              
            MMMM - bit mask of counters to change
            SS - ==0 = disable, <>0 = enable

        Data In:
            None

        This command instructs the addressed I/O unit to enable (SS <> 0)
        or disable (SS = 0) counter channels specified by the 16-bit data
        field MMMM. Before an input module can act as a counter, it must
        be configured as a counter input (default configuration) and the
        counter channel must be enabled.
        """
        return self.send_receive_2(aa,'ENABLE/DISABLE COUNTER GROUP',inspect.currentframe())

    @utl.logger
    def enable_disable_counter(self,aa,CC,SS):
        """
        Send a 'ENABLE/DISABLE COUNTER'
            MsgFmt(cmd='b CC SS', rsp=None)
                    
        Data Out              
            CC - channel number
            SS - ==0 = disable, <>0 = enable

        Data In:
            None

        This command instructs the addressed I/O unit to enable or disable
        a counter channel specified by data field CC. Enable or disable is
        specified by data field SS. Before an input module can act as a
        counter, it must be configured as a counter input (default config-
        uration) and the counter channel must be enabled.

        Upon power up all counter channels are disabled.  Disabling a counter
        makes it act as a standard input. Maximum counter frequency is 25 KHz
        The channel number is in range(16).
        """
        return self.send_receive_2(aa,'ENABLE/DISABLE COUNTER',inspect.currentframe())

    @utl.logger
    def read_16_bit_counter(self,aa,CC):
        """
        Send a 'READ 16 BIT COUNTER'
            MsgFmt(cmd='m CC', rsp='DDDD')
                            
        Data Out              
            CC - channel number

        Data In:
            DDDD - least significant word of 32 bit counter

        This command is used to read the least significant 16 bits of the
        current counter value for the counter input channel number specified
        by data field CC.

        The channel specified must be configured as a counter before issuing
        this command. Only the least significant 16 bits of the current counter
        value are returned.
        """
        return self.send_receive_2(aa,'READ 16 BIT COUNTER',inspect.currentframe())

    @utl.logger
    def read_32_bit_counter_group(self,aa,MMMM):
        """
        Send a 'READ 32 BIT COUNTER GROUP'
            MsgFmt(cmd='T MMMM', rsp='DDDDDDDD')
                                    
        Data Out              
            MMMM - mask of channels to read

        Data In:
            DDDDDDDD - counter value

        This command causes the addressed unit to read the counter channels
        specified by data field MMMM and to send back the current 32-bit
        counter value for each of the specified counter channels.

        The channel specified must be configured as a counter before issuing
        this command. The current 32-bit counter value for each of the specified
        channels is returned.
        
        Note:  The users guide says MMMM is optional for ASCII, but it is
        unclear how it handles mixed I/O module configurations, so we're
        requiring MMMM.
        """
        return self.send_receive_2(aa,'READ 32 BIT COUNTER GROUP',inspect.currentframe())

    @utl.logger
    def read_32_bit_counter(self,aa,CC):
        """
        Send a 'READ 32 BIT COUNTER'
            MsgFmt(cmd='l CC', rsp='DDDDDDDD')
            
        Data Out              
            CC - channel number

        Data In:
            DDDD - least significant word of 32 bit counter

        This command returns the current counter value (32 bits) for the
        counter input channel number specified by data field CC.  All 32
        bits of the current counter value are returned.
        
        The channel specified must be configured as a counter before
        issuing this command. 
        """
        return self.send_receive_2(aa,'READ 32 BIT COUNTER',inspect.currentframe())

    @utl.logger
    def read_and_clear_32_bit_counter_group(self,aa,MMMM):
        """
        Send a 'READ AND CLEAR 32 BIT COUNTER GROUP'
            MsgFmt(cmd='U MMMM', rsp='DDDDDDDD')

        Data Out              
            MMMM - channel mask of channels to be read

        Data In:
            DDDDDDDD - tuple of 32 bit counter values

        This command causes the addressed unit to read the counter channels
        specified by data field MMMM and to send back the current 32-bit
        counter value for each of the specified counter channels. Counters are
        cleared after they are read.
        """
        return self.send_receive_2(aa,'READ AND CLEAR 32 BIT COUNTER GROUP',inspect.currentframe())

    @utl.logger
    def read_and_clear_32_bit_counter(self,aa,CC):
        """
        Send a 'READ AND CLEAR 32 BIT COUNTER'
            MsgFmt(cmd='n CC', rsp='DDDDDDDD')
                    
        Data Out              
            CC - channel number

        Data In:
            DDDDDDDD - 32 bit counter value

        This command returns the current counter value (32 bits) for
        the counter channel specified by command data field CC. Counter
        is cleared after the counter is read.
        """
        return self.send_receive_2(aa,'READ AND CLEAR 32 BIT COUNTER',inspect.currentframe())

    @utl.logger
    def read_counter_enable_disable_status(self,aa):
        """
        Send a 'READ COUNTER ENABLE/DISABLE STATUS'
            MsgFmt(cmd='u', rsp='DDDD')        
                    
        Data Out: 
            None

        Data In:
            DDDD - bit mask of counter enable flags

        This command causes the addressed unit to send back a 16-bit
        response data field to the host that identifies the counters
        that are currently enabled. A 1 indicates that the counter is
        enabled and a 0 indicates that the counter is disabled.
        """
        return self.send_receive_2(aa,'READ COUNTER ENABLE/DISABLE STATUS',inspect.currentframe())

    @utl.logger
    def read_frequency_measurement(self,aa,CC):
        """
        Send a 'READ FREQUENCY MEASUREMENT'
            MsgFmt(cmd='t CC', rsp='DDDD')
                            
        Data Out              
            CC - channel number

        Data In:
            DDDD - frequency value

        This command returns the current frequency measurement for the
        channel specified by command data field CC. Returns a 16-bit
        data field which represents frequency in units of 1 Hertz or
        10 Hertz. Maximum frequency input is 25 KHz.

        The channel specified must be configured for Frequency Measurement
        Input before issuing this command. Use Command G or Command a. The
        current frequency value (16 bits) for the specified channel number
        (CC) is returned. Bit 0 in the Option Control Byte determines the
        frequency units. A 0 in bit 0 will return a frequency value in units
        of 1 Hertz. A 1 in bit 0 will return a frequency value in units of
        10 Hertz. Use Command C (Set System Options) to set or clear bits
        in the Option Control Byte.
        """
        return self.send_receive_2(aa,'READ FREQUENCY MEASUREMENT',inspect.currentframe())

    @utl.logger
    def read_frequency_measurement_group(self,aa,MMMM):
        """
        Send a 'READ FREQUENCY MEASUREMENT GROUP'
            MsgFmt(cmd='Z MMMM', rsp='DDDD')
                            
        Data Out              
            MMMM - mask specifying channels to be read

        Data In:
            DDDD - tuple of frequncy values

        This command returns the current frequency measurement for the
        channels specified by command data field MMMM. Returns a 16-bit
        data field which represents frequency in units of 1 Hertz or
        10 Hertz for each specified channel.  Maximum frequency input
        is 25 KHz.

        The channel specified must be configured for Frequency Measurement
        Input before issuing this command. Use Command G or Command a. The
        current frequency value (16 bits) for the specified channel number
        (CC) is returned. Bit 0 in the Option Control Byte determines the
        frequency units. A 0 in bit 0 will return a frequency value in units
        of 1 Hertz. A 1 in bit 0 will return a frequency value in units of
        10 Hertz. Use Command C (Set System Options) to set or clear bits
        in the Option Control Byte.        
        """
        return self.send_receive_2(aa,'READ FREQUENCY MEASUREMENT GROUP',inspect.currentframe())

    # Chapter 9: Digital Time Delay/Pulse Output Commands
    # Command Name, Command Format, Version
    @utl.logger
    def set_time_proportional_output_period(self,aa,CC,TTTTTTTT):
        """
        Send a 'SET TIME PROPORTIONAL OUTPUT PERIOD'
            MsgFmt(cmd='] CC TTTTTTTT', rsp=None)
                                    
        Data Out              
            CC - channel number
            TTTTTTTT - period in 100 usec ticks

        Data In:
            None

        This command is used to set the period of a time proportional output.
        It is the first part of a two part command set. The other part is the
        Set TPO Percentage command (Command j). This command must be executed
        before using the Set TPO Percentage command. Time setting is in units
        of 100 microseconds. A minimum setting of 100 milliseconds is required.
        """
        return self.send_receive_2(aa,'SET TIME PROPORTIONAL OUTPUT PERIOD',inspect.currentframe())

    @utl.logger
    def set_tpo_percentage(self,aa,CC,PPPPPPPP):
        """
        Send a 'SET TPO PERCENTAGE'
            MsgFmt(cmd='j CC PPPPPPPP', rsp=None)
                                            
        Data Out              
            CC - channel number
            PPPPPPPP - period in 100 usec ticks

        Data In:
            None

        This command instructs the addressed I/O unit to set the time pro-
        portional output percentage to the value specified by data field
        PPPPPPPP. Data field PPPPPPPP is a 32-bit unsigned integer. Units
        are in 1/65,536 of a percent thus the most significant 16 bits
        represent the whole number part and the least significant 16 bits
        represent the fractional part of the percentage. The channel number
        is specified by data field CC.
        """
        return self.send_receive_2(aa,'SET TPO PERCENTAGE',inspect.currentframe())

    @utl.logger
    def start_off_pulse(self,aa,CC,TTTTTTTT):                                               
        """
        Send a 'START OFF PULSE'
            MsgFmt(cmd='g CC TTTTTTTT', rsp=None)
                                            
        Data Out              
            CC - channel number
            TTTTTTTT - pulse width in 100 usec ticks

        Data In:
            None

        This command is used to generate a retriggerable OFF pulse at the
        standard output channel specified by data field CC. The delay time
        is specified by the data field TTTTTTTT. Time setting is in units
        of 100 micro seconds. Minimum pulse time is 0.5 milliseconds. An
        error is returned if the time is set for less than 0.5 milliseconds
        (TTTTTTTT < 5).
        """
        return self.send_receive_2(aa,'START OFF PULSE',inspect.currentframe())
                                                   
    @utl.logger
    def start_on_pulse(self,aa,CC,TTTTTTTT):                                               
        """
        Send a 'START ON PULSE'
            MsgFmt(cmd='g CC TTTTTTTT', rsp=None)
                                            
        Data Out              
            CC - channel number
            TTTTTTTT - pulse width in 100 usec ticks

        Data In:
            None

        This command is used to generate a retriggerable OFF pulse at the
        standard output channel specified by data field CC. The delay time
        is specified by the data field TTTTTTTT. Time setting is in units
        of 100 micro seconds. Minimum pulse time is 0.5 milliseconds. An
        error is returned if the time is set for less than 0.5 milliseconds
        (TTTTTTTT < 5).
        """
        return self.send_receive_2(aa,'START ON PULSE',inspect.currentframe())

    @utl.logger
    def generate_n_pulses(self,aa,CC,NNNNNNNN,FFFFFFFF,XXXXXXXX):
        """
        Send a 'GENERATE N PULSES'
            MsgFmt(cmd='i CC NNNNNNNN FFFFFFFF XXXXXXXX', rsp=None)
                                                    
        Data Out
            CC - channel number
            NNNNNNNN - pulse on time in 100us ticks
            FFFFFFFF - pulse off time in 100us ticks
            XXXXXXXX - number of pulses

        Data In:
            None

        This command causes the addressed unit to generate an output pulse
        stream having a specified ON time, OFF time and number of pulses.
        Command data field NNNNNNNN specifies the ON time in 100 microsecond
        units. Command data field FFFFFFFF specifies the OFF time in 100
        microsecond units. Command data field XXXXXXXX specifies the number
        of pulses to be output. Both the ON time and the OFF time must be
        greater than or equal to 1 millisecond.
        """
        return self.send_receive_2(aa,'GENERATE N PULSES',inspect.currentframe())

    @utl.logger
    def read_output_timer_counter(self,aa,CC):
        """
        Send a 'READ OUTPUT TIMER COUNTER'
            MsgFmt(cmd='k CC', rsp='TTTTTTTT')
            
        Data Out
            CC - channel number
            
        Data In:
            TTTTTTTT - time remaining in 100us ticks

        This command returns the current time delay value of an output channel
        which may have a delay in progress. The value represents the time re-
        maining for the delay. A time of zero indicates the delay has finished
        or is not active.

        """
        return self.send_receive_2(aa,'READ OUTPUT TIMER COUNTER',inspect.currentframe())

    @utl.logger
    def start_continuous_square_wave(self,aa,CC,NNNNNNNN,FFFFFFFF):
        """
        Send a 'START CONTINUOUS SQUARE WAVE'
            MsgFmt(cmd='h CC NNNNNNNN FFFFFFFF', rsp=None)

        Data Out
            CC - channel number
            NNNNNNNN - pulse on time in 100us ticks
            FFFFFFFF - pulse off time in 100us ticks

        Data In:
            None

        This command is used to start a continuous square wave at the output
        channel specified by data field CC. Data field NNNNNNNN specifies the
        ON time in 100 microsecond units. Data field FFFFFFFF specifies the
        OFF time in 100 microsecond units.
        """
        return self.send_receive_2(aa,'START CONTINUOUS SQUARE WAVE',inspect.currentframe())

    # Chapter 10: Digital Pulse/Period Measurement Commands
    # Command Name, Command Format, Version
    @utl.logger
    def read_32_bit_pulse_period_measurement(self,aa,CC):
        """
        Send a 'READ 32 BIT PULSE/PERIOD MEASUREMENT'
            MsgFmt(cmd='p CC', rsp='DDDDDDDD')
            
        Data Out
            CC - channel number

        Data In:
            DDDDDDDD - 32 bit period in 100us ticks

        This command returns the pulse/period measurement data (32-bits) for
        the input channel number specified by data field CC.

        The channel specified must be configured for pulse measurement, period
        measurement or pulse totalization before issuing this command. All
        32-bits of the pulse/period measurement data are returned. Returned data
        is in units of 100 microseconds.
        """
        return self.send_receive_2(aa,'READ 32 BIT PULSE/PERIOD MEASUREMENT',inspect.currentframe())

    @utl.logger
    def read_and_restart_16_bit_pulse_period(self,aa,CC):
        """
        Send a
        """
        return self.send_receive_2(aa,'READ AND RESTART 16 BIT PULSE/PERIOD',inspect.currentframe())

    @utl.logger
    def read_and_restart_32_bit_pulse_period(self,aa,CC):
        """
        Send a 'READ AND RESTART 32 BIT PULSE/PERIOD'
            MsgFmt(cmd='p CC', rsp='DDDDDDDD')
            
        Data Out
            CC - channel number

        Data In:
            DDDDDDDD - 32 bit period in 100us ticks

        This command reads the pulse/period measurement data (32-bits) for
        the input channel number specified by data field CC, then clears it.

        The channel specified must be configured for pulse measurement, period
        measurement or pulse totalization before issuing this command. All
        32-bits of the pulse/period measurement data are returned. Returned data
        is in units of 100 microseconds.
        """
        return self.send_receive_2(aa,'READ AND RESTART 32 BIT PULSE/PERIOD',inspect.currentframe())

    @utl.logger
    def read_16_bit_pulse_period_measurement(self,aa,CC):
        """
        Send a 'READ 16 BIT PULSE/PERIOD MEASUREMENT'
            MsgFmt(cmd='p CC', rsp='DDDD')
            
        Data Out
            CC - channel number

        Data In:
            DDDD - 16 bit period in 100us ticks

        This command returns the pulse/period measurement data (16-bits) for
        the input channel number specified by data field CC.

        The channel specified must be configured for pulse measurement, period
        measurement or pulse totalization before issuing this command. Returned
        data is in units of 100 microseconds.
        """
        return self.send_receive_2(aa,'READ 16 BIT PULSE/PERIOD MEASUREMENT',inspect.currentframe())

    @utl.logger
    def read_32_bit_pulse_period_group(self,aa,MMMM):
        """
        Send a 'READ 32 BIT PULSE/PERIOD GROUP'
            MsgFmt(cmd='p MMMM', rsp='DDDDDDDD')
            
        Data Out
            MMMM - bit mask of channels to be read

        Data In:
            DDDDDDDD - tuple of 32 bit period values in 100us ticks

        This command reads the pulse/period measurement data (32-bits) for
        the input channels specified by data field MMMM.

        The channels specified must be configured for pulse measurement, period
        measurement or pulse totalization before issuing this command. All
        32-bits of the pulse/period measurement data are returned. Returned data
        is in units of 100 microseconds.
        """
        return self.send_receive_2(aa,'READ 32 BIT PULSE/PERIOD GROUP',inspect.currentframe())

    @utl.logger
    def read_and_restart_32_bit_pulse_period_group(self,aa,MMMM):
        """
        Send a 'READ AND RESTART 32 BIT PULSE/PERIOD GROUP'
            MsgFmt(cmd='p MMMM', rsp='DDDDDDDD')
            
        Data Out
            MMMM - bit mask of channels to be read/cleared

        Data In:
            DDDDDDDD - tuple of 32 bit period values in 100us ticks

        This command reads the pulse/period measurement data (32-bits) for
        the input channels specified by data field MMMM, and then clears them.

        The channels specified must be configured for pulse measurement, period
        measurement or pulse totalization before issuing this command. All
        32-bits of the pulse/period measurement data are returned. Returned data
        is in units of 100 microseconds.
        """
        return self.send_receive_2(aa,'READ AND RESTART 32 BIT PULSE/PERIOD GROUP',inspect.currentframe())

    @utl.logger
    def read_pulse_period_complete_status(self,aa):
        """
        Send a 'READ PULSE/PERIOD COMPLETE STATUS'
            MsgFmt(cmd='V', rsp='DDDD')

        Data Out:
            None

        Data In:
            DDDD - bit mask of complete period measurements

        This command causes the addressed unit to send back a 16-bit response
        data field to the host that identifies the channels which have measured
        one complete pulse or one complete period (cycle). A 1 indicates that
        the measurement is complete and that data is ready. A 0 indicates that
        the measurement is in progress or waiting for an edge trigger to occur
        or else the channel is not configured for pulse/period measurements.
        """
        return self.send_receive_2(aa,'READ PULSE/PERIOD COMPLETE STATUS',inspect.currentframe())

    # Chapter 11: Digital Event/Reaction Commands
    # Command Name, Command Format, Version
    @utl.logger
    def enable_disable_event_entry_group(self,aa,GG,MMMM,NNNN):
        """
        Send a 'ENABLE/DISABLE EVENT ENTRY GROUP'
            MsgFmt(cmd='{ GG MMMM NNNN', rsp=None)

        Data Out:
            GG - event entry group number
            MMMM - entries to enable
            NNNN - entries to disable

        Data In:
            None

        This command is used to enable and/or disable a selected group of 16
        event table entries in the event/reaction table. Data field GG spec-
        ifies which group of entries is to be enabled (or disabled). The most
        significant nibble (the upper 4 bits) of GG is used to determine
        which group of 16 entries will be enabled (or disabled). The lower
        nibble is ignored. See Remarks for examples. Data field MMMM is a bit-
        mask representing entries to be enabled. Data field NNNN is a bitmask
        representing entries to be disabled. Setting the same bit in both
        enable and disable bitmasks will result in an error being returned and
        the command is not executed. An entry which is not enabled is not
        checked for it’s event occurrence by the CPU.
        """
        return self.send_receive_2(aa,'ENABLE/DISABLE EVENT ENTRY GROUP',inspect.currentframe())

    @utl.logger
    def enable_disable_event_table_entry(self,aa,EE,SS):
        """
        Send a 'ENABLE/DISABLE EVENT TABLE ENTRY'
            MsgFmt(cmd='N EE SS', rsp=None)

        Data Out:
            EE - table number
            SS - action
                ==0 - disable
                <>0 - enable

        Data In:
            None
            
        This command is used to enable or to disable an event table entry
        specified by data field EE. A zero value in data field SS specifies
        that the entry is to be disabled. A non-zero value specifies that
        the entry is to be enabled.
        """
        return self.send_receive_2(aa,'ENABLE/DISABLE EVENT TABLE ENTRY',inspect.currentframe())

    @utl.logger
    def read_and_clear_event_latches(self,aa,EE):
        """
        Send a 'READ AND CLEAR EVENT LATCHES'
            MsgFmt(cmd='Q EE', rsp='DDDD')

        Data Out:
            EE - event entry number

        Data In:
            DDDD - bitmask of event latches

        This command is used to enable or to disable an event table entry spec-
        ified by data field EE. A zero value in data field SS specifies that the
        entry is to be disabled. A non-zero value specifies that the entry is to
        be enabled. 
        """
        return self.send_receive_2(aa,'READ AND CLEAR EVENT LATCHES',inspect.currentframe())

    @utl.logger
    def read_event_data_holding_buffer(self,aa,EE):
        """
        Send a 'READ EVENT DATA HOLDING BUFFER'
            MsgFmt(cmd='I EE', rsp='DDDDDDDD')

        Data Out:
            EE - event entry number

        Data In:
            DDDDDDDD - event data

        This command is used to read data which has been previously read and
        held by the execution of digital reaction command 08. The returned data
        will be a 32-bit number.

        Data DDDDDDDD will only be valid if the digital reaction command 08 has
        been executed. See Digital event/reaction Command M. Data field DDDDDDDD
        is 32 bits. Units are in counts.
        """
        return self.send_receive_2(aa,'READ EVENT DATA HOLDING BUFFER',inspect.currentframe())

    @utl.logger
    def read_event_entry_enable_disable_status(self,aa,EE):
        """
        Send a 'READ EVENT ENTRY ENABLE/DISABLE STATUS'
            MsgFmt(cmd='v EE', rsp='DDDD')

        Data Out:
            EE - event table entry group

        Data In:
            DDDD - tuple of event entry statuses
            
        This command returns 16 status bits or 256 status bits. Each status
        bit represents one event table entry. The status bit will be a ‘1’
        for event table entries which are enabled and a ‘0’ for entries which
        are disabled. If data field EE = FF Hex, then 256 status bits are
        returned, otherwise only 16 status bits are returned. The most signif-
        icant 4 bits of data field EE determine which group of 16 event table
        entry status bits will be returned. The least significant bits of data
        field EE are ignored (unless EE = FF Hex).
        """
        return self.send_receive_2(aa,'READ EVENT ENTRY ENABLE/DISABLE STATUS',inspect.currentframe())

    @utl.logger
    def read_event_latches(self,aa,EE):
        """
        Send a 'READ EVENT LATCHES'
            MsgFmt(cmd='P EE', rsp='DDDD')

        Data Out:
            EE - event table entry group

        Data In:
            DDDD - mask of event latches
           

        This command is used to read the event latches. If command data field
        EE = FF Hex, then all 256 latch bits (one for each latch) are returned,
        otherwise only 16 bits are returned. The most significant 4 bits of
        data field EE determine which group of 16 latch bits will be returned.
        The least significant bits of data field EE are ignored
        (unless EE = FF Hex).
        """
        return self.send_receive_2(aa,'READ EVENT LATCHES',inspect.currentframe())

    @utl.logger
    def set_event_on_timer_counter_gte(self,aa,EE,CC,NNNNNNNN):
        """
        Send a 'SET EVENT ON COUNTER/TIMER >='
            MsgFmt(cmd='L EE CC NNNNNNNN', rsp=None)

        Data Out:
            EE - event table entry group
            CC - channel number
            NNNNNNNN - counter compare value

        Data In:
            None

        This command places an event entry in the Event/Reaction Table at the
        entry number specified by data field EE. The event specification defines
        a counter/timer input channel CC and a 32-bit count value NNNNNNNN.
        The event occurs when the count on input channel CC equals or exceeds
        NNNNNNNN.

        The channel specified may be configured as either a quadrature or
        standard counter, pulse duration or ON/OFF time totalizer. Command ‘G’
        or command ‘a’ may be used. A counter channel should be enabled with
        command ‘H’ or command ‘b’ and the event table entry should be enabled
        using command ‘N’ before this command will be effective.
        
        When the count of a quadrature or standard counter is less than NNNNNNN,
        then becomes greater than or equal to NNNNNNN, the event has occurred
        and the reaction command will be executed. Quadrature counter values are
        interpreted as signed numbers (i.e., FFFFFFFE Hex = -2 decimal). Standard
        counter values are unsigned.
        """
        return self.send_receive_2(aa,'SET EVENT ON COUNTER/TIMER >=',inspect.currentframe())

    @utl.logger
    def set_event_on_timer_counter_lte(self,aa,EE,CC,NNNNNNNN):
        """
        Send a 'SET EVENT ON COUNTER/TIMER <='
            MsgFmt(cmd='} EE CC NNNNNNNN', rsp=None)            

        Data Out:
            EE - event table entry group
            CC - channel number
            NNNNNNNN - counter compare value

        Data In:
            None

        This command places an event entry in the Event/Reaction Table at the
        entry number specified by data field EE. The event specification defines
        a counter/timer input channel CC and a 32-bit count value NNNNNNNN.
        The event occurs when the count on input channel CC ecomes less than or
        equal to NNNNNNNN.

        The channel specified may be configured as either a quadrature or
        standard counter, pulse duration or ON/OFF time totalizer. Command ‘G’
        or command ‘a’ may be used. A counter channel should be enabled with
        command ‘H’ or command ‘b’ and the event table entry should be enabled
        using command ‘N’ before this command will be effective.
        
        When the count of a quadrature or standard counter is less than NNNNNNN,
        then becomes greater than or equal to NNNNNNN, the event has occurred
        and the reaction command will be executed. Quadrature counter values are
        interpreted as signed numbers (i.e., FFFFFFFE Hex = -2 decimal). Standard
        counter values are unsigned.
        """
        return self.send_receive_2(aa,'SET EVENT ON COUNTER/TIMER <=',inspect.currentframe())

    @utl.logger
    def clear_event_reaction_table(self,aa):
        """
        Send a 'CLEAR EVENT/REACTION TABLE'
            MsgFmt(cmd='_', rsp=None)

        Data Out:
            None

        Data In:
            None

        This command is used to clear the entire event/reaction Table. All event
        latches and interrupts are also cleared.

        The first 32 entries in the event/reaction table are restored from EEPROM
        upon power-up or when the Reset command is executed.
        """
        return self.send_receive_2(aa,'CLEAR EVENT/REACTION TABLE',inspect.currentframe())

    @utl.logger
    def clear_event_table_entry(self,aa,EE):
        """
        Send a 'CLEAR EVENT TABLE ENTRY'
            MsgFmt(cmd='\\ EE', rsp=None)

        Data Out:
            EE - event table entry number

        Data In:
            None

        This command is used to clear a single entry in the Event/Reaction Table.
        Data field EE specifies the entry to be cleared.

        The command data field EE specifies the event/reaction table entry to be
        cleared. If the specified entry is 00 to 1F Hex, then it will be restored
        from EEPROM upon power-up or any type of reset.
        """
        return self.send_receive_2(aa,'CLEAR EVENT TABLE ENTRY',inspect.currentframe())

    @utl.logger
    def clear_interrupt(self,aa):
        """
        Send a 'CLEAR INTERRUPT'
            MsgFmt(cmd='zB', rsp=None)

        Data Out:
            None

        Data In:
            None

        This command is used to clear the Event/Reaction interrupt output. Event
        latches are not affected.  This command will only clear the interrupt output
        from the I/O unit. It will not clear any event latch or return data.
        """
        return self.send_receive_2(aa,'CLEAR INTERRUPT',inspect.currentframe())

    @utl.logger
    def read_and_optionally_clear_event_latch(self,aa,EE,FF):
        """
        Send a 'READ AND OPTIONALLY CLEAR EVENT LATCH'
            MsgFmt(cmd='zA EE FF', rsp='DD')

        This command returns event latch data and the current event entry enable
        and interrupt enable status for the entry specified by data field EE.
        Data field FF indicates whether or not to clear the latch.

        Data field FF in the command specifies whether the event latch is to be
        cleared. If FF is not equal to 00, the event latch will be cleared after
        it is read. If FF is equal to 00, the event latch will not be cleared.
        The data byte returned by this command (DD) contains only 3 bits that
        are of any significance to the user.
        
        Bit 0:
            if 1, the event latch is set; the event has occurred.
            if 0, the event latch is cleared; the event has not occurred
                since the last time this latch was read and cleared.
        Bit 6:
            if 1, the event interrupt is enabled.
            if 0, the event interrupt is disabled.
        Bit 7:
            if 1, the event entry is enabled and will be scanned.
            if 0, the event entry is disabled and will not be scanned.
        """
        return self.send_receive_2(aa,'READ AND OPTIONALLY CLEAR EVENT LATCH',inspect.currentframe())

    @utl.logger
    def read_event_table_entry(self,aa,EE):
        """
        Send a 'READ EVENT TABLE ENTRY'
            MsgFmt(cmd='O EE', rsp='CB CC RC RCC ED RES NNNNNNNN TTTTTTTT')

        Data Out:
            CB - control byte
            CC = channel
            RC - reaction command
            RCC - reaction channel number
            ED - enable/disable flag
            RES - reserved
            NNNNNNNN - Compare Must ON/Must OFF mask or Counter Value
            TTTTTTTT - Reaction Must ON/Must OFF mask or Time Delay Value            

        Data In:
            None

        The Control Byte contains flag bits which are defined as follows:
            Bit 7: Event Scan Enable Flag. This bit is set if the event is
                enabled for scanning.
            Bit 6: Interrupt Enable Flag. This bit is set if the event interrupt
                is enabled.
            Bit 5: Last Entry Flag. This bit is set if this event is the last
                valid entry.
            Bit 4, 3: Event Type
                00 = Must On - Must Off Compare
                01 = Communications Link Watchdog Monitor
                10 = Counter Channel <= Setpoint Compare
                11 = Counter Channel >= Setpoint Compare
            Bit 2: Valid Entry Flag. If this bit is clear, this entry is considered
                a null entry and is not being scanned. All other data is therefore
                meaningless.
            Bit 1: Match Latch. This bit is set when an event condition is matched
                and cleared when that condition no longer exists.
            Bit 0: Event Latch. This bit is set by an event condition match and can
                only be cleared by the host.

        The compare channel is valid only for events which must monitor data of a
        certain channel. The reaction command number is the command which will be
        executed when this event occurs.
        
        The reaction channel number is only valid for reaction commands which
        require a channel or event number.
        
        The enable/disable flag is only valid for reaction commands which require
        an enable or disable flag. A zero byte represents a disable flag, and a
        nonzero byte an enable flag.
        
        The compare value field will contain the compare data for this event. If
        the event is monitoring a must ON/must OFF condition, the most signifi-
        cant word of this field will contain the must OFF mask and the least sig-
        nificant word will contain the must ON mask. If the event is monitoring a
        counter value, this field will contain the compare value. The most signif-
        icant byte is first and the least significant byte is last. This field is
        meaningless for events which monitor the communications watchdog time-out
        condition.
        
        The reaction data field is valid only for those reaction commands which
        require data. If the reaction command is a 01, “Set Output Module State
        -Group” command, the most significant 16-bit word of this field will con-
        tain the mask of modules to be deactivated, (must OFF). The least signif-
        icant 16-bit word will hold the mask of modules to be activated, (must ON).
        If the reaction command is a 02 (Start ON Pulse) or 03 (Start OFF Pulse)
        this field will contain the delay value in 100 microsecond units.
        
        This command returns the event/reaction Table Entry for the entry number
        specified by data field EE. Fourteen (14) bytes of data are returned for
        the binary protocol and 28 Hex bytes for the ASCII protocol.
        """
        return self.send_receive_2(aa,'READ EVENT TABLE ENTRY',inspect.currentframe())

    @utl.logger
    def set_event_interrupt_status(self,aa,EE,SS):
        """
        Send a 'SET EVENT INTERRUPT STATUS'
            MsgFmt(cmd='I EE SS', rsp=None)

        Data Out:
            EE - event entry table number
            SS -
                ==0 - disabled
                <>0 - enabled

        Data In:
            None

        This command is used to enable or to disable the interrupt output func-
        tion of an entry in the event/reaction Table Entry specified by data
        field EE.  A zero value in data field SS specifies that the interrupt
        is to be disabled. A non-zero value specifies that the interrupt is to
        be enabled.

        Before issuing this command, you must first issue a K, L, y or }
        command to place an entry in the event/reaction Table. The Global Event
        Interrupt Enable Bit (bit 7) in the Option Control Byte must be set to
        a ‘1’ before interrupts will be sent to the host computer. An interrupt
        is generated whenever there is an event match for the event/reaction
        Table Entry whose interrupt status is a ‘1’. Interrupts are sent to the
        host computer by placing a logic zero on the INT line. An interrupt will
        be generated in the host computer if the communications line is setup to
        whenever the INT line goes low (logic zero).
        """
        return self.send_receive_2(aa,'SET EVENT INTERRUPT STATUS',inspect.currentframe())

    @utl.logger
    def set_event_on_comm_link_watchdog_timeout(self,aa,EE):
        """
        Send a 'SET EVENT ON COMM LINK WATCHDOG TIMEOUT'
            MsgFmt(cmd='y EE', rsp=None)

        Data Out:
            EE - event table entry number

        Data In:
            None

        This command places an event entry in the event/reaction Table. The entry
        number is specified by data field EE. The event occurs when the watchdog
        timer times out.

        In order for this event to be effective, you must (1) enable the watchdog
        timer (see command D) and (2) enable the event/reaction Table Entry (see
        command N). The channels specified by the reaction command must be config-
        ured properly.
        """
        return self.send_receive_2(aa,'SET EVENT ON COMM LINK WATCHDOG TIMEOUT',inspect.currentframe())

    @utl.logger
    def set_event_on_momo_match(self,aa,EE,MMMM,NNNN):
        """
        Send a 'SET EVENT ON MOMO MATCH'
            MsgFmt(cmd='K EE MMMM NNNN', rsp=None)

        Data Out:
            EE - event table entry number

        Data In:
            None

        This command places an event entry in the event/reaction Table at the
        entry number specified by the data field EE. The event specification
        field MMMM specifies the modules that must be ON and field NNNN speci-
        fies the modules that must be OFF for an event to occur. Modules not
        specified are ‘don’t care’.

        The channels specified in the MOMO field can be inputs or outputs. The
        event table entry EE must be enabled using command ‘N’ before the re-
        action command will be executed. An error will occur if the channels
        specified by the reaction command are not configured properly. When the
        channels specified by field MMMM are ON and the channels specified by
        field NNNN are OFF, the event has occurred. Note however, that the re-
        action will be executed only once (if the entry is enabled) even though
        the MOMO match persists. The event must change from a non-matching to a
        matching condition before the reaction will be executed again. The MOMO
        match must change to a non-matching condition first, then a matching
        condition must occur again before this can happen.
        """
        return self.send_receive_2(aa,'SET EVENT ON MOMO MATCH',inspect.currentframe())

    """
    Shared 'SET EVENT REACTION COMMAND' modifiers
    """
    shared_reaction_commands = {
        'NULL REACTION (DO NOTHING)':MsgFmt('00',None),
        'ENABLE/DISABLE EVENT TABLE ENTRY':MsgFmt('06 EE SS',None),
        'ENABLE/DISABLE EVENT ENTRY GROUP':MsgFmt('07 GG MMMM NNNN',None)
        }

    """
    These commands place a reaction command entry in the event/reaction
    Table. The event table entry contains two parts (1) an event speci-
    fication and (2) a reaction specification. The event specification
    is entered into the event/reaction Table by commands K, L, y or }.
    The reaction command can be selected from a list of nine commands.
    See “Remarks” for a list of reaction commands that can be used.

    In order for an event/reaction to be executed, you must (1) enter
    an event specification into the event/reaction Table using commands
    K, L or y, (2) enter a reaction command into the event/reaction Table
    using command M and (3) enable the table entry using command N. In
    addition to the above, you must remember to configure and/or enable
    the channels specified by the reaction command. Refer to Chapter 3
    for detailed description of event/reaction processing.
    """
    
    @utl.logger
    def on_event_do_nothing(self,aa,EE,RC):
        """
        Send a 'ON EVENT NULL REACTION (DO NOTHING)'
            MsgFmt(cmd='M EE RC', rsp=None)

        Data Out:
            EE - event table entry number
            RC - reaction command
            
        Data In:
            None

        No command is executed. The event latch is set upon event occurrence.
        """
        RC=int(self.shared_reaction_commands['NULL REACTION (DO NOTHING)'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT NULL REACTION (DO NOTHING)',inspect.currentframe())

    @utl.logger
    def on_event_enable_disable_event_table_entry(self,aa,EE,RC,ee,SS):
        """
        Send a 'ON EVENT ENABLE/DISABLE EVENT TABLE ENTRY'
            MsgFmt(cmd='M EE RC ee SS', rsp=None)

        Data Out:
            EE = Event Entry Number 
            RC - reaction command
            ee - Event Table Entry Number
            SS = Enable Setting, 0 = Disable, 1 = Enable.

        Data In:
            None

        When an event fires on EE, react by enabling/disabling an event on ee.
        
        This command is used to enable or to disable an event table entry speci-
        fied by data field ee. A zero value in data field SS specifies that the
        entry is to be disabled. A non-zero value specifies that the entry is to
        be enabled.
        """
        RC=int(self.shared_reaction_commands['ENABLE/DISABLE EVENT TABLE ENTRY'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT ENABLE/DISABLE EVENT TABLE ENTRY',inspect.currentframe())

    @utl.logger
    def on_event_enable_disable_event_entry_group(self,aa,EE,RC,GG,MMMM,NNNN):
        """
        Send a 'ON EVENT ENABLE/DISABLE EVENT ENTRY GROUP'
            MsgFmt(cmd='M EE RC GG MMMM NNNN', rsp=None)

        Data Out:
            EE - event table entry number
            RC - reaction command
            GG = Event Entry Group (A group of 16 events)
            MMMM = Bitmask for Entries to be Enabled.
            NNNN = Bitmask for Entries to be Disabled.
            
        Data In:
            None

        Setting the same bit in MMMM and NNNN will result in an error being
        returned and the command is not executed. If GG (Event Entry Group
        Number) is set to FF Hex, then ALL event table entries are specified
        and bitmask MMMM is checked for enable/disable status. If MMMM is equal
        to zero (0), then all entries are disabled. If MMMM is not equal to zero
        (0), then all entries are enabled. In either case, bitmask NNNN must be
        sent, but is ignored. See command { chapter 11. Applies to firmware
        revision 1.02 or later.
        """
        RC=int(self.shared_reaction_commands['ENABLE/DISABLE EVENT ENTRY GROUP'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT ENABLE/DISABLE EVENT ENTRY GROUP',inspect.currentframe())

    """
    Digital 'SET EVENT REACTION COMMAND' modifiers
    """
    digital_reaction_commands = {
        # REACTION COMMANDS (To be used with digital command M
        'SET OUTPUT MODULE STATE-GROUP':MsgFmt('01 MMMM NNNN',None),
        'START ON PULSE':MsgFmt('02 CC TTTTTTTT',None),
        'START OFF PULSE':MsgFmt('03 CC TTTTTTTT',None),
        'ENABLE/DISABLE COUNTER':MsgFmt('04 CC SS',None),
        'CLEAR COUNTER':MsgFmt('05 CC',None),        
        'READ AND HOLD COUNTER VALUE':MsgFmt('08 CC',None),       
        }

    @utl.logger
    def on_event_set_output_module_state_group(self,aa,EE,RC,MMMM,NNNN):
        """
        Send a 'ON EVENT SET OUTPUT MODULE STATE-GROUP'
            MsgFmt(cmd='M EE RC MMMM NNNN', rsp=None)

        Data Out:
            EE - table entry number
            RC - 1
            MMMM - outputs to enable
            NNNN - outpute to disable

        Data In:
            None

        When an event happens on EE, internally call 'SET OUTPUT MODULE STATE-GROUP'

        This command is used to set the state of any or all output channels.
        The specified channels must have previously been configured as standard
        outputs by using the ‘G’ or ‘a’ command. Returns an error if a channel
        is specified which is not a standard output. An error is also returned
        if the command specifies a module to be both turned ON and OFF. Any time
        delay or TPO function on a specified channel will be canceled.            
        """
        RC=int(self.digital_reaction_commands['SET OUTPUT MODULE STATE-GROUP'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT SET OUTPUT MODULE STATE-GROUP',inspect.currentframe())

    @utl.logger
    def on_event_start_on_pulse(self,aa,EE,RC,CC,TTTTTTTT):
        """
        Send a 'ON EVENT START ON PULSE'
            MsgFmt(cmd='M EE RC CC TTTTTTTT', rsp=None)

        Data Out:
            EE - event table entry number
            RC - 2
            CC - channel number
            TTTTTTTT - on time

        Data In:
            None

        On the occurance of the specified event on EE, send a 'START ON PULSE'
        
        This command is used to generate a retriggerable ON pulse at the
        standard output channel specified by data field CC. The delay time
        is specified by the data field TTTTTTTT. Time setting is in units
        of 100 micro seconds. Minimum pulse time is 0.5 milliseconds. An
        error is returned if the time is set for less than 0.5 milliseconds
        (TTTTTTTT < 5).
        """
        RC=int(self.digital_reaction_commands['START ON PULSE'].cmd.split(' ')[0],16)
        print('RC={:02X}'.format(RC))
        return self.send_receive_2(aa,'ON EVENT START ON PULSE',inspect.currentframe())

    @utl.logger
    def on_event_start_off_pulse(self,aa,EE,RC,CC,TTTTTTTT):
        """
        Send a 'SET EVENT START OFF PULSE'
            MsgFmt(cmd='M EE RC CC TTTTTTTT', rsp=None)

        Data Out:
            EE - event table entry number
            RC - 3
            CC - channel number
            TTTTTTTT - off time

        Data In:
            None

        On the occurance of the specified event on EE, send a 'START OFF PULSE'
        
        This command is used to generate a retriggerable OFF pulse at the
        standard output channel specified by data field CC. The delay time
        is specified by the data field TTTTTTTT. Time setting is in units
        of 100 micro seconds. Minimum pulse time is 0.5 milliseconds. An
        error is returned if the time is set for less than 0.5 milliseconds
        (TTTTTTTT < 5).
        """
        RC=int(self.digital_reaction_commands['START OFF PULSE'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'SET EVENT START OFF PULSE',inspect.currentframe())

    @utl.logger
    def on_event_enable_disable_counter(self,aa,EE,RC,CC,SS):
        """
        Send a 'ON EVENT ENABLE/DISABLE COUNTER'
        

        Data Out:
            EE - event table entry number
            RC - 4
            CC = Channel Number of Counter to enable or disable
            SS = Enable Setting, 0 = Disable, 1 = Enable.

        Data In:
            None

        On event specified in EE, call 'ENABLE/DISABLE COUNTER'
        
        This command instructs the addressed I/O unit to enable or disable
        a counter channel specified by data field CC. Enable or disable is
        specified by data field SS. Before an input module can act as a
        counter, it must be configured as a counter input (default config-
        uration) and the counter channel must be enabled.

        Upon power up all counter channels are disabled.  Disabling a counter
        makes it act as a standard input. Maximum counter frequency is 25 KHz
        The channel number is in range(16).
        """
        RC=int(self.digital_reaction_commands['ENABLE/DISABLE COUNTER'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT ENABLE/DISABLE COUNTER',inspect.currentframe())

    @utl.logger
    def on_event_clear_counter(self,aa,EE,RC,CC):
        """
        Send a
            MsgFmt(cmd='M EE RC CC SS', rsp=None)

        Data Out:
            EE - event table entry number
            RC - 5
            CC - Channel Number of Counter to be Read

        Data In:
            None

        On the event specified in table EE, clear the counter specified by CC.

        The channel specified must be configured as a counter before issuing this
        command.
        """
        RC=int(self.commands['CLEAR COUNTER'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT CLEAR COUNTER',inspect.currentframe())

    @utl.logger
    def on_event_read_and_hold_counter_value(self,aa,EE,RC,CC):
        """
        Send a 'ON EVENT READ AND HOLD COUNTER VALUE'
            MsgFmt(cmd='M EE RC CC', rsp=None)

        Data Out:
            EE - event table entry number
            RC - 6
            CC - Channel Number of Counter to be Read

        Data In:
            None

        On the event specified in table EE, clear the counter specified by CC.
        """
        RC=int(self.digital_reaction_commands['READ AND HOLD COUNTER VALUE'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT READ AND HOLD COUNTER VALUE',inspect.currentframe())

    # Chapter 12: Analog Setup/System Commands
    # Command Name, Command Format, Version
    @utl.logger
    def set_comm_link_watchdog_and_delay(self,aa,TTTT):
        """
        Send a 'SET COMM LINK WATCHDOG AND DELAY'
            MsgFmt(cmd='D TTTT', rsp=None)

        Data Out:
            TTTT - watchdog delay time

        Data In:
            None

        This command sets the communications line watchdog timeout period for
        the addressed I/O unit. When enabled, if a command (or an “>” character
        for ASCII protocol) is not received after the time delay specified by
        data field TTTT, the watchdog timer will time out. Upon time out, the
        analog output modules which have been instructed to output a specified
        value (by command H) will do so. After a watchdog timer has timed out,
        the next command sent to the I/O unit must be a Power Up Clear Command
        (Command A) otherwise an error will be returned. A delay of zero (0)
        disables the watchdog function.
        """
        return self.send_receive_2(aa,'SET COMM LINK WATCHDOG AND DELAY',inspect.currentframe())

    @utl.logger
    def cancel_comm_link_watchdog_and_delay(self,aa):
        """
        Almost impossible to clear a watchdog from the terminal
        so this should do it
        """
        rsp = self.power_up_clear(aa)
        if rsp.ack == 'A':
            return set_comm_link_watchdog_and_delay(aa,0)
        return rsp
    
    @utl.logger
    def set_comm_link_watchdog_timeout_data(self,aa,MMMM,DDDDDDDD):
        """
        Send a 'SET COMM LINK WATCHDOG TIMEOUT DATA'
            MsgFmt(cmd='H MMMM DDDDDDDD', rsp=None)

        Data Out:
            MMMM - channels to be effected
            DDDDDDDD - value to be written

        Data In:
            None

        This command is used to set the desired output state for the specified
        analog modules upon a communications link watchdog timeout condition.
        A one (1) in data field MMMM indicates that the specified output module
        will have its output changed to the value indicated by the corresponding
        field DDDDDDDD. A zero (0) in data field MMMM indicates that the value
        for the specified output module will not be changed. For each bit in
        MMMM that is set to a one (1), there must be a corresponding 8-byte
        (ASCII) or 4-byte (Binary) data field DDDDDDDD. The value in field
        DDDDDDDD is in Engineering units. It specifies the value to which the
        output module will be set.
        """
        return self.send_receive_2(aa,'SET COMM LINK WATCHDOG TIMEOUT DATA',inspect.currentframe())

    # Chapter 13: Analog I/O Configuration Commands
    # Command Name, Command Format, Version
    @utl.logger
    def calculate_and_set_adc_module_offset(self,aa,CC):
        """
        Send a 'CALCULATE AND SET ADC MODULE OFFSET'
            MsgFmt(cmd='d CC', rsp='OOOO')

        Data Out:
            CC - channel number

        Data In:
            OOOO - Offset in counts, signed 16-bit integer

        This command instructs the addressed I/O unit to use the current input
        value as the zero point input value. The current reading is multiplied
        by minus one (-1) and is used as an offset. The offset value is returned
        to the host computer. This command is intended to be used for calibra-
        tion purposes.
        """
        rsp = self.send_receive_2(aa,'CALCULATE AND SET ADC MODULE OFFSET',inspect.currentframe())
        return self.parse_mistic_response_data(('OOOO',),rsp)

    @utl.logger
    def calculate_and_set_adc_module_gain(self,aa,CC):
        """
        Send a 'CALCULATE AND SET ADC MODULE GAIN'
            MsgFmt(cmd='e CC', rsp='GGGG')

        Data Out:
            CC - channel number

        Data In:
            GGGG - Gain coefficient, unsigned 16-bit integer scaled by a factor
            of 4096.

        This command instructs the addressed I/O unit to use the current input
        value as the full scale input value. A gain coefficient is calculated
        and used for all subsequent readings. The coefficient is returned to the
        host computer. This command is intended to be used for calibration
        purposes
        """
        rsp = self.send_receive_2(aa,'CALCULATE AND SET ADC MODULE GAIN',inspect.currentframe())
        return self.parse_mistic_response_data(('GGGG',),rsp)

    @utl.logger
    def set_adc_module_offset(self,aa,CC,OOOO):
        """
        Send a 'SET ADC MODULE OFFSET'
            MsgFmt(cmd='b CC OOOO', rsp=None)

        Data Out:
            CC- channel number
            OOOO - offset counts, 0-4095

        Data In:
            None

        This command sets an offset which is added to the raw count data before
        any engineering unit scaling is done. Units are counts. Data field CC
        specifies the analog input module.
        """
        OOOO = self.int16_to_mistic_data(OOOO)
        return self.send_receive_2(aa,'SET ADC MODULE OFFSET',inspect.currentframe())

    @utl.logger
    def set_adc_module_gain(self,aa,CC,GGGG):
        """
        Send a 'SET ADC MODULE GAIN'
            MsgFmt(cmd='c CC GGGG', rsp=None)

        Data Out:
            CC - channel number
            GGGG - gain factor

        Data In:
            None

        This command is used to set the gain coefficient for the analog input
        channel specified by data field CC. The gain coefficient is specified
        by data field GGGG. The gain coefficient is an unsigned 16-bit integer
        scaled by a factor of 4096.
        """
        return self.send_receive_2(aa,'SET ADC MODULE GAIN',inspect.currentframe())

    @utl.logger
    def set_averaging_sample_weight(self,aa,CC,DDDD):
        """
        Send a 'SET AVERAGING SAMPLE WEIGHT (DIG. FILTERING)'
            MsgFmt(cmd='h CC DDDD', rsp=None)

        Data Out:
            CC - channel number
            DDDD - number of samples in average

        Data In:
            None

        This command initiates digital filtering on the channel specified by
        data field CC. Averaging is permitted for input channels only. The
        averaging algorithm is:
        
        New Average = ( (Current Reading - Old Average) / DDDD) + Old Average
            
        Data field DDDD specifies the sample weight. A value of DDDD = 0000
        specifies that averaging is to be discontinued.
        """
        return self.send_receive_2(aa,'SET AVERAGING SAMPLE WEIGHT (DIG. FILTERING)',inspect.currentframe())

    @utl.logger
    def set_totalization_sample_rate(self,aa,CC,DDDD):
        """
        Send a 'SET TOTALIZATION SAMPLE RATE'
            MsgFmt(cmd='g CC DDDD', rsp=None)

        Data Out:
            CC - channel number
            DDDD - number of samples in average

        Data In:
            None

        This command initiates totalization on the specified channel. Totali-
        zation is allowed for both input and output analog channels. Totali-
        zation is also allowed for both filtered and unfiltered readings.
        Data field CC specifies the channel and data field DDDD specifies the
        sample rate in 100 mS units. DDDD is a 16-bit unsigned integer. Bit 15
        is used as a flag to indicate whether totalization will be for filtered
        or unfiltered readings. If bit 15 is zero, the unfiltered readings will
        be totalized. If bit 15 is a one, the filtered readings will be totalized.
        A sample rate of DDDD = 0000 will discontinue totalization for the speci-
        fied channel. See remarks for further information on this command.
        """
        return self.send_receive_2(aa,'SET TOTALIZATION SAMPLE RATE',inspect.currentframe())

    @utl.logger
    def set_engineering_unit_scaling_parameters(self,aa,CC,HHHHHHHH,LLLLLLLL):
        """
        Send a 'SET ENGINEERING UNIT SCALING PARAMETERS'
            MsgFmt(cmd='f CC HHHHHHHH LLLLLLLL', rsp=None)

        Data Out:
            CC - channel number
            HHHHHHHH - Engineering units corresponding to full scale (4095 counts)
                in units of 1/65,536 of an Engineering unit.
            LLLLLLLL - Engineering units corresponding to zero scale (0000 counts)
                in units of 1/65,536 of an Engineering unit.

        Data In:
            None

        This command is used to set the Engineering units Scaling Parameters
        specified by data fields HHHHHHHH and LLLLLLLL for the channel speci-
        fied by data field CC. Data field HHHHHHHH specifies the Engineering
        units (in increments of 1/65,536 of an Engineering unit) corresponding
        to full scale (4095 counts). Data field LLLLLLLL specifies the Engineer-
        ing units (in increments of 1/65,536 of an Engineering unit) correspond-
        ing to zero scale (0000 counts).
        """
        HHHHHHHH = self.engineering_units_to_mistic_data(HHHHHHHH)
        LLLLLLLL = self.engineering_units_to_mistic_data(LLLLLLLL)           
        return self.send_receive_2(aa,'SET ENGINEERING UNIT SCALING PARAMETERS',inspect.currentframe())

    @utl.logger
    def set_tpo_resolution(self,aa,CC,SS):
        """
        Send a 'SET TPO RESOLUTION'
            MsgFmt(cmd='] CC SS', rsp=None)

        Data Out:
            CC -
            SS -

        Data In:
            None

        This command is used to set the resolution of a time proportional output
        module (G4DA9). The channel number specified by CC is in the range 00 to
        0F. The resolution setting specified by data field SS may be any number
        between 00 and FF. The resolution setting is in 500 microsecond units
        and affects the total TPO period according to the equation below:
        
        TPO Period = Resolution Setting * 0.5 milliseconds * 4,096

        The default resolution of the G4DA9 output module is 1 millisecond. The
        TPO period will be 4.096 seconds. If the Store System Configuration com-
        mand is executed, all TPO resolution settings will be saved and automat-
        ically restored upon power-up.
        """
        return self.send_receive_2(aa,'SET TPO RESOLUTION',inspect.currentframe())

    # Chapter 14: Analog Read/Write/Output Commands
    # Command Name, Command Format, Version
    @utl.logger
    def ramp_dac_output_to_endpoint(self,aa,CC,EEEEEEEE,SSSSSSSS):
        """
        Send a 'RAMP DAC OUTPUT TO ENDPOINT'
            MsgFmt(cmd='Z CC EEEEEEEE SSSSSSSS', rsp=None)

        Data Out:
            CC - channel number
            EEEEEEEE - endpoint value in engineering units
            SSSSSSSS - slope in units per second

        Data In:
            None

        This command is used to ramp an analog output DAC module from its
        present setting to a specified ramp endpoint at a specified ramp
        rate. Data field EEEEEEEE specifies the ramp endpoint in
        Engineering units. Data field SSSSSSSS specifies the ramp slope in
        Engineering units per second. Data field CC specifies the output channel.
        """
        EEEEEEEE = self.int32_to_analog(EEEEEEEE)
        return self.send_receive_2(aa,'RAMP DAC OUTPUT TO ENDPOINT',inspect.currentframe())
    
    """
    analog types for variable read commands
    """
    analog_data_types = {
        0x00:'Current Counts',
        0x01:'Average Counts',
        0x02:'Peak Counts',
        0x03:'Lowest Counts',
        0x04:'Totalized Counts',
        0x10:'Current Engineering units',
        0x11:'Average Engineering units',
        0x12:'Peak Engineering units',
        0x13:'Lowest Engineering units',
        0x14:'Totalized Engineering units',
        0x20:'Square Root of Current Counts',
        0x21:'Square Root of Average Counts',
        0x22:'Square Root of Peak Counts',
        0x23:'Square Root of Lowest Counts',
        0x24:'Square Root of Totalized Count',
        0x30:'Square Root of Current Engineering units',
        0x31:'Square Root of Average Engineering units',
        0x32:'Square Root of Peak Engineering units',
        0x33:'Square Root of Lowest Engineering units',
        0x34:'Square Root of Totalized Engineering units'
        }

    @utl.logger
    def read_and_clear_16_bit_io_module_data(self,aa,CC,TT):
        """
        Send a 'READ AND CLEAR I/O MODULE 16 BIT DATA'
            MsgFmt(cmd='s CC TT', rsp='DDDD')

        Data Out:
            CC - channel number
            TT - data type

        Data In:
            DDDD - 16 bit channel data

        TT's in (
            0x00, 0x01,0x02, 0x03
            ) are returned as 16 bit DDDD

        TT 0x00 is current and therefore not cleared
        """
        rsp = self.send_receive_2(aa,'READ AND CLEAR I/O MODULE 16 BIT DATA',inspect.currentframe())
        return self.parse_mistic_response_data(('DDDD',),rsp)

    @utl.logger
    def read_and_clear_io_module_current_counts(self,aa,CC):
        return self.read_and_clear_16_bit_io_module_data(aa,CC,TT=0x00)
    
    @utl.logger
    def read_and_clear_io_module_average_counts(self,aa,CC):
        return self.read_and_clear_16_bit_io_module_data(aa,CC,TT=0x01)

    @utl.logger
    def read_and_clear_io_module_peak_counts(self,aa,CC):
        return self.read_and_clear_16_bit_io_module_data(aa,CC,TT=0x02)

    @utl.logger
    def read_and_clear_io_module_lowest_counts(self,aa,CC):
        return self.read_and_clear_16_bit_io_module_data(aa,CC,TT=0x03)

    @utl.logger
    def read_and_clear_io_module_total_counts(self,aa,CC):
        return self.read_and_clear_16_bit_io_module_data(aa,CC,TT=0x04)

    @utl.logger
    def read_and_clear_16_bit_io_module_group_data(self,aa,MMMM,TT):
        """
        Send a 'READ AND CLEAR I/O MODULE 16 BIT DATA-GROUP'
            MsgFmt(cmd='S MMMM TT', rsp='DDDD')

        Data Out:
            MMMM - channel mask
            TT - data type

        Data In:
            DDDD - 16 bit channel data

        TT's in (
            0x00, 0x01,0x02, 0x03
            ) are returned as 16 bit DDDD

        TT 0x00 is current and therefore not cleared
        """
        rsp = self.send_receive_2(aa,'READ AND CLEAR I/O MODULE 16 BIT DATA-GROUP',inspect.currentframe())
        return self.parse_mistic_response_data(('DDDD',),rsp)

    @utl.logger
    def read_and_clear_io_module_group_current_counts(self,aa,MMMM):
        return self.read_and_clear_16_bit_io_module_group_data(aa,MMMM,TT=0x00)
    
    @utl.logger
    def read_and_clear_io_module_group_average_counts(self,MMMM):
        return self.read_and_clear_16_bit_io_module_group_data(aa,MMMM,TT=0x01)

    @utl.logger
    def read_and_clear_io_module_group_peak_counts(self,aa,MMMM):
        return self.read_and_clear_16_bit_io_module_group_data(aa,MMMM,TT=0x02)

    @utl.logger
    def read_and_clear_io_module_group_lowest_counts(self,aa,MMMM):
        return self.read_and_clear_16_bit_io_module_group_data(aa,MMMM,TT=0x03)

    @utl.logger
    def read_and_clear_io_module_group_total_counts(self,aa,MMMM):
        return self.read_and_clear_16_bit_io_module_group_data(aa,MMMM,TT=0x04)

    @utl.logger
    def read_and_clear_32_bit_io_module_data(self,aa,CC,TT):
        """
        Send a 'READ AND CLEAR I/O MODULE 32 BIT DATA'
            MsgFmt(cmd='s CC TT', rsp='DDDDDDDD')

        Data Out:
            CC - channel number
            TT - data type

        Data In:
            DDDD - 32 channel data

        TT's in (
            0x10,0x11,0x12,0x13,0x14,
            0x20,0x21,0x22,0x23,0x24,
            0x30,0x31,0x32,0x33,0x34
            ) are returned as 32 bit DDDDDDDD

        TT's in (
            0x00,0x10,0x20,0x30
            ) are current and therefore not cleared
        """
        rsp = self.send_receive_2(aa,'READ AND CLEAR I/O MODULE 32 BIT DATA',inspect.currentframe())
        return self.parse_mistic_response_data(('DDDDDDDD',),rsp)


    @utl.logger
    def read_and_clear_io_module_current_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x10)
    
    @utl.logger
    def read_and_clear_io_module_average_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x11)

    @utl.logger
    def read_and_clear_io_module_peak_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x12)

    @utl.logger
    def read_and_clear_io_module_lowest_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x13)

    @utl.logger
    def read_and_clear_io_module_total_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x14)

    @utl.logger
    def read_and_clear_io_module_sqrt_current_counts(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x20)
    
    @utl.logger
    def read_and_clear_io_module_sqrt_average_counts(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x21)

    @utl.logger
    def read_and_clear_io_module_sqrt_peak_counts(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x22)

    @utl.logger
    def read_and_clear_io_module_sqrt_lowest_counts(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x23)

    @utl.logger
    def read_and_clear_io_module_sqrt_total_counts(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x24)

    @utl.logger
    def read_and_clear_io_module_sqrt_current_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x30)
    
    @utl.logger
    def read_and_clear_io_module_sqrt_average_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x31)

    @utl.logger
    def read_and_clear_io_module_sqrt_peak_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x32)

    @utl.logger
    def read_and_clear_io_module_sqrt_lowest_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x33)

    @utl.logger
    def read_and_clear_io_module_sqrt_total_eng_units(self,aa,CC):
        return self.read_and_clear_32_bit_io_module_data(aa,CC,TT=0x34)    

    @utl.logger
    def read_and_clear_32_bit_io_module_group_data(self,aa,MMMM,TT):
        """
        Send a 'READ AND CLEAR I/O MODULE 32 BIT DATA-GROUP'
            MsgFmt(cmd='S MMMM TT', rsp='DDDDDDDD')

        Data Out:
            MMMM - channel mask
            TT - data type

        Data In:
            DDDDDDDD - 32 bit channel data

        TT's in (
            0x10,0x11,0x12,0x13,0x14,
            0x20,0x21,0x22,0x23,0x24,
            0x30,0x31,0x32,0x33,0x34
            ) are returned as 32 bit DDDDDDDD

        TT's in (
            0x00,0x10,0x20,0x30
            ) are current and therefore not cleared
        """
        rsp = self.send_receive_2(aa,'READ AND CLEAR I/O MODULE 32 BIT DATA-GROUP',inspect.currentframe())
        return self.parse_mistic_response_data(('DDDDDDDD',),rsp)

    @utl.logger
    def read_and_clear_io_module_group_current_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x10)
    
    @utl.logger
    def read_and_clear_io_module_group_average_eng_units(self,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x11)

    @utl.logger
    def read_and_clear_io_module_group_peak_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x12)

    @utl.logger
    def read_and_clear_io_module_group_lowest_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x13)

    @utl.logger
    def read_and_clear_io_module_group_total_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x14)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_current_counts(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x20)
    
    @utl.logger
    def read_and_clear_io_module_group_sqrt_average_counts(self,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x21)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_peak_counts(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x22)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_lowest_counts(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x23)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_total_counts(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x24)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_current_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x30)
    
    @utl.logger
    def read_and_clear_io_module_group_sqrt_average_eng_units(self,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x31)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_peak_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x32)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_lowest_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x33)

    @utl.logger
    def read_and_clear_io_module_group_sqrt_total_eng_units(self,aa,MMMM):
        return self.read_and_clear_32_bit_io_module_group_data(aa,MMMM,TT=0x34)

    @utl.logger
    def read_16_bit_io_module_magnitude(self,aa,CC,TT):
        """
        Send a 'READ I/O MODULE 16 BIT MAGNITUDE'
            MsgFmt(cmd='r CC TT', rsp='DDDD')

        Data Out:
            CC - channel number
            TT - data type

        Data In:
            DDDD - 16 bit channel data

        """
        rsp = self.send_receive_2(aa,'READ I/O MODULE 16 BIT MAGNITUDE',inspect.currentframe())
        return self.parse_mistic_response_data(('DDDD',),rsp)

    @utl.logger
    def read_io_module_magnitude_current_counts(self,aa,CC):
        return self.read_16_bit_io_module_magnitude(aa,CC,TT=0x00)
    
    @utl.logger
    def read_io_module_magnitude_average_counts(self,aa,CC):
        return self.read_16_bit_io_module_magnitude(aa,CC,TT=0x01)

    @utl.logger
    def read_io_module_magnitude_peak_counts(self,aa,CC):
        return self.read_16_bit_io_module_magnitude(aa,CC,TT=0x02)

    @utl.logger
    def read_io_module_magnitude_lowest_counts(self,aa,CC):
        return self.read_16_bit_io_module_magnitude(aa,CC,TT=0x03)

    @utl.logger
    def read_io_module_magnitude_total_counts(self,aa,CC):
        return self.read_16_bit_io_module_magnitude(aa,CC,TT=0x04)

    def read_32_bit_io_module_magnitude(self,aa,CC,TT):
        """
        Send a 'READ I/O MODULE 32 BIT MAGNITUDE'
            MsgFmt(cmd='r CC TT', rsp='DDDDDDDD')

        Data Out:
            CC - channel number
            TT - data type

        Data In:
            DDDDDDDD - 32 bit channel data

        """
        rsp = self.send_receive_2(aa,'READ I/O MODULE 32 BIT MAGNITUDE',inspect.currentframe())
        return self.parse_mistic_response_data(('DDDDDDDD',),rsp)
    
    @utl.logger
    def read_io_module_magnitude_current_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x10)
    
    @utl.logger
    def read_io_module_magnitude_average_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x11)
    
    @utl.logger
    def read_io_module_magnitude_peak_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x12)

    @utl.logger
    def read_io_module_magnitude_lowest_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x13)

    @utl.logger
    def read_io_module_magnitude_total_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x14)

    @utl.logger
    def read_io_module_magnitude_sqrt_current_counts(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x20)
    
    @utl.logger
    def read_io_module_magnitude_sqrt_average_counts(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x21)

    @utl.logger
    def read_io_module_magnitude_sqrt_peak_counts(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x22)

    @utl.logger
    def read_io_module_magnitude_sqrt_lowest_counts(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x23)

    @utl.logger
    def read_io_module_magnitude_sqrt_total_counts(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x24)

    @utl.logger
    def read_io_module_magnitude_sqrt_current_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x30)
    
    @utl.logger
    def read_io_module_magnitude_sqrt_average_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x31)

    @utl.logger
    def read_io_module_magnitude_sqrt_peak_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x32)

    @utl.logger
    def read_io_module_magnitude_sqrt_lowest_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x33)

    @utl.logger
    def read_io_module_magnitude_sqrt_total_eng_units(self,aa,CC):
        return self.read_32_bit_io_module_magnitude(aa,CC,TT=0x34)    

    @utl.logger
    def read_16_bit_io_module_magnitude_group(self,aa,MMMM,TT):
        """
        Send a 'READ I/O MODULE 16 BIT MAGNITUDE-GROUP'
            MsgFmt(cmd='R MMMM TT', rsp='DDDD')

        Data Out:
            MMMM - channel mask
            TT - data type

        Data In:
            DDDD - 16 bit channel data

        """
        rsp = self.send_receive_2(aa,'READ I/O MODULE 16 BIT MAGNITUDE-GROUP',inspect.currentframe())
        return self.process_mistic_response_data(('DDDD',),rsp)

    @utl.logger
    def read_io_module_magnitude_group_current_counts(self,aa,MMMM):
        return self.read_16_bit_io_module_magnitude_group(aa,MMMM,TT=0x00)
    
    @utl.logger
    def read_io_module_magnitude_group_average_counts(self,MMMM):
        return self.read_16_bit_io_module_magnitude_group(aa,MMMM,TT=0x01)

    @utl.logger
    def read_io_module_magnitude_group_peak_counts(self,aa,MMMM):
        return self.read_16_bit_io_module_magnitude_group(aa,MMMM,TT=0x02)

    @utl.logger
    def read_io_module_magnitude_group_lowest_counts(self,aa,MMMM):
        return self.read_16_bit_io_module_magnitude_group(aa,MMMM,TT=0x03)

    @utl.logger
    def read_io_module_magnitude_group_total_counts(self,aa,MMMM):
        return self.read_16_bit_io_module_magnitude_group(aa,MMMM,TT=0x04)

    @utl.logger
    def read_32_bit_io_module_magnitude_group(self,aa,MMMM,TT):
        """
        Send a 'READ I/O MODULE 32 BIT MAGNITUDE-GROUP'
            MsgFmt(cmd='R MMMM TT', rsp='DDDDDDDD')

        Data Out:
            MMMM - channel mask
            TT - data type

        Data In:
            DDDDDDDD - 32
            bit channel data

        """
        rsp = self.send_receive_2(aa,'READ I/O MODULE 32 BIT MAGNITUDE-GROUP',inspect.currentframe())
        return self.process_mistic_response_data(('DDDDDDDD',),rsp)

    @utl.logger
    def read_io_module_magnitude_group_current_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x10)
    
    @utl.logger
    def read_io_module_magnitude_group_average_eng_units(self,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x11)

    @utl.logger
    def read_io_module_magnitude_group_peak_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x12)

    @utl.logger
    def read_io_module_magnitude_group_lowest_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x13)

    @utl.logger
    def read_io_module_magnitude_group_total_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x14)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_current_counts(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x20)
    
    @utl.logger
    def read_io_module_magnitude_group_sqrt_average_counts(self,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x21)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_peak_counts(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x22)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_lowest_counts(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x23)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_total_counts(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x24)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_current_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x30)
    
    @utl.logger
    def read_io_module_magnitude_group_sqrt_average_eng_units(self,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x31)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_peak_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x32)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_lowest_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x33)

    @utl.logger
    def read_io_module_magnitude_group_sqrt_total_eng_units(self,aa,MMMM):
        return self.read_32_bit_io_module_magnitude_group(aa,MMMM,TT=0x34)

    @utl.logger
    def set_dac_module_magnitude_eng_units(self,aa,CC,DDDDDDDD):
        """
        Send a 'SET DAC MODULE MAGNITUDE, ENG. UNITS'
            MsgFmt(cmd='w CC DDDDDDDD', rsp=None)

        Data Out:
            CC -
            DDDDDDDD -

        Data In:
            None

        """
        DDDDDDDD = self.engineering_units_to_mistic_data(DDDDDDDD)
        return self.send_receive_2(aa,'SET DAC MODULE MAGNITUDE, ENG. UNITS',inspect.currentframe())

    @utl.logger
    def set_dac_module_magnitude_eng_units_group(self,aa,MMMM,DDDDDDDD):
        """
        Send a 'SET DAC MODULE MAGNITUDE, ENG. UNITS-GRP.'
            MsgFmt(cmd='W MMMM DDDDDDDD', rsp=None)

        Data Out:
            MMMM -
            DDDDDDDD -

        Data In:
            None

        """
        DDDDDDDD = tuple(self.engineering_units_to_mistic_data(v) for v in DDDDDDDD)
        return self.send_receive_2(aa,'SET DAC MODULE MAGNITUDE, ENG. UNITS-GRP.',inspect.currentframe())

    @utl.logger
    def set_dac_module_magnitude_counts(self,aa,CC,DDDD):
        """
        Send a 'SET DAC MODULE MAGNITUDE, COUNTS'
            MsgFmt(cmd='x CC DDDD', rsp=None)

        Data Out:
            CC -
            DDDD -

        Data In:
            None

        """
        return self.send_receive_2(aa,'SET DAC MODULE MAGNITUDE, COUNTS',inspect.currentframe())

    @utl.logger
    def set_dac_module_magnitude_counts_group(self,aa,MMMM,DDDD):
        """
        Send a 'SET DAC MODULE MAGNITUDE, COUNTS-GROUP'
            MsgFmt(cmd='X MMMM DDDD', rsp=None)

        Data Out:
            MMMM -
            DDDD -

        Data In:
            None

        """
        return self.send_receive_2(aa,'SET DAC MODULE MAGNITUDE, COUNTS-GROUP',inspect.currentframe())

    # Chapter 15: Analog Event/Reaction Commands
    # Command Name, Command Format, Version
    @utl.logger
    def set_event_in_io_gte_setpoint_counts(self,aa,EE,CC,TT,DDDD):
        """
        Send a 'SET EVENT ON I/O >= SETPOINT (COUNTS)'
            MsgFmt(cmd='K EE CC TT DDDD', rsp=None)

        Data Out:
            EE - event table entry number
            CC -
            TT -
            DDDD -

        Data In:
            None

        """
        return self.send_receive_2(aa,'SET EVENT ON I/O >= SETPOINT (COUNTS)',inspect.currentframe())

    @utl.logger
    def set_event_in_io_gte_setpoint_eng_units(self,aa,EE,CC,TT,DDDDDDDD):
        """
        Send a 'SET EVENT ON I/O >= SETPOINT (ENG. UNITS)'
        
        Data Out:
            EE - event table entry number
            CC -
            TT -
            DDDDDDDD -

        Data In:
            None

        """
        DDDDDDDD = self.int32_to_analog(DDDDDDDD)
        return self.send_receive_2(aa,'SET EVENT ON I/O >= SETPOINT (ENG. UNITS)',inspect.currentframe())

    @utl.logger
    def set_event_in_io_lte_setpoint_counts(self,aa,EE,CC,TT,DDDDDDDD):
        """
        Send a 'SET EVENT ON I/O <= SETPOINT (COUNTS)'
            MsgFmt(cmd='K EE CC TT DDDDDDDD', rsp=None)

        Data Out:
            EE - event table entry number
            CC -
            TT -
            DDDDDDDD -

        Data In:
            None

        """
        return self.send_receive_2(aa,'SET EVENT ON I/O <= SETPOINT (COUNTS)',inspect.currentframe())

    @utl.logger
    def set_event_in_io_lte_setpoint_counts(self,aa,EE,CC,TT,DDDDDDDD):
        """
        Send a 'SET EVENT ON I/O <= SETPOINT (ENG. UNITS)'
            MsgFmt(cmd='L EE CC TT DDDDDDDD', rsp=None)

        Data Out:
            EE - event table entry number
            CC -
            TT -
            DDDDDDDD -

        Data In:
            None

        """
        return self.send_receive_2(aa,'SET EVENT ON I/O <= SETPOINT (ENG. UNITS)',inspect.currentframe())

    """
    Analog 'SET EVENT REACTION COMMAND' modifiers
    """
    analog_reaction_commands = {
        # REACTION COMMANDS (To be used with analog command M)
        'SET DAC MODULE MAGNITUDE, COUNTS':MsgFmt('01 CC DDDD',None),
        'SET DAC MODULE MAGNITUDE, ENG. UNITS':MsgFmt('02 CC DDDDDDDD',None),
        'RAMP DAC OUTPUT TO ENDPOINT':MsgFmt('03 CC EEEEEEEE SSSSSSSS',None),
        'ENABLE/DISABLE PID LOOP':MsgFmt('04 LL SS',None),
        'SET PID LOOP SETPOINT':MsgFmt('05 LL SSSSSSSS',None),
        'READ AND HOLD I/O DATA':MsgFmt('08 CC TT',None),
        'SET PID LOOP MIN-MAX OUTPUT LIMITS':MsgFmt('09 LL HHHHHHHH LLLLLLLL',None),
        }

    @utl.logger
    def on_event_set_dac_module_magnitude_counts(self,aa,EE,RC,CC,DDDD):
        """
        Send a 'ON EVENT SET DAC MODULE MAGNITUDE, COUNTS'
            MsgFmt(cmd='M EE RC CC DDDD', rsp=None)

        Data Out:
            EE - event table entry number
            RC -
            CC -
            DDDD -

        Data In:
            None

        """
        RC=int(self.analog_reaction_commands['SET DAC MODULE MAGNITUDE, COUNTS'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT SET DAC MODULE MAGNITUDE, COUNTS',inspect.currentframe())

    @utl.logger
    def on_event_set_dac_module_magnitude_eng_units(self,aa,EE,RC,CC,DDDDDDDD):
        """
        Send a 'ON EVENT SET DAC MODULE MAGNITUDE, ENG. UNITS'
            MsgFmt(cmd='M EE RC CC DDDDDDDD', rsp=None)

        Data Out:
            EE - event table entry number
            RC -
            CC -
            TT -
            DDDDDDDD -

        Data In:
            None

        """
        RC=int(self.analog_reaction_commands['SET DAC MODULE MAGNITUDE, ENG. UNITS'].cmd.split(' ')[0],16)
        DDDDDDDD = self.engineering_units_to_mistic_data(DDDDDDDD)
        return self.send_receive_2(aa,'ON EVENT SET DAC MODULE MAGNITUDE, ENG. UNITS',inspect.currentframe())

    @utl.logger
    def on_event_ramp_dac_output_to_endpoint(self,aa,EE,RC,CC,EEEEEEEE,SSSSSSSS):
        """
        Send a 'ON EVENT RAMP DAC OUTPUT TO ENDPOINT'
            MsgFmt(cmd='M EE RC CC EEEEEEEE SSSSSSSS', rsp=None)

        Data Out:
            EE - event table entry number
            RC -
            CC -
            EEEEEEEE -
            SSSSSSSS -

        Data In:
            None

        """
        RC=int(self.analog_reaction_commands['RAMP DAC OUTPUT TO ENDPOINT'].cmd.split(' ')[0],16)
        EEEEEEEE = self.engineering_units_to_mistic_data(EEEEEEEE)
        SSSSSSSS = self.engineering_units_to_mistic_data(SSSSSSSS)
        return self.send_receive_2(aa,'ON EVENT RAMP DAC OUTPUT TO ENDPOINT',inspect.currentframe())

    @utl.logger
    def on_event_enable_disable_pid_loop(self,aa,EE,RC,LL,SS):
        """
        Send a 'ON EVENT ENABLE/DISABLE PID LOOP'
            MsgFmt(cmd='M EE RC LL SS', rsp=None)

        Data Out:
            EE - event table entry number
            RC -
            LL -
            SS -

        Data In:
            None

        """
        RC=int(self.analog_reaction_commands['ENABLE/DISABLE PID LOOP'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT ENABLE/DISABLE PID LOOP',inspect.currentframe())
        
    @utl.logger
    def on_event_set_pid_loop_setpoint(self,aa,EE,RC,LL,SSSSSSSS):
        """
        Send a 'ON EVENT SET PID LOOP SETPOINT'
            MsgFmt(cmd='M EE RC LL SSSSSSSS', rsp=None)

        Data Out:
            EE - event table entry number
            RC -
            LL -
            SSSSSSSSSS -

        Data In:
            None

        """
        RC=int(self.analog_reaction_commands['SET PID LOOP SETPOINT'].cmd.split(' ')[0],16)
        SSSSSSSS = self.engineering_units_to_mistic_data(SSSSSSSS)
        return self.send_receive_2(aa,'ON EVENT SET PID LOOP SETPOINT',inspect.currentframe())

    @utl.logger
    def on_event_read_and_hold_io_data(self,aa,EE,RC,CC,TT):
        """
        Send a 'ON EVENT READ AND HOLD I/O DATA'
            MsgFmt(cmd='M EE RC CC TT', rsp=None)

        Data Out:
            EE - event table entry number
            RC -
            CC -
            TT -

        Data In:
            None

        """
        RC=int(self.analog_reaction_commands['READ AND HOLD I/O DATA'].cmd.split(' ')[0],16)
        return self.send_receive_2(aa,'ON EVENT READ AND HOLD I/O DATA',inspect.currentframe())

    @utl.logger
    def on_event_set_pid_loop_min_max_output_limits(self,aa,EE,RC,LL,HHHHHHHH,LLLLLLLL):
        """
        Send a 'ON EVENT SET PID LOOP MIN-MAX OUTPUT LIMITS'
            MsgFmt(cmd='M EE RC LL HHHHHHHH LLLLLLLL', rsp=None)]

        Data Out:
            EE - event table entry number
            RC -
            LL -
            HHHHHHHH -
            LLLLLLLL -

        Data In:
            None

        """
        RC=int(self.analog_reaction_commands['SET PID LOOP MIN-MAX OUTPUT LIMITS'].cmd.split(' ')[0],16)
        HHHHHHHH = self.engineering_units_to_mistic_data(HHHHHHHH)
        LLLLLLLL = self.engineering_units_to_mistic_data(LLLLLLLL)
        return self.send_receive_2(aa,'ON EVENT SET PID LOOP MIN-MAX OUTPUT LIMITS',inspect.currentframe())
                                                    
    # Chapter 16: PID Loop Commands
    # Command Name, Command Format, Version
    @utl.logger
    def set_pid_loop_control_options(self,aa,CC,SSSS,CCCC):
        """
        Send a 'SET PID LOOP CONTROL OPTIONS'
            MsgFmt(cmd='j CC SSSS CCCC', rsp=None)

        Data Out:
            CC -
            SSSS -
            CCCC -

        Data In:
            None

        """
        return self.send_receive_2(aa,'SET PID LOOP CONTROL OPTIONS',inspect.currentframe())

    @utl.logger
    def set_pid_loop_derivative_rate(self,aa,LL,DDDDDDDD):
        """
        Send a 'SET PID LOOP DERIVATIVE RATE'
            MsgFmt(cmd='n LL DDDDDDDD', rsp=None)

        Data Out:
            LL -
            DDDDDDDD -

        Data In:
            None

        """
        DDDDDDDD = self.engineering_units_to_mistic_data(DDDDDDDD)
        return self.send_receive_2(aa,'SET PID LOOP DERIVATIVE RATE',inspect.currentframe())

    @utl.logger
    def set_pid_loop_gain(self,aa,LL,GGGGGGGG):
        """
        Send a 'SET PID LOOP GAIN'
            MsgFmt(cmd='I LL GGGGGGGG', rsp=None)

        Data Out:
            LL -
            GGGGGGGG -

        Data In:
            None

        """
        GGGGGGGG = self.engineering_units_to_mistic_data(GGGGGGGG)
        return self.send_receive_2(aa,'SET PID LOOP GAIN',inspect.currentframe())

    @utl.logger
    def set_pid_loop_process_variable(self,aa,LL,SSSSSSSS):
        """
        Send a 'SET PID LOOP PROCESS VARIABLE'
            MsgFmt(cmd='q LL SSSSSSSS', rsp=None)

        Data Out:
            LL -
            SSSSSSSS -

        Data In:
            None

        """
        SSSSSSSS = self.engineering_units_to_mistic_data(SSSSSSSS)
        return self.send_receive_2(aa,'SET PID LOOP PROCESS VARIABLE',inspect.currentframe())

    @utl.logger
    def set_pid_loop_setpoint(self,aa,LL,SSSSSSSS):
        """
        Send a 'SET PID LOOP SETPOINT'
            MsgFmt(cmd='k LL SSSSSSSS', rsp=None)

        Data Out:
            LL -
            SSSSSSSS -

        Data In:
            None

        """
        SSSSSSSS = self.engineering_units_to_mistic_data(SSSSSSSS)
        return self.send_receive_2(aa,'SET PID LOOP SETPOINT',inspect.currentframe())

    @utl.logger
    def initialize_pid_loop(self,aa,LL,II,SS,OO,TTTT):
        """
        Send a 'INITIALIZE PID LOOP'
            MsgFmt(cmd='i LL II SS OO TTTT', rsp=None)

        Data Out:
            LL -
            SS -
            OO -
            TTTT -

        Data In:
            None

        """
        return self.send_receive_2(aa,'INITIALIZE PID LOOP',inspect.currentframe())

    @utl.logger
    def read_all_pid_loop_parameters(self,aa,LL):
        """
        Send a 'READ ALL PID LOOP PARAMETERS'
            MsgFmt(cmd='T LL', rsp='CCCC RRRR II SS ZZ OO PPPPPPPP SSSSSSSS GGGGGGGG
            IIIIIIII DDDDDDDD HHHHHHHH LLLLLLLL ZZZZ YYYY OOOOOOOO AAAAAAAA BBBBBBBB
            QQQQQQQQ FFFFFFFF KKKKKKKK')

        Data Out:
            LL -

        Data In:
            CCCC:     Loop Control Word.
            RRRR:     Scan Rate in 100 mS units.
            II:       Input Channel Number (00-0F).
            SS:       Setpoint Channel Number (00-0F).
            ZZ:       Reserved for Future Use.
            OO:       Output Channel Number (00-0F).
            PPPPPPPP: Loop Input in Engineering units (Process Variable).
            SSSSSSSS: Loop Setpoint in Engineering units.
            GGGGGGGG: Loop Gain Constant.
            IIIIIIII: Loop Reset Rate in Repeats per Minute (RPM).
            DDDDDDDD: Loop Derivative Term in Minutes.
            HHHHHHHH: Loop Maximum Setpoint Limit.
            LLLLLLLL: Loop Minimum Setpoint Limit.
            ZZZZ:     Reserved for Future Use.
            YYYY:     Loop Output in Counts (Controlled Variable).
            OOOOOOOO: Loop Output Setting in Engineering units of Output Channel.
            AAAAAAAA: Loop Output Maximum Limit in Engineering units.
            BBBBBBBB: Loop Output Minimum Limit in Engineering units.
            QQQQQQQQ: Loop Output Maximum Change Per Scan
            FFFFFFFF: Loop Input Full Scale in Engineering units.
            KKKKKKKK: Loop Input Zero Scale in Engineering units.
        """
        rsp = self.send_receive_2(aa,'READ ALL PID LOOP PARAMETERS',inspect.currentframe())
        names = tuple(self.commands['READ ALL PID LOOP PARAMETERS'].rsp.split(' ')[6:])
        return self.parse_mistic_response_data(names,rsp)


##    @utl.logger
##    def read_pid_loop_parameter(self,aa,LL,PP):
##        """
##        Send a 'READ PID LOOP PARAMETER'
##
##        """
##        return self.send_receive_2(aa,'READ PID LOOP PARAMETER',inspect.currentframe())
##
##    Note: Although the command essentially remains 'READ PID LOOP PARAMETER', some of the
##    commands return 16 bits and some 32 bits ('DDDD' or 'DDDDDDDD').  Trying to come up
##    with a way to automate response parsing by coding the fields in self.commands.rsp

    @utl.logger
    def read_pid_loop_control_word(self,aa,LL,PP=0):
        """
        Send a 'READ PID LOOP CONTROL WORD'
            MsgFmt(cmd='t LL PP', rsp='DDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDD

        """
        return self.send_receive_2(aa,'READ PID LOOP CONTROL WORD',inspect.currentframe())

    @utl.logger
    def read_pid_loop_rate_word(self,aa,LL,PP=1):
        """
        Send a 'READ PID LOOP RATE WORD'
            MsgFmt(cmd='t LL PP', rsp='DDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDD

        """
        return self.send_receive_2(aa,'READ PID LOOP RATE WORD',inspect.currentframe())

    @utl.logger
    def read_pid_loop_output_counts(self,aa,LL,PP=2):
        """
        Send a 'READ PID LOOP OUTPUT COUNTS'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        return self.send_receive_2(aa,'READ PID LOOP OUTPUT COUNTS',inspect.currentframe())

    @utl.logger
    def read_pid_loop_input_setpoint_output_channels(self,aa,LL,PP=3):
        """
        Send a 'READ PID LOOP INPUT, SETPOINT, OUTPUT CHANNELS'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            II - input channel number
            SS - setpoint channel number
            ZZ - reserved
            OO - output channel number

        """
        return self.send_receive_2(aa,'READ PID LOOP INPUT, SETPOINT, OUTPUT CHANNELS',inspect.currentframe())

    @utl.logger
    def read_pid_loop_input_value(self,aa,LL,PP=4):
        """
        Send a 'READ PID LOOP INPUT VALUE'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP INPUT VALUE',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP INPUT VALUE'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_setpoint_value(self,aa,LL,PP=5):
        """
        Send a 'READ PID LOOP SETPOINT VALUE'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP SETPOINT VALUE',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP SETPOINT VALUE'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_output_value(self,aa,LL,PP=6):
        """
        Send a 'READ PID LOOP OUTPUT VALUE'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP OUTPUT VALUE',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP OUTPUT VALUE'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_gain_term(self,aa,LL,PP=7):
        """
        Send a 'READ PID LOOP GAIN TERM'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP GAIN TERM',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP GAIN TERM'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_integral_term(self,aa,LL,PP=8):
        """
        Send a 'READ PID LOOP INTEGRAL TERM'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP INTEGRAL TERM',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP INTEGRAL TERM'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_derivative_term(self,aa,LL,PP=9):
        """
        Send a 'READ PID LOOP DERIVATIVE TERM'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP DERIVATIVE TERM',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP DERIVATIVE TERM'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_maximum_setpoint_limit(self,aa,LL,PP=10):
        """
        Send a 'READ PID LOOP MAXIMUM SETPOINT LIMIT'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP MAXIMUM SETPOINT LIMIT',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP MAXIMUM SETPOINT LIMIT'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_minimum_setpoint_limit(self,aa,LL,PP=11):
        """
        Send a 'READ PID LOOP MINIMUM SETPOINT LIMIT'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP MINIMUM SETPOINT LIMIT',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP MINIMUM SETPOINT LIMIT'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_output_maximum_limit(self,aa,LL,PP=12):
        """
        Send a 'READ PID LOOP OUTPUT MAXIMUM LIMIT'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP OUTPUT MAXIMUM LIMIT',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP OUTPUT MAXIMUM LIMIT'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_output_minimum_limit(self,aa,LL,PP=13):
        """
        Send a 'READ PID LOOP OUTPUT MINIMUM LIMIT'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')        

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP OUTPUT MINIMUM LIMIT',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP OUTPUT MINIMUM LIMIT'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_maximum_change_per_scan(self,aa,LL,PP=14):
        """
        Send a 'READ PID LOOP OUTPUT MAXIMUM CHANGE PER SCAN'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')        

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP OUTPUT MAXIMUM CHANGE PER SCAN',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP OUTPUT MAXIMUM CHANGE PER SCAN'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_output_counts(self,aa,LL,PP=15):
        """
        Send a 'READ PID LOOP OUTPUT COUNTS'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP OUTPUT COUNTS',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP OUTPUT COUNTS'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_full_scale_in_eng_units(self,aa,LL,PP=16):
        """
        Send a 'READ PID LOOP FULL SCALE IN ENG. UNITS'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')        

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP FULL SCALE IN ENG. UNITS',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP FULL SCALE IN ENG. UNITS'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)

    @utl.logger
    def read_pid_loop_zero_scale_in_eng_units(self,aa,LL,PP=17):
        """
        Send a 'READ PID LOOP ZERO SCALE IN ENG. UNITS'
            MsgFmt(cmd='t LL PP', rsp='DDDDDDDD')        

        Data Out:
            LL -
            PP -

        Data In:
            DDDDDDDD -

        """
        rsp = self.send_receive_2(aa,'READ PID LOOP ZERO SCALE IN ENG. UNITS',inspect.currentframe())
        names = tuple(self.commands['READ PID LOOP ZERO SCALE IN ENG. UNITS'].rsp.split(' '))
        return self.parse_mistic_response_data(names,rsp)
                
    @utl.logger
    def set_pid_loop_integral_reset_rate(self,aa,LL,IIIIIIII):
        """
        Send a 'SET PID LOOP INTEGRAL RESET RATE'
            MsgFmt(cmd='m LL IIIIIIII', rsp=None)

        Data Out:
            LL -
            IIIIIIII -

        Data In:
            None

        """
        IIIIIIII = self.engineering_units_to_mistic_data(IIIIIIII)
        return self.send_receive_2(aa,'SET PID LOOP INTEGRAL RESET RATE',inspect.currentframe())

    @utl.logger
    def set_pid_loop_maximum_rate_of_change(self,aa,LL,RRRRRRRR):
        """
        Send a 'SET PID LOOP MAXIMUM RATE OF CHANGE'
            MsgFmt(cmd='u LL RRRRRRRR', rsp=None)

        Data Out:
            LL -
            RRRRRRRR -

        Data In:
            None

        """
        RRRRRRRR = self.engineering_units_to_mistic_data(RRRRRRRR)
        return self.send_receive_2(aa,'SET PID LOOP MAXIMUM RATE OF CHANGE',inspect.currentframe())

    @utl.logger
    def set_pid_loop_min_max_output_limits(self,aa,LL,HHHHHHHH,LLLLLLLL):
        """
        Send a 'SET PID LOOP MIN-MAX OUTPUT LIMITS'
            MsgFmt(cmd='p LL HHHHHHHH LLLLLLLL', rsp=None)

        Data Out:
            LL -
            HHHHHHHH -
            LLLLLLLL -

        Data In:
            None

        """
        HHHHHHHH = self.engineering_units_to_mistic_data(HHHHHHHH)
        LLLLLLLL = self.engineering_units_to_mistic_data(LLLLLLLL)
        return self.send_receive_2(aa,'SET PID LOOP MIN-MAX OUTPUT LIMITS',inspect.currentframe())

    @utl.logger
    def set_pid_loop_min_max_setpoint_limits(self,aa,LL,HHHHHHHH,LLLLLLLL):
        """
        Send a 'SET PID LOOP MIN-MAX SETPOINT LIMITS'
            MsgFmt(cmd='o LL HHHHHHHH LLLLLLLL', rsp=None)

        Data Out:
            LL -
            HHHHHHHH -
            LLLLLLLL -

        Data In:
            None

        """
        HHHHHHHH = self.engineering_units_to_mistic_data(HHHHHHHH)
        LLLLLLLL = self.engineering_units_to_mistic_data(LLLLLLLL)
        return self.send_receive_2(aa,'SET PID LOOP MIN-MAX SETPOINT LIMITS',inspect.currentframe())

    @utl.logger
    def build_ascii_command(self,aa,cmd,frame):
        """
        aa - device address in range(256)
        cmd - key to command and response format into self.commands
        frame - the callers frame so we can verify args

        EXAMPLES :
            mn.set_io_configuration_group(0x8f,15,(4,4,6,0x84))

            tx: >8FG000F8406040435\r
            rx: A41\r
            
            Sends a Set I/O Configuration - Group command in ASCII protocol
            to the I/O unit at address 8F Hex.
            
            Channels 03, 02, 01 and 00 are selected (MMMM = 000F)
            to be configured as follows:
            
            Channel 03 = 84 Hex (DA4 0 to 5 VDC analog output)
            Channel 02 = 06 Hex (AD6 0 to 5 VDC analog input)
            Channel 01 = 04 Hex (AD4 ICTD analog input)
            Channel 00 = 04 Hex (AD4 ICTD analog input)
            Data verification method is 8-bit checksum.
        """
        cmdfmt = CmdFmt(self.commands[cmd].cmd.split(' ')[0],\
                         tuple(self.commands[cmd].cmd.split(' ')[1:]))
        # start with header
        txpkt = '>{:02X}'.format(aa)
        txpkt += cmdfmt.cmd
        if len(cmdfmt.args) > 0:
            # get args passed to the function
            fun_args = tuple(inspect.getargvalues(frame).args)
            # if args are required, make sure we have them all
            if fun_args[-len(cmdfmt.args):] == cmdfmt.args:
                # build kwargs
                values = inspect.getargvalues(frame)[3]
                # beware, can't pass kwargs as parameter to a function
                # without losing the OrderedDict nature
                kwargs = OrderedDict()
                for arg in cmdfmt.args:
                    kwargs[arg] = values[arg]
                for key in kwargs:
                    if isinstance(kwargs[key],int):
                        txpkt += '{:0{:d}X}'.format(kwargs[key],len(key))
                    elif isinstance(kwargs[key],tuple):
                        # we're assuming data is passed starting with lower index
                        # so reverse the order for transmit
                        for e in reversed(kwargs[key]):
                            txpkt += '{:0{:d}X}'.format(e,len(key))            
        if self.crc_mode:
            txpkt += '{:04X}\r'.format(self.crc(aa,txpkt[1:]))
        else:
            txpkt += '{:02X}\r'.format(self.chksum(aa,txpkt[1:]))
        return txpkt

    @utl.logger
    def build_binary_command(self,aa,cmd,frame):        
        """
        """
        cmdfmt = CmdFmt(self.commands[cmd].cmd.split(' ')[0],\
                         tuple(self.commands[cmd].cmd.split(' ')[1:]))
        # start with header
        txpkt = bytearray()
        txpkt += (aa).to_bytes(1,byteorder='big')
        txpkt += (0).to_bytes(1,byteorder='big')
        txpkt += cmdfmt.cmd.encode()
        if len(cmdfmt.args) > 0:
            # get args passed to the function
            fun_args = tuple(inspect.getargvalues(frame).args)
            # if args are required, make sure we have them all
            if fun_args[-len(cmdfmt.args):] == cmdfmt.args:
                # build kwargs
                values = inspect.getargvalues(frame)[3]
                # beware, can't pass kwargs as parameter to a function
                # without losing the OrderedDict nature
                kwargs = OrderedDict()
                for arg in cmdfmt.args:
                    kwargs[arg] = values[arg]
                for key in kwargs:
                    if isinstance(kwargs[key],int):
                        txpkt += (kwargs[key]).to_bytes(len(key)//2,byteorder='big')
                    elif isinstance(kwargs[key],tuple):
                        # we're assuming data is passed starting with lower index
                        # so reverse the order for transmit
                        for e in reversed(kwargs[key]):
                            txpkt += e.to_bytes(len(key)//2,byteorder='big')
        if self.crc_mode:
            txpkt[1] = len(txpkt[2:]) + 2
            txpkt += (self.crc(aa,txpkt)).to_bytes(2,byteorder='big')
        else:
            txpkt[1] = len(txpkt[2:]) + 1
            txpkt += (self.chksum(aa,txpkt)).to_bytes(1,byteorder='big')
        return txpkt
            
    @utl.logger
    def build_command(self,aa,cmd,frame):
        """
        """
        # create Mistic packet to be sent
        if self.binary_mode:
            txpkt = self.build_binary_command(aa,cmd,frame)            
        else:
            txpkt = self.build_ascii_command(aa,cmd,frame)
        return txpkt

    @utl.logger
    def get_ascii_response(self,aa):
        """
        """
        rxPkt = ''
        # while response timer running
        tb = perf_counter()
        timeout = self.get_response_timeout(aa)
        utl.log_info_message('Response timeout is {:f} secs'.format(timeout-perf_counter()))
        rxPkt = ''
        while perf_counter() < timeout:
            # if there are bytes waiting to be read
            n = self.tty.rx_bytes_available()
            if n > 0:
                # read all of them
                s = self.tty.read(n)
                rxPkt += s
                an = re.search(r'[AN]',rxPkt)
                if an:
                    rxPkt = rxPkt[an.start():]
                    cr = re.search(r'\r',rxPkt)
                    if cr:
                        rxPkt = rxPkt[:cr.end()]
                        if self.verify_dvf(aa,rxPkt):
                            if self.crc_mode:
                                rsp = (rxPkt[0:1],rxPkt[1:-5])
                            else:
                                rsp = (rxPkt[0:1],rxPkt[1:-3])                                
                            utl.log_info_message('Response {} time took {:f} secs'.format(rsp,perf_counter()-tb))
                            return RspMsg(*rsp)
                        else:
                            return RspMsg('E',2)
                else:
                    # if there are no 'A's or 'N's, packet can't possibly be
                    # the start of something important, could be leftovers
                    rxPkt = ''
        else:
            # Had an issue where max read was 64 bytes with USB device.
            # Perhaps a sleep(0) will give kernel time to send more USB
            # IRPs.
            sleep(0)
        return RspMsg('E',29)

    @utl.logger
    def get_binary_response(self,aa):
        """
        """
        # while response timer running
        tb = perf_counter()
        timeout = self.get_response_timeout(aa)
        utl.log_info_message('Response timeout is {:f} secs'.format(timeout-perf_counter()))
        rx = bytearray()
        while perf_counter() < timeout:
            # if there are bytes waiting to be read
            n = self.tty.rx_bytes_available()
            if n > 0:
                # append data to buffer
                rx += self.tty.read(n,self.binary_mode)
                if self.crc_mode and len(rx) >= 4:
                    if self.verify_dvf(aa,rx):
                        if rx[1] == 0:
                            rsp = ('A',rx[2:-2])
                        else:
                            rsp = ('N',rx[1])
                        utl.log_info_message('Response {} time took {:f} secs'.format(rsp,perf_counter()-tb))
                        return RspMsg(*rsp)
                elif len(rx) >= 3: 
                    if self.verify_dvf(aa,rx):
                        if rx[1] == 0:
                            rsp = ('A',rx[2:-1])
                        else:
                            rsp = ('N',rx[1])
                        utl.log_info_message('Response {} time took {:f} secs'.format(rsp,perf_counter()-tb))
                        return RspMsg(*rsp)
                    else:
                        rx = bytearray()
            else:
                # Had an issue where max read was 64 bytes with USB device.
                # Perhaps a sleep(0) will give kernel time to send more USB
                # IRPs.
                sleep(0)
        return RspMsg('E',29)
        
    @utl.logger
    def get_response(self,aa):
        """
        """
        if self.binary_mode:
            return self.get_binary_response(aa)
        else:
            return self.get_ascii_response(aa)
    
    @utl.logger
    def parse_ascii_response(self,aa,cmd,ntrsp):
        """
        split up the data into fields based on self.commands[].rspfmt
        """
        if ntrsp.ack == 'A':
            rspfmt = self.commands[cmd].rsp
            if rspfmt:
                fields = rspfmt.split(' ')
                fields = [field for field in fields]
                ntflds = namedtuple('ntflds',fields)
                widths = tuple(len(field) for field in ntflds._fields)
                # data length expected
                explen = sum(widths)
                data = ntrsp.data
                lflds = []
                if len(fields) > 1:
                    for width in widths:
                        lflds += [int(data[:width],16)]
                        data = data[width:]
                elif len(fields) == 1:
                    width = widths[0]
                    for i in range(0,len(data),width):
                        lflds += [int(data[:width],16)]
                        data = data[width:]
                    # we're assuming data is returned starting with higher index
                    # so reverse the order for easer indexing
                    lflds = [tuple(reversed(lflds))]
                else:
                    lflds = []
                rsp = RspMsg(ntrsp.ack,ntflds(*lflds))
                return rsp
            else:
                t = (ntrsp.ack,(None,))
                ntrsp = namedtuple('ntrsp',['ack','data'])
                return ntrsp(*t)
                
        # Mistic returns a hex error
        elif ntrsp.ack == 'N':
            ntflds = namedtuple('ntflds',['ERROR'])
            lflds = [int(ntrsp.data,16)]
            return RspMsg(ntrsp.ack,ntflds(*lflds))
        # we return TBD errors
        elif ntrsp.ack == 'E':
            ntflds = namedtuple('ntflds',['ERROR'])
            if isinstance(ntrsp.data,int):
                lflds = [int(ntrsp.data)]
            else:
                lflds = [ntrsp.data]
            return RspMsg(ntrsp.ack,ntflds(*lflds))
        return RspMsg(ntrsp.ack,None)

    @utl.logger
    def parse_binary_response(self,aa,cmd,ntrsp):
        """
        split up the data into fields based on self.commands[].rspfmt
        """
        if ntrsp.ack == 'A':
            rspfmt = self.commands[cmd].rsp
            if rspfmt:
                fields = rspfmt.split(' ')
                fields = [field for field in fields]
                ntflds = namedtuple('ntflds',fields)
                widths = tuple(len(field) for field in ntflds._fields)
                # data length expected
                explen = sum(widths)
                data = bytearray(ntrsp.data)
                lflds = []
                if len(fields) > 1:
                    for width in widths:
                        lflds += [int.from_bytes(data[:width//2],byteorder='big')]
                        data = data[width//2:]
                elif len(fields) == 1:
                    width = widths[0]
                    for i in range(0,len(data),width//2):
                        lflds += [int.from_bytes(data[:width//2],byteorder='big')]
                        data = data[width//2:]
                    # we're assuming data is returned starting with higher index
                    # so reverse the order for easer indexing
                    lflds = [tuple(reversed(lflds))]
                else:
                    lflds = []
                rsp = RspMsg(ntrsp.ack,ntflds(*lflds))
                return rsp
            else:
                t = (ntrsp.ack,(None,))
                ntrsp = namedtuple('ntrsp',['ack','data'])
                return ntrsp(*t)
                
        # Mistic returns a hex error
        elif ntrsp.ack == 'N':
            ntflds = namedtuple('ntflds',['ERROR'])
            lflds = [ntrsp.data]
            return RspMsg(ntrsp.ack,ntflds(*lflds))
        # we return TBD errors
        elif ntrsp.ack == 'E':
            ntflds = namedtuple('ntflds',['ERROR'])
            if isinstance(ntrsp.data,int):
                lflds = [int(ntrsp.data)]
            else:
                lflds = [ntrsp.data]
            return RspMsg(ntrsp.ack,ntflds(*lflds))
        return RspMsg(ntrsp.ack,None)
        
    @utl.logger
    def parse_response_data(self,aa,cmd,pkt):
        """
        """
        if self.binary_mode:
            return self.parse_binary_response(aa,cmd,pkt)
        else:
            return self.parse_ascii_response(aa,cmd,pkt)
    
    @utl.logger
    def send_receive_2(self,aa,cmd,frame):
        """
        """
        # for 'REPEAT LAST RESPONSE' or possible retries?
        if cmd != 'REPEAT LAST RESPONSE':
            self.last_command[aa] = cmd
        txpkt = self.build_command(aa,cmd,frame)
        for i in range(3):
            utl.log_info_message(txpkt)
            # start with a clean slate
            self.tty.flush_input_buffer(self.binary_mode)
            # send the bytes
            self.tty.write(txpkt,self.binary_mode)
            rsp = self.get_response(aa)
            utl.log_info_message(rsp)
            if rsp.ack == 'A':
                break
        # send last command response format for parsing
        if cmd == 'REPEAT LAST RESPONSE':
            return self.parse_response_data(aa,self.last_command[aa],rsp)
        return self.parse_response_data(aa,cmd,rsp)

    @utl.logger
    def crc(self,aa,data):
        """
        compute the crc.
        """
        crc = 0
        # if this is a string
        if isinstance(data,str):
            utl.log_info_message('Init crc to {:04X}'.format(crc))
            utl.log_info_message('computing crc16r')
            for ch in data:
                i = crc ^ ord(ch)
                crc = (crc >> 8) ^ self.crc16r[i & 255]
            return crc & 0xffff
        # elif this is an array of bytes
        elif isinstance(data,bytes) or isinstance(data,bytearray):
            utl.log_info_message('Init crc to {:04X}'.format(crc))
            utl.log_info_message('computing crc16r')             
            for b in data:
                i = crc ^ b
                crc = (crc >> 8) ^ self.crc16r[i & 255]
            return crc & 0xffff
        return 0
        
    @utl.logger
    def chksum(self,aa,data):
        """
        compute the 8 bit checksum. Since the same checksum calc is used in both
        ASCII and BINARY mode, data can be a string, bytes, or a bytearray.
        """
        if isinstance(data,str):
            cks = 0
            for ch in data:
                cks += ord(ch)
            return cks % 256
        elif isinstance(data,bytes) or isinstance(data,bytearray):
            cks = 0
            for b in data:
                cks += b    
            return cks % 256
        return 0        

    def crcchrev(self,poly,data):
        """
        compute a reverse crc
        """
        a = 0
        data <<= 1
        for i in range(8):
            data >>= 1
            if ((data ^ a) & 0x0001):
                a = ((a >> 1) ^ poly)
            else:
                a >>= 1
        return a
        
    def generate_crc_table(self,poly):
        """
        generate a crc table
        """
        tbl = []
        for i in range(256):
            if poly == 0x1021 or poly == 0x8005:
                tbl += [self.crcch(poly,i)]
            else:
                tbl += [self.crcchrev(poly,i)]
        return tuple(tbl)

    """
    crc tables constructed in init
    """
    crc16r = None
        
    def list_mistic_devices(self):
        """
        Build a list of Mistic devices by sending a
        'Power Up Clear' to all addresses and looking
        for an ACK.

        If we get an ACK, get the type of device for grins
        """
        devices = []
        for address in range(256):
            msg = 'checking address {:02X}'.format(address)
            print(msg,end='',flush=True)
            print(chr(8)*len(msg),end='',flush=True)
            rtn = self.power_up_clear(address)
            if rtn[0] == 'A':
                devices += [address]
        print('\nFound {:d} Mistic devices'.format(len(devices)))
        return devices

if __name__ == "__main__":             
    # create the OmuxNET object
    mn = OmstNET()
    # list the available ttys
    ttys = mn.tty.list_ttys()
    # print a menu
    print('List of available ttys')
    for tty in ttys:
        print(tty,ttys[tty])
    # ask user to select a port
    ttychoice = int(input('Select tty: '),10)
    print('\n')
    if ttychoice in ttys:
        print('Check baudrate jumpers')
        baudrates = mn.tty.list_baudrates()
        for baudrate in baudrates:
            print(baudrate,baudrates[baudrate])
        baudratechoice = int(input('Select baudrate: '),10)
        if baudratechoice in mn.tty.baudrates:
            baudrate = int(mn.tty.baudrates[baudratechoice])
        print('\n')
        # open the port
        if mn.tty.open(ttys[ttychoice],int(mn.tty.baudrates[baudratechoice])):

            print('Check binary mode jumper')
            print(0,'ASCII mode')
            print(1,'BINARY mode')
            chmode = int(input('Select character mode: '),10)
            mn.binary_mode = chmode == 1
            print('\n')
            
            print('Check verification mode jumper')
            print(0,'Use CHECKSUM')
            print(1,'use CRC16')
            dvfmode = int(input('Select verification mode: '),10)
            mn.crc_mode = dvfmode == 1
            print('\n')
            
            mn.devices = mn.list_mistic_devices()
            for device in mn.devices:
                rsp = mn.what_am_i(device)
                if rsp.ack == 'A':
                    print('{:s} found @ {:02X}'.format(rsp.data.BrainBoardType,device))
        else:
            print('() open failed'.format(ttys[choice]))
    else:
        print('{:d} : invalid port selection'.format(choice))
