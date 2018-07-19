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

import re                               # needed for parsing optomux ASCII packets
from   time import perf_counter,sleep   # needed for computing turnaround timeouts
from   itertools import chain           # needed for combining ranges in arg verification
import OmuxTTY as tty                   # communications via serial port
import OmuxUTL as utl                   # logging and TBD
import datetime                         # for timedelta
from   collections import namedtuple

# init the utils such as logging
utl._init()

class OmuxNET:
    
    def __init__(self):
        self.tty = tty.OmuxTTY()
        self.turnaround_delay = {}
        self.timer_resolution = {}
        self.temperature_probes = {}
        self.read_as_analog_input = {}

    def compute_pkt_checksum(self,pkt):
        """
        computes the checksum an optomux command packet
        for example, the command on.initiate_square_wave(0,15,1,1)
        generates the packet '>00Z000FL01019E\r'
        Note: this function assumes the packet does not yet
        have an appended checksum or \r and therefore is using
        len(pkt) as the final character to be summed. 

        compute_pkt_checksum computes the checksum of '00Z000FL0101'
        returning the result as an integer
        """
        cks = 0
        for i in range(1,len(pkt)):
            cks = ((cks + ord(pkt[i])) & 255)
        return cks
    
    def compute_data_checksum(self,data):
        """
        computes the checksum an optomux response packet
        for example, the command on.read_configuration(0)
        generated the packet '>00jCA\r'
        The optomux device responded with 'A000FD6\r'
        
        compute_data_checksum computes the checksum of '000F'
        which is the data portion of the response
        """
        cks = 0
        for i in range(len(data)):
            cks = ((cks + ord(data[i])) & 255)
        return cks

    """
    errors returned by Optomux controller
    and our emulation of the Optoware driver
    """
    errors = {
        # Optomux errors returned by the controller or brain
         0:'Power-Up Clear Expected - Command Ignored',
        -1:'Power Up Clear Expected',
        -2:'Undefined Command',
        -3:'Checksum Error',
        -4:'Input Buffer Overrun',
        -5:'Non-printable ASCII Character Received',
        -6:'Data Field Error',
        -7:'Serial Watchdog Time-out',
        -8:'Invalid Limit Set',
        # Optoware errors returned when packet is verified
        -20:'Invalid Command Number',
        -21:'Invalid Module Position',
        -22:'Data Range Error',
        -23:'Invalid First Modifiers',
        -24:'Invalid Second Modifiers',
        -25:'Invalid Address',
        -27:'Not Enough Return Data',
        -28:'Invalid Return Data',
        -29:'Turnaround Time Out',
        -30:'Input Buffer Overrun',
        -31:'Checksum Error',
        -33:'Send Error',
        -34:'Incorrect Command Echo In Four-Pass'
        }
    
    uint16_valid_range = utl.RangeList([range(65536)])
    uint15_valid_range = utl.RangeList([range(32768)])
    uint8_valid_range  = utl.RangeList([range(256)])
    uint12_valid_range = utl.RangeList([range(4096)])

    """
    dcit of valid turn around delay values
    """
    turnaround_delay = {
        0:'No Delay',
        1:'10 ms',
        2:'100 ms',
        3:'500 ms'
        }
    """
    range of valid turn around values
    """
    turnaround_delay_valid_range = \
        utl.RangeList([range(len(turnaround_delay))])

    """
    dict digital watchdog values
    """
    digital_watchdog_delay = {
        0:'Disable',
        1:'10s, All outputs off',
        2:'1m,  All outouts off',
        3:'10m, All outputs off',
        4:'Disable',
        5:'10s, 0 output on, rest off',
        6:'1m,  0 output on, rest off',
        7:'10m, 0 output on, rest off'
        }
    
    """
    set of valid digital watchdog delay values
    """
    digital_watchdog_delay_valid_range = \
        utl.RangeList([range(len(digital_watchdog_delay)),
            range(2,65536)])

    """
    dict analog watchdog times and actions
    """
    analog_watchdog_delay = {
        0:'Disable',
        1:'10s, Zero Scale',
        2:'1m,  Zero Scale',
        3:'10m, Zero Scale',
        4:'Disable',
        5:'10s, Full Scale',
        6:'1m,  Full Scale',
        7:'10m, Full Scale'
        }
    """
    set of valid digital watchdog delay values
    """
    analog_watchdog_delay_valid_range = \
        utl.RangeList([range(len(analog_watchdog_delay)),
            range(20,65536)])
    """
    dict of protocol pass values
    """
    protocol_passes = {
        0:'2 pass',
        1:'4 pass'
        }
    """
    range of valid protocol pass values
    """
    protocol_passes_valid_range = \
        utl.RangeList([range(len(protocol_passes))])
    """
    dict of optomux controller types
    """
    optomux_type = {
        0:'Digital',
        1:'Analog'
        }
    """
    range of optomux controller types
    """
    optomux_type_valid_range = \
        utl.RangeList([range(len(optomux_type))])
    """
    set of valid enhanced digital watchdog values
    """
    enhanced_digital_watchdog_delay_valid_range = utl.RangeList(\
        [range(0),range(200,65536)])
    """
    range of valid timer resolutions, note: 0 means the max time of 2.56 seconds
    """
    timer_resolution_valid_range = \
        utl.RangeList([range(256)])
    """
    dict of temperature probe types
    """
    temperature_probe_types = {
        0:'no temperature probe',
        1:'ICTD probe',
        2:'10 ohm RTD probe',
        3:'100 ohm RTD probe',
        4:'Type J thermocouple',
        5:'Type K thermocouple',
        6:'Type R thermocouple',
        7:'Type S thermocouple',
        8:'Type T thermocouple',
        9:'Type E thermocouple',
        }
    """
    'Set Temperature Probe Type' valid range
    """
    temperature_probe_type_valid_range = \
        utl.RangeList([range(len(temperature_probe_types))])
    """
    'Initiate Square Wave' widths
    """
    square_wave_on_off_time_valid_range = \
        uint8_valid_range
    """
    'Set Time Delay' valid range
    """
    set_time_delay_valid_range = \
        uint16_valid_range
    """
    'Generate N Pulses' pulse width valid range
    """
    generate_pulses_on_time_valid_range = \
        uint8_valid_range
    """
    'Generate N Pulses' valid count
    """
    generate_pulses_n_valid_range = \
        uint16_valid_range
    """
    'Set Number of Averages' valid range
    """
    number_of_averages_valid_range = \
        utl.RangeList([range(1,256)])
    """
    0 NOP
    1 cancels within 1 timer tick
    otherwise, multiplied by timer resolution
    """
    start_on_off_pulse_valid_range = \
        uint16_valid_range
    """
    Analog ADC/DAV values fir in 12 bits
    """
    analog_value_valid_range = \
        uint12_valid_range
    """
    time for 1/2 square wave or a sawtooth ramp
    """
    output_waveform_time = {
        0: 'Disable Waveform',
        1: '2.18  minutes',
        2: '3.28  minutes',
        3: '4.37  minutes',
        4: '5.46  minutes',
        5: '6.56  minutes',
        6: '7.65  minutes',
        7: '8.74  minutes',
        8: '1.09  minutes',
        9: '32.8  seconds',
        10:'21.89 seconds',
        11:'16.4  seconds',
        12:'13.1  seconds',
        13:'10.9  seconds',
        14:'9.4   seconds',
        15:'8.2   seconds',
        }
    """
    'Set Output Waveform' valid periods
    """
    output_waveform_time_valid_range = \
        utl.RangeList([range(1,len(output_waveform_time))])
    """
    'Set Output Waveform" valid shapes
    for the Optomux ASCII command
    """
    output_waveform_type = {
        0:'No waveform',
        1:'Triangle up',
        2:'Ramp to FS',
        3:'Sawtooth up',
        4:'Square wave',
        5:'Triangle down',
        6:'Ramp to ZS',
        7:'Sawtooth down'
        }
    """
    'Set Output Waveform' valid types
    """
    output_waveform_type_valid_range = \
        utl.RangeList([range(1,len(output_waveform_type))])
    """
    'Set Output Waveform' limits
    """
    output_waveform_lo_hi_limit_valid_range = \
        uint8_valid_range
    """
    'Enhanced Output Waveform' valid types
    for the Optomux ASCII command
    """
    enhanced_output_waveform_type = {
        0:'Turn waveform off',
        1:'Triangle wave with positive inital slope',
        2:'Ramp up—waveform terminates upon reaching the upper limit',
        3:'Sawtooth, continuous ramp up',
        4:'Square wave (50 % duty cycle)',
        5:'Triangle wave with negative initial slope',
        6:'Ramp down—waveform termihnates at lower limit',
        7:'Sawtooth, continuous ramp down'
        }
    """
    'Enhanced Output Waveform' valid range
    """
    enhanced_output_waveform_type_valid_range = \
        utl.RangeList([range(1,len(enhanced_output_waveform_type))])
    """
    list of optomux command format and name strings
    keyed by optoware command number
    """
    commands = {
        0: ('A', 'Power Up Clear'),
        1: ('B', 'Reset'),
        2: ('C[data]', 'Set Turnaround Delay'),
        3: ('D[data]', 'Set Digital Watchdog'),
        4: ('E[data]', 'Set Protocol'),
        5: ('F', 'Identify Optomux Type'),
        6: ('G[positions]', 'Configure Positions'),
        7: ('H[positions]', 'Configure As Inputs'),
        8: ('I[positions]', 'Configure As Outputs'),
        9: ('J[positions]', 'Write Digital Outputs'),
        10: ('K[positions]', 'Activate Digital Outputs'),
        11: ('L[positions]', 'Deactivate Digital Outputs'),
        12: ('M', 'Read On Off Status'),
        13: ('N[positions]', 'Set Latch Edges'),
        14: ('O[positions]', 'Set Off To On Latches'),
        15: ('P[positions]', 'Set On To Off Latches'),
        16: ('Q', 'Read Latches'),
        17: ('R[positions]', 'Read and Clear Latches'),
        18: ('S[positions]', 'Clear Latches'),
        19: ('T[positions]', 'Start and Stop Counters'),
        20: ('U[positions]', 'Start Counters'),
        21: ('V[positions]', 'Stop Counters'),
        22: ('W[positions]', 'Read Counters'),
        23: ('X[positions]', 'Read and Clear Counters'),
        24: ('Y[positions]', 'Clear Counters'),
        25: ('Z[positions][modifiers][data]', 'Set Time Delay'),
        26: ('Z[positions][modifiers][data]', 'Initiate Square Wave'),
        27: ('Z[positions][modifiers]', 'Turn Off Time Delay Square Wave'),
        28: ('a[positions]', 'Set Pulse Trigger Polarity'),
        29: ('b[positions]', 'Trigger On Positive Pulse'),
        30: ('c[positions]', 'Trigger On Negative Pulse'),
        31: ('d', 'Read Pulse Complete Bits'),
        32: ('e[positions]', 'Read Pulse Duration Counters'),
        33: ('f[positions]', 'Read and Clear Duration Counters'),
        34: ('g[positions]', 'Clear Duration Counters'),
        35: ('J[positions][data]', 'Write Analog Outputs'),
        36: ('K[positions]', 'Read Analog Outputs'),
        37: ('L[positions]', 'Read Analog Inputs'),
        38: ('M[positions][data]', 'Average and Read Input'),
        39: ('N[positions][data]', 'Set Input Range'),
        40: ('O', 'Read Out Of Range Latches'),
        41: ('P[positions]', 'Read and Clear Out Of Range Latches'),
        42: ('Q[positions]', 'Clear Out Of Range Latches'),
        43: ('R[positions][modifiers][data]', 'Set Output Waveform'),
        44: ('R[positions][modifiers]', 'Turn Off Existing Waveforms'),
        45: ('D[positions][data]', 'Set Analog Watchdog'),
        46: ('S[positions][data]', 'Update Analog Outputs'),
        47: ('T[positions][data]', 'Start Averaging Inputs'),
        48: ('i', 'Read Average Complete Bits'),
        49: ('U[positions]', 'Read Averaged Inputs'),
        50: ('V[positions][modifiers][data]', 'Enhanced Output Waveform'),
        51: ('V[positions][modifiers]', 'Cancel Enhanced Waveforms'),
        52: ('g[positions]', 'Calculate Offsets'),
        53: ('W[positions][data]', 'Set Offsets'),
        54: ('h[positions]', 'Calculate and Set Offsets'),
        55: ('X[positions]', 'Calculate Gain Coefficients'),
        56: ('Y[positions][data]', 'Set Gain Coefficients'),
        57: ('Z[positions]', 'Calculate and Set Gain Coefficients'),
        58: ('a[positions]', 'Read Lowest Values'),
        59: ('b[positions]', 'Clear Lowest Values'),
        60: ('c[positions]', 'Read and Clear Lowest Values'),
        61: ('d[positions]', 'Read Peak Values'),
        62: ('e[positions]', 'Clear Peak Values'),
        63: ('f[positions]', 'Read and Clear Peak Values'),
        64: ('M', 'Read Binary On Off Status'),
        65: ('J[positions]', 'Write Binary Outputs'),
        66: ('Q', 'Read Binary Latches'),
        67: ('R[positions]', 'Read and Clear Binary Latches'),
        68: ('Z[positions][modifiers][data]', 'High Resolution Square Wave'),
        69: ('h[positions]', 'Retrigger Time Delay'),
        70: ('j', 'Read Configuration'),
        71: ('m[positions][data]', 'Set Enhanced Digital Watchdog'),
        72: ('i[positions][modifiers][data]', 'Generate N Pulses'),
        73: ('k[positions][data]', 'Start On Pulse'),
        74: ('l[positions][data]', 'Start Off Pulse'),
        75: ('n[data]', 'Set Timer Resolution'),
        76: ('k[positions][data]', 'Set Temperature Probe Type'),
        77: ('l[positions]', 'Read Temperature Inputs'),
        78: ('m[positions][data]', 'Set Analog Watchdog Timeout'),
        79: ('o[positions]', 'Read Average Temperature Inputs'),
        80: ('`', 'Date Of Firmware')
        }

    """
    create a dictionary mapping optoware command description strings
    to command numbers.  These are used in various functions to
    look up the command number as an index into the commands dictionary.
    """
    command_name = {v[1]:k for (k,v) in commands.items()}
    
    @utl.logger
    def verify_address(self,address):
        """
        valid addresses are 0 to 255
        """
        if isinstance(address,int) and address in range(0,256):
            return (0,'{:02X}'.format(address))
        return ('E',-25)
    
    @utl.logger
    def verify_command(self,command):
        """
        Optoware commands fit range(80) and each command value
        is a key in the commands dictionary
        """
        # check the command number
        if isinstance(command,int) and command in self.commands.keys():
            return (0,'{:s}'.format(self.commands[command][0][0]))
        return ('E',-20)

    @utl.logger
    def verify_positions(self,command,afmt,**kwargs):
        """
        Positions may be a 16 bit mask or a
        tuple of range(len(16)) having a value in range(16).
        """
        if 'positions' in afmt:
            if 'positions' in kwargs:
                if isinstance(kwargs['positions'],tuple):
                    # at least 1 but not more than 16 points
                       # max from 0 to 16
                           # min from 0 to 16
                    if len(kwargs['positions']) in range(1,17) \
                       and max(kwargs['positions']) in range(16) \
                       and min(kwargs['positions']) in range(16):
                        mask = 0
                        for position in kwargs['positions']:
                            # build a bit mask
                            mask |= (1 << position)
                        return (0,'{:04X}'.format(mask))
                # allow passing bitmask or single position directly
                elif isinstance(kwargs['positions'],int):
                    # 'Average and Read Input' takes a single position and
                    # B3000 seems to return 'N01\r' which states in the docs:
                    # "Undefined Command. The command character was not a
                    # legal command character. This error may also be received
                    # if the unit is an older model that cannot recognize a
                    # newer command.* and it does this for the exact example
                    # given in the Optomux guide for this command
                    # >03M70A58\r
                    
                    if command == self.command_name['Average and Read Input']:
                        if kwargs['positions'] in range(16):
                            return (0,'{:X}'.format(kwargs['positions']))
                    # must fit in uint16
                    elif kwargs['positions'] in range(1,65536):
                        return (0,'{:04X}'.format(kwargs['positions']))
                # some commands assume all 16 points are written if
                # optomux message doesn't include positions
                elif kwargs['positions'] == None:
                    return (0,'')        
                return ('E',-21)
        return (0,'')

    @utl.logger
    def verify_single_modifier(self,command,modifier):
        """
        commands which take a singleASCII char or an int can be
        passed in a list of len 1, or directly as a str or int    
        """
        if isinstance(modifier,str):
## 'HIJK'         25: ('Z[positions][modifiers][data]', 'Set Time Delay'),
            if command == self.command_name['Set Time Delay'] \
               and modifier in 'HIJK':
                    return (0,modifier)
## 'L'            26: ('Z[positions][modifiers][data]', 'Initiate Square Wave'),
            elif command == self.command_name['Initiate Square Wave'] \
               and modifier == 'L':
                    return (0,modifier)
## 'G'            27: ('Z[positions][modifiers]', 'Turn Off Time Delay Square Wave'),
            elif command == self.command_name['Turn Off Time Delay Square Wave'] \
                and modifier == 'G':
                    return (0,modifier)
## 'M'            68: ('Z[positions][modifiers][data]', 'High Resolution Square Wave'),
            elif command == self.command_name['Turn Off Time Delay Square Wave'] \
                and modifier == 'M':
                    return (0,modifier)
# single int modifier               
        elif isinstance(modifier,int):
##                50: ('V[positions][modifiers][data]', 'Enhanced Output Waveform'),
            if command == self.command_name['Enhanced Output Waveform'] \
                and modifier in self.enhanced_output_waveform_type_valid_range:
                    return (0,'{:X}'.format(modifier))
##                51: ('V[positions][modifiers]', 'Cancel Enhanced Waveforms'),
            elif command == self.command_name['Cancel Enhanced Waveforms'] \
                and modifier == 0:
                    return (0,'0')
##                44: ('R[positions][modifiers]', 'Turn Off Existing Waveforms'),
            elif command == self.command_name['Turn Off Existing Waveforms'] \
                and modifier == 0:
                    return (0,'0')
##                72: ('i[positions][modifiers][data]', 'Generate N Pulses'),
            elif command == self.command_name['Generate N Pulses'] \
                and modifier in self.uint8_valid_range:
                    return (0,'{:02X}'.format(modifier))
        return ('E',-21)

    @utl.logger
    def verify_double_modifier(self,command,modifiers):
        """
        arguments which take two modifiers
        """
        if command == self.command_name['Set Output Waveform'] \
            and modifiers[0] in self.output_waveform_time_valid_range \
             and modifiers[1] in self.output_waveform_type_valid_range:
                return (0,'{:X}{:X}'.format(modifiers[0],modifiers[1]))
        return ('E',-21)
    
    @utl.logger
    def verify_modifiers(self,command,afmt,**kwargs):
        """
        not all commands require modifiers, so this function compares
        the optomux command format string and the kwargs to make sure
        modifiers is present in both.

        if present, the value or values must be checked for validity
        as the number and range differ from command to command
        """
        # optomux command format contains 'modifiers'
        if 'modifiers' in afmt:
            if 'modifiers' in kwargs:
                if isinstance(kwargs['modifiers'],tuple):
                    if len(kwargs['modifiers']) == 1:
                        return self.verify_single_modifier(command,kwargs['modifiers'][0])
                    elif len(kwargs['modifiers']) == 2:
                        return self.verify_double_modifier(command,kwargs['modifiers'])
                elif isinstance(kwargs['modifiers'],str) \
                    or isinstance(kwargs['modifiers'],int):
                        return self.verify_single_modifier(command,kwargs['modifiers'])
                return ('E',-23)
        return (0,'')
    
    @utl.logger
    def verify_single_data_value(self,command,value):
        """
        convert a single data value to a string appropriate for the command number
        """
##        2: ('C[data]', 'Set Turnaround Delay'),
        if command == self.command_name['Set Turnaround Delay']\
            and value in self.turnaround_delay_valid_range:
                return (0,'{:X}'.format(value))
##        3: ('D[data]', 'Set Digital Watchdog'),
        elif command == self.command_name['Set Digital Watchdog']\
            and value in self.digital_watchdog_delay_valid_range:
                return (0,'{:X}'.format(value))
##        4: ('E[data]', 'Set Protocol'),
        elif command == self.command_name['Set Protocol']\
            and value in self.protocol_passes_valid_range:
                return (0,'{:X}'.format(value))
##        25: ('Z[positions][modifiers][data]', 'Set Time Delay'),
        elif command == self.command_name['Set Time Delay']\
            and value in self.set_time_delay_valid_range:
                return (0,'{:X}'.format(value))
##        35: ('J[positions][data]', 'Write Analog Outputs'),
        elif command == self.command_name['Write Analog Outputs']\
            and value in self.analog_value_valid_range:
                return (0,'{:03X}'.format(value))
##        38: ('M[positions][data]', 'Average and Read Input'),
        elif command == self.command_name['Average and Read Input']\
            and value in self.number_of_averages_valid_range:
                return (0,'{:02X}'.format(value))
##        45: ('D[positions][data]', 'Set Analog Watchdog'),
        elif command == self.command_name['Set Analog Watchdog']:
            if value in self.analog_watchdog_delay_valid_range:
                return [0,'{:X}'.format(value)]
            else:
                print('Set Analog Watchdog',value)
##        47: ('T[positions][data]', 'Start Averaging Inputs'),
        elif command == self.command_name['Start Averaging Inputs']\
            and value in self.number_of_averages_valid_range:
                return (0,'{:02X}'.format(value))
##        71: ('m[positions][data]', 'Set Enhanced Digital Watchdog'),
        elif command == self.command_name['Set Enhanced Digital Watchdog']\
            and value in self.enhanced_digital_watchdog_valid_range:
                return (0,'{:X}'.format(value))
##        72: ('i[positions][modifiers][data]', 'Generate N Pulses'),
        elif command == self.command_name['Generate N Pulses']\
             and value in self.generate_pulses_n_valid_range:
                return (0,'{:X}'.format(value))
##        73: ('k[positions][data]', 'Start On Pulse'),
        elif command == self.command_name['Start On Pulse']\
             and value in self.start_on_off_pulse_valid_range:
                return (0,'{:X}'.format(value))
##        74: ('l[positions][data]', 'Start Off Pulse'),
        elif command == self.command_name['Start Off Pulse']\
             and value in self.start_on_off_pulse_valid_range:
                return (0,'{:X}'.format(value))
##        75: ('n[data]', 'Set Timer Resolution'),
        elif command == self.command_name['Set Timer Resolution']\
             and value in self.uint8_valid_range:
                return (0,'{:02X}'.format(value))
##        76: ('k[positions][data]', 'Set Temperature Probe Type'),
        elif command == self.command_name['Set Temperature Probe Type']\
             and value in self.temperature_probe_type_valid_range:
                return (0,'{:X}'.format(value))
        return ('E',-22)

    @utl.logger
    def verify_list_of_data_values(self,command,values):
        """
        commands which require multiple values in the info array but are
        shared by all points in the position mask
        """
##        26: ('Z[positions]L[data]', 'Initiate Square Wave'),
        if command == self.command_name['Initiate Square Wave']\
            and values[0] in range(256) \
            and values[1] in range(256):
                return [0,'{:02X}{:02X}'.format(values[0],values[1])]          
##        39: ('N[positions][data]', 'Set Input Range'),
        elif command == self.command_name['Set Input Range']\
            and values[0] in self.analog_value_valid_range\
            and values[1] in self.analog_value_valid_range:
                return [0,'{:04X}{:04X}'.format(values[0],values[1])]          
            
##        43: ('R[positions][modifiers][data]', 'Set Output Waveform'),
        elif command == self.command_name['Set Output Waveform']\
             and values[0] in self.analog_value_valid_range\
             and values[1] in self.analog_value_valid_range:
            # allowing 12 bit value entry because it seems more
            # reasonable but command only uses the top 8 bits
            # thus the divide by 16
                return [0,'{:02X}{:02X}'.format(values[0]>>4,values[1]>>4)]          
##        50: ('V[positions][modifiers][data]', 'Enhanced Output Waveform'),
        elif command == self.command_name['Enhanced Output Waveform']\
             and values[0] in self.analog_value_valid_range\
             and values[1] in self.analog_value_valid_range\
             and values[2] in range(1,32768):
                return [0,'{:03X}{:03X}{:04X}'.format(\
                    values[0],values[1],values[2])]            
##        68: ('Z[positions]M[data]', 'High Resolution Square Wave'),
        elif command == self.command_name['High Resolution Square Wave']\
             and values[0] in self.uint8_valid_range\
             and values(1) in self.uint8_valid_range:
                return [0,'{:02x}{:02X}'.format(values[0],values[1])]          
        return ('E',-22)

    @utl.logger
    def verify_list_of_data_values_per_point(self,command,positions,values):
        """
        commands which output a value in the info array for each
        point in the positions mask are handled here    
        """
        # if a position mask
        if isinstance(positions,int):
            # make a tuple
            positions = self.positions_mask_to_tuple(positions)
        # if a single value
        if isinstance(values,int):
            # make a tuple
            values = (values,)
        # if there is a value for each position
        if len(positions) == len(values):
            v = ''
    ##        46: ('S[positions][data]', 'Update Analog Outputs'),
            if command == self.command_name['Update Analog Outputs']:
                for i in reversed(range(len(values))):
                    v += '{:03X}'.format(values[i])
                return [0,v]        
    ##        53: ('W[positions][data]', 'Set Offsets'),
            elif command == self.command_name['Set Offsets']:
                # the user's guide indicates the highest position index
                # is the first value returned, but in the Optoware info
                # array it looks like the lowest index value is first
                for i in reversed(range(len(values))):
                    v += '{:04X}'.format(values[i])
                return [0,v]        
    ##        56: ('Y[positions][data]', 'Set Gain Coefficients'),
            elif command == self.command_name['Set Gain Coefficients']:
                # the user's guide indicates the highest position index
                # is the first value returned, but in the Optoware info
                # array it looks like the lowest index value is first
                for i in reversed(range(len(values))):
                    v += '{:04X}'.format(values[i])
                return [0,v]        
    ##        78: ('m[positions][data]', 'Set Analog Watchdog Timeout'),
            elif command == self.command_name['Set Analog Watchdog Timeout']:
                # the user's guide indicates the highest position index
                # is the first value returned, but in the Optoware info
                # array it looks like the lowest index value is first
                for i in reversed(range(len(values))):
                    v += '{:03X}'.format(values[i])
                return [0,v]
        return ('E',-22)

    @utl.logger
    def verify_data(self,command,afmt,**kwargs):
        """
        not all commands require data, so this function compares
        the optomux command format string and the kwargs to make sure
        data is present in both.

        if present, the value or values must be checked for validity
        as the number and range differ from command to command
        """
        if 'data' in afmt:
            if 'data' in kwargs:
                # commands which take one data value
                if command in (2,3,4,25,35,38,45,47,71,72,73,74,75,76):
                    if isinstance(kwargs['data'],tuple)\
                       and len(kwargs['data']) == 1\
                       and isinstance(kwargs['data'],int):
                        return self.verify_single_data_value(command,kwargs['data'][0])
                    elif isinstance(kwargs['data'],int):
                        return self.verify_single_data_value(command,kwargs['data'])   
                # commands which take several data elements
                elif command in (26,39,43,50,68):
                    if isinstance(kwargs['data'],tuple):
                        return self.verify_list_of_data_values(command,kwargs['data'])   
                # commands which take one data value per position
                elif command in (46,53,56,78):
                    if isinstance(kwargs['data'],tuple):
                        return self.verify_list_of_data_values_per_point(\
                            command,kwargs['positions'],kwargs['data'])
                return ('E',-22)
        return (0,'')

    def parse_format_string(self,sfmt):
        """
        parse the optomux command format string
        example commands to be parsed are:
            12: ('M', 'Read On Off Status'),
            24: ('Y[positions]', 'Clear Counters'),
            25: ('Z[positions][modifiers][data]', 'Set Time Delay'),
            26: ('Z[positions]L[data]', 'Initiate Square Wave'),
            27: ('Z[positions]G', 'Turn Off Time Delay Square Wave'),
            73: ('k[positions][data]', 'Start On Pulse'),
            75: ('n[data]', 'Set Timer Resolution'),
        typical would be
            m.groups(1) = command char
            m.groups(2) = positions or data
            m.groups(3) = modifiers or data
            m.groups(4) = data
        """
        p = re.compile(\
            r'^([A-Za-o`])'\
            +'\[?(\w*)\]?'\
            +'\[?([GHIJKLM]|\w*)\]?'\
            +'\[?(\w*)\]?$')
        m = p.search(sfmt)
        
        # remove empty groups
        return list(filter(lambda x: x, m.groups()))
    
    @utl.logger
    def build_packet(self,address,command,**kwargs):
        """
        Purpose:
            Verifies the arguments passed based on the command, and appends
            the required fields as required.
        Parameters:
            address - optomux device address
            command - optoware command number
            **kwargs - command dependent values
                positions - an int or tuple of points to be changed
                modifiers - command specific modifier
                data - command dependent int or tuple of values
        Note:
        A single element tuple must have a comma, ie:
            (0,) is evaluated as a tuple but
            (0)  is evaluated as an int
        """
        pkt = '>'
        if address not in self.timer_resolution:
            self.timer_resolution[address] = 256

        try:
            # verify the address
            rtn = self.verify_address(address)
            if rtn[0] == 'E':
                utl.log_error_message('verify_address failed')
                raise ValueError(rtn[1])
            pkt += rtn[1]

            # verify the command
            rtn = self.verify_command(command)
            if rtn[0] == 'E':
                utl.log_error_message('verify_command failed')
                raise ValueError(rtn[1])
            pkt += rtn[1]
            
            # use the command value to get the optomax command format string
            sfmt = self.commands[command][0]
            afmt = self.parse_format_string(sfmt)
            
            # verify the positions
            rtn = self.verify_positions(command,afmt,**kwargs)
            if rtn[0] == 'E':
                utl.log_error_message('verify_positions failed')
                raise ValueError(rtn[1])            
            pkt += rtn[1]
            
            # verify the modifiers
            rtn = self.verify_modifiers(command,afmt,**kwargs)
            if rtn[0] == 'E':
                utl.log_error_message('verify_modifiers failed')
                raise ValueError(rtn[1])           
            pkt += rtn[1]
            
            # verify the data
            rtn = self.verify_data(command,afmt,**kwargs)
            if rtn[0] == 'E':
                utl.log_error_message('verify_data failed')
                raise ValueError(rtn[1])
            pkt += rtn[1]
            pkt += '{:02X}\r'.format(self.compute_pkt_checksum(pkt)) # '??\r'

            return (0,pkt)
        
        except Exception as ex:
            return rtn
            
    def get_response_timeout(self,address):
        """
        Compute a response timeout so read won't hang forever.
        It is based on the time at the current baudrate of
        sending the longest command and receiving the longest
        response, plus the 'Set Turnaround Delay' time plus
        a 10ms buffer starting at the current perf_counter
        reading.
        """
        if address not in self.turnaround_delay:
            self.turnaround_delay[address] = 0
        # likely the longest response packet
        # would be reading 16 analog values
        max_opto_msg = \
            'A' + \
            '00000000000000000000000000000000000' + \
            '00000000000000000000000000000000000' + \
            '\r'
        ta_secs = [0,0.010,0.100,0.500]
        # time required to send two max sized messages
        # could scope this
        # 2 * secs/char * len(maxmsg)
        # + turnaround delay setting
        # + current perf_counter
        # + 10 ms fudge factor
        to = 2 * 10/self.tty.baud * len(max_opto_msg) \
            + ta_secs[self.turnaround_delay[address]] \
            + perf_counter() \
            + 0.010
        return to

    @utl.logger
    def send_receive(self,address,command,**kwargs):
        """
        Build the Optomux ASCII packet from the args.  Flush the
        input buffer because the only way to sync a response to a
        command is that it immediately follows.  Then write the
        packet over the serial port, start a response timer,and
        collect input until we either timeout or receive a valid
        response packet.
        """
        pkt = self.build_packet(address,command,**kwargs)
        if pkt[0] != 'E':
            rxPkt = ''
            # start with a clean slate
            self.tty.flush_input_buffer()
            # send the bytes
            tb = perf_counter()
            self.tty.write(pkt[1])
            # while response timer running
            timeout = self.get_response_timeout(address)
            utl.log_info_message('Response timeout is {:f} secs'.format(timeout-perf_counter()))
            while perf_counter() < timeout:
                # if there are bytes waiting to be read
                n = self.tty.rx_bytes_available()
                if n > 0:
                    # read all of them
                    s = self.tty.read(n)
                    rxPkt += s
                    # if an optomux response is complete
                    if rxPkt.startswith(('A','N')) and rxPkt.endswith('\r'):
                        rsp = (rxPkt[0:1],rxPkt)
                        utl.log_info_message('Response time took {:f} secs'.format(perf_counter()-tb))
                        return self.process_response(command,rsp[1])
            else:
                # turnaround timeout
                err = -29
                utl.log_error_message(self.errors[err])
                return ('E',err)
        else:
            # parrot caller's error
            err = -pkt[1]
            utl.log_error_message(self.errors[err])
            return pkt

    @utl.logger
    def process_response(self,command,rsp):
        """
        Check the response against the command that supposidly
        initiated it and parse the returned data.
        """
        if rsp.startswith('A'):
            if rsp == 'A\r':
                return ('A',0)
            else:
                data = rsp[1:-3]
                chks = int(rsp[-3:-1],16)
                if self.compute_data_checksum(data) == chks:
                    if command in [5,64,66,67]:
                        return ('A',int(rsp[1:-3],16))
                    elif command in [12,16,17,31,48,70]:
                        return ('A',self.optomux_data_to_binary_tuple(rsp[1:-3]))
                    # counters use full 16 bits
                    elif command in [22,23,32,33]:
                        return ('A',self.optomux_data_to_counter_tuple(rsp[1:-3]))
                    elif command in [37,38,49,58,60,61,63]:
                        return ('A',self.optomux_data_to_analog_input_tuple(rsp[1:-3]))
                    # reading temperatures
                    elif command in [77,79]:
                        return ('A',self.optomux_data_to_temperature_tuple(rsp[1:-3]))
                    # reading back what we sent to an analog output
                    elif command in [36]:
                        return ('A',self.optomux_data_to_analog_output_tuple(rsp[1:-3]))
                    elif command in [40,41]:
                        # opto returns HHHHLLLL nibbles
                        rsp = (0,self.optomux_data_to_tuple(rsp[1:-3]))
                        # rsp[1][0:16]  contains hi limit violation flags
                        # rsp[1][16:32] contains lo limit violation flags
                        # the values are massaged to:
                        #   0 - no limit violation
                        #   1 - lo limit violation
                        #   2 - hi limit violation
                        #   3 - hi and lo limit violation
                        hilo = tuple(rsp[i]<<1|rsp[i+16] for i in range(16))
                        # change the answer
                        return ('A',hilo)
                    elif command in [80]:
                        # B1, B2, E1, E2'ish
                        # A07/05/05*B9
                        if rsp[3] == '/':
                            return ('A',rsp[1,-3])
                        # B3000'ish
                        # A811609019911050100300000B7
                        else:
                            return ('A',rsp[5:7]+'/'+rsp[7:9]+'/'+rsp[9:11])            


                else:
                    err = -31
                    utl.log_error_message(self.errors[err])
                    return ('E',err)
        elif rsp.startswith('N'):
            err = -int(rsp[1:-1],16)
            utl.log_error_message(self.errors[err])
            return ('N',err)
        else:
            return rsp
            
##        0: ('A', 'Power Up Clear'),
    @utl.logger
    def power_up_clear(self,address):
        """
        Purpose:
            Provide a command to clear the Power-Up Clear Expected error.

        Parameters:
            address - optomux controller address in range(256)

        Description:
            On powerup of a device, a Power-Up Clear Expected error 'N00\r'
            will be returned in response to the first Optomux command sent.
            There is no harm in sending the 'Power Up Clear' command first
            if one knows that a power loss has occured.

            Once the device has sent the 'N00\r' error, it responds normally
            to further commands.  Just resend the rejected command.

            This command has NO effect on the Optomux unit’s operation or
            setup—the Power-up Clear Expected error provides an indication
            to the host that there has been a power failure and that Optomux
            has been reset to power-up conditions (see page 44).
        """
        cmd = self.command_name['Power Up Clear']
        return self.send_receive(address,cmd)
    
##        1: ('B', 'Reset'),
    @utl.logger
    def reset(self,address):
        """
        Purpose:
            Resets the Optomux unit to power-up conditions.

        Parameters:
            address - optomux controller address in range(256)

        Description:
            Digital device conditions on reset:        
                All outputs turned off
                All points then configured as inputs
                Protocol as set by jumper B10 (not on B3000)
                Watchdog timer disabled
                Turnaround delay = 0
                Counters/duration timers cancelled
                Latches cleared
                Timer resolution = 10 ms. 0 scale written to all output points
            Analog device conditions on reset:
                All points then configured as inputs
                Protocol as set by jumper B10 (not on B3000)
                Watchdog timer disabled
                Turnaround delay = 0
                All offsets set to 0
                All gain coefficients set to 1
                All averaging cancelled
                All temperature probe types cancelled
        
        NOTE:
            After using a Reset command, the Optomux application should wait
            before trying to communicate the reset device.
                For a B1, E1, B2, or E2 brain board, wait about 100ms.
                For a B3000, wait about 800 ms.

            If you are using a B3000 brain, a Reset command affects all four
            virtual addresses within the brain; you cannot reset just one.
            After sending a Reset command to a B3000 brain, you must send a
            Power-Up Clear command. Otherwise, the brain responds with an error.            
        """
        cmd = self.command_name['Reset']
        return self.send_receive(address,cmd)
        
##        2: ('C[data]', 'Set Turnaround Delay'),
    @utl.logger
    def set_turnaround_delay(self,address,data=0):
        """
        Purpose:
            Allow the host to tell the Optomux unit to delay
            before responding to commands sent from host.
            This command is helpful in some half-duplex radio
            modem systems.

        Parameters:
            address - optomux controller address in range(256)
            data - one of 4 delays in range(len(turnaround_delay))
                turnaround_delay = {
                    0:'No Delay',
                    1:'10 ms',
                    2:'100 ms',
                    3:'500 ms'
                    }

        Description:
            If no delay is specified, delay = 0 is assumed.
            On power-up, delay = 0.
        """
        kwargs = {'data':data}
        cmd = self.command_name['Set Turnaround Delay']
        rsp = self.send_receive(address,cmd,**kwargs)
        # change local setting if command has been acked
        if rsp[0] == 'A':
            self.turnaround_delay[address] = data
        return rsp

##        3: ('D[data]', 'Set Digital Watchdog'),
    @utl.logger
    def set_digital_watchdog(self,address,positions,action=0):
        """
        Purpose:
            Instructs a digital Optomux unit to monitor activity on the
            communications link and to take a predetermined action if
            there is no activity within a specified time. No activity
            means no activity of any kind on the serial link, or no
            communication with this brain board on the Ethernet link.

        Parameters:
            address - optomux device address in range(256)
            positions - tuple of up to 16 ints in range(16),
                    or a unit16 mask in range(65536)
        action - int in range(8) corresponding to these preset actions
            0 -- Watchdog disabled
            1 10 seconds Turn all outputs OFF
            2 1 minute Turn all outputs OFF
            3 10 minutes Turn all outputs OFF
            4 -- Watchdog disabled
            5 10 seconds Turn output 0 on, all other outputs OFF
            6 1 minute Turn output 0 on, all other outputs OFF
            7 10 minutes Turn output 0 on, all other outputs OFF
        
        Notes:
            Watchdog is disabled on power-up.
            The Optomux unit will respond with 'N06\r' to the first command
            following a watchdog timeout as a warning to let the host know a
            watchdog timeout occurred, unless the command is a'Power Up Clear'
            which get's an 'A\r'.
        """
        kwargs = {
            'positions':positions,
            'data':action
            }
        cmd = self.command_name['Set Digital Watchdog']        
        return self.send_receive(address,cmd,**kwargs)

##        4: ('E[data]', 'Set Protocol'),
    @utl.logger
    def set_protocol(self,address,protocol=0):
        """
        Purpose:
            Some Optomux boards allow a diagnostic 4 pass communications mode.
            In the 4 pass mode, the brain replaces the leading '>' with a 'A'
            in the command and returns it.  The host knows the command made it
            through and can then send an 'E' command for execute.

        Parameters:
            address - optomux device address in range(256)
            protocol - in range (2)
                0 2-pass protocol
                1 4-pass protocol

        Notes:
            4-pass is used for diagnostics only.
            Does not apply to B3000 Optomux units.
        """
        kwargs = {
            'data':protocol
            }
        cmd = self.command_name['Set Protocol']        
        return self.send_receive(address,cmd,**kwargs)

##        5: ('F', 'Identify Optomux Type'),
    @utl.logger
    def identify_optomux_type(self,address):
        """
        Purpose:
            Instructs the Optomux unit to identify itself as digital or analog.
            
        Parameters:
            address - device address in range(256)

        Returns:
            ('A',0) = digital
            ('A',1) = analog

        B3000 Example:
            >>> on.identify_type(0)
            ('A', 0)
            >>> on.identify_type(2)
            ('A', 1)

        """
        cmd = self.command_name['Identify Optomux Type']
        return self.send_receive(address,cmd)
    
##        6: ('G[positions]', 'Configure Positions'),
    @utl.logger
    def configure_positions(self,address,positions):
        """
        Purpose:
            Points to function as outputs. Points not specified
            here are configured as inputs.
            
        Parameters:
            address - optomux device address in range(256)
            positions - mask or tuple of positions to be made outputs
        
        Remarks:
            If the configuration for any point is changed by this command,
            then any time delay, latch, etc. is cleared. On power up, all
            points are configured as inputs.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Configure Positions']        
        return self.send_receive(address,cmd,**kwargs)

##        7: ('H[positions]', 'Configure As Inputs'),
    @utl.logger
    def configure_as_inputs(self,address,positions):
        """
        Purpose:
            Positions array contains points to function as inputs.
            Points not specified are left unchanged.

        Parameters:
            address - optomux device address in range(256)
            positions - mask or tuple of positions to be made inputs
        
        Remarks:
            If the configuration for any point is changed by this command,
            then any time delay, latch, etc. is cleared. On power up, all
            points are configured as inputs.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Configure As Inputs']        
        return self.send_receive(address,cmd,**kwargs)

##        8: ('I[positions]', 'Configure As Outputs'),
    @utl.logger
    def configure_as_outputs(self,address,positions):
        """
        Purpose:
            Positions array contains points to function as outputs.
            Points not specified are left unchanged.

        Parameters:
            address - optomux device address in range(256)
            positions - mask or tuple of positions to be made outputs
        
        Remarks:
            If the configuration for any point is changed by this command,
            then any time delay, latch, etc. is cleared. On power up, all
            points are configured as inputs.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Configure As Outputs']        
        return self.send_receive(address,cmd,**kwargs)

##        9: ('J[positions]', 'Write Digital Outputs'),
    @utl.logger
    def write_digital_outputs(self,address,positions=None):
        """
        Purpose:
            Points specified in positions array are turned on,
            those not specified are turned off.

        Parameters:
            address - optomux device address in range(256)
            positions - mask or tuple of positions to be turned on
        
        Remarks:
            Time delays, if set, are implemented when this command is
            executed. Points that have been configured to function as
            inputs are not affected by this command.

        Note:
            Optomux manual says all outputs are turned on if positions
            are not specified.  Here positions=None is used for that.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Write Digital Outputs']        
        return self.send_receive(address,cmd,**kwargs)

##        10: ('K[positions]', 'Activate Digital Outputs'),
    @utl.logger
    def activate_digital_outputs(self,address,positions=None):
        """
        Purpose:
            Points specified in positions array are turned on,
            those not specified are unaffected.

        Parameters:
            address - optomux device address in range(256)
            positions - mask or tuple of positions to turned on
        
        Remarks:
            Time delays, if set, are implemented when this command is
            executed. Points that have been configured to function as
            inputs are not affected by this command.

        Note:
            Optomux manual says all outputs are turned on if positions
            are not specified.  Here a positions=None is used for that.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Activate Digital Outputs']        
        return self.send_receive(address,cmd,**kwargs)

##        11: ('L[positions]', 'Deactivate Digital Outputs'),
    @utl.logger
    def deactivate_digital_outputs(self,address,positions=None):
        """
        Purpose:
            Points specified in positions array are turned off,
            those not specified are unaffected.

        Parameters:
            address - optomux device address in range(256)
            positions - mask or tuple of positions to turned off
        
        Remarks:
            Time delays, if set, are implemented when this command is
            executed. Points that have been configured to function as
            inputs are not affected by this command.

        Note:
            Optomux manual says all outputs are turned off if positions
            are not specified.  Here a positions=None is used for that.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Deactivate Digital Outputs']        
        return self.send_receive(address,cmd,**kwargs)

##        12: ('M', 'Read On Off Status'),
    @utl.logger
    def read_on_off_status(self,address):
        """
        Purpose:
            Returns the current on/off status of all 16 points.

        Parameters:
            address: optomux device address in range(256)
            
        Returns:
            A tuple with a 1 (on) or 0 (off) in each position.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.read_on_off_status(0)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.activate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_on_off_status(0)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

        """
        cmd = self.command_name['Read On Off Status']        
        return self.send_receive(address,cmd)
    
##        13: ('N[positions]', 'Set Latch Edges'),
    @utl.logger
    def set_latch_edges(self,address,positions):
        """
        Purpose:
            All points specified in positions will latch ON-to-OFF, others
            are OFF-to_ON.
            
        Parameters:
            address - optomux device address in range(256)
            Positions: Input points to latch on ON-to-OFF transitions.
                All other points remain unchanged.

        Description:
            The default on power up is OFF-to-ON.
            Points configured as outputs are not affected by this command.
            On power up, all points are set to latch on OFF-to-ON transitions.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.read_on_off_status(0)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.set_latch_edges(4,12)
            ('A', 0)
            >>> on.activate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_on_off_status(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.deactivate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_on_off_status(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Set Latch Edges']        
        return self.send_receive(address,cmd,**kwargs)
    
##        14: ('O[positions]', 'Set Off To On Latches'),
    @utl.logger
    def set_off_to_on_latches(self,address,positions):
        """
        Purpose:
            Sets input points to latch on OFF-to-ON transitions.

        Parameters:
            address - optomux device address in range(256)
            Positions: Input points to latch on OFF-to-ON transitions.
                All other points remain unchanged.
                
        Description:
            On power up, all points are set to latch OFF-to-ON transitions.
            Points configured as outputs are not affected by this command.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.clear_latches(4)
            ('A', 0)
            >>> on.set_on_to_off_latches(4,15)
            ('A', 0)
            >>> on.set_off_to_on_latches(4,12)
            ('A', 0)
            >>> on.read_latches(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.activate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_on_off_status(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.deactivate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_on_off_status(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))       
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Set Off To On Latches']        
        return self.send_receive(address,cmd,**kwargs)

##        15: ('P[positions]', 'Set On To Off Latches'),
    @utl.logger
    def set_on_to_off_latches(self,address,positions):
        """
        Purpose:
            Sets input points to latch on ON-to-OFF transitions.
            
        Parameters:
            address - optomux device address in range(256)
            Positions: Input points to latch on ON-to-OFF transitions.
                All other points remain unchanged.
                
        Description:
            On power up, all points are set to latch OFF-to-ON transitions.
            Points configured as outputs are not affected by this command.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.set_on_to_off_latches(4,3)
            ('A', 0)
            >>> on.clear_latches(4)
            ('A', 0)
            >>> on.read_latches(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.activate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_on_off_status(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.deactivate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_on_off_status(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Set On To Off Latches']        
        return self.send_receive(address,cmd,**kwargs)

##        16: ('Q', 'Read Latches'),
    @utl.logger
    def read_latches(self,address):
        """
        Purpose:
            Returns data indicating which of the inputs have latched.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input points to latch on ON-to-OFF transitions.
                All other points remain unchanged.

        Description:
            This command does not clear the latches. Subsequent
            Read Latches commands will return the same results.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.activate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_latches(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

        """
        cmd = self.command_name['Read Latches']        
        return self.send_receive(address,cmd)

##        17: ('R[positions]', 'Read and Clear Latches'),
    @utl.logger
    def read_and_clear_latches(self,address,positions=None):
        """
        Purpose:
            Returns data indicating which inputs have latched and
            then resets specified latches.

        Parameters:
            address - optomux device address in range(256)
            positions - Input points to latch on ON-to-OFF transitions.
                All other points remain unchanged.

        Description:
            This command returns the latch status for all points.
            It clears latches only for the specified points. All
            other latches remain unchanged.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.activate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_latches(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_and_clear_latches(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.read_latches(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read and Clear Latches']        
        return self.send_receive(address,cmd,**kwargs)

##        18: ('S[positions]', 'Clear Latches'),
    @utl.logger
    def clear_latches(self,address,positions=None):
        """
        Purpose:
            Clear latches specified inpositions array.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input latches to clear.

        Description:
            This command clears the specified input latches.
            All other latches remain unchanged.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.read_latches(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.activate_digital_outputs(0,15)
            ('A', 0)
            >>> on.read_latches(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.clear_latches(4)
            ('A', 0)
            >>> on.read_latches(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Clear Latches']        
        return self.send_receive(address,cmd,**kwargs)
    
##        19: ('T[positions]', 'Start and Stop Counters'),
    @utl.logger
    def start_and_stop_counters(self,address,positions):
        """
        Purpose:
            Positions specifies counters to be enabled.
            Others are disabled.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input counters to enable

        Description:
            This command has no effect on the stored count.
            Counting can start or resume at any time. The
            maximum count is 65,535; after that, the count
            resets to 0.

        Notes:
            Frequencies up to 400 Hz (50% duty cycle, minimum
            pulse width of 1.25 milliseconds) can be counted.
            
            Using the Generate N Pulses command 72 (i) will
            degrade the maximum counting frequency to about
            350 Hz.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.start_and_stop_counters(4,0x0005)
            ('A', 0)
            >>> on.generate_n_pulses(0,15,25,10)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (10, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.start_and_stop_counters(4,0x000A)
            ('A', 0)
            >>> on.generate_n_pulses(0,15,25,15)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (10, 15, 10, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Start and Stop Counters']        
        return self.send_receive(address,cmd,**kwargs)
    
##        20: ('U[positions]', 'Start Counters'),
    @utl.logger
    def start_counters(self,address,positions):
        """
        Purpose:
            Positions specifies counters to be enabled.
            Others are unaffected.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input counters to enable.

        Description:
            This command has no effect on the stored count.
            Counting can start or resume at any time. The
            maximum count is 65,535; after that, the count
            resets to 0.

        Notes:
            Frequencies up to 400 Hz (50% duty cycle, minimum
            pulse width of 1.25 milliseconds) can be counted.
            
            Using the Generate N Pulses command 72 (i) will
            degrade the maximum counting frequency to about
            350 Hz.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.start_counters(4,0x0005)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.generate_n_pulses(0,15,25,10)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (10, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.start_counters(4,0x000A)
            ('A', 0)
            >>> on.generate_n_pulses(0,15,25,10)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (20, 10, 20, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Start Counters']        
        return self.send_receive(address,cmd,**kwargs)
    
##        21: ('V[positions]', 'Stop Counters'),
    @utl.logger
    def stop_counters(self,address,positions):
        """
        Purpose:
            Positions specifies counters to be disabled.
            Others are unaffected.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input counters to disable.

        Description:
            This command has no effect on the stored count.
            Counting can start or resume at any time. The
            maximum count is 65,535; after that, the count
            resets to 0.

        Notes:
            Frequencies up to 400 Hz (50% duty cycle, minimum
            pulse width of 1.25 milliseconds) can be counted.
            
            Using the Generate N Pulses command 72 (i) will
            degrade the maximum counting frequency to about
            350 Hz.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.start_counters(4,15)
            ('A', 0)
            >>> on.generate_n_pulses(0,15,25,10)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.stop_counters(4,0x000a)
            ('A', 0)
            >>> on.generate_n_pulses(0,15,25,10)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (20, 10, 20, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Stop Counters']        
        return self.send_receive(address,cmd,**kwargs)
    
##        22: ('W[positions]', 'Read Counters'),
    @utl.logger
    def read_counters(self,address,positions=None):
        """
        Purpose:
            Positions specifies counters to be read.
            Counts are unaffected.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input counters to read.

        Description:
            This command has no effect on the stored count.
            Counting can start or resume at any time. The
            maximum count is 65,535; after that, the count
            resets to 0.

        Notes:
            Frequencies up to 400 Hz (50% duty cycle, minimum
            pulse width of 1.25 milliseconds) can be counted.
            
            Using the Generate N Pulses command 72 (i) will
            degrade the maximum counting frequency to about
            350 Hz.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.start_counters(4,15)
            ('A', 0)
            >>> on.generate_n_pulses(0,15,25,10)
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read Counters']        
        return self.send_receive(address,cmd,**kwargs)
    
##        23: ('X[positions]', 'Read and Clear Counters'),
    @utl.logger
    def read_and_clear_counters(self,address,positions=None):
        """
        Purpose:
            Positions specifies counters to be read then cleared.
            Other counts are unaffected.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input counters to read/clear.

        Description:
            This command has no effect on the stored count.
            Counting can start or resume at any time. The
            maximum count is 65,535; after that, the count
            resets to 0.

        Notes:
            Frequencies up to 400 Hz (50% duty cycle, minimum
            pulse width of 1.25 milliseconds) can be counted.
            
            Using the Generate N Pulses command 72 (i) will
            degrade the maximum counting frequency to about
            350 Hz.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.start_counters(4,(0,1,2,3))
            ('A', 0)
            >>> on.generate_n_pulses(0,(0,1,2,3),25,10)
            ('A', 0)
            >>> on.read_and_clear_counters(4,(0,1,2,3))
            ('A', (10, 10, 10, 10))
            >>> on.read_counters(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read and Clear Counters']        
        return self.send_receive(address,cmd,**kwargs)

##        24: ('Y[positions]', 'Clear Counters'),
    @utl.logger
    def clear_counters(self,address,positions=None):
        """
        Purpose:
            Positions specifies counters to be cleared.
            Other counts are unaffected.
            
        Parameters:
            address - optomux device address in range(256)
            positions - Input counters to clear.

        Description:
            This command has no effect on the stored count.
            Counting can start or resume at any time. The
            maximum count is 65,535; after that, the count
            resets to 0.

        Notes:
            Frequencies up to 400 Hz (50% duty cycle, minimum
            pulse width of 1.25 milliseconds) can be counted.
            
            Using the Generate N Pulses command 72 (i) will
            degrade the maximum counting frequency to about
            350 Hz.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.start_counters(4,(0,1,2,3))
            ('A', 0)
            >>> on.generate_n_pulses(0,(0,1,2,3),25,10)
            ('A', 0)
            >>> on.read_counters(4,(0,1,2,3))
            ('A', (10, 10, 10, 10))
            >>> on.clear_counters(4,(0,1,2,3))
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Clear Counters']        
        return self.send_receive(address,cmd,**kwargs)

##        25: ('Z[positions][modifiers][data]', 'Set Time Delay'),
    @utl.logger
    def pulse_on(self,address,positions,ticks):
        """
        Purpose:
            Arm the outputs specified in positions to generate a single
            on pulse of duration ticks * 10ms * timer resolution
            when the output is next activated.

        Parameters:
            address - optomux device address in range(256)
            positions - Outputs to pulse on
            ticks - number of timer ticks
            
        Description:
            Modifier in optomux ASCII command is 'H'.
            Desired time = delay length × timer resolution × 10 ms
            Valid delay lengths are 0 through 65,535. A 0 value is
            equal to a delay length of 65,535 (FFFF).
        """
        kwargs = {
            'positions':positions,
            'modifiers':'H',
            'data':ticks
            }
        cmd = self.command_name['Set Time Delay']        
        return self.send_receive(address,cmd,**kwargs)

    @utl.logger
    def delay_on(self,address,positions,ticks):
        """
        Purpose:
            Arm the outputs specified in positions to delay the next
            OFF-to-ON request by ticks * 10ms * timer resolution.

        Parameters:
            address - optomux device address in range(256)
            positions - Outputs to delat
            ticks - number of timer ticks
            
        Description:
            Modifier in optomux ASCII command is 'I'.
            Desired time = delay length × timer resolution × 10 ms
            Valid delay lengths are 0 through 65,535. A 0 value is
            equal to a delay length of 65,535 (FFFF).
        """
        kwargs = {
            'positions':positions,
            'modifiers':'I',
            'data':ticks
            }
        cmd = self.command_name['Set Time Delay']        
        return self.send_receive(address,cmd,**kwargs)

    @utl.logger
    def pulse_off(self,address,positions,ticks):
        """
        Purpose:
            Arm the outputs specified in positions to generate a single
            off pulse of duration ticks * 10ms * timer resolution
            when the output is next activated.

        Parameters:
            address - optomux device address in range(256)
            positions - Outputs to pulse on
            ticks - number of timer ticks
            
        Description:
            Modifier in optomux ASCII command is 'J'.
            Desired time = delay length × timer resolution × 10 ms
            Valid delay lengths are 0 through 65,535. A 0 value is
            equal to a delay length of 65,535 (FFFF).
        """
        kwargs = {
            'positions':positions,
            'modifiers':'J',
            'data':ticks
            }
        cmd = self.command_name['Set Time Delay']        
        return self.send_receive(address,cmd,**kwargs)

    @utl.logger
    def delay_off(self,address,positions,delay):
        """
        Purpose:
            Arm the outputs specified in positions to delay the next
            ON-to-OFF request by ticks * 10ms * timer resolution.

        Parameters:
            address - optomux device address in range(256)
            positions - Outputs to delat
            ticks - number of timer ticks
            
        Description:
            Modifier in optomux ASCII command is 'K'.
            Desired time = delay length × timer resolution × 10 ms
            Valid delay lengths are 0 through 65,535. A 0 value is
            equal to a delay length of 65,535 (FFFF).
        """
        kwargs = {
            'positions':positions,
            'modifiers':'K',
            'data':delay
            }
        cmd = self.command_name['Set Time Delay']        
        return self.send_receive(address,cmd,**kwargs)
        return rsp
    
##        26: ('Z[positions][modifiers][data]', 'Initiate Square Wave'),
    @utl.logger
    def initiate_square_wave(self,address,positions,on_ticks,off_ticks):
        """
        Purpose:
            Starts a continuous square wave at specified output points.

        Parameters:
            address - optomux device address in range(256)
            positions - points to output a square wave
            on_ticks  - numnber of timer ticks to stay on
            off_ticks - number of timer ticks to stay off
        
        Description:
            The square wave continues until it is turned off using
            command 27 (Z) or modified with Set Time Delay command
            25 (Z) on page 81 or High Resolution Square Wave command
            68 (Z) on page 85.

            Current timer resolution (Set Timer Resolution command 75)
            and values in the Info Array or [data] fields are used to
            calculate on and off times of the square wave, as described
            below.
            
            On time = timer resolution x 256 x first element value
            Off time = timer resolution x 256 x second element value.
            
            Values can be between 0 and 255 (0 = 256). Maximum for on
            and off times is 2.56 seconds x 256 x 256 (2796.20 minutes
            or 46.6 hours).

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.start_counters(4,15)
            ('A', 0)
            >>> on.initiate_square_wave(0,15,1,1)
            ('A', 0)
            >>> on.read_counters(4,15)
            ('A', (5, 5, 5, 5))
            >>> on.stop_counters(4,(0,2))
            ('A', 0)
            >>> on.read_counters(4)
            ('A', (8, 10, 8, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.turn_off_time_delay_or_square_wave(0,(1,3))
            ('A', 0)
            >>> on.read_counters(4,15)
            ('A', (8, 21, 8, 21))
            >>> on.read_counters(4,15)
            ('A', (8, 21, 8, 21))
            >>> on.start_counters(4,(0,2))
            ('A', 0)
            >>> on.read_counters(4,15)
            ('A', (12, 21, 12, 21))
            >>> on.turn_off_time_delay_or_square_wave(0,15)
            ('A', 0)
            >>> on.read_and_clear_counters(4,15)
            ('A', (16, 21, 16, 21))
            >>> on.read_counters(4,15)
            ('A', (0, 0, 0, 0))
        """
        cmd = self.command_name['Initiate Square Wave']        
        kwargs = {'positions':positions,
                  'modifiers':'L',
                  'data':(on_ticks,off_ticks)}
        return self.send_receive(address,cmd,**kwargs)
    
##        27: ('Z[positions][modifiers]', 'Turn Off Time Delay Square Wave'),
    @utl.logger
    def turn_off_time_delay_or_square_wave(self,address,positions):
        """
        Purpose:
            Turns off existing time delay on specified outputs.
            
        Parameters:
            address - optomux device address in range(256)
            positions - outputs which are to discontinue square wave

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.reset(4)
            ('A', 0)
            >>> on.start_counters(4,15)
            ('A', 0)
            >>> on.initiate_square_wave(0,15,1,1)
            ('A', 0)
            >>> on.read_counters(4,15)
            ('A', (5, 5, 5, 5))
            >>> on.turn_off_time_delay_or_square_wave(0,(1,3))
            ('A', 0)
            >>> on.read_counters(4,15)
            ('A', (9, 7, 9, 7))
            >>> on.read_counters(4,15)
            ('A', (11, 7, 11, 7))
            >>> on.turn_off_time_delay_or_square_wave(0,(0,2))
            ('A', 0)
            >>> on.read_counters(4,15)
            ('A', (14, 7, 14, 7))
            >>> on.read_counters(4,15)
            ('A', (14, 7, 14, 7))
        """
        kwargs = {'positions':positions,
                  'modifiers':'G'}
        cmd = self.command_name['Turn Off Time Delay Square Wave']        
        return self.send_receive(address,cmd,**kwargs)

##        28: ('a[positions]', 'Set Pulse Trigger Polarity'),
    @utl.logger
    def set_pulse_trigger_polarity(self,address,positions):
        """
        Purpose:
            Configure the specified inputs to measure the duration
            of the next ON pulse.  The others will measure off
            pulse duration.
        
        Parameters:
            address - optomux device address in range(256)
            positions - specified inputs measure ON time,
                others measure OFF time.

        Description:
            This command measures the duration of the first pulse of the
            appropriate level and stores the result to be recalled. As soon
            as a complete pulse is measured for a point, a pulse complete bit
            is set. Send a “Read Pulse Complete Bits” command to find out
            when the duration is done.
            
            The resolution for the duration counters is dependent upon the
            current timer resolution (see “Set Timer Resolution” on page 52).
            The default value of 10 ms allows you to measure a pulse of up
            to 10.92 minutes. At the lowest resolution (2.56 seconds as
            opposed to 0.01 seconds), you could measure a pulse of up to
            2,796.16 minutes (46.6 hours).
            
            This command does not clear preexisting duration counter values
            or pulse complete bits. If a point’s pulse complete bit has been
            previously set, no measurements are made until that point’s pulse
            complete bit and duration counter are cleared by sending either
            “Clear Duration Counters” or “Read and Clear Duration Counters”.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Set Pulse Trigger Polarity']        
        return self.send_receive(address,cmd,**kwargs)

##        29: ('b[positions]', 'Trigger On Positive Pulse'),
    @utl.logger
    def trigger_on_positive_pulse(self,address,positions):
        """
        Purpose:
            Configure the specified inputs to measure the duration
            of the next ON pulse.  The others are unaffected.
        
        Parameters:
            address - optomux device address in range(256)
            positions - specified inputs ON time is measured.

        Description:
            This command measures the duration of the first pulse of the
            appropriate level and stores the result to be recalled. As soon
            as a complete pulse is measured for a point, a pulse complete bit
            is set. Send a “Read Pulse Complete Bits” command to find out
            when the duration is done.
            
            The resolution for the duration counters is dependent upon the
            current timer resolution (see “Set Timer Resolution” on page 52).
            The default value of 10 ms allows you to measure a pulse of up
            to 10.92 minutes. At the lowest resolution (2.56 seconds as
            opposed to 0.01 seconds), you could measure a pulse of up to
            2,796.16 minutes (46.6 hours).
            
            This command does not clear preexisting duration counter values
            or pulse complete bits. If a point’s pulse complete bit has been
            previously set, no measurements are made until that point’s pulse
            complete bit and duration counter are cleared by sending either
            “Clear Duration Counters” or “Read and Clear Duration Counters”.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Trigger On Positive Pulse']        
        return self.send_receive(address,cmd,**kwargs)

##        30: ('c[positions]', 'Trigger On Negative Pulse'),
    @utl.logger
    def trigger_on_negative_pulse(self,address,positions):
        """
        Purpose:
            Configure the specified inputs to measure the duration
            of the next OFF pulse.  The others are unaffected.
        
        Parameters:
            address - optomux device address in range(256)
            positions - specified inputs ON time is measured.

        Description:
            This command measures the duration of the first pulse of the
            appropriate level and stores the result to be recalled. As soon
            as a complete pulse is measured for a point, a pulse complete bit
            is set. Send a “Read Pulse Complete Bits” command to find out
            when the duration is done.
            
            The resolution for the duration counters is dependent upon the
            current timer resolution (see “Set Timer Resolution” on page 52).
            The default value of 10 ms allows you to measure a pulse of up
            to 10.92 minutes. At the lowest resolution (2.56 seconds as
            opposed to 0.01 seconds), you could measure a pulse of up to
            2,796.16 minutes (46.6 hours).
            
            This command does not clear preexisting duration counter values
            or pulse complete bits. If a point’s pulse complete bit has been
            previously set, no measurements are made until that point’s pulse
            complete bit and duration counter are cleared by sending either
            “Clear Duration Counters” or “Read and Clear Duration Counters”.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Trigger On Negative Pulse']        
        return self.send_receive(address,cmd,**kwargs)

##        31: ('d', 'Read Pulse Complete Bits'),
    @utl.logger
    def read_pulse_complete_bits(self,address):
        """
        Purpose:
            Allows the host computer to determine which points
            have finished measuring pulse duration.

        Parameters:
            address - optomux device address in range(256)

        Description:
            When a duration measurement completes, a Pulse Complete bit
            is set which causes the duration counter to hold it's value.

            This command does not clear preexisting duration counter values
            or pulse complete bits. If a point’s pulse complete bit has been
            previously set, no measurements are made until that point’s pulse
            complete bit and duration counter are cleared by sending either
            “Clear Duration Counters” or “Read and Clear Duration Counters”.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.set_timer_resolution(0,1)
            ('A', 0)
            >>> on.set_timer_resolution(4,1)
            ('A', 0)
            >>> on.trigger_on_positive_pulse(4,15)
            ('A', 0)
            >>> on.clear_duration_counters(4,15)
            ('A', 0)
            >>> on.read_pulse_duration_counters(4,15)
            ('A', (0, 0, 0, 0))
            >>> on.start_on_pulse(0,15,100)
            ('A', 0)
            >>> on.read_pulse_complete_bits(4)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>> on.clear_duration_counters(4,15)
            ('A', 0)
            >>> on.read_pulse_complete_bits(4)
            ('A', (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            >>>
        """
        cmd = self.command_name['Read Pulse Complete Bits']        
        return self.send_receive(address,cmd)
    
##        32: ('e[positions]', 'Read Pulse Duration Counters'),
    @utl.logger
    def read_pulse_duration_counters(self,address,positions):
        """
        Purpose:
            Allows the host to read the current value of the pulse duration
            counters specified in positions.
        
        Parameters:
            address - optomux device address in range(256)
            positions - specified inputs whose duration counter is to be read.

        Description:
            Values are returned in the current timer resolution For example,
            a value of 2 equals a pulse length of: 2 x timer resolution ).
            Pulses up to 10.92 minutes can be timed with a resolution of 10 ms.
            If this command is used before the pulse is finished, the current
            duration is returned.
            
            If the final duration is desired, use the read_pulse_complete_bits
            command to poll the device.

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.set_timer_resolution(0,1)
            ('A', 0)
            >>> on.set_timer_resolution(4,1)
            ('A', 0)
            >>> on.trigger_on_positive_pulse(4,15)
            ('A', 0)
            >>> on.read_and_clear_pulse_duration_counters(4,15)
            ('A', (0, 0, 0, 0))
            >>> on.start_on_pulse(0,15,100)
            ('A', 0)
            >>> on.read_pulse_duration_counters(4,15)
            ('A', (99, 100, 99, 100))
            >>> on.clear_duration_counters(4,15)
            ('A', 0)
            >>> on.read_pulse_duration_counters(4,15)
            ('A', (0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read Pulse Duration Counters']        
        return self.send_receive(address,cmd,**kwargs)
    
##        33: ('f[positions]', 'Read and Clear Duration Counters'),
    @utl.logger
    def read_and_clear_pulse_duration_counters(self,address,positions):
        """
        Purpose:
            Allows the host to read/clear the current value of the pulse duration
            counters specified in positions.
        
        Parameters:
            address - optomux device address in range(256)
            positions - specified inputs whose duration counter is to be read/cleared.

        Description:
            Values are returned in the current timer resolution For example,
            a value of 2 equals a pulse length of: 2 x timer resolution ).
            Pulses up to 10.92 minutes can be timed with a resolution of 10 ms.
            If this command is used before the pulse is finished, the current
            duration is returned.
            
            If the final duration is desired, use the read_pulse_complete_bits
            command to poll the device.        

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.set_timer_resolution(0,1)
            ('A', 0)
            >>> on.set_timer_resolution(4,1)
            ('A', 0)
            >>> on.trigger_on_positive_pulse(4,15)
            ('A', 0)
            >>> on.read_and_clear_pulse_duration_counters(4,15)
            ('A', (0, 0, 0, 0))
            >>> on.start_on_pulse(0,15,100)
            ('A', 0)
            >>> on.read_pulse_duration_counters(4,15)
            ('A', (99, 100, 99, 100))
            >>> on.clear_duration_counters(4,15)
            ('A', 0)
            >>> on.read_pulse_duration_counters(4,15)
            ('A', (0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read and Clear Duration Counters']        
        return self.send_receive(address,cmd,**kwargs)

##        34: ('g[positions]', 'Clear Duration Counters'),
    @utl.logger
    def clear_duration_counters(self,address,positions):
        """
        Purpose:
            Allows the host to clear the current value of the pulse duration
            counters specified in positions.
        
        Parameters:
            address - optomux device address in range(256)
            positions - specified inputs whose duration counter is to be cleared.

        Description:
            Counters not specified are left unchanged.            

        Example:
            Address 0 outputs (0,1,2,3) wired to
            Address 4 inputs  (0,1,2,3)
            >>> on.set_timer_resolution(0,1)
            ('A', 0)
            >>> on.set_timer_resolution(4,1)
            ('A', 0)
            >>> on.trigger_on_positive_pulse(4,15)
            ('A', 0)
            >>> on.read_and_clear_pulse_duration_counters(4,15)
            ('A', (0, 0, 0, 0))
            >>> on.start_on_pulse(0,15,100)
            ('A', 0)
            >>> on.read_pulse_duration_counters(4,15)
            ('A', (99, 100, 99, 100))
            >>> on.clear_duration_counters(4,15)
            ('A', 0)
            >>> on.read_pulse_duration_counters(4,15)
            ('A', (0, 0, 0, 0))
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Clear Duration Counters']        
        return self.send_receive(address,cmd,**kwargs)

##        35: ('J[positions][data]', 'Write Analog Outputs'),
    @utl.logger
    def write_analog_outputs(self,address,positions,value):
        """
        Purpose:
            Writes a specified value to one or more analog outputs.

        Description:
            Writes the same value to all specified in outputs.
        """
        kwargs = {
            'positions':positions,
            'data':value
            }
        cmd = self.command_name['Write Analog Outputs']        
        return self.send_receive(address,cmd,**kwargs)

##        36: ('K[positions]', 'Read Analog Outputs'),
    @utl.logger
    def read_analog_outputs(self,address,positions):
        """
        Purpose:
            Reads values from specified outputs.

        Description:
            Response tuple contains data for specified outputs.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read Analog Outputs']        
        return self.send_receive(address,cmd,**kwargs)

##        37: ('L[positions]', 'Read Analog Inputs'),
    @utl.logger
    def read_analog_inputs(self,address,positions):
        """
        Purpose:
            Reads values from specified inputs.

        Description:
            Response tuple contains data for specified inputs.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read Analog Inputs']        
        return self.send_receive(address,cmd,**kwargs)

##        38: ('M[positions][data]', 'Average and Read Input'),
    @utl.logger
    def average_and_read_input(self,address,position,samples):
        """
        Purpose:
            Averages the value of a single point over a specified
            number of samples and returns the result.
            
        Description:
            This command returns a response only when it is finished
            averaging. If the number of samples is very large and the
            system is communicating serially, it can tie up the bus
            while waiting for an acknowledgment message. You can use
            Start Averaging Inputs, Read Average Complete Bits, and
            Read Averaged Inputs instead.
            .
            Averaging is done using a continuous running average with
            a sample rate of 100 milliseconds. After the number of samples
            has been reached, the value is returned to the host. The
            following equation shows how the average is calculated:
                Average = ((N-1) (Old Average) + (New Reading))/N

            Even though the positions argument may be a tuple which is
            not sorted, the averages are returned in ascending positions
            order.
        """
        if isinstance(position,list):
            position = position[0]
        kwargs = {
            'positions':position,
            'data':samples
            }
        cmd = self.command_name['Average and Read Input']        
        return self.send_receive(address,cmd,**kwargs)

##        39: ('N[positions][data]', 'Set Input Range'),
    @utl.logger
    def set_input_range(self,address,position,hi_limit,lo_limit):
        """
        Purpose:
            Defines the high and low limits for the specified input points.
            
        Parameters:
            address  - optomux device address in range(256)
            positions - points to read/clear
            hi_limit - hi limit latch trigger value
            lo_limit - lo limit latch trigger value

        Description:
            This command defines a range for the specified inputs. If an input
            is out of the specified range, one of two latches is set: high or
            low limit exceeded. To read these latches, send the command
            Read Out-of-Range Latches.

            All specified inputs are set to the same limits.
        """
        kwargs = {
            'positions':positions,
            'data':(hi_limit,lo_limit)
            }
        cmd = self.command_name['Set Input Range']        
        return self.send_receive(address,cmd,**kwargs)
    
##        40: ('O', 'Read Out Of Range Latches'),
    @utl.logger
    def read_out_of_range_latches(self,address):
        """
        Purpose:
            Returns the high and low out-of-range latches for all points.
            Remarks: Before using this command, send Set Input Range to
            set high and low imits for the points. This command does not
            clear the latches.
            
        Parameters:
            address - optomux device address in range(256)

        Returns:
            A tuple of limit flags for all points:
            0 = point has remained within limits or is an output.
            1 = low-limit latch has been set
            2 = high-limit latch has been set
            3 = low-limit and high-limit latches have been set     
        """
        cmd = self.command_name['Read Out Of Range Latches']        
        return self.send_receive(address,cmd)
    
##        41: ('P[positions]', 'Read and Clear Out Of Range Latches'),
    @utl.logger
    def read_and_clear_out_of_range_latches(self,address,positions):
        """
        Purpose:
            Reads/clears the high and low out-of-range latches
            for specified points.
            
        Parameters:
            address - optomux device address in range(256)
            positions - points to read/clear

        Description:
            Before using this command, send Set Input Range to set the
            value of high and low limits for the specified points. This
            command does not clear the latches.

        Returns:
            A tuple of limit flags for all points:
            0 = point has remained within limits or is an output.
            1 = low-limit latch has been set
            2 = high-limit latch has been set
            3 = both low- and high-limit latches have been set     
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read and Clear Out Of Range Latches']        
        return self.send_receive(address,cmd,**kwargs)

##        42: ('Q[positions]', 'Clear Out Of Range Latches'),
    @utl.logger
    def clear_out_of_range_latches(self,address,positions=None):
        """
        Purpose:
            Clears the high and low out-of-range latches for the
            specified points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - points to clear

        Description:
            Before using this command, send Set Input Range to set the
            value of high and low limits for the specified points.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Clear Out Of Range Latches']        
        return self.send_receive(address,cmd,**kwargs)
    
##        43: ('R[positions][modifiers][data]', 'Set Output Waveform'),
    @utl.logger
    def set_output_waveform(self,address,positions,rate,shape,hi_limit,lo_limit):
        """
        Purpose:
            Starts a constant waveform at specified output points.

        Parameters:
            address   - optomux device address in range(256)
            positions - points to generate waveform
            rate      - period of ramp or 1/2 period of
                        square wave or sawtooth
            shape     - waveform shape
            hi_limit  - max value of waveform
            lo_limit  - min value of waveform

        Description:
            The rate sets the period of a ramp one-half the period of
            a triangle or square wave and assume a full scale change.

            Valid waveform rates:
                0    Disable waveform
                1    2.18 minutes
                2    3.28 minutes
                3    4.37 minutes
                4    5.46 minutes
                5    6.56 minutes
                6    7.65 minutes
                7    8.74 minutes
                8    1.09 minutes
                9    32.8 seconds
                10   21.8 seconds
                11   16.4 seconds
                12   13.1 seconds
                13   10.9 seconds
                14   9.4  seconds
                15   8.2  seconds
                
            Valid waveform shapes:
                0    No waveform
                1    Triangle wave with positive initial slope
                2    Ramp up—waveform terminates upon reaching the upper limit
                3    Continuous ramp up
                4    Square wave (50 % duty cycle)
                5    Triangle wave with negative initial slope
                6    Ramp down—waveform terminates at lower limit
                7    Continuous ramp down
        """
        kwargs = {
            'positions':positions,
            'modifiers':(rate,shape),
            'data':(hi_limit,lo_limit)
            }
        cmd = self.command_name['Set Output Waveform']        
        return self.send_receive(address,cmd,**kwargs)
    
##        44: ('R[positions][modifiers]', 'Turn Off Existing Waveforms'),
    @utl.logger
    def turn_off_existing_waveform(self,address,positions):
        """
        Purpose:
            Turns off existing waveforms that were started with
            Set Output Waveforms

        Parameters:
            address   - optomux device address in range(256)
            positions - points having waveform canceled

        Description:
            Cancels waveforms that were started using 'Set Output Waveform'.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Turn Off Existing Waveforms']        
        return self.send_receive(address,cmd,**kwargs)

##        45: ('D[positions][data]', 'Set Analog Watchdog'),
    @utl.logger
    def set_analog_watchdog(self,address,positions,action):
        """
        Purpose:
            Instructs an analog Optomux unit to monitor activity on the
            communications link and to take a predetermined action if
            there is no activity within a specified time. No activity
            means no activity of any kind on the serial link, or no
            communication with this brain board on the Ethernet link.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - points having waveform canceled
            action - timeout and output action to be taken
            
        Description:
            Valid times and actions are:
            0 -- Watchdog disabled
            1 10 seconds Write zero-scale
            2 1 minute Write zero-scale
            3 10 minutes Write zero-scale
            4 -- Watchdog disabled
            5 10 seconds Write full-scale
            6 1 minute Write full-scale
            7 10 minutes Write full-scale

            20–65,535 Sets timeout value to action x 10 ms.
                Use 'Set Analog Watchdog Timeout'
                command to determine output value
            
            8–19 -- invalid, returns limit error

            If no data is specified, 0 (watchdog disabled) is assumed.
            
            The Optomux unit will respond to the first command after a
            watchdog timeout with an error -7 for the driver or N06cr
            for the protocol, and the command will NOT be executed
            (unless it is a PUC). The error message is a warning to
            let the host know a watchdog timeout occured.
        """
        kwargs = {
            'positions':positions,
            'data':action
            }
        cmd = self.command_name['Set Analog Watchdog']        
        return self.send_receive(address,cmd,**kwargs)   

    @utl.logger
    def disable_analog_watchdog(self,address,positions):
        """
        Purpose:
            When typing at the terminal, short watchdog delays make it
            virtually impossible to recover.  It takes two commands to
            clear the error since the first is rejected with a N06\r.

        Parameters:
            address   - optomux device address in range(256)
            positions - points having waveform canceled

        Description:
            Send a 'Power Up Clear' to get rid of the N06\r error, then
            follow immediately with a 'Set Analog Watchdog' type 0 to
            disable.
        """
        self.power_up_clear(address)
        self.set_analog_watchdog(address,positions,0)
        
##        46: ('S[positions][data]', 'Update Analog Outputs'),
    @utl.logger
    def update_analog_outputs(self,address,positions,values):
        """
        Purpose:
            Writes values to one or more analog outputs.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of points to cancel wave
            values    - tuple of values for specified positions
            
        Description:
            Use this command to write different values to multiple outputs.
            Use 'Write Analog Outputs' to write the same value to multiple
            outputs. Attempts to write values to points configured as inputs
            will be ignored.
        Driver
        """
        kwargs = {
            'positions':positions,
            'data':values
            }
        cmd = self.command_name['Update Analog Outputs']        
        return self.send_receive(address,cmd,**kwargs)

##        47: ('T[positions][data]', 'Start Averaging Inputs'),
    @utl.logger
    def start_averaging_inputs(self,address,positions,samples):
        """
        Purpose:
            Averages the value of a single point over a specified
            number of samples and returns the result.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of points to cancel wave
            samples   - number of samples to average

        Description:
            Averaging is done using a continuous running average with
            a sample rate of 100 milliseconds. The equation is:            
                Average = ((N-1) (Old Average) + (New Reading))/N

            Poll with 'Read Average Complete Bits' in (samples * 100)ms
            to see if average is ready.
        """
        kwargs = {
            'positions':positions,
            'data':samples
            }
        cmd = self.command_name['Start Averaging Inputs']        
        return self.send_receive(address,cmd,**kwargs)
    
##        48: ('i', 'Read Average Complete Bits'),
    @utl.logger
    def read_average_complete_bits(self,address):
        """
        Purpose:
            Allows the host to determine which points have completed averaging.
            
        Parameters:
            address   - optomux device address in range(256)

        Description:
            Use Start Averaging Inputs (page 104) before using this command. A 1 bit
            indicates that input averaging has been completed; a 0 bit indicates that
            it has not. Ignore response bits corresponding to points configured as
            outputs or where averaging has not been started.
        """
        cmd = self.command_name['Read Average Complete Bits']        
        return self.send_receive(address,cmd)
    
##        49: ('U[positions]', 'Read Averaged Inputs'),
    @utl.logger
    def read_averaged_inputs(self,address,positions):
        """
        Purpose:
            Read the averaged analog values of the specified input positions
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of points to read averages

        Description:
            Input averaging must have already been started using 'Start Averaging Inputs'.
            Check whether averaging is completed by using 'Read Average Complete Bits'
            If averaging has not been completed, 'Read Averaged Inputs' returns the current
            value of the average.
        """
        kwargs = {
            'positions':positions
            }
        cmd = self.command_name['Read Averaged Inputs']        
        return self.send_receive(address,cmd,**kwargs)

##        50: ('V[positions][modifiers][data]', 'Enhanced Output Waveform'),
    @utl.logger
    def enhanced_output_waveform(self,address,positions,shape,hi_limit,lo_limit,period):
        """
        Purpose:
            Starts a constant waveform at specified output points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to generate waveform
            shape     - waveform shape
            hi_limit  - max value of waveform
            lo_limit  - min value of waveform
            period    - period of ramp or 1/2 period of
                        square wave or sawtooth

        Description:
            This command offers greater flexibility in setting the period
            of the waveform than the older command 'Set Output Waveform'.
            
            Valid waveform types are:
                0   Triangle wave with positive inital slope
                1   Ramp up—waveform terminates upon reaching the upper limit
                2   Sawtooth, continuous ramp up
                3   Square wave (50 % duty cycle)
                4   Triangle wave with negative initial slope
                5   Ramp down—waveform termihnates at lower limit
                6   Sawtooth, continuous ramp down
        """
        kwargs = {
            'positions':positions,
            'modifiers':shape,
            'data':(hi_limit,lo_limit,period)
            }
        cmd = self.command_name['Enhanced Output Waveform']        
        return self.send_receive(address,cmd,**kwargs)
    
##        51: ('V[positions][modifiers]', 'Cancel Enhanced Waveforms'),
    @utl.logger
    def cancel_enhanced_waveforms(self,address,positions):
        """
        Purpose:
            Turns off existing waveforms that were started with
            'Enhanced Output Waveform'
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to generate waveform

        Description:
            This command cancels waveforms that were started using command
            'Enhanced Output Waveform'. If the 'Set Output Waveform' command
            was used, then the 'Turn Off Existing Waveforms' must be used.
        """
        kwargs = {
            'positions':positions,
            'modifiers':0
            }
        cmd = self.command_name['Cancel Enhanced Waveforms']        
        return self.send_receive(address,cmd,**kwargs)

##        52: ('g[positions]', 'Calculate Offsets'),
    @utl.logger
    def calculate_offsets(self,address,positions):
        """
        Purpose:
            Calculates and returns offsets for specified input points.

        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to calculate offsets

        Description:
            Because offset values are calculated using the current values of the inputs,
            use this command when the specified points are receiving the value you wish
            to consider zero scale. This is usually done during system installation and
            calibration, when known inputs (zero scale) can be applied to the points.
        
            Use the offset values obtained from this command to set offset values during
            Optomux initialization.

            Always set offsets before calculating gain coefficients (see page 126).
        """
        kwargs = {
            'positions':positions,
            }
        cmd = self.command_name['Calculate Offsets']        
        return self.send_receive(address,cmd,**kwargs)
    
##        53: ('W[positions][data]', 'Set Offsets'),
    @utl.logger
    def set_offsets(self,address,positions,offsets):
        """
        Purpose:
            Set analog offsets for specified input points.

        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to receive offsets
            offsets -  tuple of offsets from a previous calibration

        Description:
            Use the offset values obtained from 'Calculate Offsets' in this
            command to set offset values during Optomux initialization.

            Always set offsets before calculating gain coefficients.
        """
        kwargs = {
            'positions':positions,
            'data':offsets
            }
        cmd = self.command_name['Set Offsets']        
        return self.send_receive(address,cmd,**kwargs)

##        54: ('h[positions]', 'Calculate and Set Offsets'),
    @utl.logger
    def calculate_and_set_offsets(self,address,positions):
        """
        Purpose:
            Calculates and sets offsets for specified input points,
            and then returns caluclated  offsets to the host.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to calculate offsets

        Description:
            Because offset values are calculated using the current values of the inputs,
            use this command when the specified points are receiving the value you wish
            to consider zero scale. This is usually done during system installation and
            calibration, when known inputs (zero scale) can be applied to the points.
        
            Always set offsets before calculating gain coefficients.
        """
        kwargs = {
            'positions':positions,
            }
        cmd = self.command_name['Calculate and Set Offsets']        
        return self.send_receive(address,cmd,**kwargs)

##        55: ('X[positions]', 'Calculate Gain Coefficients'),
    @utl.logger
    def calculate_gain_coefficients(self,address,positions):
        """
        Purpose:
            Calculates and returns gain coeffients for specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to calculate gains

        Description:
            Before using this command, first calculate and set the offsets. Because
            gain values are calculated using the current values of the inputs, use
            this command when the specified points are receiving the value you wish
            to consider full scale. This is usually done during system installation
            and calibration, when known inputs (full scale) can be applied to the
            points. Use the values obtained from this command to set gain coefficient
            values during Optomux initialization.

        Returns:
            Tuple of elements elements containing gain coefficient values for the
            corresponding points. Values returned are 10,000 times the actual gain
            coefficients.
            
        Note:
            A returned value of 14,000 represents a gain coefficient of 1.40. 
        """
        kwargs = {
            'positions':positions,
            }
        cmd = self.command_name['Calculate Gain Coefficients']        
        return self.send_receive(address,cmd,**kwargs)
    
##        56: ('Y[positions][data]', 'Set Gain Coefficients'),
    @utl.logger
    def set_gain_coefficients(self,address,positions,gains):
        """
        Purpose:
            Sets the gain coeffients for specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to set gains
            gains     - tuple of gain values from earlier calibartion
            
        Description:
            It is assumed that the offsets and gains have been conputed
            using calibrated input values previously previously.  Use this
            command when necessary to set the gains to a previously good
            calibration value.
        """
        kwargs = {
            'positions':positions,
            'data':gains
            }
        cmd = self.command_name['Set Gain Coefficients']        
        return self.send_receive(address,cmd,**kwargs)

##        57: ('Z[positions]', 'Calculate and Set Gain Coefficients'),
    @utl.logger
    def calculate_and_set_gain_coefficients(self,address,positions):
        """
        Purpose:
            Calculates and sets the gain coeffients for specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to set calculate/gains
            
        Description:
            It is assumed that the offsets and gains have been conputed
            using calibrated input values previously previously.  Use this
            command to calibrate the gain when a cal source is applied
            to the input.
        """
        kwargs = {
            'positions':positions,
            }
        cmd = self.command_name['Calculate and Set Gain Coefficients']        
        return self.send_receive(address,cmd,**kwargs)

##        58: ('a[positions]', 'Read Lowest Values'),
    @utl.logger
    def read_lowest_values(self,address,positions):
        """
        Purpose:
            Returns the lowest readings at specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to read lowest values
            
        Description:
            The reading will be the lowest the unit has encountered since last
            receiving a 'Read and Clear Lowest Values' or 'Clear Lowest Values'.
            
            Units set all low values to an extreme over range of 2000 hex
            (for the driver) or 3000 hex (for the protocol) upon power-up.

        Returns:
            A tuple of lowest values for the specified positions
        """
        kwargs = {
            'positions':positions
            }
        cmd = self.command_name['Read Lowest Values']        
        return self.send_receive(address,cmd,**kwargs)

##        59: ('b[positions]', 'Clear Lowest Values'),
    @utl.logger
    def clear_lowest_values(self,address,positions=None):
        """
        Purpose:
            Clears the lowest reading for specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to clear
            
        Description:
            Units clear the lowest readings by setting the lowest value to an
            extreme over range of 2000 hex (for the driver) or 3000 hex
            (for the protocol). This allows the unit to store the lowest value
            encountered in subsequent readings.
        """
        kwargs = {
            'positions':positions,
            }
        cmd = self.command_name['Clear Lowest Values']        
        return self.send_receive(address,cmd,**kwargs)
    
##        60: ('c[positions]', 'Read and Clear Lowest Values'),
    @utl.logger
    def read_and_clear_lowest_values(self,address,positions):
        """
        Purpose:
            Returns/clears the lowest readings at specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to read lowest values
            
        Description:
            The reading will be the lowest the unit has encountered since last
            receiving a 'Read and Clear Lowest Values' or 'Clear Lowest Values'.
            
            Units set all low values to an extreme over range of 2000 hex
            (for the driver) or 3000 hex (for the protocol) upon power-up.

        Returns:
            A tuple of lowest values for the specified positions
        """
        kwargs = {
            'positions':positions
            }
        cmd = self.command_name['Read and Clear Lowest Values']        
        return self.send_receive(address,cmd,**kwargs)

##        61: ('d[positions]', 'Read Peak Values'),
    @utl.logger
    def read_peak_values(self,address,positions):
        """
        Purpose:
            Returns the highest readings at specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to read highest values
            
        Description:
            The reading will be the highest the unit has encountered since last
            receiving a 'Read and Clear Peak Values' or 'Clear Peak Values'.
            
            Units set all low values to an extreme over range of 0000 hex
            upon power-up.

        Returns:
            A tuple of highest values for the specified positions
        """
        kwargs = {
            'positions':positions
            }
        cmd = self.command_name['Read Peak Values']        
        return self.send_receive(address,cmd,**kwargs)

##        62: ('e[positions]', 'Clear Peak Values'),
    @utl.logger
    def clear_peak_values(self,address,positions=None):
        """
        Purpose:
            Clears the peak reading for specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to clear
            
        Description:
            Units clear the peak readings by setting the them to 0.
        """
        kwargs = {
            'positions':positions,
            }
        cmd = self.command_name['Clear Peak Values']        
        return self.send_receive(address,cmd,**kwargs)

##        63: ('f[positions]', 'Read and Clear Peak Values'),
    @utl.logger
    def read_and_clear_peak_values(self,address,positions=None):
        """
        Purpose:
            Returns/clears the highest readings at specified input points.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to read/clear
            
        Description:
            The reading will be the highest the unit has encountered since last
            receiving a 'Read and Clear Peak Values' or 'Clear Peak Values'.
            
            Units set all peak values to 0000 hex upon power-up.

        Returns:
            A tuple of highest values for the specified positions
        """
        kwargs = {
            'positions':positions
            }
        cmd = self.command_name['Read and Clear Peak Values']        
        return self.send_receive(address,cmd,**kwargs)

##        64: ('M', 'Read Binary On Off Status'),
    @utl.logger
    def read_binary_on_off_status(self,address):
        """
        Purpose:
            Returns the on/off status of all 16 points in the form of
            a 16-bit binary number.
            
        Parameters:
            address   - optomux device address in range(256)

        Returns:
            A bitmask (0=off, 1=on)
        """
        cmd = self.command_name['Read Binary On Off Status']        
        return self.send_receive(address,cmd)

##        65: ('J[positions]', 'Write Binary Outputs'),
    @utl.logger
    def write_binary_outputs(self,address,mask):
        """
        Purpose:
            Writes all 16 points with the value form a bitmask
            
        Parameters:
            address - optomux device address in range(256)
            mask - uint16 with a bit for each output, 0=off, 1=on
        """
        kwargs = {'positions':mask}
        rsp = self.write_digital_outputs(address,**kwargs)
        return rsp

##        66: ('Q', 'Read Binary Latches'),
    @utl.logger
    def read_binary_latches(self,address):
        """
        Purpose:
            Returns data indicating which of the inputs have latched.
            
        Parameters:
            address   - optomux device address in range(256)

        Description:
            This command does not clear the latches. Subsequent Read Latches commands will
            return the same results.
        """
        cmd = self.command_name['Read Binary Latches']        
        return self.send_receive(address,cmd)

##        67: ('R[positions]', 'Read and Clear Binary Latches'),
    @utl.logger
    def read_and_clear_binary_latches(self,address,positions):
        """
        Purpose:
            Returns data indicating which of the inputs have latched
            and clears the latches.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - specifies which latches to read/clear
            
        Description:
            This command first reads then clears the latches.
            """
        kwargs = {'positions':positions}
        cmd = self.command_name['Read and Clear Binary Latches']        
        return self.send_receive(address,cmd,**kwargs)

##        68: ('Z[positions][modifiers][data]', 'High Resolution Square Wave'),
    @utl.logger
    def high_resolution_square_wave(self,address,positions,on_time,off_time):
        """
        Purpose:
            Starts a continuous square wave at specified output points.

        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to output squarewave
            on_ticks  - timer ticks to stay on
            off_ticks - timer ticks to stay off

        Description:
            Before using this command, use the Set Timer Resolution.

            On time = timer resolution x first element value
            Off time = timer resolution x second element value.
            Maximum for on and off times is 256 x 2.56 seconds (10.92 minutes).

            This command operates the same as the standard square wave command except
            that it uses the current timer resolution instead of a resolution of 2.56
            seconds.

            The square wave stops by sending one of the commands:
                'Turn Off Time Delay/Square Wave' is sent
                'Set Time Delay'
                'Initiate Square Wave'

            Write , Activate, and Deactivate Digital Outputs have no effect
        """
        kwargs = {
            'positions':positions,
            'modifiers':'M',
            'data':(on_time,off_time)
            }
        cmd = self.command_name['High Resolution Square Wave']        
        return self.send_receive(address,cmd,**kwargs)
    
##        69: ('h[positions]', 'Retrigger Time Delay'),
    @utl.logger
    def retrigger_time_delay(self,address,positions=None):
        """
        Purpose:
            Restarts or triggers an existing time delay.

        Parameters:
            address   - optomux device address in range(256)
            positions - position to retrigger
            
        Description:
            Use this command along with 'Set Time Delay' to  dynamically
            change an active time delay. This command overrides an
            existing time delayed output by setting the time delay counter
            to the value established with 'Set Time Delay'.
        """
        kwargs = {'positions':positions}
        cmd = self.command_name['Retrigger Time Delay']        
        return self.send_receive(address,cmd,**kwargs)
    
##        70: ('j', 'Read Configuration'),
    @utl.logger
    def read_configuration(self,address):
        """
        Purpose:
            Returns the current input/output configuration for all 16 points.

        Parameters:
            address   - optomux device address in range(256)

        Description:
            0=Input
            1=Output

        Returns:
            Tuple of bits

        Example:
            >>> on.read_configuration(0)
            ('A', (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        """         
        cmd = self.command_name['Read Configuration']        
        return self.send_receive(address,cmd)

    """
    on timeout, outputs set in positions are turned on, others off
    """
##        71: ('m[positions][data]', 'Set Enhanced Digital Watchdog'),
    @utl.logger
    def set_enhanced_digital_watchdog(self,address,positions,ticks):
        """
        Purpose:
            Instructs a digital Optomux unit to monitor activity on the
            communications link and to take a specified action if there
            is no activity within a specified time.
            
        Parameters:
            address   - optomux device address in range(256)
            positions - mask or tuple of positions to output squarewave
            ticks     - number of 10ms ticks before generating a watchdog

        Description:
            After this command is issued, if a character is not received
            within the time specified delay, the unit will turn on or off
            all outputs specified. All time delay outputs are cancelled.
            Inputs are not affected.
            
            The delay time is set delay * 10 milliseconds. Delays of less
            than 200 milliseconds (except 0) result in a limit error and
            the command is not executed. A delay time of zero disables
            the digital watchdog function. If no delay time is sent, 0
            is assumed.
            
            The Optomux unit will respond to the first command after a
            watchdog timeout with N06cr (Optomux protocol) error, and
            the command will NOT be executed (unless it is a PUC). This
            error code is sent as a warning to let the host know a watchdog
            timeout occurred.
        """
        kwargs = {'positions':positions,
                  'data':ticks}
        cmd = self.command_name['Set Enhanced Digital Watchdog']        
        return self.send_receive(address,cmd,**kwargs)
    
##        72: ('i[positions][modifiers][data]', 'Generate N Pulses'),
    @utl.logger
    def generate_n_pulses(self,address,positions,ticks,count):
        """
        Purpose:
            Instructs Optomux unit to output a counted string of
            pulses of a specified duration.

        Parameters:
            address   - target optomux device address
            positions - outputs to pulse
            ticks     - 50% time in ticks (see 'Set Timer Resolution'
            count     - number of pulses

        Description:
            Generates count pulses on specified outputs with a period of
            2 * timer resolution * ticks            
        """
        kwargs = {'positions':positions,
                  'modifiers':ticks,
                  'data':count}
        cmd = self.command_name['Generate N Pulses']        
        return self.send_receive(address,cmd,**kwargs)

##        73: ('k[positions][data]', 'Start On Pulse'),
    @utl.logger
    def start_on_pulse(self,address,positions,ticks):
        """
        Purpose:
            Turns on specified outputs for a the specified number of
            timer ticks, then turns them off.

        Parameters:
            address   - target optomux device address
            positions - outputs to pulse
            ticks     - pulse on time in ticks (see 'Set Timer Resolution'
            
        Description:
            Assumes the 'Set Timer Resolution' command has been used to
            tick rate.
            
            Because this command is retriggerable, it can be used as a watchdog
            circuit by continuously sending this command at a rate faster than
            the pulse length.
            
            To cancel this command, resend it with ticks = 1
        """
        kwargs = {
            'positions':positions,
            'data':time
            }
        cmd = self.command_name['Start On Pulse']        
        return self.send_receive(address,cmd,**kwargs)

##        74: ('l[positions][data]', 'Start Off Pulse'),
    @utl.logger
    def start_off_pulse(self,address,positions,time):
        """
        Purpose:
            Turns off specified outputs for a the specified number of
            timer ticks, then turns them on.

        Parameters:
            address   - target optomux device address
            positions - outputs to pulse
            ticks     - pulse off time in ticks (see 'Set Timer Resolution'
            
        Description:
            Assumes the 'Set Timer Resolution' command has been used to
            tick rate.
            
            Because this command is retriggerable, it can be used as a watchdog
            circuit by continuously sending this command at a rate faster than
            the pulse length.
            
            To cancel this command, resend it with ticks = 1
        """
        kwargs = {
            'positions':positions,
            'data':time
            }
        cmd = self.command_name['Start Off Pulse']        
        return self.send_receive(address,cmd,**kwargs)
    
##        75: ('n[data]', 'Set Timer Resolution'),
    @utl.logger
    def set_timer_resolution(self,address,ticks):
        """
        Purpose:
            Sets a global timer value for all timing functions
            on the Optomux digital brain.
            
        Parameters:
            address   - target optomux device address
            ticks     - timer period in 10ms ticks

        Description:
            If the value is 0, the timer resolution is 2.56 seconds. This command is a
            global command and affects the timing resolution for the following commands
                SET TIME DELAY
                INITIATE SQUARE WAVE
                HIGH RESOLUTION SQUARE WAVE
                RETRIGGER TIME DELAY
                GENERATE N PULSES
                START ON PULSE
                START OFF PULSE
                READ PULSE COMPLETE BITS
                READ PULSE DURATION COUNTERS
                READ AND CLEAR DURATION COUNTERS
        """
        self.timer_resolution[address] = ticks
        kwargs = {'data':ticks}
        cmd = self.command_name['Set Timer Resolution']        
        return self.send_receive(address,cmd,**kwargs)

##        76: ('k[positions][data]', 'Set Temperature Probe Type'),
    @utl.logger
    def set_temperature_probe_type(self,address,positions,probe,useReadAnalogInputs=False):
        """
        Purpose:
            Sets the probe type for points using temperature input
            modules (thermocouples, ICTDs, and RTDs), so that the
            READ TEMPERATURE INPUTS command can be used to read the
            temperature directly.
            
        Parameters:
            address   - target optomux device address
            positions - outputs to pulse
            type      - probe type

        Description:
            Valid probe types are:
                0 no temperature probe
                1 ICTD probe
                2 10 ohm RTD probe
                3 100 ohm RTD probe
                4 Type J thermocouple
                5 Type K thermocouple
                6 Type R thermocouple
                7 Type S thermocouple
                8 Type T thermocouple
                9 Type E thermocouple

        Notes:
            For some reason the B3000 changes the value returned by
            'Read Analog Inputs' if a probe type was set using
            'Set Temperature Probe Type'.  Therefore, don't send the
            'Set Temperature Probe Type' command if we want to compute
            our own temperature using the equations/tables found in
            the users manual.  
        """
        kwargs = {
            'positions':positions,
            'data':probe
            }
        if not useReadAnalogInputs:
            cmd = self.command_name['Set Temperature Probe Type']        
            rtn = self.send_receive(address,cmd,**kwargs)
        else:
            rtn = ('A',0)
        if rtn[0] == 'A':
            if isinstance(positions,int):
                positions = self.positions_mask_to_tuple(kwargs['positions'])
            for position in positions:
                key = address<<4+position
                self.temperature_probes[key] = probe
                self.read_as_analog_input[key] = useReadAnalogInputs
        return rtn
                        
    @utl.logger
    def convert_to_icdt(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        ICTD Input Module—AD4, p155
        """
        v = ((0.08262 * value) - 188.4)
        return v
    
    @utl.logger
    def convert_to_10_ohm_rtd(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        """
        return value
    
    @utl.logger
    def convert_to_100_ohm_rtd(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        100 Ohm RTD Input Module—AD10T2, p158
        """
        rtd100row = namedtuple('rtd100row',['A0', 'A1', 'A2', 'A3', 'A4', 'A5'])
        rtd100tbl = (
            rtd100row(range(-32768,2110), 0, 9.474182E-02, 50, -0.000156, 0.0156, -1.11),
            rtd100row(range(2111,4095), 2111, 0.1008065, 100, -0.000156, 0, 248.45),
            rtd100row(range(4095,6219), 4095, 0.1082863, 115, -0.00017, 0, 462.76),
            rtd100row(range(6219,32767), 6219, 0.118525, 135, -0.000188, 0, 711.56)
            )

        for rtd100row in rtd100tbl:
            if value in rtd100tbl.range:
                tv0 = (value - rtd100row.A0) * rtd100row.A1 - rtd100row.A2
                return tv0 - (rtd100row.A3 * tv0**2) - (rtd100row.A4 * tv0) + rtd100row.A5
        return None

    @utl.logger
    def convert_to_type_j_thermocouple(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        Type J Thermocouple—AD5, AD5T, p156
        """
        # linearization table
        jrow = namedtuple('jrow',['range','A','B','C'])
        jtbl = (
            jrow(range(-4096,2),104,0.1923076,20.15),
            jrow(range(1,162),0,0.1863354,0.10),
            jrow(range(162,355),161,0.1813472,30.19),
            jrow(range(355,552),354,0.1776649,65.10),
            jrow(range(552,868),551,0.1740506,100.15),
            jrow(range(868,1767),867,0.1724137,155.10),
            jrow(range(1767,2547),1766,0.1730769,310.00),
            jrow(range(2547,2896),2546,0.1719197,445.05),
            jrow(range(2896,3192),2895,0.1689189,505.10),
            jrow(range(3192,3465),3191,0.1654411,555.10),
            jrow(range(3465,3744),3464,0.1612903,600.20),
            jrow(range(3744,3901),3743,0.1572327,645.17),
            jrow(range(3901,4096),3901,0.1546391,670.10))

        for jrow in jtbl:
            if value in jrow.range:
                # TEMP = (VALUE% - A) * B + C
                return (value - jrow.A) * jrow.B + jrow.C
        return None
    
    @utl.logger
    def convert_to_type_k_thermocouple(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        Type K Thermocouple—AD8, AD8T, p157
        """
        krow = namedtuple('krow',['range', 'A', 'B', 'C'])
        ktbl = (
            krow(range(-32768,97), 0, 0.3167899, -99.6),
            krow(range(96,199), 95, 0.2892961, -69.6),
            krow(range(199,349), 198, 0.2675585, -39.8),
            krow(range(349,579), 348, 0.2518454, 0.3),
            krow(range(579,910), 478, 0.2478090, 57.9),
            krow(range(910,1366), 909, 0.2541073, 140.2),
            krow(range(1366,1871), 1365, 0.2456905, 256.2),
            krow(range(1871,3085), 1870, 0.2405867, 380.1),
            krow(range(3085,3622), 3084, 0.2456271, 671.8),
            krow(range(3622,4096), 3621, 0.2532714, 803.8),
            krow(range(4096,4525), 4095, 0.2611331, 924.0),
            krow(range(4525,4875), 4524, 0.2685714, 1035.9),
            krow(range(4875,5171), 4874, 0.2770270, 1130.0),
            krow(range(5171,5422), 5170, 0.2858277, 1211.9),
            krow(range(5422,32767), 5422, 0.2959973, 1283.9)
            )
        for krow in ktbl:
            if value in krow.range:
                # TEMP = (VALUE% - A) * B + C
                return (value - krow.A) * krow.B + krow.C
        return None

    @utl.logger
    def convert_to_type_r_thermocouple(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        Type R Thermocouple—AD17T, p159
        """
        rrow = namedtuple('rrow',['range','A0', 'A1', 'A2', 'A3', 'A4'])
        rtbl = (
            rrow(range(-32768,740), 0, 0.1625144, -2.045438E-05, 2.540494E-09, -1.7679E-13),
            rrow(range(740,32767), 46.67453, 0.1117991, -2.565926E-06, 5.347317E-11, 0))
        for rrow in rtbl:
            if value in rrow.range:
                tv0 = 2.43663 * value
                # temp = A0 + (A1 * TV0) + (A2 * (TV0^2)) + (A3 * (TV0^3)) + (A4 * (TV0^4))
                return (rrow.A0 \
                        + (rrow.A1 * tv0) \
                        + (rrow.A2 * tv0**2) \
                        + (rrow.A3 * tv0**3) \
                        + (rrow.A4 * tv0**4))
        return None

    @utl.logger
    def convert_to_type_s_thermocouple(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        Type S Thermocouple—AD17T, p160
        """
        srow = namedtuple('srow',['range','A0', 'A1', 'A2', 'A3', 'A4'])
        stbl = (
            srow(range(-32768,479), 0, 0.1641405, -2.024176E-05, 2.784973E-09, -1.41721E-13),
            srow(range(479,32767), 30.1319, 0.1215561, -2.752449E-06, 6.475822E-11, 0)
            )
        for srow in stbl:
            if value in srow.range:
                tv0 = 2.43663 * value
                # temp = A0 + (A1 * TV0) + (A2 * (TV0^2)) + (A3 * (TV0^3)) + (A4 * (TV0^4))
                return (srow.A0 \
                        + (srow.A1 * tv0) \
                        + (srow.A2 * tv0**2) \
                        + (srow.A3 * tv0**3) \
                        + (srow.A4 * tv0**4))
        return None

    @utl.logger
    def convert_to_type_t_thermocouple(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        Type T Thermocouple—AD18T, p161
        """
        trow = namedtuple('trow',['range','A0', 'A1', 'A2', 'A3', 'A4'])
        ttbl = (
            trow(range(-32768,1419), 0, 2.383709E-02, -2.987884E-06, -7.194581E-10, -1.004194E-13),
            trow(range(1419,32767), 0, 0.0256613, -6.195487E-07, 2.218164E-11, -3.55009E-16)
            )
        for trow in ttbl:
            if value in trow.range:
                tv0 = 3.951286 * value + 5602.92
                return (trow.A0 \
                        + (trow.A1 * tv0) \
                        + (trow.A2 * tv0**2) \
                        + (trow.A3 * tv0**3) \
                        + (trow.A4 * tv0**4))
        return None

    @utl.logger
    def convert_to_type_e_thermocouple(self,value):
        """
        Optomux Protocol Guide, Form 1572-140618—June 2014
        Converting Temperature Readings
        Type E Thermocouple—AD19T, p162
        """
        erow = namedtuple('erow',['range','A0', 'A1', 'A2', 'A3', 'A4'])
        etbl = (
            erow(range(-32768,367), 0, 1.572665E-02, -1.210215E-06, -1.95778E-10, -1.66963E-14),
            erow(range(367,3784), 0, 1.702253E-02, -2.209724E-07, 5.480931E-12, -5.766989E-17),
            erow(range(3784,32767), 19.66945, 1.420774E-02, -5.184451E-08, 5.636137E-13, -1.564634E-18)
            )
        for erow in etbl:
            if value in erow.range:
                tv0 = 3.951286 * value + 5602.92
                return (trow.A0 \
                        + (erow.A1 * tv0) \
                        + (erow.A2 * tv0**2) \
                        + (erow.A3 * tv0**3) \
                        + (erow.A4 * tv0**4))
        return None

    @utl.logger
    def convert_probe_temperature_readings(self,address,positions,values):
        """
        Convert analog reading to temperature based on probe type
        """
        if isinstance(positions,int):
            positions = sorted(self.positions_mask_to_tuple(positions))
        if isinstance(values,int):
            values = list(values)
        # make mutable
        for position in positions:
            value = values[positions.index(position)]
            probe_type = -1
            try:
                key = address<<4+position
                probe_type = self.temperature_probes[key]
                if probe_type in self.temperature_probe_type_valid_range:
                    if isinstance(value,int):
                        value /= 16
                    else:
                        value = -4096
            except:
                value = -4096
        return values
        
    @utl.logger
    def convert_analog_temperature_readings(self,address,positions,values):
        """
        Refer to:
            Optomux Protocol Guide, Form 1572-140618—June 2014
            Converting Temperature Readings, p155

        There seems to be an issue with the B3000 changing the value returned
        by 'Read Analog Inputs' when the temperature probe type changes.

        Thus it is necessary to avoid sending the 'Set Temperature Probe Type'
        command to the B3000 if the temperature module is to be read using the
        'Read Analog Inputs' command.

        The set_temperature_probe_type function
        """
        if isinstance(positions,int):
            positions = self.positions_mask_to_tuple(positions)
        if isinstance(values,int):
            values = tuple(values)
        values = list(values)
        
        # make mutable
        for position in positions:
            i = positions.index(position)
            value = values[i]
            probe_type = -1
            try:
                key = address<<4+position
                probe_type = self.temperature_probes[key]
                if probe_type in self.temperature_probe_type_valid_range:
                    if self.temperature_probe_types[probe_type] == 'ICTD probe':
                        value = self.convert_to_icdt(value)
                    elif self.temperature_probe_types[probe_type] == '10 ohm RTD probe':
                        value = self.convert_to_10_ohm_rtd(value)
                    elif self.temperature_probe_types[probe_type] == '100 ohm RTD probe':
                        value = self.convert_to_100_ohm_rtd(value)
                    elif self.temperature_probe_types[probe_type] == 'Type J thermocouple':
                        value = self.convert_to_type_j_thermocouple(value)
                    elif self.temperature_probe_types[probe_type] == 'Type K thermocouple':
                        value = self.convert_to_type_k_thermocouple(value)
                    elif self.temperature_probe_types[probe_type] == 'Type R thermocouple':
                        value = self.convert_to_type_r_thermocouple(value)
                    elif self.temperature_probe_types[probe_type] == 'Type S thermocouple':
                        value = self.convert_to_type_s_thermocouple(value)
                    elif self.temperature_probe_types[probe_type] == 'Type T thermocouple':
                        value = self.convert_to_type_t_thermocouple(value)
                    elif self.temperature_probe_types[probe_type] == 'Type E thermocouple':
                        value = self.convert_to_type_e_thermocouple(value)
                    else:
                        value = -4096
            except:
                value = -4096
                utl.log_error_message('Invalid Temperature Probe Type {:d}'.format(probe_type))
            values[i] = value
        return tuple(values)

##        77: ('l[positions]', 'Read Temperature Inputs'),
    @utl.logger
    def read_temperature_inputs(self,address,positions):
        """
        Purpose:
            Returns the temperature at the specified input points.
            
        Parameters:
            address   - target optomux device address
            positions - positions for which temps are to be read

        Remarks:
            Before using this command, set the temperature probe type
            For additional information on reading temperatures, see
            Appendix C, “Reading Negative Numbers and Temperature."

        Returns:
            A tuple with temperature readings in 1/16ths of degrees C.

            -4096 means no probe type set
        """
        if isinstance(positions,int):
            positions = self.positions_mask_to_tuple(positions)

        # create empty lists of positions
        temperature_positions = []
        analog_positions = []
        
        # for each position specified
        for position in positions:
            # compute key
            key = address << 4 + position
            # using 'Read Analog Inputs'?
            if key in self.read_as_analog_input \
               and self.read_as_analog_input[key]:
                # append to simulation list
                analog_positions.append(position)
            # append to real temps list
            else:
                temperature_positions.append(position)
                
        # back to tuples
        temperature_positions = tuple(sorted(temperature_positions))
        analog_positions = tuple(sorted(analog_positions))
        
        # if there are real temp modules to be read
        if len(temperature_positions):
            # send the 'Read Temperature Inputs' command
            kwargs = {'positions':temperature_positions}
            cmd = self.command_name['Read Temperature Inputs']        
            temperature_response = self.send_receive(address,cmd,**kwargs)
            if temperature_response[0] == 'A':
                temperature_response = ('A',self.convert_probe_temperature_readings(\
                    address,temperature_positions,temperature_response[1]))
                
        # if there are simulated temps to read
        if len(analog_positions):
            # send the 'Read Analog Inputs' command
            kwargs = {'positions':analog_positions}
            cmd = self.command_name['Read Analog Inputs']
            analog_response = self.send_receive(address,cmd,**kwargs)
            if analog_response[0] == 'A':
                analog_response = ('A',self.convert_analog_temperature_readings(\
                    address,analog_positions,analog_response[1]))

        # combine simulated and read temps
        positions_and_values = sorted(\
         list(zip(temperature_positions,temperature_response[1])) +
         list(zip(analog_positions,analog_response[1])),\
         key=lambda pv: pv[0])
        
        # extract values
        return ('A',tuple([pv[1] for pv in positions_and_values]))
    
##        78: ('m[positions][data]', 'Set Analog Watchdog Timeout'),
    @utl.logger
    def set_analog_watchdog_timeout(self,address,positions,values):
        kwargs = {
            'positions':positions,
            'data':values
            }
        cmd = self.command_name['Set Analog Watchdog Timeout']        
        return self.send_receive(address,cmd,**kwargs)
    
##        79: ('o[positions]', 'Read Average Temperature Inputs'),
    @utl.logger
    def read_average_temperature_inputs(self,address,positions):
        """
        Purpose:
            Returns the temperature at the specified input points.
            
        Parameters:
            address   - target optomux device address
            positions - positions for which temps are to be read

        Remarks:
            Before using this command, set the temperature probe type,
            set number of averages using 'Start Averaging Inputs'.
            Poll 'Check Average Complete Bits' to see if an average is
            ready.
            
            For additional information on reading temperatures, see
            Appendix C, “Reading Negative Numbers and Temperature."

        Returns:
            A tuple with temperature readings in 1/16ths of degrees C.

            -4096 means no probe type set
        """
        if isinstance(positions,int):
            positions = self.positions_mask_to_tuple(positions)

        # create empty lists of positions
        temperature_positions = []
        analog_positions = []
        
        # for each position specified
        for position in positions:
            # compute key
            key = address << 4 + position
            # using 'Read Analog Inputs'?
            if key in self.read_as_analog_input \
               and self.read_as_analog_input[key]:
                # append to simulation list
                analog_positions.append(position)
            # append to real temps list
            else:
                temperature_positions.append(position)
                
        # back to tuples
        temperature_positions = tuple(sorted(temperature_positions))
        analog_positions = tuple(sorted(analog_positions))
        
        # if there are real temp modules to be read
        if len(temperature_positions):
            # send the 'Read Temperature Inputs' command
            kwargs = {'positions':temperature_positions}
            cmd = self.command_name['Read Average Temperature Inputs']        
            temperature_response = self.send_receive(address,cmd,**kwargs)
            if temperature_response[0] == 'A':
                temperature_response = ('A',self.convert_probe_temperature_readings(\
                    address,temperature_positions,temperature_response[1]))
                
        # if there are simulated temps to read
        if len(analog_positions):
            # send the 'Read Analog Inputs' command
            kwargs = {'positions':analog_positions}
            cmd = self.command_name['Read Averaged Inputs']
            analog_response = self.send_receive(address,cmd,**kwargs)
            if analog_response[0] == 'A':
                analog_response = ('A',self.convert_analog_temperature_readings(\
                    address,analog_positions,analog_response[1]))

        # combine simulated and read temps
        positions_and_values = sorted(\
         list(zip(temperature_positions,temperature_response[1])) +
         list(zip(analog_positions,analog_response[1])),\
         key=lambda pv: pv[0])
        
        # extract values
        return ('A',tuple([pv[1] for pv in positions_and_values]))
   
##        80: ('`', 'Date Of Firmware')   
    @utl.logger
    def date_of_firmware(self,address):
        """
        Purpose:
            Identifies brain firmware revision by date of release.
            
        Parameters:
            address   - target optomux device address

        Description:
            ‘ is the single quote under the tilde sign on the computer
            keyboard and is an ASCII 60 hex.
        
            Response from a B1, B2, E1, or E2:
                A07/05/05*B9
                
            Response from a B3000:
                A811609019911050100300000B7
                
            Very old brain boards may not understand this command
            and return an error.
        """
        cmd = self.command_name['Date Of Firmware']        
        return self.send_receive(address,cmd)

    @utl.logger
    def get_timer_resolution(self,address):
        """
        A way to get the timer resolution from an instance
        variable for use in computations of time related
        delays and waveform periods.  Not part of Optomux
        command set but there is no corresponding Optomux
        command to read the info from the controller.
        """
        return self.timer_resolution[address]

    @utl.logger
    def get_initiate_square_wave_on_off_time_limits(self,address):
        """
        A way to get the limits for square wave pulse widths.  This
        is meant to help determine if the current timer resolution
        set with 'Set Timer Resolution' will support the desired
        pulse width.
        """
        square_wave_time_limits = [
                  256 * 0.010 * self.timer_resolution[address],
            256 * 256 * 0.010 * self.timer_resolution[address]
            ]
        return square_wave_time_limits

    @utl.logger
    def positions_tuple_to_mask(self,positions):
        """
        convert a positions tuple into a bit mask
        """
        if isinstance(positions,list):
            mask = 0
            for position in positions:
                mask |= (1<<position)
            return mask
        elif isinstance(positions,int):
            return positions
        return 0

    @utl.logger
    def positions_mask_to_tuple(self,positions):
        """
        convert a positions mask into a tuple
        """
        return tuple(i for i in range(16) if (positions & (1 << i)) != 0)                                                            

    @utl.logger
    def optomux_data_to_binary_tuple(self,data):
        """
        create a tuple of bit values
        """
        return self.optomux_data_to_tuple(data)

    @utl.logger
    def optomux_data_to_counter_tuple(self,data):
        """
        create a tuple from 16 bit counter values
        """
        return self.optomux_data_to_tuple(data,16)
        
    @utl.logger
    def optomux_data_to_analog_input_tuple(self,data):
        """
        create a tuple from analog input values
        """
        values = list(self.optomux_data_to_tuple(data,16))
        for i in range(len(values)):
            if isinstance(values[i],int):
                values[i] -= 4096
        return tuple(values)
    
    @utl.logger
    def optomux_data_to_temperature_tuple(self,data):
        """
        Temperature Readings—Some Optomux commands are specifically
        designed for temperature measurement (“Read Temperature Inputs”
        on page 107 and “Read Average Temperature Inputs” on page 109).
        When you use a Read Temperature command, the brain board normally
        takes care of thermocouple linearization and returns temperature
        in degrees C. However, the commands are valid only when the
        temperature is within the nominal range for the module. If
        temperature is outside the module’s nominal range (or if your
        software does not support the Read Temperature commands), then
        you will need to read counts from the module, linearize the counts,
        and convert them to temperature. More information and equations
        showing how to linearize and convert readings from temperature
        modules are in “Converting Temperature Readings” on page 155.

        Points that read below the scale of the set probe type return a
        value of -273 C.  Points that read above the scale for the set
        probe type return 2047 C.
        """
        values = list(self.optomux_data_to_tuple(data,16))
        for i in range(len(values)):
            if isinstance(values[i],int):
                # if sign bit is set
                if values[i] & 0x8000:
                    # handle twos complement
                    values[i] -= 0x10000
        return tuple(values)
    
    @utl.logger
    def optomux_data_to_analog_output_tuple(self,data):
        return self.optomux_data_to_tuple(data,12)        

    @utl.logger
    def optomux_data_to_tuple(self,data,bits=1):
        """
        Return data in a tuple.  Since optomux returns it as an array of
        hex nibbles and the command determines bits per field, this routine
        first converts from hex to binary, then extracts the bits per the
        width arg.
        Special '?' twiddling needs to be done because optomux returns '?'s
        in a field where data makes no sense.  For example, reading analog
        averages of analog outputs, latched digital outputs, etc.
        """
        b = ''
        # for each hex digit in string
        for i in range(len(data)):
            # get the next nibble
            c = data[i]
            # if a valid hex digit
            if c in '0123456789abcdefABCDEF':
                # conver to int
                n = int(data[i],16)
                # for each bit in the nibble starting at msb
                for j in reversed(range(4)):
                    # append a binary digit
                    b += chr(ord('0') + ((n >> j) & 1))
            # tried to read an output counter
            elif c == '?':
                # 4 binary '?' s
                for i in range(4):
                    b += '?'
        # create a tuple of ints using substrings of binary width bits
        # and expand optomux '????' as if the '?'s were binary digits
        # of all fields will be the same width
        lv = []
        for i in reversed(range(0,len(b),bits)):
            # read bits worth of binary digits
            v = b[i:i+bits]
            # try to convert to an int using base 2
            try:
                n = int(v,2)
            # poke a '?' placeholder so caller knows not to use
            # the value
            except:
                n = '?'
            # append the value to the list
            finally:
                lv.append(n)
        return tuple(lv)

    def list_optomux_devices(self):
        """
        Build a list of optomux devices by sending a
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
                rtn = self.identify_optomux_type(address)
                if rtn[0] == 'A':
                    print('Found {:s} device at address {:02X}'\
                          .format(self.optomux_type[int(rtn[1])],address))
                    devices.append(address)
        print('\nDone')
        return devices
        
if __name__ == "__main__":
    # createthe OmuxNET object
    on = OmuxNET()
    # list the available ttys
    ttys = on.tty.list_ttys()
    # print a menu
    for tty in ttys:
        print(tty,ttys[tty])
    # ask user to select a port
    ttychoice = int(input('choose a tty by number from the above list: '),10)
    print('\n')
    if ttychoice in ttys:
        baudrates = on.tty.list_baudrates()
        for baudrate in baudrates:
            print(baudrate,baudrates[baudrate])
        baudratechoice = int(input('choose a baudrate (check brain jumpers): '),10)
        if baudratechoice in on.tty.baudrates:
            baudrate = int(on.tty.baudrates[baudratechoice])
        print('\n')
        # open the port
        if on.tty.open(ttys[ttychoice],int(on.tty.baudrates[baudratechoice])):
            # build a list of devices by seeing which
            # addresses ACK a 'Power Up Clear' command
            devices = on.list_optomux_devices()
            for device in devices:
                # for grins, get the firmware date
                rtn = on.date_of_firmware(device)
                if rtn[0] == 'A':
                    print('Date Of Firmware: {}'.format(rtn[1]))
        else:
            print('() open failed'.format(ttys[choice]))
    else:
        print('{:d} : invalid port selection'.format(choice))

        
    

