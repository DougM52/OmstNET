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

import OmstNET
import time
import operator

def verify_command(mn,aa,cmd,why=None,**kwargs):
    """
    mn - the OmstNET instance
    aa - address of device
    cmd - function with args to be called
    why - an optional text string to display between tests
    **kwargs - optional compare values when reading back data

    Sends commands, verifies responses, and possibly verifies returned data.

    Note:
    Some of the tests will fail data compare because a loopback connection may
    be required.  Fo example, 'GENERATE N PULSES', 'START ON PULSE', 'START OFF
    PULSE', 'START CONTINUOUS SQUARE WAVE' are used to test counters, latch edges,
    period, frequency, etc.  But the latter require inputs which must have been
    wired to outputs.  
    """

    if why:
        print('*****',why,'*****')
    if cmd.ack == 'A':
        print('{} acked'.format(mn.last_command[aa]))
        if kwargs:
            diff = {}
            for field in cmd.data._fields:
                act = getattr(cmd.data,field)
                if field in kwargs:
                    exp = kwargs[field]
                    if exp != act:
                        diff[field] = 'exp {} != act {}'.format(exp,act)
            if len(diff) > 0:
                print('{} readback miscompares, \n\t{}'.format(mn.last_command[aa],diff))
            else:
                print('{} readback data compare passed'.format(mn.last_command[aa]))
    else:
        print('{} naked, error = {}'.format(mn.last_command[aa],cmd))
    return cmd
            
        
def verify_commands(mn,aa,cfg=tuple([0 if i & 4 else 128 for i in range(16)])):
    """
    This is a pretty extensive example of how to send commands and process
    responses for many Mistic commands.

    mn - OmstNET instance
    aa - address of device to be tested
    cfg - configuration suitable for set_io_configuration_group, etc.

    The default one assumes modules are ODC, IDC, ODC, IDC so they can
    be easily looped back.
    """
    MMMM = sum(tuple(1<<i for i in range(16) if cfg[i] == 128))
    verify_command(mn,aa,mn.power_up_clear(aa))
    verify_command(mn,aa,mn.identify_type(aa))
    verify_command(mn,aa,mn.repeat_last_response(aa),\
                   "Request a 'REPEAT LAST RESPONSE' to 'IDENTIFY TYPE' command")
    time.sleep(1)
    verify_command(mn,aa,mn.reset_all_parameters_to_default(aa),\
                   'Start with a clean slate')
    verify_command(mn,aa,mn.set_io_configuration_group(aa,0xffff,cfg),\
                   'Set module configuration and verify with readback')
    kwargs = {'TT':cfg}
    verify_command(mn,aa,mn.read_module_configuration(aa),**kwargs)

    kwargs = {'PPPP':0,'NNNN':0}
    verify_command(mn,aa,mn.read_and_optionally_clear_latches_group(aa,3),\
                   'Clear all latches')
    verify_command(mn,aa,mn.read_and_optionally_clear_latches_group(aa,3),\
                   'Verify latches cleared',**kwargs)

    kwargs = {'MMMM':MMMM}
    verify_command(mn,aa,mn.set_output_module_state_group(aa,MMMM,0),\
                   'Activate outputs verify with readback')
    verify_command(mn,aa,mn.read_module_status(aa),**kwargs)

    kwargs = {'PPPP':MMMM,'NNNN':0}
    verify_command(mn,aa,mn.read_and_optionally_clear_latches_group(aa,1),\
                   'Read, clear, verify positive latches',**kwargs)
    
    kwargs = {'NNNN':MMMM}
    verify_command(mn,aa,mn.set_output_module_state_group(aa,0,kwargs['NNNN']),
                   'Deactivate outputs and verify with readback')
    verify_command(mn,aa,mn.read_module_status(aa),**kwargs)

    kwargs = {'PPPP':0,'NNNN':MMMM}
    verify_command(mn,aa,mn.read_and_optionally_clear_latches_group(aa,2),\
                   'Read, clear, verify negative latches',**kwargs)
    
    kwargs = {'DD':(5,)}
    verify_command(mn,aa,mn.activate_output(aa,0),\
                   'Verify positive can be set and cleared')
    verify_command(mn,aa,mn.read_and_optionally_clear_latch(aa,0,1),**kwargs)
    kwargs['DD'] = (4,)
    verify_command(mn,aa,mn.read_and_optionally_clear_latch(aa,0,0),**kwargs)

    kwargs['DD'] = (2,)
    verify_command(mn,aa,mn.deactivate_output(aa,0),\
                   'Verify negative latch can be set and cleared')
    verify_command(mn,aa,mn.read_and_optionally_clear_latch(aa,0,2),**kwargs)
    kwargs['DD'] = (0,)
    verify_command(mn,aa,mn.read_and_optionally_clear_latch(aa,0,0),**kwargs)
    
    kwargs = {'MMMM':MMMM}
    verify_command(mn,aa,mn.set_comm_link_watchdog_momo_and_delay(aa,MMMM,0,50),
                   'Verify digital watchdog activates specified outputs')
    time.sleep(1)
    mn.power_up_clear(aa)
    verify_command(mn,aa,mn.read_module_status(aa),**kwargs)
    
    kwargs = {'NNNN':MMMM}
    verify_command(mn,aa,mn.set_comm_link_watchdog_momo_and_delay(aa,0,MMMM,50),
                   'Verify digital watchdog deactivates specified outputs')
    time.sleep(1)
    mn.power_up_clear(aa)
    verify_command(mn,aa,mn.read_module_status(aa),**kwargs)
                   
    verify_command(mn,aa,mn.cancel_comm_link_watchdog_momo_and_delay(aa),\
                   'sleep(1) and try to send a command, should be acked')
    time.sleep(1)
    verify_command(mn,aa,mn.identify_type(aa),\
                   'If identify type is acked, wd was disabled successfully')
    
    mmmm = operator.xor(MMMM,0xffff)
    verify_command(mn,aa,mn.enable_disable_counter_group(aa,mmmm,1),
                   'Enable inputs as counters')
    kwargs = {'DDDDDDDD':tuple(0 for i in range(16) if (mmmm & (1 << i)))}
    verify_command(mn,aa,mn.read_and_clear_32_bit_counter_group(aa,mmmm),\
                   'Make sure counters can be cleared')
    verify_command(mn,aa,mn.read_32_bit_counter_group(aa,mmmm),**kwargs)
    
    print('*****','Generate 10 pulses on output channels','*****')
    for i in range(16):
        if (MMMM & (1 << i)):
            verify_command(mn,aa,mn.generate_n_pulses(aa,i,500,500,10))
    time.sleep(1)

    kwargs = {'DDDDDDDD':tuple(10 for i in range(16) if (mmmm & (1 << i)))}
    verify_command(mn,aa,mn.read_32_bit_counter_group(aa,mmmm),
                   'Verify counts, (assuming outputs looped back to inputs)',\
                   **kwargs)

    print('*****','Verify 16 bit counters can be read and cleared','*****')
    for i in range(16):
        if (mmmm & (1 << i)):
            rsp = mn.read_32_bit_counter(aa,i)
            if rsp.ack == 'A':
                kwargs = {'DDDD':(rsp.data.DDDDDDDD[0],)}
                verify_command(mn,aa,mn.read_and_clear_16_bit_counter(aa,i),**kwargs)
                kwargs = {'DDDD':(0,)}
                verify_command(mn,aa,mn.read_16_bit_counter(aa,i),**kwargs)

    pper = tuple(3 for i in range(16) if (mmmm & (1 << i)))
    verify_command(mn,aa,mn.set_io_configuration_group(aa,mmmm,pper),\
                   'Reconfigure inputs to measure period')
    verify_command(mn,aa,mn.read_and_restart_32_bit_pulse_period_group(aa,mmmm))
    for i in range(16):
        if (MMMM & (1 << i)):
            verify_command(mn,aa,mn.generate_n_pulses(aa,i,500,500,10))
    time.sleep(1)
    kwargs = {'DDDD':mmmm}
    rsp = verify_command(mn,aa,mn.read_pulse_period_complete_status(aa),**kwargs)
    if rsp.ack == 'A':
        pppp = rsp.data.DDDD[0]
        kwargs = {'DDDDDDDD':tuple(1000 for i in range(16) if (pppp & (1 << i)))}
        verify_command(mn,aa,mn.read_32_bit_pulse_period_group(aa,pppp),**kwargs)
        
    pfrq = tuple(4 for i in range(16) if (mmmm & (1 << i)))
    verify_command(mn,aa,mn.set_io_configuration_group(aa,mmmm,pfrq),\
                   'Reconfigure inputs to measure frequency')
    for i in range(16):
        if (MMMM & (1 << i)):
            verify_command(mn,aa,mn.start_continuous_square_wave(aa,i,1000,1000))
    time.sleep(1.1)
    kwargs = {'DDDD':tuple(5 for i in range(16) if (mmmm & (1 << i)))}
    verify_command(mn,aa,mn.read_frequency_measurement_group(aa,mmmm),**kwargs)
    verify_command(mn,aa,mn.set_output_module_state_group(aa,0,MMMM))
    
    pneg = tuple(2 for i in range(16) if (mmmm & (1 << i)))
    verify_command(mn,aa,mn.set_io_configuration_group(aa,mmmm,pneg),\
                   'Reconfigure inputs to measure positive pulse duration')
    verify_command(mn,aa,mn.set_output_module_state_group(aa,MMMM,0))
    verify_command(mn,aa,mn.read_and_restart_32_bit_pulse_period_group(aa,mmmm))
    for i in range(16):
        if (MMMM & (1 << i)):
            verify_command(mn,aa,mn.start_off_pulse(aa,i,5000))
    time.sleep(0.5)
    kwargs = {'DDDD':mmmm}
    rsp = verify_command(mn,aa,mn.read_pulse_period_complete_status(aa),**kwargs)
    if rsp.ack == 'A':
        pppp = rsp.data.DDDD[0]
        kwargs = {'DDDDDDDD':tuple(5000 for i in range(16) if (pppp & (1 << i)))}
        verify_command(mn,aa,mn.read_32_bit_pulse_period_group(aa,pppp),**kwargs)

    ppls = tuple(1 for i in range(16) if (mmmm & (1 << i)))
    verify_command(mn,aa,mn.set_io_configuration_group(aa,mmmm,ppls),\
                   'Reconfigure inputs to measure positive pulse duration')
    verify_command(mn,aa,mn.set_output_module_state_group(aa,0,MMMM))
    verify_command(mn,aa,mn.read_and_restart_32_bit_pulse_period_group(aa,mmmm))
    for i in range(16):
        if (MMMM & (1 << i)):
            verify_command(mn,aa,mn.start_on_pulse(aa,i,5000))
    time.sleep(0.5)
    kwargs = {'DDDD':mmmm}
    rsp = verify_command(mn,aa,mn.read_pulse_period_complete_status(aa),**kwargs)
    if rsp.ack == 'A':
        pppp = rsp.data.DDDD[0]
        kwargs = {'DDDDDDDD':tuple(5000 for i in range(16) if (pppp & (1 << i)))}
        verify_command(mn,aa,mn.read_32_bit_pulse_period_group(aa,pppp),**kwargs)


if __name__ == "__main__":                 
    # createthe OmuxNET object
    mn = OmstNET.OmstNET()
    # list the available ttys
    ttys = mn.tty.list_ttys()
    # print a menu
    for tty in ttys:
        print(tty,ttys[tty])
    # ask user to select a port
    ttychoice = int(input('choose a tty by number from the above list: '),10)
    print('\n')
    if ttychoice in ttys:
        baudrates = mn.tty.list_baudrates()
        for baudrate in baudrates:
            print(baudrate,baudrates[baudrate])
        baudratechoice = int(input('choose a baudrate (check brain jumpers): '),10)
        if baudratechoice in mn.tty.baudrates:
            baudrate = int(mn.tty.baudrates[baudratechoice])
        print('\n')
        # open the port
        if mn.tty.open(ttys[ttychoice],int(mn.tty.baudrates[baudratechoice])):
            # build a list of devices by seeing which
            # addresses ACK a 'Power Up Clear' command
            devices = mn.list_mistic_devices()
            for device in devices:
                # for grins, get the identity string
                rsp = mn.what_am_i(device)
                if rsp.ack == 'A':
                    print('{:s} found @ {:02X}'.format(rsp.data.BrainBoardType,device))
            if len(devices) > 0:
                verify_commands(mn,devices[0])
        else:
            print('() open failed'.format(ttys[choice]))
    else:
        print('{:d} : invalid port selection'.format(choice))


