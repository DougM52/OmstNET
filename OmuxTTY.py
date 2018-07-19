##Copyright (c) 2018 Douglas E. Moore
##dougmo52@gmail.com
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

import termios                      # configuring the serialport
import struct                       # reading output from ioctls
import re                           # parsing directories to find ttys
import fcntl                        # getting awaiting byte count for os.read
import os                           # open, read, write, close
from   functools import cmp_to_key  # sorting tty names
import OmuxUTL as utl               # various utilities like logging

class OmuxTTY:
    
    def __init__(self):
        self.tty = None
        self.fd = None
        self.baud = None
        
    """
    field indices in termios structure
    """
    offset_termios_flags = [
        'iflag',
        'oflag',
        'cflag',
        'lflag',
        'ispeed',
        'ospeed',
        'cc'
    ]

    """
    field indices of cc values in termios structure
    """
    offset_termios_cc = [
        'VINTR',
        'VQUIT',
        'VERASE',
        'VKILL',
        'VEOF',
        'VTIME',
        'VMIN',
        'VSWTC',
        'VSTART',
        'VSTOP',
        'VSUSP',
        'VEOL',
        'VREPRINT',
        'VDISCARD',
        'VWERASE',
        'VLNEXT',
        'VEOL2'
    ]        

    """
    baudrates
    """
    baudrates = (
        '300',
        '600',
        '1200',
        '2400',
        '4800',
        '9600',
        '19200',
        '38400',
        '57600',
        '76800',
        '115200'
        )
    
    @staticmethod
    def compare_ttys(tty1,tty2):
        """
        callback that sorts ttys first by name prefix, then by numeric suffix
        ['ttyS0', 'ttyUSB0', 'ttyUSB1']
        """
        p = re.compile("^(tty(S|USB))(\d+)$")
        m1 = p.match(tty1)
        m2 = p.match(tty2)
        if m1.group(1) == m2.group(1):
            return int(m1.group(3)) - int(m2.group(3))
        else:
            if m1.group(1) < m2.group(1):
                return -1
            else:
                if m1.group(1) > m2.group(1):
                    return 1
            return 0

    @staticmethod
    def list_ttys():
        """
        list ttys that were found on PCI bus at system boot
        and also by PNP such as FTDI USB devices which may
        come and go
        """
        ttys = []
        base = "/sys/devices"
        rx = "^.*(pnp|pci).*/tty/(tty(S|USB)\d+)$"
        p1 = re.compile(rx)
        for root, dirs, files in os.walk(base):
            m = p1.match(root)
            if m:
                ttys.append(m.group(2))
        ttys = sorted(ttys,key=cmp_to_key(OmuxTTY().compare_ttys))
        return {key: '/dev/'+ttys[key] for key in range(len(ttys))}

    @staticmethod
    def list_baudrates():
        return {key:OmuxTTY.baudrates[key] for key in range(len(OmuxTTY.baudrates))}
    
    @utl.logger
    def cfsetspeed(self,tios,speed):
        """
        put the baudrate in the termios speed members
        """
        tios[self.offset_termios_flags.index('ispeed')] = speed
        tios[self.offset_termios_flags.index('ospeed')] = speed
        self.baud = speed
        return tios

    @utl.logger
    def open(self,tty,baud=38400):
        """
        open the specified tty at the specified baudrate
        """
        if self.tty != None:
            raise ValueError('tty is allready open on {:s}'.format(self.tty))
        try:
            fd = os.open(tty,os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            if fd:
                tios = termios.tcgetattr(fd)
                tios[self.offset_termios_flags.index('iflag')] = 0
                tios[self.offset_termios_flags.index('oflag')] = 0
                tios[self.offset_termios_flags.index('lflag')] = 0
                tios[self.offset_termios_flags.index('cflag')] = \
                    getattr(termios,'B{:d}'.format(baud)) \
                            | termios.CLOCAL \
                            | termios.CREAD \
                            | termios.CS8
                tios[self.offset_termios_flags.index('cc')][self.offset_termios_cc.index('VMIN')] = 1
                tios[self.offset_termios_flags.index('cc')][self.offset_termios_cc.index('VTIME')] = 0   
                self.cfsetspeed(tios,getattr(termios,'B{:d}'.format(baud)))
                termios.tcsetattr(fd,termios.TCSAFLUSH,tios)
                self.tty = tty
                self.fd = fd
                self.baud = baud
        except:
            if os.istty(fd):
                os.close(fd)
                return False
        finally:
            print('opened \'{:s}:{:d},N,8,1\''.format(self.tty,self.baud))
            return True

    @utl.logger
    def close(self):
        """
        Close the posr
        """
        if self.fd != None \
           and os.isatty(self.fd):
            os.close(self.fd)
            print('closed \'{:s}\''.format(self.tty))
            self.tty = None
            self.fd = None
        
    @utl.logger
    def write(self,pkt):
        """
        write data to port
        """
        # send the bytes
        os.write(self.fd,pkt.encode())

    @utl.logger
    def read(self,n):
        """
        read data from port
        """
        return os.read(self.fd,n).decode()

    @utl.logger
    def rx_bytes_available(self):
        """
        only safe way to prevent read blocking is to make sure
        there is something to be read
        """
        if os.isatty(self.fd):
            #unlikely to get here if nothing to read, but just in case
            s = fcntl.ioctl(self.fd, termios.TIOCINQ,struct.pack('I',0))
            #how many bytes available?
            n = struct.unpack('I', s)[0]
            return n
        return 0

    @utl.logger
    def flush_input_buffer(self):
        """
        optomux responses do not maintain any type of message number
        so the only way to make sure to tie a command to a response
        is to make sure the input buffer is empty before sending a
        command.
        """
        while True:
            n = self.rx_bytes_available()
            if n != 0:
                self.read(n)
            else:
                break

          
