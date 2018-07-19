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

import logging
import functools
import inspect

__loggingEnabled__ = False

def _init():
    global __loggingEnabled__
    __loggingEnabled = False
    logging.basicConfig(
        format="%(message)s",
        handlers=[logging.StreamHandler()],
        level=logging.INFO
        )
    
def start_logging():
    """
    enable logging
    """
    global __loggingEnabled__
    __loggingEnabled__ = True
    print('logging started')

def stop_logging():
    """
    disable logging
    """
    global __loggingEnabled__
    __loggingEnabled__ = False
    print('logging stopped')
    
def logger(func):
    """
    function decorator that logs the function name and args
    """
    global __loggingEnabled__
    @functools.wraps(func)
    def wrap(self, *args, **kwargs):
        if __loggingEnabled__:
            # get a list of args, kwargs
            arg_names = func.__code__.co_varnames[:func.__code__.co_argcount]
            # if class method, drop leading 'self' because args doesn't contain it
            if arg_names[0] == 'self':
                arg_names = arg_names[1:]
            # build a tuple of name=value pairs
            arg_equals_value = tuple('{}={}'.format(arg_names[i],args[i])\
                                     for i in range(len(args)))
            # construct an appropriate function signature as called
            s = '{}('.format(func.__qualname__)
            for i in range(len(arg_equals_value)):
                if i > 0:
                    s += ','
                s += '{}'.format(arg_equals_value[i])
            if len(kwargs) > 0:
                s += ',{}'.format(str(kwargs))
            s += ')'
            # log the call
            logging.info(s.replace('\\\\','\\'))
            # call the wrapped function
        return func(self, *args, **kwargs)
    return wrap

def log_info_message(msg):
    """
    if logging is enabled, log an info message
    """
    if __loggingEnabled__:
        logging.info(msg)
    
def log_error_message(msg):
    """
    if logging is enabled, log an error message
    """
    if __loggingEnabled__:
        logging.error(msg)

def bytearray_to_hex(data):
    sx = ''
    for b in data:
        sx += '{:02X} '.format(b)
    return sx

def hex_print_bytearray(data):
    print(bytearray_to_hex)
        
"""
a class to use the in operator to see if a value is
in a list of ranges
"""
class RangeList:
    def __init__(self,listOfRanges):
        """
        create self.listOfRanges
        """
        self.listOfRanges = listOfRanges

    def __contains__(self,value):
        """
        See if value is in oune of the ranges in the list
        """
        for r in self.listOfRanges:
            if value in r:
                return True
        return False
    
    def index(self,value):
        """
        return the index of the first range containing the value
        """
        for i in range(len(self.listOfRanges)):
            if value in self.listOfRanges[i]:
                return i
        return -1
