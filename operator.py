#!/usr/bin/env python
#
# 900 Computer Operator
#
# Derived from pyserial miniterm by Andrew Herbert 08/08/2023
#
# Very simple serial terminal
#
# This file is copied from part of pySerial.
# https://github.com/pyserial/pyserial
# (C)2002-2020 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause

from __future__ import absolute_import

import codecs
import os
import sys
import threading
import time

import serial
from serial.tools.list_ports import comports

# pylint: disable=wrong-import-order,wrong-import-position

try:
    raw_input
except NameError:
    # pylint: disable=redefined-builtin,invalid-name
    raw_input = input   # in python3 it's "raw"
    unichr = chr


def key_description(character):
    """generate a readable description for a key"""
    ascii_code = ord(character)
    if ascii_code < 32:
        return 'Ctrl+{:c}'.format(ord('@') + ascii_code)
    else:
        return repr(character)


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class ConsoleBase(object):
    """OS abstraction for console (input/output codec, no echo)"""

    def __init__(self, miniterm):
        self.miniterm = miniterm
        if sys.version_info >= (3, 0):
            self.byte_output = sys.stdout.buffer
        else:
            self.byte_output = sys.stdout
        self.output = sys.stdout

    def setup(self):
        """Set console to read single characters, no echo"""

    def cleanup(self):
        """Restore default console settings"""

    def getkey(self):
        """Read a single key from the console"""
        return None

    def write_bytes(self, byte_string):
        """Write bytes (already encoded)"""
        self.byte_output.write(byte_string)
        self.byte_output.flush()

    def write(self, text):
        """Write string"""
        self.output.write(text)
        self.output.flush()

    def cancel(self):
        """Cancel getkey operation"""

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
    # context manager:
    # switch terminal temporary to normal mode (e.g. to get user input)

    def __enter__(self):
        self.cleanup()
        return self

    def __exit__(self, *args, **kwargs):
        self.setup()


if os.name == 'nt':  # noqa
    import msvcrt
    import ctypes
    import platform

    class Out(object):
        """file-like wrapper that uses os.write"""

        def __init__(self, fd):
            self.fd = fd

        def flush(self):
            pass

        def write(self, s):
            os.write(self.fd, s)

    class Console(ConsoleBase):
        fncodes = {
            ';': '\x1bOP',  # F1
            '<': '\x1bOQ',  # F2
            '=': '\x1bOR',  # F3
            '>': '\x1bOS',  # F4
            '?': '\x1b[15~',  # F5
            '@': '\x1b[17~',  # F6
            'A': '\x1b[18~',  # F7
            'B': '\x1b[19~',  # F8
            'C': '\x1b[20~',  # F9
            'D': '\x1b[21~',  # F10
        }
        navcodes = {
            'H': '\x1b[A',  # UP
            'P': '\x1b[B',  # DOWN
            'K': '\x1b[D',  # LEFT
            'M': '\x1b[C',  # RIGHT
            'G': '\x1b[H',  # HOME
            'O': '\x1b[F',  # END
            'R': '\x1b[2~',  # INSERT
            'S': '\x1b[3~',  # DELETE
            'I': '\x1b[5~',  # PAGE UP
            'Q': '\x1b[6~',  # PAGE DOWN
        }

        def __init__(self, miniterm):
            super(Console, self).__init__(miniterm)
            self._saved_ocp = ctypes.windll.kernel32.GetConsoleOutputCP()
            self._saved_icp = ctypes.windll.kernel32.GetConsoleCP()
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
            ctypes.windll.kernel32.SetConsoleCP(65001)
            # ANSI handling available through SetConsoleMode since Windows 10 v1511 
            # https://en.wikipedia.org/wiki/ANSI_escape_code#cite_note-win10th2-1
            if platform.release() == '10' and int(platform.version().split('.')[2]) > 10586:
                ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
                import ctypes.wintypes as wintypes
                if not hasattr(wintypes, 'LPDWORD'):  # PY2
                    wintypes.LPDWORD = ctypes.POINTER(wintypes.DWORD)
                SetConsoleMode = ctypes.windll.kernel32.SetConsoleMode
                GetConsoleMode = ctypes.windll.kernel32.GetConsoleMode
                GetStdHandle = ctypes.windll.kernel32.GetStdHandle
                mode = wintypes.DWORD()
                GetConsoleMode(GetStdHandle(-11), ctypes.byref(mode))
                if (mode.value & ENABLE_VIRTUAL_TERMINAL_PROCESSING) == 0:
                    SetConsoleMode(GetStdHandle(-11), mode.value | ENABLE_VIRTUAL_TERMINAL_PROCESSING)
                    self._saved_cm = mode
            self.output = codecs.getwriter('UTF-8')(Out(sys.stdout.fileno()), 'replace')
            # the change of the code page is not propagated to Python, manually fix it
            sys.stderr = codecs.getwriter('UTF-8')(Out(sys.stderr.fileno()), 'replace')
            sys.stdout = self.output
            self.output.encoding = 'UTF-8'  # needed for input

        def __del__(self):
            ctypes.windll.kernel32.SetConsoleOutputCP(self._saved_ocp)
            ctypes.windll.kernel32.SetConsoleCP(self._saved_icp)
            try:
                ctypes.windll.kernel32.SetConsoleMode(ctypes.windll.kernel32.GetStdHandle(-11), self._saved_cm)
            except AttributeError:  # in case no _saved_cm
                pass

        def getkey(self):
            while True:
                z = msvcrt.getwch()
                if z == unichr(13):
                    return unichr(10)
                elif z is unichr(0) or z is unichr(0xe0):
                    try:
                        code = msvcrt.getwch()
                        if z is unichr(0):
                            return self.fncodes[code]
                        else:
                            return self.navcodes[code]
                    except KeyError:
                        pass
                else:
                    return z

        def cancel(self):
            # CancelIo, CancelSynchronousIo do not seem to work when using
            # getwch, so instead, send a key to the window with the console
            hwnd = ctypes.windll.kernel32.GetConsoleWindow()
            ctypes.windll.user32.PostMessageA(hwnd, 0x100, 0x0d, 0)

elif os.name == 'posix':
    import atexit
    import termios
    import fcntl
    import signal

    class Console(ConsoleBase):
        def __init__(self, miniterm):
            super(Console, self).__init__(miniterm)
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            atexit.register(self.cleanup)
            signal.signal(signal.SIGINT, self.sigint)
            if sys.version_info < (3, 0):
                self.enc_stdin = codecs.getreader(sys.stdin.encoding)(sys.stdin)
            else:
                self.enc_stdin = sys.stdin

        def setup(self):
            new = termios.tcgetattr(self.fd)
            new[3] = new[3] & ~termios.ICANON & ~termios.ECHO & ~termios.ISIG
            new[6][termios.VMIN] = 1
            new[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, new)

        def getkey(self):
            c = self.enc_stdin.read(1)
            if c == unichr(0x7f):
                c = unichr(8)    # map the BS key (which yields DEL) to backspace
            return c

        def cancel(self):
            fcntl.ioctl(self.fd, termios.TIOCSTI, b'\0')

        def cleanup(self):
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)

        def sigint(self, sig, frame):
            """signal handler for a clean exit on SIGINT"""
            self.miniterm.stop()
            self.cancel()

else:
    raise NotImplementedError(
        'Sorry no implementation for your platform ({}) available.'.format(sys.platform))


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def ask_for_port():
    """\
    Show a list of ports and ask the user for a choice. To make selection
    easier on systems with long device names, also allow the input of an
    index.
    """
    sys.stderr.write('\n--- Available ports:\n')
    ports = []
    for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
        sys.stderr.write('--- {:2}: {:20} {!r}\n'.format(n, port, desc))
        ports.append(port)
    while True:
        sys.stderr.write('--- Enter port index or full name: ')
        port = raw_input('')
        try:
            index = int(port) - 1
            if not 0 <= index < len(ports):
                sys.stderr.write('--- Invalid index!\n')
                continue
        except ValueError:
            pass
        else:
            port = ports[index]
        return port


class Miniterm(object):
    """\
    Terminal application. Copy data from serial port to console and vice versa.
    Handle special keys from the console to show menu etc.
    """

    def __init__(self, serial_instance):
        self.console = Console(self)
        self.serial = serial_instance
        self.echo = True
        self.raw = True
        self.input_encoding = 'UTF-8'
        self.output_encoding = 'UTF-8'
        self.exit_character = unichr(0x1d)  # GS/CTRL+]
        self.menu_character = unichr(0x14)  # Menu: CTRL+T
        self.alive = None
        self._reader_alive = None
        self.receiver_thread = None
        self.reader_file = None # holds uploaded paper tape input
        self.reader_index = 0 # position of next character in reader_file if open
        self.punch_file = None # file to receive paper tape punch output
        self.tty_buffer = None # holds buffered character from console
        self.power_cycle_needed = False
        self.punch_buffer = None
        self.change_speed = False

    def _start_reader(self):
        """Start reader thread"""
        self._reader_alive = True
        # start serial->console thread
        self.receiver_thread = threading.Thread(target=self.reader, name='rx')
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

    def _stop_reader(self):
        """Stop reader thread only, wait for clean exit of thread"""
        self._reader_alive = False
        if hasattr(self.serial, 'cancel_read'):
            self.serial.cancel_read()
        self.receiver_thread.join()

    def start(self):
        """start worker threads"""
        self.alive = True
        self._start_reader()
        # enter console->serial loop
        self.transmitter_thread = threading.Thread(target=self.writer, name='tx')
        self.transmitter_thread.daemon = True
        self.transmitter_thread.start()
        self.console.setup()

    def stop(self):
        """set flag to stop worker threads"""
        self.alive = False

    def join(self, transmit_only=False):
        """wait for worker threads to terminate"""
        self.transmitter_thread.join()
        if not transmit_only:
            if hasattr(self.serial, 'cancel_read'):
                self.serial.cancel_read()
            self.receiver_thread.join()

    def close(self):
        self.serial.close()

    def dump_port_settings(self):
        """Write current settings to sys.stderr"""
        sys.stderr.write("\n--- Settings: {p.name}  {p.baudrate},{p.bytesize},{p.parity},{p.stopbits}\n".format(p=self.serial))

    def add_parity(self, ch):
        """compute parity for 7 bit byte"""
        bit = 0
        parity = False
        x = ch
        while x:
            parity = not parity
            x = x & (x - 1)
        return (ch + 128 if parity else ch )

    def read_from_paper_tape(self):
        """read next character from paper tape, uploading a new file if necessary"""
        buffer = bytearray(b'0')
        warned = False
        while self.alive:
            if self.reader_file:
                # reader file available
                if self.reader_index < len(self.reader_file):
                    # and data available in the file
                    buffer[0] = self.reader_file[self.reader_index]
                    self.reader_index += 1
                    if buffer[0] > 127:
                        sys.stderr.write('\n--- Non-ASCII character ignored\n')
                        sys.stderr.flush()
                        buffer[0] = 255 # replace by DEL
                    if buffer[0] == 255: # DEL is sent as 255, 255
                        self.serial.write(buffer)
                        self.serial.write(buffer)
                    else:
                        self.serial.write(buffer)
                    return # transfer complete
                else:
                    # reached end of file
                    self.reader_file = None # run off end of file
                    
            # Here if either no file uploaded or run off end
            if not warned: # advise the operator
                sys.stderr.write('--- Paper tape reader empty\n')
                sys.stderr.flush()
                warned = True
                
            # wait until file becomes available        
            if not self.alive:
                return
            elif self.power_cycle_needed: # cancel wait on power cycle
                buffer[0] = 255 # reply of 255, 0 cancels the read
                self.serial.write(buffer)
                buffer[0] = 0
                self.serial.write(buffer)
                self.tape_file = None
                sys.stderr.write('--- Paper tape input cancelled\n')
                sys.stderr.flush()
                return # read transfer complete
            else:
                time.sleep(0.1) # continue to wait for file

    def read_from_tty(self):
        """read next character from console"""
        buffer = bytearray(b'0')
        while self.alive:
            if self.tty_buffer:
                # character available
                if self.tty_buffer > 127:
                    sys.stderr.write('\n--- Non-ASCII character ignored\n')
                    sys.stderr.flush()
                    buffer[0] = 255 # replace by DEL
                else:
                    buffer[0] = self.add_parity(self.tty_buffer)
                self.tty_buffer = None
                if buffer[0] == 255: # DEL is sent as 255, 255
                    self.serial.write(buffer)
                    self.serial.write(buffer)
                    return # transfer complete
                else:
                    self.serial.write(buffer)
                    return # transfer complete
                
            elif self.power_cycle_needed: # cancel wait on power cycle
                self.power_cycle_needed = False
                buffer[0] = 255 # reply of 255, 0 cancels the read
                self.serial.write(buffer)
                buffer[0] = 0
                self.serial.write(buffer)
                sys.stderr.write('--- Teletype input cancelled\n')
                sys.stderr.flush()
                return # read transfer complete
            else:
                time.sleep(0.1) # continue to wait for data

    def punch_to_tty(self):
        """take next character from teletype"""
        buffer = bytearray(b'0')
        data = self.serial.read()
        buffer[0] = data[0] & 127 # strip off parity bit
        self.console.write_bytes(buffer)
        # send back ack or power_cycle_needed
        if self.power_cycle_needed:
            self.power_cycle_needed = False
            sys.stderr.write('--- Teletype output cancelled\n')
            sys.stderr.flush()
            buffer[0] = 1
        else:
            buffer[0] = 0
        self.serial.write(buffer)

    def punch_to_tape(self):
        """take next character from paper tape punch"""
        buffer = bytearray(b'0')
        ch = self.serial.read()
        if not self.punch_buffer:
            self.punch_buffer = bytes(ch)
        else:
            self.punch_buffer += ch
        # send back ack or power_cycle_needed
        if self.power_cycle_needed:
            self.power_cycle_needed = False
            sys.stderr.write('--- Teletype input cancelled\n')
            sys.stderr.flush()
            buffer[0] = 1
        else:
            buffer[0] = 0
        serial.write(buffer)

    def power_cycle(self):
        self.power_cycle_needed = False
        self.serial.write(b'N')
        self.tty_buffer = None

    def reader(self):
        """loop and copy serial->console"""
        buffer = bytearray(b'0')
        try:
            while self.alive:
                # read or wait for one byte
                if self.power_cycle_needed:
                    self.power_cycle()
                elif self.change_speed:
                    self.change_speed = False
                    self.serial.write(b'D')
                else:
                    self.serial.timeout = 0.1
                    data = self.serial.read()
                    self.serial.timeout = None
                    if data:
                        ch = chr(data[0])
                        if   ch == '\x00':
                            sys.stderr.write('--- NUL ignored\n')
                            
                        elif ch == 'L': # Logging message
                            self.console.write_bytes(self.serial.read_until())
                            
                        elif ch == 'R': # Read from paper tape
                            self.read_from_paper_tape()
                            
                        elif ch == 'S': # read from teleprinter
                            self.read_from_tty()

                        elif ch == 'P': # punch to paper tape
                            self.punch_to_tape()
 
                        elif ch == 'Q': # punch  teltoetype
                            self.punch_to_tty()
                            
                        else:
                            sys.stderr.write('--- Unexpected code ')
                            sys.stderr.write(str(data[0]))
                            sys.stderr.write('\n')
        except serial.SerialException:
            self.alive = False
            self.console.cancel()
            raise       # XXX handle instead of re-raise?

    def writer(self):
        """\
        Loop and copy console->serial until self.exit_character character is
        found. When self.menu_character is found, interpret the next key
        locally.
        """
        menu_active = False
        try:
            while self.alive:
                try:
                    c = self.console.getkey()
                except KeyboardInterrupt:
                    c = '\x03'
                if not self.alive:
                    break
                if menu_active:
                    self.handle_menu_key(c)
                    menu_active = False
                elif c == self.menu_character:
                    menu_active = True      # next char will be for menu
                elif c == self.exit_character:
                    self.stop()             # exit app
                    break
                elif self.tty_buffer is None:
                    self.tty_buffer=ord(c)
                    self.console.write(c)
        except:
            self.alive = False
            raise

    def handle_menu_key(self, c):
        """Implement a simple menu / settings"""
        if c == self.menu_character or c == self.exit_character:
            # Menu/exit character again -> send itself
            self.serial.write(ord(c[0]))          
        elif c in '\x08hH?':                   # CTRL+H, h, H, ? -> Show help
            sys.stderr.write(self.get_help_text())
        elif c in 'rR':                        # R -> upload paper tape reader file
            self.upload_file()
        elif c in 'pP':                        # P -> save punch output
            self.download_file()
        elif c in 'nN':                        # N -> toggle 920M power
            sys.stderr.write('--- Toggle 920M Power\n')
            self.power_cycle_needed = True
        elif c in 'dD':                        # D -> change device speeds
            self.change_speed = True
            sys.stderr.write("--- Change speed\n")
        elif c in 'fF':                        # F -> flags and variables
            self.status()
        elif c in 'iI':                        # I -> info
            self.dump_port_settings()
        elif c in 'cC':                        # C -> change port
            self.change_port()
        elif c in 'zZ':                        # Z -> suspend / open port temporarily
            self.suspend_port()
        elif c in 'bB':                        # B -> change baudrate
            self.change_baudrate()
        elif c in 'qQ':                        # Q -> exit app
            self.stop()                        
        else:
            sys.stderr.write('--- unknown menu character {}\n'.format(key_description(c)))

    def status(self):
        if not self.reader_file:
            sys.stderr.write('--- No paper tape file loaded\n')
        elif self.reader_index >= len(self.reader_file):
            sys.stderr.write('--- Run off end of paper tape\n')
        if self.punch_buffer:
            sys.stderr.write('--- ' + str(len(punch_buffer)) +
                             ' paper tape characters punched\n');
        if self.tty_buffer:
            sys.stderr.write('--- Teleprinter character buffered\n')
        if self.power_cycle_needed:
            sys.stderr.write('--- Power cycle needed')

    def download_file(self):
        """Ask user for punch file"""
        if not self.punch_buffer: # check there is something to save
            sys.stderr.write('--- Nothing on punch to save\n')
            return

        sys.stderr.write('--- Paper tape output file: ')
        sys.stderr.flush()
        with self.console:
            filename = sys.stdin.readline().rstrip('\r\n')
            if filename:
                try:
                    with open(filename, 'wb') as f:
                        f.write(self.punch_buffer)
                    sys.stderr.write('--- Punch output saved to file {}\n'.format(filename))
                except IOError as e:
                    sys.stderr.write('--- ERROR opening file {}: {}\n'.format(filename, e))
            else:
                sys.stderr.write('--- Paper tape output discarded\n')
            self.punch_buffer = None
            
    def upload_file(self):
        """Ask user for paper tape input file"""
        sys.stderr.write('--- Paper tape input file: ')
        sys.stderr.flush()
        with self.console:
            filename = sys.stdin.readline().rstrip('\r\n')
            if filename:
                try:
                    with open(filename, 'rb') as f:
                        self.reader_file = f.read(-1)
                        self.reader_index = 0
                    sys.stderr.write('--- File {} loaded in paper tape reader\n'.format(filename))
                except IOError as e:
                    sys.stderr.write('--- ERROR opening file {}: {}\n'.format(filename, e))
            else:
                self.reader_file = None
                sys.stderr.write('--- Paper tape reader unloaded\n')

    def change_baudrate(self):
        """change the baudrate"""
        sys.stderr.write('\n--- Baudrate: ')
        sys.stderr.flush()
        with self.console:
            backup = self.serial.baudrate
            try:
                self.serial.baudrate = int(sys.stdin.readline().strip())
            except ValueError as e:
                sys.stderr.write('--- ERROR setting baudrate: {}\n'.format(e))
                self.serial.baudrate = backup
            else:
                self.dump_port_settings()

    def change_port(self):
        """Have a conversation with the user to change the serial port"""
        with self.console:
            try:
                port = ask_for_port()
            except KeyboardInterrupt:
                port = None
        if port and port != self.serial.port:
            # reader thread needs to be shut down
            self._stop_reader()
            # save settings
            settings = self.serial.getSettingsDict()
            try:
                new_serial = serial.serial_for_url(port, do_not_open=True)
                # restore settings and open
                new_serial.applySettingsDict(settings)
                new_serial.rts = self.serial.rts
                new_serial.dtr = self.serial.dtr
                new_serial.open()
                new_serial.break_condition = self.serial.break_condition
            except Exception as e:
                sys.stderr.write('--- ERROR opening new port: {}\n'.format(e))
                new_serial.close()
            else:
                self.serial.close()
                self.serial = new_serial
                sys.stderr.write('--- Port changed to: {}\n'.format(self.serial.port))
            # and restart the reader thread
            self._start_reader()

    def suspend_port(self):
        """\
        open port temporarily, allow reconnect, exit and port change to get
        out of the loop
        """
        # reader thread needs to be shut down
        self._stop_reader()
        self.serial.close()
        sys.stderr.write('\n--- Port closed: {}\n'.format(self.serial.port))
        do_change_port = False
        while not self.serial.is_open:
            sys.stderr.write('--- Quit: {exit} | p: port change | any other key to reconnect\n'.format(
                exit=key_description(self.exit_character)))
            k = self.console.getkey()
            if k == self.exit_character:
                self.stop()             # exit app
                break
            elif k in 'pP':
                do_change_port = True
                break
            try:
                self.serial.open()
            except Exception as e:
                sys.stderr.write('--- ERROR opening port: {}\n'.format(e))
        if do_change_port:
            self.change_port()
        else:
            # and restart the reader thread
            self._start_reader()
            sys.stderr.write('--- Port opened: {}\n'.format(self.serial.port))

    def get_help_text(self):
        """return the help text"""
        # help text, starts with blank line!
        return """
--- 920M Operator - help
---
--- {exit:8} Exit program (alias {menu} Q)
--- {menu:8} Menu escape key, followed by:
--- Menu keys:
---    {menu:7} Send the menu character itself to remote
---    {exit:7} Send the exit character itself to remote
---    
---    H Help
---
----   D Changed device speeds
---    F Flags and variables (debugging information)    
---    R Upload tape reader file (prompt will be shown)
---    P Save punch output to file (prompt will be shown
---    N Power cycle 920M (toggle NOPOWER)
---
---    I Show serial port info
---    S change serial port
---    B change baud rate
""".format(version=getattr(serial, 'VERSION', 'unknown version'),
           exit=key_description(self.exit_character),
           menu=key_description(self.menu_character))


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# default args can be used to override when calling main() from an other script
# e.g to create a miniterm-my-device.py
def main(default_port=None, default_baudrate=115250,  serial_instance=None):
    """Command line tool, entry point"""

    import argparse

    parser = argparse.ArgumentParser(
        description='Miniterm - A simple terminal program for the serial port.')

    parser.add_argument(
        'port',
        nargs='?',
        help='serial port name ("-" to show port list)',
        default=default_port)

    parser.add_argument(
        'baudrate',
        nargs='?',
        type=int,
        help='set baud rate, default: %(default)s',
        default=default_baudrate)

    group = parser.add_argument_group('port settings')

    group.add_argument(
        '--non-exclusive',
        dest='exclusive',
        action='store_false',
        help='disable locking for native ports',
        default=True)

    group.add_argument(
        '--ask',
        action='store_true',
        help='ask again for port when open fails',
        default=False)

    group = parser.add_argument_group('hotkeys')

    group.add_argument(
        '--exit-char',
        type=int,
        metavar='NUM',
        help='Unicode of special character that is used to exit the application, default: %(default)s',
        default=0x1d)  # GS/CTRL+]

    group.add_argument(
        '--menu-char',
        type=int,
        metavar='NUM',
        help='Unicode code of special character that is used to control miniterm (menu), default: %(default)s',
        default=0x14)  # Menu: CTRL+T

    group = parser.add_argument_group('diagnostics')

    group.add_argument(
        '-q', '--quiet',
        action='store_true',
        help='suppress non-error messages',
        default=False)

    group.add_argument(
        '--develop',
        action='store_true',
        help='show Python traceback on error',
        default=False)

    args = parser.parse_args()

    if args.menu_char == args.exit_char:
        parser.error('--exit-char can not be the same as --menu-char')

    while serial_instance is None:
        # no port given on command line -> ask user now
        if args.port is None or args.port == '-':
            try:
                args.port = ask_for_port()
            except KeyboardInterrupt:
                sys.stderr.write('\n')
                parser.error('user aborted and port is not given')
            else:
                if not args.port:
                    parser.error('port is not given')
        try:
            serial_instance = serial.serial_for_url(
                args.port,
                args.baudrate,
                parity='N', 
                rtscts=0,   
                xonxoff=0,  
                do_not_open=True)

            if not hasattr(serial_instance, 'cancel_read'):
                # enable timeout for alive flag polling if cancel_read is not available
                serial_instance.timeout = 1

            if isinstance(serial_instance, serial.Serial):
                serial_instance.exclusive = args.exclusive

            serial_instance.open()
        except serial.SerialException as e:
            sys.stderr.write('could not open port {!r}: {}\n'.format(args.port, e))
            if args.develop:
                raise
            if not args.ask:
                sys.exit(1)
            else:
                args.port = '-'
        else:
            break

    miniterm = Miniterm(serial_instance)
    miniterm.exit_character = unichr(args.exit_char)
    miniterm.menu_character = unichr(args.menu_char)
    
    if not args.quiet:
        sys.stderr.write('--- 920M Operator on {p.name}  {p.baudrate},{p.bytesize},{p.parity},{p.stopbits}\n'.format(
            p=miniterm.serial))
        sys.stderr.write('--- Quit: {} | Menu: {} | Help: {} followed by {} ---\n'.format(
            key_description(miniterm.exit_character),
            key_description(miniterm.menu_character),
            key_description(miniterm.menu_character),
            key_description('\x08')))

    miniterm.start()
    try:
        miniterm.join(True)
    except KeyboardInterrupt:
        pass
    if not args.quiet:
        sys.stderr.write('\n--- exit\n')
    miniterm.join()
    miniterm.close()


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if __name__ == '__main__':
    main()
