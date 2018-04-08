from future import standard_library
standard_library.install_aliases()
################
#
# Modul:        PaderKicker Monitor
# File:         pk_monitor_main_window.py
#
# Author:       Bernd Kleinjohann
# Created:      June 2011
# Version:      2.0
# Contents:     this class contains all common data for themonitor thread
#
import tkinter as tk
import tkinter.messagebox

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_init_node import RCInitTkGui
from .rc_frame import RCFrame
from .rc_port_dialog import RCPortDialog


class RCSend(tk.LabelFrame):

    def __init__(self, master=None, init_data=None, init_inport=None, init_outport=None,
                 width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT, cmd_side='s', **kwd):

        self.tk_master = master
        self.init_data = init_data

        self.height = height
        self.width = width
        self.default_relief = relief
        self.rc_event = None

        self.cmd_side = cmd_side
        self.cmd_frame = None

        self.rc_slave_widget = None
        self.rc_slave_widget_cmd = None
        self.cmd_frame = None

        self.local_init_inport = {}
        self.local_internal_inport = {}
        self.local_external_inport = {}
        if init_inport is not None:
            self.init_data.update_port_tab(init_inport, self.local_init_inport,
                                           self.local_internal_inport, self.local_external_inport)
        self.local_init_outport = {}
        self.local_internal_outport = {}
        self.local_external_outport = {}
        if init_outport is not None:
            self.init_data.update_port_tab(init_outport, self.local_init_outport,
                                           self.local_internal_outport, self.local_external_outport)
        self.create_widgets()
        return

    def create_widgets(self):
        tk.LabelFrame.__init__(self, self.tk_master, width=self.width,
                               height=self.height, relief=self.default_relief)
        self.tk_master['bg'] = rc_util.rc_bg_frame_color
        self.tk_master.act_color = rc_util.rc_bg_frame_color

        self.act_color = self['bg']
        self.rowconfigure(0, weight=1, minsize=rc_util.rc_min_size)
        self.columnconfigure(0, weight=1, minsize=rc_util.rc_min_size)
        if self.cmd_side in ('nw', 'n', 'ne', 'sw', 's', 'se'):
            self.vert = False
        elif self.cmd_side in ('wn', 'w', 'ws', 'en', 'e', 'es'):
            self.vert = True
        else:
            self.cmd_side = 's'
            self.vert = False

        # create label Buttons
        self.cmd_frame = tk.Frame(self, bd=0, padx=0, pady=0, relief=tk.FLAT, bg=rc_util.rc_bg_color)

        self.i_port_button = tk.Button(self.cmd_frame, text='Input Ports', command=self.cmd_i_port_dialog)
        self.i_port_button.config(bd=0, activebackground=rc_util.rc_button_color, relief=tk.FLAT)
        self.o_port_button = tk.Button(self.cmd_frame, text='Output Ports', command=self.cmd_o_port_dialog,
                                       bd=0, activebackground=rc_util.rc_button_color, relief=tk.FLAT)
        self.orient_button = tk.Button(self.cmd_frame, text='CMD orient', command=self.cmd_orient,
                                       padx=0, pady=0, bd=0, activebackground=rc_util.rc_button_color, relief=tk.FLAT)
        self.send_button = tk.Button(self.cmd_frame, text='Send', command=self.cmd_send,
                                     padx=0, pady=0, bd=0, activebackground=rc_util.rc_button_color, relief=tk.FLAT)

        if self.cmd_side in ('nw', 'n', 'ne', 'sw', 's', 'se'):
            self.vert = True
        elif self.cmd_side in ('wn', 'w', 'ws', 'en', 'e', 'es'):
            self.vert = False
        if self.vert:
            side = tk.LEFT
        else:
            side = tk.TOP
        self.i_port_button.pack(side=side)
        self.o_port_button.pack(side=side)
        self.orient_button.pack(side=side)
        self.send_button.pack(side=side)

        self.config(labelwidget=self.cmd_frame, labelanchor=self.cmd_side)

        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.rc_slave_widget = RCFrame(master=self, init_data=self.init_data,
                                       width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
        self.rc_slave_widget.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)

   # change bg color iff curser in frame
        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

    def get_installed_widgets(self):
        if self.rc_slave_widget is None:
            widget_structure = ['Send', {'init_inport': self.local_init_inport,
                                         'init_outport': self.local_init_outport}, None]
        else:
            widget_structure = ['Send', {'init_inport': self.local_init_inport, 'init_outport': self.local_init_outport},
                                {'widget_structure': self.rc_slave_widget.get_installed_widgets()}]
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        if widget_structure is None:
            return

        if self.rc_slave_widget is not None:
            self.rc_slave_widget.destroy()
            self.rc_slave_widget = None

        self.rc_slave_widget_cmd = widget_structure[0]
        widget_cmd = RCInitTkGui.rc_menu_widgets[self.rc_slave_widget_cmd][0]
        self.rc_slave_widget = widget_cmd(master=self, init_data=self.init_data, **widget_structure[1])
        self.rc_slave_widget.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)
        self.config(bd=rc_util.rc_border, relief=tk.FLAT)
        self.rc_event = None

        if widget_structure[2] is not None:
            self.rc_slave_widget.install_widgets(**widget_structure[2])
        return

    def connect_widgets(self, id_n, get_src, set_src, **kwd):
        if self.rc_slave_widget is None:
            return id_n, get_src, set_src
        else:
            ###
            # neue code_tabs aufsetzen und code fuer die Ports erzeugen
            ###
            return self.rc_slave_widget.connect_widgets(id_n, get_src, set_src, **kwd)

    def i_port_ret_fkt(self, port_tab):
        if port_tab is not None:
            self.init_data.update_port_tab(port_tab, self.local_init_inport, self.local_internal_inport,
                                           self.local_external_inport, reset=True)

    def cmd_i_port_dialog(self):
        RCPortDialog(self.local_init_inport, ret_fkt=self.i_port_ret_fkt,
                     name=self.init_data.rc_name + ' (local)', direction='Input')
        return

    def o_port_ret_fkt(self, port_tab):
        if port_tab is not None:
            self.init_data.update_port_tab(port_tab, self.local_init_outport, self.local_internal_outport,
                                           self.local_external_outport, reset=True)

    def cmd_o_port_dialog(self):
        RCPortDialog(self.local_init_outport, ret_fkt=self.o_port_ret_fkt,
                     name=self.init_data.rc_name + ' (local)', direction='Output')
        return

    def cmd_send(self):
        # warnig widge
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** info *** RCWindow: Send Command or not implemented')
        tkinter.messagebox.showwarning(parent=self.tk_master, title='Warning',
                                       message="Send command not implemented")
        return

    def _cmd_set_highlight(self, event):
        self.rc_event = event
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** frame set highlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_reset_highlight()
        self.rc_set_highlight()
        return

    def rc_set_highlight(self):
        self.config(bg=rc_util.rc_hl_color)
        return

    def _cmd_reset_highlight(self, event):
        self.rc_event = event
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** frame reset hightlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_set_highlight()
        self.rc_reset_highlight()
        return

    def rc_reset_highlight(self):
        self.config(bg=self.act_color)
        return

    def update_layout(self):
        self.cmd_frame.pack_forget()
        if self.cmd_side in ('nw', 'n', 'ne', 'sw', 's', 'se'):
            self.vert = False
        elif self.cmd_side in ('wn', 'w', 'ws', 'en', 'n', 'es'):
            self.vert = True

        if self.vert:
            side = tk.TOP
        else:
            side = tk.LEFT
        self.i_port_button.pack(side=side)
        self.o_port_button.pack(side=side)
        self.orient_button.pack(side=side)
        self.send_button.pack(side=side)

        self.config(labelwidget=self.cmd_frame, labelanchor=self.cmd_side)
        return

    def cmd_orient(self):
        anchor = ['s', 'sw', 'ws', 'w', 'wn', 'nw', 'n', 'ne', 'e', 'en', 'es', 'se']
        i = anchor.index(self.cmd_side)
        if i >= len(anchor) - 1:
            self.cmd_side = anchor[0]
        else:
            self.cmd_side = anchor[i + 1]
        self.update_layout()
        return
