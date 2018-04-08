################
#
# Modul:        RC_Control
# File:         rc_monitor.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2015
# Version:      1.0
#
# Contents:
"""
Remote control for robots
Thread environment and top level Widget
for RCMonitor based on tk
"""
from future import standard_library
standard_library.install_aliases()
from builtins import str
import os
import copy

import tkinter as tk
import tkinter.messagebox

from framework.smart_node import SMONodeInterface as RCInit
from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_init_node import RCInitTkGui
from .rc_frame import RCFrame
from .rc_port_dialog import RCPortDialog
from .rc_edge_dialog import RCInEdgeDialog, RCOutEdgeDialog
from .rc_monitor import  RCWindow


class RCHeader(tk.Frame):

    def __init__(self, master=None, width=rc_util.rc_min_size,
                 height=rc_util.rc_min_size, relief=tk.FLAT, init_data=None, **kwd):

        self.tk_master = master
        self.init_data = init_data
        if self.init_data is None:
            self.init_data = RCInitTkGui()
        self.rc_name = None
        self.rc_path = None

        self.height = height
        self.width = width
        self.default_relief = relief

        self.rc_event = None

        self.create_widgets()

        RCInitTkGui.rc_window_lst.append(self)
        return

# def _update_widgets(self):
# calculate the proper dimensions and control connections ???
# return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master, bd=0)

        self.act_color = self['bg']
        self.columnconfigure(0, weight=1, minsize=rc_util.rc_min_size)
        self.rowconfigure(0, weight=0, minsize=rc_util.rc_min_size)
        self.rowconfigure(1, weight=1, minsize=rc_util.rc_min_size)

        # create comand frame
        self.cmd_frame = tk.Frame(self)
        self.cmd_frame.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)

        # create file menu
        self.cmd_frame.rowconfigure(0, weight=0)    # fixed height
        self.cmd_frame.columnconfigure(0, weight=0)  # fixed width

        self.button_file_menu = tk.Menubutton(self.cmd_frame, text="File")
        self.button_file_menu.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
        self.file_menu = tk.Menu(self.button_file_menu, tearoff=0)
        self.button_file_menu.menu = self.file_menu
        self.button_file_menu["menu"] = self.file_menu

        # self.button_file_menu.menu.add
        self.file_menu.add_command(label="RC Window", command=self.cmd_new_rc_window)
        self.file_menu.add_command(label="New", command=self.cmd_new_rc_window)
        self.file_menu.add_command(label="Open", command=self.cmd_new_rc_window)
        self.file_menu.add_command(label="Save", command=self.cmd_new_rc_window)
        self.file_menu.add_command(label="Close", command=self.cmd_exit)
        self.file_menu.add_command(label="Exit", command=self.cmd_exit_all)

        # connection menue
        self.cmd_frame.rowconfigure(1, weight=0)    # fixed height
        self.cmd_frame.columnconfigure(1, weight=0)  # fixed width

        self.button_connect_menu = tk.Menubutton(self.cmd_frame, text="Connection")
        self.button_connect_menu.grid(row=0, column=1, sticky=tk.N + tk.S + tk.E + tk.W)
        self.connect_menu = tk.Menu(self.button_connect_menu, tearoff=0)
        self.button_connect_menu.menu = self.connect_menu
        self.button_connect_menu["menu"] = self.connect_menu

        self.connect_menu.add_command(label="Inports", command=self.cmd_i_port_dialog)
        self.connect_menu.add_command(label="Outports", command=self.cmd_o_port_dialog)
        self.connect_menu.add_command(label="Inedge", command=self.cmd_i_edge_dialog)
        self.connect_menu.add_command(label="Outedge", command=self.cmd_o_edge_dialog)

        # control menue
        self.cmd_frame.rowconfigure(2, weight=0)    # fixed height
        self.cmd_frame.columnconfigure(2, weight=0)  # fixed width

        self.button_control_menu = tk.Menubutton(self.cmd_frame, text="Control")
        self.button_control_menu.grid(row=0, column=2, sticky=tk.N + tk.S + tk.E + tk.W)
        self.control_menu = tk.Menu(self.button_control_menu, tearoff=0)
        self.button_control_menu.menu = self.control_menu
        self.button_control_menu["menu"] = self.control_menu

        self.control_menu.add_command(label="Register", command=self.cmd_none)
        self.control_menu.add_command(label="Sample", command=self.cmd_none)
        self.control_menu.add_command(label="Start", command=self.cmd_none)
        self.control_menu.add_command(label="Stop", command=self.cmd_none)

        # configur space between buttons
        self.cmd_frame.columnconfigure(3, weight=1)  # width may change aenderbar

        # exit button
        self.cmd_frame.columnconfigure(4, weight=0)  # width may change aenderbar
        self.exit_all_button = tk.Button(self.cmd_frame, text='Exit All', command=self.cmd_exit_all,
                                         relief=tk.FLAT,
                                         highlightbackground=rc_util.rc_l_blue,
                                         highlightcolor=rc_util.rc_l_red,
                                         highlightthickness=4)
        self.exit_all_button.grid(row=0, column=4, sticky=tk.N + tk.S + tk.E + tk.W)

        # create content widget
        self.cont_frame = RCFrame(master=self, init_data=self.init_data,
                                  width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
        self.cont_frame.grid(row=1, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
        self.grid()
        # change bg color iff curser in frame

        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

    def get_installed_widgets(self):
        if self.init_data is None:
            widget_structure = ['Header', {}, None]
        else:
            widget_structure = ['Header', {}, {'rc_path': self.rc_path, 'rc_name': self.rc_name, 'rekursive': False,
                                               'widget_structure': self.rc_slave_widget.get_installed_widgets()}]
        print(widget_structure)
        return widget_structure

    def install_widgets(self, parent=None, rc_path=None, rc_name=None, rekursive=False, widget_structure=None, **kwd):
        if widget_structure is None:
            return

        self.rc_slave_widget_cmd = widget_structure[0]
        widget_cmd = RCInitTkGui.rc_menu_widgets[self.rc_slave_widget_cmd]
        self.rc_slave_widget = widget_cmd(master=self, init_data=self.init_data, **widget_structure[1])
        self.rc_slave_widget.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)
        self.config(bd=rc_util.rc_border, relief=tk.FLAT)
        self.rc_event = None

        if rekursive:
            self.rc_path = rc_path
            if self.rc_path is None or self.rc_path == '':
                self.rc_path = RCInitTkGui.rc_init_tk_path
            self.rc_name = rc_name
            if self.rc_name is not None and self.rc_name != '' and self.rc_name not in RCInit.node_tab:
                rc_fname = os.path.join(self.rc_path, '%s.py' % self.rc_name)
                rc_init_obj = None
                rc_init_obj = RCInitTkGui(smo_init_mode='open', smo_filename=rc_fname, glob=globals())

            if rc_init_obj is None or rc_init_obj.valid == False:
                # create default rc init object and store it in robot_back_fname
                if SMOBaseObject.warning:
                    SMOBaseObject.debug_handler.out(
                        '*** WARNING *** main: try to create a new default initialization')
            if rc_init_obj is None:
                rc_init_obj = RCInitTkGui()

                self.rc_path = RCInitTkGui.rc_init_tk_path
                self.rc_name = rc_util.rc_default_model_name
                if self.rc_name in RCInit.node_tab:
                    self.rc_name = self.rc_name + '_' + str(RCInitTkGui.obj_no)
                tkinter.messagebox.showwarning(parent=self.tk_master, title='Warning',
                                               message="No node data, ceate new %s" % self.rc_name)
                rc_fname = os.path.join(self.rc_path, '%s.py' % self.rc_name)
                rc_init_obj.smo_save(smo_filename=rc_fname)

            if self.rc_slave_widget is not None:
                # achtung aufraeumen
                self.rc_slave_widget.destroy()
                self.rc_slave_widget = None
            RCInit.node_tab[self.rc_name] = rc_init_obj
            selt.init_data = rc_init_obj
            self.rc_slave_widget.install_widgets(parent=self.rc_slave_widget, **rc_init_obj.widget_structure)
        return

    def cmd_none(self):
        # warnig widge
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** info *** RCWindow: None Command or not implemented')
        tkinter.messagebox.showwarning(parent=self.tk_master, title='Warning',
                                       message="Command or not implemented")
        return

    def cmd_new_rc_window(self):
        RCWindow(init_data=self.init_data)
        return

    def cmd_exit(self):
        result = tkinter.messagebox.askquestion(parent=self.tk_master, title='Warning',
                                                message="Do you really want to close this Window?")
        if result == 'yes':
            self.tk_master.destroy()
            RCInitTkGui.rc_window_lst.remove(self)
            if len(RCInitTkGui.rc_window_lst) <= 0:
                self.quit()
            return

    def cmd_exit_all(self):
        result = tkinter.messagebox.askquestion(parent=self.tk_master, title='Warning',
                                                message="Do you really want to exit all RC windows?")
        if result == 'yes':
            self.tk_master.destroy()
            if self in RCInitTkGui.rc_window_lst:
                RCInitTkGui.rc_window_lst.remove(self)
            for x in RCInitTkGui.rc_window_lst:
                try:
                    x.tk_master.destroy()
                except:
                    if SMOBaseObject.info:
                        SMOBaseObject.debug_handler.out(
                            '*** info *** RCMonitor: Exception during destroy gui window')
            self.quit()

    def cmd_i_port_dialog(self):
        port_tab = copy.deepcopy(self.init_data.init_inport)
        RCPortDialog(port_tab, name=self.init_data.rc_name, direction='Input')
        return

    def cmd_o_port_dialog(self):
        port_tab = copy.deepcopy(self.init_data.init_outport)
        RCPortDialog(port_tab, name=self.init_data.rc_name, direction='Output')
        return

    def cmd_i_edge_dialog(self):
        i_edge_tab = copy.deepcopy(self.init_data.in_pred)
        RCInEdgeDialog(i_edge_tab, name=self.init_data.rc_name)
        return

    def cmd_o_edge_dialog(self):
        o_edge_tab = copy.deepcopy(self.init_data.out_succ)
        RCOutEdgeDialog(o_edge_tab, name=self.init_data.rc_name)
        return

    def _cmd_set_highlight(self, event):
        self.rc_event = event
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** header set highlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
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
                '*** INFO *** header reset hightlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_set_highlight()
        self.rc_reset_highlight()
        return

    def rc_reset_highlight(self):
        self.config(bg=self.act_color)
        return
