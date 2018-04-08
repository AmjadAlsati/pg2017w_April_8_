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
import os
import copy
import threading

import Tkinter as tk
import tkMessageBox
import tkFileDialog
import tkSimpleDialog

from framework.robots.rc_simple_local_robot import RCLocalRobot_1
from framework.robots.rc_simple_call_robot import RCCallRobotConnect_1
from framework.smart_node import SMONodeInterface as RCInit
from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_init_node import RCInitTkGui
from .rc_frame import RCFrame
from .rc_port_dialog import RCPortDialog
from .rc_edge_dialog import RCInEdgeDialog, RCOutEdgeDialog


class RCHeader(tk.Frame):

    def __init__(self, robot_host, robot_port, master=None, width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT, gui_filename=None, init_data=None, rekursive=False, **kwd):

        self.tk_master = master
        self.init_data = init_data

        self._robot_host = robot_host
        self._robot_port = robot_port
        self._gui_filename = gui_filename

# if self.init_data == None:
##            self.init_data = ()

        self.init_data = RCInitTkGui()
        self.init_data.rc_header_widget = self
        self.init_data.rc_top_level_widget = None

        self.rc_path = self.init_data.rc_path
        self.rc_name = self.init_data.rc_name
        self.connection_name = self.init_data.connection_name
        self.rekursive = rekursive

        self.height = height
        self.width = width
        self.default_relief = relief

        self.rc_event = None

        self.create_widgets()

        # for test
        self.rob = None

        if self._gui_filename is not None:
            self.cmd_open_init_data(self._gui_filename)
        return

# def _update_widgets(self):
# calculate the proper dimensions and control connections ???
# return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master, bd=0)

        self.tk_master['bg'] = rc_util.rc_bg_frame_color
        self.tk_master.act_color = rc_util.rc_bg_frame_color

        self['bg'] = rc_util.rc_bg_color
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
        self.button_file_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)
        self.button_file_menu.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
        self.file_menu = tk.Menu(self.button_file_menu, tearoff=0)
        self.file_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                              activeforeground=rc_util.rc_black)
        self.button_file_menu.menu = self.file_menu
        self.button_file_menu["menu"] = self.file_menu

        self.file_menu.add_command(label="RC Window", command=self.cmd_new_rc_window)
        self.file_menu.add_command(label="Close Window", command=self.cmd_exit)
        self.file_menu.add_command(label="Exit all Windows", command=self.cmd_exit_all)
        # self.file_menu.add_separator()

        self.cmd_frame.columnconfigure(1, weight=0)  # fixed width
        self.cmd_frame.rowconfigure(0, weight=0)    # fixed height
        self.button_widget_menu = tk.Menubutton(self.cmd_frame, text="Widget")
        self.button_widget_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)
        self.button_widget_menu.grid(row=0, column=1, sticky=tk.N + tk.S + tk.E + tk.W)
        self.widget_menu = tk.Menu(self.button_widget_menu, tearoff=0)
        self.widget_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                                activeforeground=rc_util.rc_black)
        self.button_widget_menu.menu = self.widget_menu
        self.button_widget_menu["menu"] = self.widget_menu

        self.widget_menu.add_command(label="New Widget", command=self.cmd_new_init_data)
        self.widget_menu.add_command(label="Open Widget", command=self.cmd_open_init_data)
        self.widget_menu.add_command(label="Save Widget", command=self.cmd_save_init_data)
        self.widget_menu.add_command(label="Close Widget", command=self.cmd_close_init_data)

        # connection menue
        # self.cmd_frame.rowconfigure(1,weight=0)    # fixed height
        self.cmd_frame.columnconfigure(2, weight=0)  # fixed width

        self.button_connect_menu = tk.Menubutton(self.cmd_frame, text="Connection")
        self.button_connect_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)
        self.button_connect_menu.grid(row=0, column=2, sticky=tk.N + tk.S + tk.E + tk.W)
        self.connect_menu = tk.Menu(self.button_connect_menu, tearoff=0)
        self.connect_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                                 activeforeground=rc_util.rc_black)

        self.button_connect_menu.menu = self.connect_menu
        self.button_connect_menu["menu"] = self.connect_menu

        self.connect_menu.add_command(label="Inports", command=self.cmd_i_port_dialog)
        self.connect_menu.add_command(label="Outports", command=self.cmd_o_port_dialog)
        self.connect_menu.add_command(label="Inedge", command=self.cmd_i_edge_dialog)
        self.connect_menu.add_command(label="Outedge", command=self.cmd_o_edge_dialog)

        # control menue
        # self.cmd_frame.rowconfigure(3,weight=0)    # fixed height
        self.cmd_frame.columnconfigure(3, weight=0)  # fixed width

        self.button_control_menu = tk.Menubutton(self.cmd_frame, text="Control")
        self.button_control_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)

        self.button_control_menu.grid(row=0, column=3, sticky=tk.N + tk.S + tk.E + tk.W)
        self.control_menu = tk.Menu(self.button_control_menu, tearoff=0)
        self.control_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                                 activeforeground=rc_util.rc_black)
        self.button_control_menu.menu = self.control_menu
        self.button_control_menu["menu"] = self.control_menu

        self.control_menu.add_command(label="Register", command=self.cmd_register)
        self.control_menu.add_command(label="Update", command=self.cmd_update)
        self.control_menu.add_command(label="Step", command=self.cmd_step)
        self.control_menu.add_command(label="Start", command=self.cmd_start)
        self.control_menu.add_command(label="Stop", command=self.cmd_stop)

        # node control menu
        self.cmd_frame.columnconfigure(4, weight=0)  # fixed width

        self.button_node_menu = tk.Menubutton(self.cmd_frame, text="Robot Nodes")
        self.button_node_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)

        self.button_node_menu.grid(row=0, column=4, sticky=tk.N + tk.S + tk.E + tk.W)
        self.node_menu = tk.Menu(self.button_node_menu, tearoff=0)
        self.node_menu.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                              activeforeground=rc_util.rc_black)
        self.button_node_menu.menu = self.node_menu
        self.button_node_menu["menu"] = self.node_menu

        self.node_menu.add_command(label="Local Robot", command=self.cmd_simple_local_robot)
        self.node_menu.add_command(label="Local Robot Start", command=self.cmd_start_simple_local_robot)
        self.node_menu.add_command(label="Local Robot Stop", command=self.cmd_stop_simple_local_robot)

        self.node_menu.add_command(label="Call Robot", command=self.cmd_simple_call_robot)
        self.node_menu.add_command(label="Call Robot Start", command=self.cmd_start_simple_call_robot)
        self.node_menu.add_command(label="Call Robot Stop", command=self.cmd_stop_simple_call_robot)

        self.node_menu.add_command(label="Pi Robot", command=self.cmd_pi_robot)
        self.node_menu.add_command(label="Pi Robot Start", command=self.cmd_start_pi_robot)
        self.node_menu.add_command(label="Pi Robot Stop", command=self.cmd_stop_pi_robot)

        # configur space between buttons
        self.cmd_frame.columnconfigure(5, weight=1)  # width may change aenderbar

        # exit button
        self.cmd_frame.columnconfigure(6, weight=0)  # width may change aenderbar
        self.exit_all_button = tk.Button(self.cmd_frame, text='Exit All', command=self.cmd_exit_all)
        self.exit_all_button.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)
        self.exit_all_button.grid(row=0, column=6, sticky=tk.N + tk.S + tk.E + tk.W)

        # create content widget
        self.cont_frame = RCFrame(master=self, init_data=self.init_data,
                                  width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
        self.cont_frame.grid(row=1, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
        # self.grid()
        # change bg color iff curser in frame

        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

# def destroy(self):
# Frame.destroy(self)
# print 'yyy'
##        if self.i_port_dialog != None: self.i_port_dialog.destroy()
##        if self.o_port_dialog != None: self.o_port_dialog.destroy()
##        if self.i_edge_dialog != None: self.i_edge_dialog.destroy()
##        if self.o_edge_dialog != None: self.o_edge_dialog.destroy()
# print 'xxxx'
# Frame.destroy(self)
# print 'zzzz'
# return

    def get_installed_widgets(self):
        if self.rekursive:
            widget_structure = ['Header', {}, {'rc_path': self.rc_path,
                                               'rc_name': self.rc_name,
                                               'connection_name': self.connection_name,
                                               'rekursive': self.rekursive}]
        else:
            widget_structure = ['Header', {}, {'rc_path': self.rc_path,
                                               'rc_name': self.rc_name,
                                               'connection_name': self.connection_name,
                                               'rekursive': self.rekursive,
                                               'init_inport': self.init_data.init_inport,
                                               'init_in_pred': self.init_data.init_in_pred,
                                               'init_outport': self.init_data.init_outport,
                                               'init_out_succ': self.init_data.init_out_succ,
                                               'widget_structure': self.cont_frame.get_installed_widgets()}]
        return widget_structure

    def install_widgets(self, rc_path=None, rc_name=None, connection_name=None,
                        init_inport=None, init_in_pred=None, init_outport=None, init_out_succ=None,
                        rekursive=False, widget_structure=None, **kwd):

        if rc_path is not None and rc_path != '':
            self.rc_path = rc_path
        else:
            self.rc_path = None

        if rc_name is not None and rc_name != '':
            self.rc_name = rc_name
        else:
            self.rc_name = None

        if self.connection_name is not None and self.connection_name != '':
            self.connection_name = self.connection_name
        else:
            self.rc_name = None

        if rekursive:
            if self.rc_name is not None and self.rc_name is not None:
                rc_fname = os.path.join(self.rc_path, '%s.py' % self.rc_name)
                rc_init_obj = None
                rc_init_obj = RCInitTkGui(smo_init_mode='open', smo_filename=rc_fname, glob=RCInitTkGui.glob)
            if rc_init_obj is None or not isinstance(rc_init_obj, RCInitTkGui):
                tkMessageBox.showwarning(parent=self.tk_master, title='Warning',
                                         message="No node data, ceate new %s" % self.rc_name)
                self.cmd_new_init_data()
                return
            self.init_data = rc_init_obj
            self.init_data.rc_header_widget = self
            rc_init_obj.rc_path = self.rc_path
            rc_init_obj.rc_name = self.rc_name
            # install widgets
            if self.cont_frame is not None:
                self.cont_frame.destroy()
            self.cont_frame = RCFrame(master=self, init_data=self.init_data,
                                      width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
            self.cont_frame.grid(row=1, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
            self.cont_frame.install_widgets(**widget_structure[2])
            # etwas fehlt in der struktur!!!
            # self.cont_frame.install_widgets(**widget_structure)

        else:
            self.init_data = RCInitTkGui()
            self.init_data.rc_header_widget = self
            if self.rc_path is not None and self.rc_path != '':
                self.init_data.rc_path = self.rc_path
            if self.rc_name is not None and self.rc_name != '':
                self.init_data.rc_name = self.rc_name
            if self.connection_name is not None and self.connection_name != '':
                self.init_data.connection_name = self.rc_name

            if init_inport is not None:
                self.init_data.update_port_tab(init_inport, self.init_data.init_inport, self.init_data.internal_inport,
                                               self.init_data.external_inport, reset=True)
            if init_in_pred is not None:
                self.init_data.update_connection_tab(init_in_pred, self.init_data.init_in_pred,
                                                     self.init_data.in_pred, reset=True)
            if init_outport is not None:
                self.init_data.update_port_tab(init_outport, self.init_data.init_outport, self.init_data.internal_outport,
                                               self.init_data.external_outport, reset=True)
            if init_out_succ is not None:
                self.init_data.update_connection_tab(init_out_succ, self.init_data.init_out_succ,
                                                     self.init_data.out_succ, reset=True)

            if self.cont_frame is not None:
                self.cont_frame.destroy()
            self.cont_frame = RCFrame(master=self, init_data=self.init_data,
                                      width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
            self.cont_frame.grid(row=1, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
            print widget_structure
            if widget_structure is not None:
                self.cont_frame.install_widgets(**widget_structure[2])
        return

    def connect_widgets(self, id_n, get_src, set_src, recursive=False, **kwd):
        if recursive == False or self.cont_frame == None:
            return id_n, get_src, set_src
        else:
            return self.cont_frame.connect_widgets(id_n, get_src, set_src, **kwd)


# def connect_widgets(get_code_tab, set_code_tab, id_n):
# if self.recursive:
# self.init:data.connect_widgets()
###
# neue code_tabs aufsetzen und code fuer die Ports init_data erzeugen
###
# return get_code_tab, set_code_tab, id_n

    def generate_widget_connect(self):
        # generierung der get set funktionen

        if self.cont_frame is None:
            return
        # for input/output execution
        self.local_dict = {'get_input_fkt': None, 'set_output_fkt': None}
        self.global_dict = {'copy': copy}
        self.get_src = ''
        self.set_src = ''
        self.widget_init_tab = {}
        id_n = 0
        id_n, self.get_src, self.set_src =\
            self.cont_frame.connect_widgets(id_n, self.get_src, self.set_src,
                                            widget_init_tab=self.widget_init_tab,
                                            init_inport=self.init_data.init_inport,
                                            init_outport=self.init_data.init_outport)

        self.init_data.widget_init_tab = self.widget_init_tab

        get_prefix = "def get_input(port_tab, get_tab):\n"
        get_postfix = "    return\nget_input_fkt = get_input\n"
        self.get_src = get_prefix + self.get_src + get_postfix
###
###
        print '+++ get_source'
        print self.get_src, '\n'
        exec self.get_src in self.global_dict, self.local_dict
        self.init_data.get_input_fkt = self.local_dict['get_input_fkt']

        set_prefix = "def set_output(port_tab, set_tab):\n"
        set_postfix = "    return\nset_output_fkt = set_output\n"
        self.set_src = set_prefix + self.set_src + set_postfix

###
###
        print '+++ set_source'
        print self.set_src, '\n'
        exec self.set_src in self.global_dict, self.local_dict
        self.init_data.set_output_fkt = self.local_dict['set_output_fkt']
        return

    def cmd_none(self):
        # warnig widge
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** info *** RCWindow: None Command or not implemented')
        tkMessageBox.showwarning(parent=self.tk_master, title='Warning',
                                 message="Command not implemented")
        return

    def cmd_new_rc_window(self):
        rc_init_obj = RCInitTkGui()
        RCWindow(init_data=rc_init_obj)
        return

    def cmd_exit(self):
        result = tkMessageBox.askquestion(parent=self, title='Warning',
                                          message="Do you really want to close this Window?")
        if result == 'yes':
            # self.init_data.rc_top_level_widget.tk_master.destroy()
            # RCInitTkGui.rc_window_lst.remove(self.init_data.rc_top_level_widget)
            top_level = self.winfo_toplevel()
            RCInitTkGui.rc_window_lst.remove(top_level)
            top_level.destroy()
            if len(RCInitTkGui.rc_window_lst) <= 0:
                self.quit()
            return

    def cmd_exit_all(self):
        result = tkMessageBox.askquestion(parent=self, title='Warning',
                                          message="Do you really want to exit all RC windows?")
        if result == 'yes':
            for x in RCInitTkGui.rc_window_lst:
                try:
                    # x.tk_master.destroy()
                    x.destroy()
                except:
                    if SMOBaseObject.error:
                        SMOBaseObject.debug_handler.out(
                            '*** info *** RCMonitor: Exception during destroy gui window')
            RCInitTkGui.rc_window_lst = []
            self.quit()
        return

###
    def cmd_new_init_data(self):
        self.init_data = RCInitTkGui()
        self.init_data.rc_header_widget = self
        self.rc_path = self.init_data.rc_path
        self.rc_name = self.init_data.rc_name

        self.cont_frame.destroy()
        self.cont_frame = RCFrame(master=self, init_data=self.init_data,
                                  width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
        self.cont_frame.grid(row=1, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
        return

    def cmd_open_init_data(self, filename=None):
        name = self.rc_name + '.py'
        if filename is None:
            fname = tkFileDialog.askopenfilename(parent=self, title='Load GUI initialization data',
                                             initialdir=self.rc_path, initialfile=name,
                                             defaultextension='.py', filetypes=[('Init Data', ".py"), ('Alles', "*.*")])
        else:
            fname = filename

        print fname
            
        if fname is None or fname == '':
            return
# or exists
        init_data = RCInitTkGui(smo_init_mode='open', smo_filename=fname, glob=RCInitTkGui.glob)
        if not isinstance(init_data, RCInitTkGui):
            tkMessageBox.showwarning(parent=self, title='Warning',
                                     message="File contains no TK GUI initialization data")
            return
        self.init_data = init_data
        self.init_data.rc_header_widget = self

        path = os.path.dirname(fname)
        fbasename = os.path.basename(fname)
        if fbasename[-3:] == '.py':
            fbasename = fbasename[0:-3]
        self.rc_path = path
        self.init_data.rc_path = self.rc_path
        self.rc_name = fbasename
        self.init_data.rc_name = self.rc_name

        # install widgets
        self.cont_frame.destroy()
        self.cont_frame = RCFrame(master=self, init_data=self.init_data,
                                  width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
        self.cont_frame.grid(row=1, column=0, sticky=tk.N + tk.S + tk.E + tk.W)

# self.cont_frame.install_widgets(widget_structure=self.init_data.rc_widget_structure)

        if self.init_data.rc_widget_structure[2] is not None:
            self.cont_frame.install_widgets(**self.init_data.rc_widget_structure[2])
        return

    def cmd_save_init_data(self):
        name = self.rc_name + '.py'
        fname = tkFileDialog.asksaveasfilename(parent=self, title='Save GUI initialization data',
                                               initialdir=self.rc_path, initialfile=name,
                                               defaultextension='.py', filetypes=[('Init Data', ".py"), ('Alles', "*.*")])
        if fname is None or fname == '':
            return

        path = os.path.dirname(fname)
        fbasename = os.path.basename(fname)
        if fbasename[-3:] == '.py':
            fbasename = fbasename[0:-3]
        self.rc_path = path
        self.init_data.rc_path = self.rc_path
        self.rc_name = fbasename
        self.init_data.rc_name = self.rc_name

        if self.cont_frame is not None:
            self.init_data.rc_widget_structure =\
                self.cont_frame.get_installed_widgets()
        else:
            self.init_data.rc_widget_structure = None

        self.init_data.smo_save(smo_filename=fname)
        return

    def cmd_close_init_data(self):

        self.rc_path = self.init_data.rc_path
        self.rc_name = self.init_data.rc_path
        fname = os.path.join(self.rc_path, '%s.py' % self.rc_name)

        self.init_data.smo_save(smo_filename=fname)

        self.cmd_new_init_data()
        return

    def i_port_ret_fkt(self, port_tab):
        if port_tab is not None:
            self.init_data.update_port_tab(port_tab, self.init_data.init_inport,
                                           self.init_data.internal_inport,
                                           self.init_data.external_inport, reset=True)
        return

    def cmd_i_port_dialog(self):
        RCPortDialog(self.init_data.init_inport, ret_fkt=self.i_port_ret_fkt,
                     name=self.init_data.rc_name, direction='Input')
        return

    def o_port_ret_fkt(self, port_tab):
        if port_tab is not None:
            self.init_data.update_port_tab(port_tab, self.init_data.init_outport,
                                           self.init_data.internal_outport,
                                           self.init_data.external_outport, reset=True)
        return

    def cmd_o_port_dialog(self):
        RCPortDialog(self.init_data.init_outport, ret_fkt=self.o_port_ret_fkt,
                     name=self.init_data.rc_name, direction='Output')
        return

    def i_edge_ret_fkt(self, edge_tab):
        if edge_tab is not None:
            self.init_data.update_connection_tab(edge_tab, self.init_data.init_in_pred,
                                                 self.init_data.in_pred, reset=True)
        return

    def cmd_i_edge_dialog(self):
        RCInEdgeDialog(self.init_data.init_in_pred, ret_fkt=self.i_edge_ret_fkt,
                       name=self.init_data.rc_name)
        return

    def o_edge_ret_fkt(self, edge_tab):
        if edge_tab is not None:
            self.init_data.update_connection_tab(edge_tab, self.init_data.init_out_succ,
                                                 self.init_data.out_succ, reset=True)
        return

    def cmd_o_edge_dialog(self):
        RCOutEdgeDialog(self.init_data.init_out_succ, ret_fkt=self.o_edge_ret_fkt,
                        name=self.init_data.rc_name)
        return

    def cmd_register(self):
        name = tkSimpleDialog.askstring(self.tk_master, prompt='Connection Name?', initialvalue=self.connection_name)
        if name != '' and name is not None:
            self.connection_name = name
            self.init_data.connection_name = name
        return

    def cmd_update(self):
        self.generate_widget_connect()
        return

    def cmd_step(self):
        # check if node is in node list
        if self.connection_name not in RCInit.node_tab:
            RCInit.node_tab[self.connection_name] = self.init_data
        if RCInit.node_tab[self.connection_name] != self.init_data:
            tkMessageBox.showerror(parent=self, title='Warning', message="Connection Name exists")
            return
        self.generate_widget_connect()
        self.init_data.rc_step()
        del RCInit.node_tab[self.connection_name]
        return

    def cmd_start(self):
        self.generate_widget_connect()
        if self.connection_name not in RCInit.node_tab:
            RCInit.node_tab[self.connection_name] = self.init_data
        if RCInit.node_tab[self.connection_name] != self.init_data:
            tkMessageBox.showerror(parent=self, title='Warning', message="Connection Name exists")
            return
        self.init_data.sync_run = True
        self.init_data.sync_widget = self
        self.after(self.init_data.sync_period, self.init_data.rc_sync)
        return

    def cmd_stop(self):
        self.init_data.sync_run = False
        if self.connection_name in RCInit.node_tab:
            del RCInit.node_tab[self.connection_name]
        return

    def _cmd_set_highlight(self, event):
        self.rc_event = event
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** header set highlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_reset_highlight()
        self.rc_set_highlight()
        return

###
###
###

    def cmd_simple_local_robot(self):
        self.rob = RCLocalRobot_1(cam_index=0)  # nicht in init initialisiert !!!
        self.rob_name = 'robby'        # nicht in init initialisiert !!!
        return

    def cmd_start_simple_local_robot(self):
        self.rob_name = 'robby'
        #name = tkSimpleDialog.askstring(self.tk_master, prompt='Connection Name?', initialvalue=self.connection_name)
        if self.rob_name is None or self.rob_name == '':
            self.rob_name = tkSimpleDialog.askstring(self.tk_master, prompt='Connection Name?', initialvalue='robby')
        if self.rob_name != '' and self.rob_name is not None:
            if self.rob_name in RCInit.node_tab:
                tkMessageBox.showerror(parent=self, title='Warning', message="Connection Name exists")
                return
            RCInit.node_tab[self.rob_name] = self.rob
            if self.rob is not None:
                #self.rob.connection_name = self.rob_name
                self.rob.start()
        return

    def cmd_stop_simple_local_robot(self):
        if self.rob_name in RCInit.node_tab:
            del RCInit.node_tab[self.rob_name]
        self.rob.stop()
        return
###
###
###

    def cmd_pi_robot(self):
        from framework.robots.pi_rpc_robot import CCSCallRobotConnect_1
        self.call_rob = CCSCallRobotConnect_1(host=self._robot_host, port=self._robot_port)
        self.call_rob_name = 'robby'

    def cmd_start_pi_robot(self):
        self.call_rob_name = 'robby'
        if self.call_rob_name is None or self.call_rob_name == '':
            self.call_rob_name = tkSimpleDialog.askstring(
                self.tk_master, prompt='Connection Name?', initialvalue='robby')
        if self.call_rob_name != '' and self.call_rob_name is not None:
            if self.call_rob_name in RCInit.node_tab:
                tkMessageBox.showerror(parent=self, title='Warning', message="Connection Name exists")
                return
            RCInit.node_tab[self.call_rob_name] = self.call_rob
            if self.call_rob is not None:
                self.call_rob.start()
        else:
            tkMessageBox.showerror(parent=self, title='Warning', message="No legal Name exists")

        return

    def cmd_stop_pi_robot(self):
        if self.call_rob_name in RCInit.node_tab:
            del RCInit.node_tab[self.call_rob_name]
        self.call_rob.stop()
        return

# Pi Robot

    def cmd_simple_call_robot(self):
        ###
        # self.call_rob = RCCallRobotConnect_1(host='192.168.178.101', port=8008) # nicht in init initialisiert !!!
        self.call_rob = RCCallRobotConnect_1()  # nicht in init initialisiert !!!
        self.call_rob_name = 'robby'        # nicht in init initialisiert !!!
        return

    def cmd_start_simple_call_robot(self):
        self.call_rob_name = 'robby'
        if self.call_rob_name is None or self.call_rob_name == '':
            self.call_rob_name = tkSimpleDialog.askstring(
                self.tk_master, prompt='Connection Name?', initialvalue='robby')
        if self.call_rob_name != '' and self.call_rob_name is not None:
            if self.call_rob_name in RCInit.node_tab:
                tkMessageBox.showerror(parent=self, title='Warning', message="Connection Name exists")
                return
            RCInit.node_tab[self.call_rob_name] = self.call_rob
            if self.call_rob is not None:
                self.call_rob.start()
        else:
            tkMessageBox.showerror(parent=self, title='Warning', message="No legal Name exists")

        return

    def cmd_stop_simple_call_robot(self):
        if self.call_rob_name in RCInit.node_tab:
            del RCInit.node_tab[self.call_rob_name]
        self.call_rob.stop()
        return
###
# end robot cmds
###

    def rc_set_highlight(self):
        self.config(bg=rc_util.rc_hl_color)
        return

    def _cmd_reset_highlight(self, event):
        self.pkm_event = event
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** header reset hightlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_set_highlight()
        self.rc_reset_highlight()
        return

    def rc_reset_highlight(self):
        self.config(bg=self.act_color)
        return


class RCWindow(tk.Frame):

    def __init__(self, robot_host, robot_port, init_data=None, gui_filename=None):
        if init_data is None:
            self.init_data = RCInitTkGui()
        else:
            self.init_data = init_data
        self._robot_host = robot_host
        self._robot_port = robot_port
        self.tk_master = tk.Toplevel()
        self.init_data.rc_top_level_widget = self
        self.gui_filename = gui_filename
        # RCInitTkGui.rc_window_lst.append(self)
        RCInitTkGui.rc_window_lst.append(self.tk_master)

        self.create_widgets()
        return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master)
        # make toplevel window resizeble and set title and geometry
        self.tk_master.protocol("WM_DELETE_WINDOW", self.cmd_exit)
        self.tk_master.resizable(width=1, height=1)
        self.tk_master.rowconfigure(0, weight=1)
        self.tk_master.columnconfigure(0, weight=1)

        self.rc_window_title = tk.StringVar()
        self.rc_window_title.set('RC: ' + self.init_data.rc_name)
        self.tk_master.title(self.rc_window_title.get())

        ws = self.tk_master.winfo_screenwidth()
        hs = self.tk_master.winfo_screenheight()
        x = (ws / 2) - (rc_util.rc_window_init_xsize / 2)
        y = (hs / 2) - (rc_util.rc_window_init_ysize / 2)
        self.tk_master.geometry('%dx%d+%d+%d' % (rc_util.rc_window_init_xsize, rc_util.rc_window_init_ysize, x, y))

        # structure main window
        ## self.act_color = self['bg']
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)


# create menu bar
##        self.menu_bar = Menu(self.tk_master)
##        self.tk_master["menu"] = self.menu_bar
##
# create file menu
##        self.file_menu = Menu(self.menu_bar, tearoff=0)
##        self.menu_bar.add_cascade(label="File", menu=self.file_menu)
##
##        self.file_menu.add_command(label="New Window", command=self.cmd_new_rc_window)
##        self.file_menu.add_command(label="Exit", command=self.cmd_exit)
##        self.file_menu.add_command(label="Exit All", command=self.cmd_exit_all)
##        self.file_menu.add_command(label="None Test", command=self.cmd_none)

# create node type menu
##        self.node_menu = Menu(self.menu_bar, tearoff=0)
##        self.menu_bar.add_cascade(label="Nodes", menu=self.node_menu)
# for ntype in PKMonitorBaseClass.rc_node_types:
# if PKBaseObject.info: pk_debug_handler.out(
# '*** INFO *** PKMonitorFrame: Define submenu node type: %s'%ntype)
# def cmd_node_type_handler(self=self, ntype=ntype):
##                new_node_type = PKMonitorBaseClass.rc_node_types[ntype]
##                new_node_type[0](rc_root=self.rc_root, rc_master=Toplevel(), **new_node_type[1])
# return
##            self.node_menu.add_command(label=ntype, command=cmd_node_type_handler)

        # create content widget
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        self.content = RCHeader(robot_host=self._robot_host, robot_port=self._robot_port, master=self, init_data=self.init_data, gui_filename=self.gui_filename)
        self.content.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        self['bg'] = rc_util.rc_bg_color
        self.act_color = self['bg']
        if self.init_data.rc_widget_structure is not None:
            self.content.cont_frame.install_widgets(widget_structure=self.init_data.rc_widget_structure)  # [2] ???
        return

    def cmd_exit(self):
        result = tkMessageBox.askquestion(parent=self, title='Warning',
                                          message="Do you really want to close this Window?")
        if result == 'yes':
            self.tk_master.destroy()
            # RCInitTkGui.rc_window_lst.remove(self)
            RCInitTkGui.rc_window_lst.remove(self.tk_master)
            if len(RCInitTkGui.rc_window_lst) <= 0:
                self.quit()
            return

# def cmd_exit_all(self):
# result = tkMessageBox.askquestion(parent=self.tk_master, title='Warning', \
# message="Do you really want to exit all RC windows?")
# if result == 'yes':
# for x in RCInitTkGui.rc_window_lst:
# try:
# x.tk_master.destroy()
# except:
# if SMOBaseObject.info: SMOBaseObject.debug_handler.out(
# '*** info *** RCMonitor: Exception during destroy gui window')
# self.quit()

    def rc_set_highlight(self):
        # dummy for layout highlight
        return

    def rc_reset_highlight(self):
        # dummy for layout highlight
        return


# realize gui as an own SINGLE TK thread
class RCMonitor(threading.Thread):

    def __init__(self, robot_host, robot_port, gui_filename=None, init_data=None):
        # setup monitor base class

        threading.Thread.__init__(self)
        self.setDaemon(1)
        self._robot_host = robot_host
        self._robot_port = robot_port
        self._gui_filename = gui_filename
        self.init_data = init_data
        if self.init_data is None:
            self.init_data = RCInitTkGui()

        return

    def run(self):
        self._start_gui()
        return

    def start_as_thread(self):
        self.start()
        return

    def start_in_main(self):
        self._start_gui()
        return

    def _start_gui(self):
        # setup monitor
        if RCInitTkGui.rc_tk_root is None:

            # define tk setup
            RCInitTkGui.rc_tk_root = tk.Tk()

            # define tk setup
            RCInitTkGui.rc_tk_root.option_add("*borderWidth", rc_util.rc_border)  # for relief=SUNKEN
            RCInitTkGui.rc_tk_root.option_add("*background", rc_util.rc_bg_color)
            RCInitTkGui.rc_tk_root.option_add("*troughcolor", rc_util.rc_bg_color)

            #RCInitTkGui.rc_tk_root.option_add("*foreground", rc_fg_color)
            #RCInitTkGui.rc_tk_root.option_add("*highlightThickness", rc_opt_bd)
            #RCInitTkGui.rc_tk_root.option_add("*activeForeground", rc_opt_act_bg)
            #RCInitTkGui.rc_tk_root.rc_master.option_add("*activeBackground", rc_opt_act_fg)
            #RCInitTkGui.rc_tk_root.option_add("*highlightBackground", rc_opt_hl_bg)
            #RCInitTkGui.rc_tk_root.option_add("*disabledForeground", rc_opt_dis_fg)
            #RCInitTkGui.rc_tk_root.option_add("*troughColor", rc_opt_dis_fg)

            RCInitTkGui.rc_tk_root.withdraw()

            # start initial main window
            RCWindow(robot_host=self._robot_host, robot_port=self._robot_port, init_data=self.init_data,gui_filename=self._gui_filename)
            RCInitTkGui.rc_tk_root.mainloop()
            RCInitTkGui.rc_tk_root.destroy()
        else:
            RCWindow(robot_host=self._robot_host, robot_port=self._robot_port, init_data=self.init_data, gui_filename=self._gui_filename)
        return
