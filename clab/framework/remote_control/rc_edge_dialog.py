from __future__ import division
from future import standard_library
standard_library.install_aliases()
from builtins import str
from past.utils import old_div
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
import copy

import tkinter as tk
import tkinter.messagebox

from . import rc_util


class RCInEdgeDialog(tk.Frame):

    def __init__(self, edge_tab, ret_fkt=None, name='UNDEF'):
        self.name = name
        self.edge_tab = copy.deepcopy(edge_tab)
        self.ret_fkt = ret_fkt
        self.tk_master = tk.Toplevel()
        self.create_widgets()
        return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master)
        self.tk_master.protocol("WM_DELETE_WINDOW", self.cmd_exit)
        self.tk_master.resizable(width=1, height=1)
        self.tk_master.rowconfigure(0, weight=1)
        self.tk_master.columnconfigure(0, weight=1)
        self.node_title = tk.StringVar()
        title_str = 'Input Edges (pull) from Nodes: %s' % str(self.name)
        self.node_title.set(title_str)
        self.tk_master.title(self.node_title.get())
        ws = self.tk_master.winfo_screenwidth()
        hs = self.tk_master.winfo_screenheight()
        x = (old_div(ws, 2)) - (old_div(rc_util.rc_window_init_xsize, 2))
        y = (old_div(hs, 2)) - (old_div(rc_util.rc_window_init_ysize, 4))
        #self.tk_master.geometry('%dx%d+%d+%d'%(rc_window_init_xsize, rc_window_init_ysize, x, y))
        self.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        # structure main window
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=3)
        self.rowconfigure(2, weight=0)
        self.rowconfigure(3, weight=0)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)

        # row 0 subtitle
        self.s_node_label = tk.Label(self, text='Source Node', relief=tk.FLAT)
        self.s_node_label.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        self.s_port_label = tk.Label(self, text='Source Outport', relief=tk.FLAT)
        self.s_port_label.grid(row=0, column=1, sticky=tk.N + tk.E + tk.S + tk.W)

        self.t_port_label = tk.Label(self, text='Target Inport', relief=tk.FLAT)
        self.t_port_label.grid(row=0, column=3, sticky=tk.N + tk.E + tk.S + tk.W)

        # row 1 listbox

        widget_line = min((len(self.edge_tab) + 3, 3))
        #widget_line = 3

        self.s_node_lst_var = tk.StringVar()
        self.s_node_lst_widget = tk.Listbox(self, listvariable=self.s_node_lst_var, bg=rc_util.rc_bg_txt_color, exportselection=0,
                                            selectmode=tk.SINGLE, width=10, height=widget_line, bd=0, highlightthickness=0,
                                            selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.s_node_lst_widget.grid(row=1, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.s_port_lst_var = tk.StringVar()
        self.s_port_lst_widget = tk.Listbox(self, listvariable=self.s_port_lst_var, bg=rc_util.rc_bg_txt_color, exportselection=0,
                                            selectmode=tk.SINGLE, width=10, height=widget_line, bd=0, highlightthickness=0,
                                            selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.s_port_lst_widget.grid(row=1, column=1, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.edge_lst_label = tk.Label(self, text='>>>>>', relief=tk.FLAT)
        self.edge_lst_label.grid(row=1, column=2, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.t_port_lst_var = tk.StringVar()
        self.t_port_lst_widget = tk.Listbox(self, listvariable=self.t_port_lst_var, bg=rc_util.rc_bg_txt_color, exportselection=0,
                                            selectmode=tk.SINGLE, width=10, height=widget_line, bd=0, highlightthickness=0,
                                            selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.t_port_lst_widget.grid(row=1, column=3, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        # row 2 entry
        self.s_node_entry = tk.Entry(self, highlightthickness=0, width=10, relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.s_node_entry.grid(row=2, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.s_port_entry = tk.Entry(self, highlightthickness=0, width=10, relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.s_port_entry.grid(row=2, column=1, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.t_port_entry = tk.Entry(self, highlightthickness=0, width=10, relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.t_port_entry.grid(row=2, column=3, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        # row 3 Buttons
        self.add_button = tk.Button(self, text='New', command=self.cmd_new,
                                    bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.add_button.grid(row=3, column=0, sticky=tk.E + tk.W, padx=4, pady=8)
        self.del_button = tk.Button(self, text='Delete', command=self.cmd_del,
                                    bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.del_button.grid(row=3, column=1, sticky=tk.E + tk.W, padx=4, pady=8)
        self.cancel_button = tk.Button(self, text='Cancel', command=self.cmd_cancel,
                                       bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.cancel_button.grid(row=3, column=2, sticky=tk.E + tk.W, padx=4, pady=8)
        self.ok_button = tk.Button(self, text='OK', command=self.cmd_ok, bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.ok_button.grid(row=3, column=3, sticky=tk.E + tk.W, padx=4, pady=8)

        self._init_widgets()
        return

    def _init_widgets(self):
        s_node_str = ()
        s_port_str = ()
        t_port_str = ()
        for no in self.edge_tab:
            for src in self.edge_tab[no]:
                s_node_str += (no,)
                s_port_str += (src,)
                t_port_str += (self.edge_tab[no][src],)
        self.s_node_lst_var.set(s_node_str)
        self.s_port_lst_var.set(s_port_str)
        self.t_port_lst_var.set(t_port_str)
        return

    def cmd_new(self):
        s_node_name = self.s_node_entry.get()
        s_node_name = s_node_name.strip()
        if not s_node_name.isalnum() or s_node_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Source Node Identifiere', message='Illegal Identifiere')
            return

        s_port_name = self.s_port_entry.get()
        s_port_name = s_port_name.strip()
        if not s_port_name.isalnum() or s_port_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Source Port Identifiere', message='Illegal Identifiere')
            return

        t_port_name = self.t_port_entry.get()
        t_port_name = t_port_name.strip()
        if not t_port_name.isalnum() or t_port_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Target Port Identifiere', message='Illegal Identifiere')
            return

        if s_node_name in self.edge_tab and\
           s_port_name in self.edge_tab[s_node_name]:
            msg = 'Edge fom node ' + s_node_name + ' and port ' + s_port_name + ' exists'
            tkinter.messagebox.showwarning(title='Input Edge', message=msg)
            return

        self.s_node_lst_widget.insert(tk.END, s_node_name)
        self.s_port_lst_widget.insert(tk.END, s_port_name)
        self.t_port_lst_widget.insert(tk.END, t_port_name)

        if s_node_name in self.edge_tab:
            self.edge_tab[s_node_name][s_port_name] = t_port_name
        else:
            self.edge_tab[s_node_name] = {s_port_name: t_port_name}
        return

    def cmd_del(self):
        s_node_index = self.s_node_lst_widget.curselection()
        s_port_index = self.s_port_lst_widget.curselection()
        t_port_index = self.t_port_lst_widget.curselection()

        # print '*** selection ', s_node_index, s_port_index, t_port_index

        if s_node_index != ():
            index = int(s_node_index[0])
        elif s_port_index != ():
            index = int(s_port_index[0])
        elif t_port_index != ():
            index = int(t_port_index[0])
        else:
            tkinter.messagebox.showwarning(title='Delete Edge', message='No Selection')
            return

        # print '*** index ', index

        s_node_name = self.s_node_lst_widget.get(index)
        s_port_name = self.s_port_lst_widget.get(index)
        t_port_name = self.t_port_lst_widget.get(index)

        # print '*** names ', s_node_name, s_port_name, t_port_name

        if s_node_name in self.edge_tab:
            if s_port_name in self.edge_tab[s_node_name]:
                if t_port_name != self.edge_tab[s_node_name][s_port_name]:
                    tkinter.messagebox.showwarning(title='Delete Edge', message='No Target Port Selection')
                else:
                    del self.edge_tab[s_node_name][s_port_name]
                    if self.edge_tab[s_node_name] == {}:
                        del self.edge_tab[s_node_name]
                    self.s_node_lst_widget.delete(index)
                    self.s_port_lst_widget.delete(index)
                    self.t_port_lst_widget.delete(index)
            else:
                tkinter.messagebox.showwarning(title='Delete Edge', message='No Source Port  Selection')
        else:
            tkinter.messagebox.showwarning(title='Delete Edge', message='No Source Node Selection')
        return

    def cmd_cancel(self):
        self.edge_tab = None
        if self.ret_fkt is not None:
            self.ret_fkt(self.edge_tab)
        self.tk_master.destroy()
        return

    def cmd_ok(self):
        if self.ret_fkt is not None:
            self.ret_fkt(self.edge_tab)
        self.edge_tab = None
        self.tk_master.destroy()
        return

    def cmd_exit(self):
        self.edge_tab = None
        if self.ret_fkt is not None:
            self.ret_fkt(self.edge_tab)
        self.tk_master.destroy()
        return

    def cmd_none(self):
        return


class RCOutEdgeDialog(tk.Frame):

    def __init__(self, edge_tab, ret_fkt=None, name='UNDEF'):
        self.name = name
        self.edge_tab = copy.deepcopy(edge_tab)
        self.ret_fkt = ret_fkt
        self.tk_master = tk.Toplevel()
        self.create_widgets()
        return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master)
        self.tk_master.protocol("WM_DELETE_WINDOW", self.cmd_exit)
        self.tk_master.resizable(width=1, height=1)
        self.tk_master.rowconfigure(0, weight=1)
        self.tk_master.columnconfigure(0, weight=1)
        self.node_title = tk.StringVar()
        title_str = 'Output Edges (push) from Nodes: %s' % str(self.name)
        self.node_title.set(title_str)
        self.tk_master.title(self.node_title.get())
        ws = self.tk_master.winfo_screenwidth()
        hs = self.tk_master.winfo_screenheight()
        x = (old_div(ws, 2)) - (old_div(rc_util.rc_window_init_xsize, 2))
        y = (old_div(hs, 2)) - (old_div(rc_util.rc_window_init_ysize, 4))
        #self.tk_master.geometry('%dx%d+%d+%d'%(rc_window_init_xsize, rc_window_init_ysize, x, y))
        self.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        # structure main window
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=3)
        self.rowconfigure(2, weight=0)
        self.rowconfigure(3, weight=0)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)

        # row 0
        self.s_port_label = tk.Label(self, text='Source Outport', relief=tk.FLAT)
        self.s_port_label.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        self.t_node_label = tk.Label(self, text='Target Node', relief=tk.FLAT)
        self.t_node_label.grid(row=0, column=2, sticky=tk.N + tk.E + tk.S + tk.W)

        self.t_port_label = tk.Label(self, text='Target Inport', relief=tk.FLAT)
        self.t_port_label.grid(row=0, column=3, sticky=tk.N + tk.E + tk.S + tk.W)

        # row 1 listbox

        widget_line = min((len(self.edge_tab) + 3, 3))
        #widget_line = 3

        self.s_port_lst_var = tk.StringVar()
        #self.s_port_lst_var.set('po1 p2 p3')
        self.s_port_lst_widget = tk.Listbox(self, listvariable=self.s_port_lst_var, bg=rc_util.rc_bg_txt_color, exportselection=0,
                                            selectmode=tk.SINGLE, width=10, height=widget_line, bd=0, highlightthickness=0,
                                            selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.s_port_lst_widget.grid(row=1, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.edge_lst_label = tk.Label(self, text='>>>>>', relief=tk.FLAT)
        self.edge_lst_label.grid(row=1, column=1, sticky=tk.N + tk.E + tk.S + tk.W)

        self.t_node_lst_var = tk.StringVar()
        #self.t_node_lst_var.set('w1 w2 w3')
        self.t_node_lst_widget = tk.Listbox(self, listvariable=self.t_node_lst_var, bg=rc_util.rc_bg_txt_color, exportselection=0,
                                            selectmode=tk.SINGLE, width=10, height=widget_line, bd=0, highlightthickness=0,
                                            selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.t_node_lst_widget.grid(row=1, column=2, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.t_port_lst_var = tk.StringVar()
        #self.t_port_lst_var.set('pw1 pw2 pw3')
        self.t_port_lst_widget = tk.Listbox(self, listvariable=self.t_port_lst_var, bg=rc_util.rc_bg_txt_color, exportselection=0,
                                            selectmode=tk.SINGLE, width=10, height=widget_line, bd=0, highlightthickness=0,
                                            selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.t_port_lst_widget.grid(row=1, column=3, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        # row 2 entry
        self.s_port_entry = tk.Entry(self, highlightthickness=0, width=10, relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.s_port_entry.grid(row=2, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.t_node_entry = tk.Entry(self, highlightthickness=0, width=10, relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.t_node_entry.grid(row=2, column=2, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.t_port_entry = tk.Entry(self, highlightthickness=0, width=10, relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.t_port_entry.grid(row=2, column=3, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        # row 3 Buttons
        self.add_button = tk.Button(self, text='New', command=self.cmd_new,
                                    bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.add_button.grid(row=3, column=0, sticky=tk.E + tk.W, padx=4, pady=8)
        self.del_button = tk.Button(self, text='Delete', command=self.cmd_del,
                                    bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.del_button.grid(row=3, column=1, sticky=tk.E + tk.W, padx=4, pady=8)
        self.cancel_button = tk.Button(self, text='Cancel', command=self.cmd_cancel,
                                       bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.cancel_button.grid(row=3, column=2, sticky=tk.E + tk.W, padx=4, pady=8)
        self.ok_button = tk.Button(self, text='OK', command=self.cmd_ok, bd=0, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.ok_button.grid(row=3, column=3, sticky=tk.E + tk.W, padx=4, pady=8)

        self._init_widgets()
        return

    def _init_widgets(self):
        s_port_str = ()
        t_node_str = ()
        t_port_str = ()

        for no in self.edge_tab:
            for src in self.edge_tab[no]:
                s_port_str += (src,)
                t_node_str += (no,)
                t_port_str += (self.edge_tab[no][src],)
        self.s_port_lst_var.set(s_port_str)
        self.t_node_lst_var.set(t_node_str)
        self.t_port_lst_var.set(t_port_str)
        return

    def cmd_new(self):
        s_port_name = self.s_port_entry.get()
        s_port_name = s_port_name.strip()
        if not s_port_name.isalnum() or s_port_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Source Port Identifiere', message='Illegal Identifiere')
            return

        t_node_name = self.t_node_entry.get()
        t_node_name = t_node_name.strip()
        if not t_node_name.isalnum() or t_node_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Target Node Identifiere', message='Illegal Identifiere')
            return

        t_port_name = self.t_port_entry.get()
        t_port_name = t_port_name.strip()
        if not t_port_name.isalnum() or t_port_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Target Port Identifiere', message='Illegal Identifiere')
            return

        if t_node_name in self.edge_tab and\
           s_port_name in self.edge_tab[t_node_name]:
            msg = 'Edge fom port ' + s_port_name + ' to Node ' + t_node_name + ' exists'
            tkinter.messagebox.showwarning(title='Output Edge', message=msg)
            return

        self.s_port_lst_widget.insert(tk.END, s_port_name)
        self.t_node_lst_widget.insert(tk.END, t_node_name)
        self.t_port_lst_widget.insert(tk.END, t_port_name)

        if t_node_name in self.edge_tab:
            self.edge_tab[t_node_name][s_port_name] = t_port_name
        else:
            self.edge_tab[t_node_name] = {s_port_name: t_port_name}
        return

    def cmd_del(self):
        s_port_index = self.s_port_lst_widget.curselection()
        t_node_index = self.t_node_lst_widget.curselection()
        t_port_index = self.t_port_lst_widget.curselection()

        # print '*** selection ', s_port_index, t_node_index, t_port_index

        if s_port_index != ():
            index = int(s_port_index[0])
        elif t_node_index != ():
            index = int(t_node_index[0])
        elif t_port_index != ():
            index = int(t_port_index[0])
        else:
            tkinter.messagebox.showwarning(title='Delete Edge', message='No Selection')
            return

        # print '*** index ', index

        s_port_name = self.s_port_lst_widget.get(index)
        t_node_name = self.t_node_lst_widget.get(index)
        t_port_name = self.t_port_lst_widget.get(index)

        # print '*** names ',  s_port_name, t_node_name, t_port_name

        if t_node_name in self.edge_tab:
            if s_port_name in self.edge_tab[t_node_name]:
                if t_port_name != self.edge_tab[t_node_name][s_port_name]:
                    tkinter.messagebox.showwarning(title='Delete Edge', message='No Target Port Selection')
                else:
                    del self.edge_tab[t_node_name][s_port_name]
                    if self.edge_tab[t_node_name] == {}:
                        del self.edge_tab[t_node_name]
                    self.s_port_lst_widget.delete(index)
                    self.t_node_lst_widget.delete(index)
                    self.t_port_lst_widget.delete(index)
            else:
                tkinter.messagebox.showwarning(title='Delete Edge', message='No Source Port  Selection')
        else:
            tkinter.messagebox.showwarning(title='Delete Edge', message='No Source Node Selection')
        return

    def cmd_cancel(self):
        self.edge_tab = None
        if self.ret_fkt is not None:
            self.ret_fkt(self.edge_tab)
        self.tk_master.destroy()
        return

    def cmd_ok(self):
        if self.ret_fkt is not None:
            self.ret_fkt(self.edge_tab)
        self.edge_tab = None
        self.tk_master.destroy()
        return

    def cmd_exit(self):
        self.edge_tab = None
        if self.ret_fkt is not None:
            self.ret_fkt(self.edge_tab)
        self.tk_master.destroy()
        return

    def cmd_none(self):
        self.ok_cmd = False
        return
