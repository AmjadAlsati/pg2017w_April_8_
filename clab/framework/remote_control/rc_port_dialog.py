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
import cmath
from traceback import extract_tb

import tkinter as tk
import tkinter.messagebox

from . import rc_util


class RCPortDialog(tk.Frame):

    def __init__(self, port_tab, ret_fkt=None, name='UNDEF', direction='Input'):
        self.direction = direction
        self.name = name
        self.port_tab = copy.deepcopy(port_tab)
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

        if self.direction == 'Input':
            title_str = 'Input Ports for Node: '
        else:
            title_str = 'Output Ports for Node: '
        title_str += str(self.name)
        self.node_title.set(title_str)

        self.tk_master.title(self.node_title.get())
        ws = self.tk_master.winfo_screenwidth()
        hs = self.tk_master.winfo_screenheight()
        x = (old_div(ws, 2)) - (old_div(rc_util.rc_window_init_xsize, 2))
        y = (old_div(hs, 2)) - (old_div(rc_util.rc_window_init_ysize, 4))
        #self.tk_master.geometry('%dx%d+%d+%d'%(rc_window_init_xsize, rc_window_init_ysize, x, y))
        self.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        # structure main window
        self.act_color = self['bg']

        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=3)
        self.rowconfigure(2, weight=0)
        self.rowconfigure(3, weight=0)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.columnconfigure(3, weight=1)

        # row 0 listbox
        self.port_label = tk.Label(self, text='Port Name', relief=tk.FLAT)
        self.port_label.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        self.p_type_label = tk.Label(self, text='Port Type', relief=tk.FLAT)
        self.p_type_label.grid(row=0, column=1, sticky=tk.N + tk.E + tk.S + tk.W)

        self.p_default_label = tk.Label(self, text='Default Value:', relief=tk.FLAT)
        self.p_default_label.grid(row=0, column=2, columnspan=2, sticky=tk.N + tk.E + tk.S + tk.W)

        # row 1 name type
        widget_line = min((len(self.port_tab) + 5, 5))

        self.port_lst_var = tk.StringVar()
        self.port_lst_widget = tk.Listbox(self, listvariable=self.port_lst_var, selectmode=tk.SINGLE, exportselection=0,
                                          bg=rc_util.rc_bg_txt_color, width=10, height=widget_line, bd=0, highlightthickness=0,
                                          selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.port_lst_widget.grid(row=1, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.p_type_lst_var = tk.StringVar()
        self.p_type_lst_widget = tk.Listbox(self, listvariable=self.p_type_lst_var, selectmode=tk.SINGLE, exportselection=0,
                                            bg=rc_util.rc_bg_txt_color, width=10, height=widget_line, bd=0, highlightthickness=0,
                                            selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.p_type_lst_widget.grid(row=1, column=1, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.p_default_lst_var = tk.StringVar()
        self.p_default_lst_widget = tk.Listbox(self, listvariable=self.p_default_lst_var, selectmode=tk.SINGLE, exportselection=0,
                                               bg=rc_util.rc_bg_txt_color, width=10, height=widget_line, bd=0, highlightthickness=0, relief=tk.FLAT)
        self.p_default_lst_widget.grid(row=1, column=2, columnspan=2, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        # row 2 name type
        self.port_entry = tk.Entry(self, highlightthickness=0, width=12, relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.port_entry.grid(row=2, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.p_type_lst = ('Value', 'Token', ' List ')  # keep order
        self.p_type_var = tk.StringVar()
        self.p_type_option = tk.OptionMenu(self, self.p_type_var, *self.p_type_lst)
        self.p_type_option.config(bg=rc_util.rc_bg_txt_color, borderwidth=0, highlightbackground=rc_util.rc_hl_color,
                                  highlightcolor=rc_util.rc_black, activebackground=rc_util.rc_hl_color, relief=tk.FLAT)

        self.p_type_option['menu'].config(activebackground=rc_util.rc_button_color, activeforeground=rc_util.rc_black)

        self.p_type_var.set(self.p_type_lst[1])
        self.p_type_option.grid(row=2, column=1, sticky=tk.N + tk.E + tk.S + tk.W, padx=2, pady=2)

        self.p_default_entry = tk.Entry(self, highlightthickness=0, width=12,
                                        relief=tk.FLAT, bg=rc_util.rc_bg_txt_color)
        self.p_default_entry.grid(row=2, column=2, columnspan=2, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)
        self.p_default_entry.insert(0, 'None')

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
        # init widgets
        port_lst_str = ()
        p_type_lst_str = ()
        p_default_lst_str = ()

        for po in self.port_tab:
            port_lst_str += (po,)
            if self.port_tab[po][1] == rc_util.rc_value:
                po_t = self.p_type_lst[0]
            elif self.port_tab[po][1] == rc_util.rc_token:
                po_t = self.p_type_lst[1]
            elif self.port_tab[po][1] == rc_util.rc_list:
                po_t = self.p_type_lst[2]
            else:
                po_t = 'undef'
            p_type_lst_str += (po_t,)
            p_default_lst_str += (repr(self.port_tab[po][0]),)

        self.port_lst_var.set(port_lst_str)
        self.p_type_lst_var.set(p_type_lst_str)
        self.p_default_lst_var.set(p_default_lst_str)
        return

    def cmd_new(self):
        p_name = self.port_entry.get()
        p_name = p_name.strip()
        if not p_name.isalnum() or p_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Portdefinition', message='Illegal Identifiere')
            return
        if p_name in self.port_tab:
            tkinter.messagebox.showwarning(title='Portdefinition', message='Port: ' + p_name + ' exists')
            return

        p_type = self.p_type_var.get()
        p_type_str = p_type
        if p_type == self.p_type_lst[0]:
            p_type = rc_util.rc_value
        elif p_type == self.p_type_lst[1]:
            p_type = rc_util.rc_token
        elif p_type == self.p_type_lst[2]:
            p_type = rc_util.rc_list
        else:
            tkinter.messagebox.showwarning(title='Portdefinition', message='Illegal Type; ' + str(p_type))
            return
        try:
            p_default_str = self.p_default_entry.get()
            p_default_val = eval(p_default_str, vars(cmath), {})
            p_default_str = repr(p_default_val)
        except:
            error_list = extract_tb(tk.sys.exc_info()[2])
            error = error_list[len(error_list) - 1]
            error_str = 'Error in Default Value:\n    %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
                % (tk.sys.exc_info()[1], error[0], error[1], error[2], error[3])
            tkinter.messagebox.showerror('Port Default Value', error_str)
            return
        self.port_tab[p_name] = (p_default_val, p_type)

        self.port_lst_widget.insert(tk.END, p_name)
        self.p_type_lst_widget.insert(tk.END, p_type_str)
        self.p_default_lst_widget.insert(tk.END, p_default_str)

        self.port_entry.delete(0, last=tk.END)
        self.p_type_var.set(self.p_type_lst[0])
        self.p_default_entry.delete(0, last=tk.END)
        self.p_default_entry.insert(0, 'None')
        return

    def cmd_del(self):
        p_name_index = self.port_lst_widget.curselection()
        p_type_index = self.p_type_lst_widget.curselection()
        p_default_index = self.p_default_lst_widget.curselection()

        if p_name_index != ():
            index = int(p_name_index[0])
        elif p_type_index != ():
            index = int(p_type_index[0])
        elif p_default_index != ():
            index = int(p_default_index[0])
        else:
            tkinter.messagebox.showwarning(title='Delete Port', message='No Selection')
            return

        p_name = self.port_lst_widget.get(index)

        if p_name in self.port_tab:
            del self.port_tab[p_name]
            self.port_lst_widget.delete(index)
            self.p_type_lst_widget.delete(index)
            self.p_default_lst_widget.delete(index)
        else:
            tkinter.messagebox.showwarning(title='Delete Port', message='Can not find Port Name')
        return

    def cmd_cancel(self):
        self.port_tab = None
        if self.ret_fkt is not None:
            self.ret_fkt(self.port_tab)
        self.tk_master.destroy()
        return

    def cmd_ok(self):
        if self.ret_fkt is not None:
            self.ret_fkt(self.port_tab)
        self.port_tab = None
        self.tk_master.destroy()
        return

    def cmd_exit(self):
        self.port_tab = None
        if self.ret_fkt is not None:
            self.ret_fkt(self.port_tab)
        self.tk_master.destroy()
        return

    def cmd_none(self):
        return
