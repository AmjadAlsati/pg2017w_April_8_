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


class RCIODialog(tk.Frame):

    def __init__(self, io_tab, port_init_tab, widget_data_tab, ret_fkt=None, name='UNDEF', direction='IN'):

        self.direction = direction
        self.name = name

        if io_tab is None:
            self.io_tab = []
        else:
            self.io_tab = copy.deepcopy(io_tab)

        if port_init_tab is None:
            self.port_init_tab = {}
        else:
            self.port_init_tab = port_init_tab

        if widget_data_tab is None:
            self.widget_tab = {}
        else:
            self.widget_tab = widget_data_tab

        self.data = ()
        for x in self.widget_tab:
            self.data += (x,)
        if self.data == () or self.data is None:
            tkinter.messagebox.showwarning(title='Get/Set Widget Data', message='Can not set/get any widget data')
            return

        self.ret_fkt = ret_fkt

        self.tk_master = tk.Toplevel()
        self.create_widgets()
        self._init_widgets()

        return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master)
        self.tk_master.protocol("WM_DELETE_WINDOW", self.cmd_exit)
        self.tk_master.resizable(width=1, height=1)
        self.tk_master.rowconfigure(0, weight=1)
        self.tk_master.columnconfigure(0, weight=1)
        self.node_title = tk.StringVar()

        if self.direction == 'IN':
            title_str = 'Get Data from Widget %s for Output Ports' % str(self.name)
            label_0_txt = 'Get Data'
            label_1_txt = 'Selector'
            label_3_txt = 'Output Port'
            label_4_txt = 'Selector'

            label_5_txt = 'Data Init'
            label_8_txt = 'Port Init'
        else:
            title_str = 'Set Data from Input Ports to Widget %s' % str(self.name)
            label_0_txt = 'Input Port'
            label_1_txt = 'Selector'
            label_3_txt = 'Set Data'
            label_4_txt = 'Selector'

            label_5_txt = 'Port Init'
            label_8_txt = 'Data Init'

        self.node_title.set(title_str)
        self.tk_master.title(self.node_title.get())
        ws = self.tk_master.winfo_screenwidth()
        hs = self.tk_master.winfo_screenheight()
        x = (old_div(ws, 2)) - (old_div(rc_util.rc_window_init_xsize, 4))
        y = (old_div(hs, 2)) - (old_div(rc_util.rc_window_init_ysize, 4))
        #self.tk_master.geometry('%dx%d+%d+%d'%(rc_window_init_xsize, rc_window_init_ysize, x, y))
        self.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        # structure main window
        self['bg'] = rc_util.rc_bg_color
        self.act_color = self['bg']

        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=3)
        self.rowconfigure(2, weight=0)
        self.rowconfigure(3, weight=0)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=0)
        self.columnconfigure(3, weight=1)
        self.columnconfigure(4, weight=1)

        # row 0 listbox
        self.label_0 = tk.Label(self, text=label_0_txt, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.label_0.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)

        self.label_1 = tk.Label(self, text=label_1_txt, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.label_1.grid(row=0, column=1, sticky=tk.N + tk.E + tk.S + tk.W)

        self.label_3 = tk.Label(self, text=label_3_txt, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.label_3.grid(row=0, column=3, sticky=tk.N + tk.E + tk.S + tk.W)

        self.label_4 = tk.Label(self, text=label_4_txt, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.label_4.grid(row=0, column=4, sticky=tk.N + tk.E + tk.S + tk.W)

        # row 1 name type
        widget_line = min((len(self.port_init_tab) + 3, 3))
        #widget_line = 3

        self.lst_0_var = tk.StringVar()
        self.lst_0_widget = tk.Listbox(self, listvariable=self.lst_0_var, selectmode=tk.SINGLE, exportselection=0,
                                       width=10, height=widget_line, bg=rc_util.rc_bg_txt_color, bd=0, highlightthickness=0, relief=tk.FLAT)
        self.lst_0_widget.grid(row=1, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.lst_1_var = tk.StringVar()
        self.lst_1_widget = tk.Listbox(self, listvariable=self.lst_1_var, selectmode=tk.SINGLE, exportselection=0,
                                       width=10, height=widget_line, bg=rc_util.rc_bg_txt_color, bd=0, highlightthickness=0,
                                       selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.lst_1_widget.grid(row=1, column=1, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.edge_lst_label = tk.Label(self, text='=', bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.edge_lst_label.grid(row=1, column=2, sticky=tk.N + tk.E + tk.S + tk.W)

        self.lst_3_var = tk.StringVar()
        self.lst_3_widget = tk.Listbox(self, listvariable=self.lst_3_var, selectmode=tk.SINGLE, exportselection=0,
                                       width=10, height=widget_line, bg=rc_util.rc_bg_txt_color, bd=0, highlightthickness=0,
                                       selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.lst_3_widget.grid(row=1, column=3, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        self.lst_4_var = tk.StringVar()
        self.lst_4_widget = tk.Listbox(self, listvariable=self.lst_4_var, selectmode=tk.SINGLE, exportselection=0,
                                       width=10, height=widget_line, bg=rc_util.rc_bg_txt_color, bd=0, highlightthickness=0,
                                       selectforeground=rc_util.rc_black, selectbackground=rc_util.rc_hl_color, relief=tk.FLAT)
        self.lst_4_widget.grid(row=1, column=4, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        # row 2 name type
        self.port_entry = tk.Entry(self, relief=tk.FLAT, highlightthickness=0, bg=rc_util.rc_bg_txt_color)

        self.port_selector_entry = tk.Entry(self, relief=tk.FLAT, highlightthickness=0, bg=rc_util.rc_bg_txt_color)

        self.data_entry_var = tk.StringVar()
        self.data_option = tk.OptionMenu(self, self.data_entry_var, *self.data)
        self.data_option['menu'].config(activebackground=rc_util.rc_button_color, activeforeground=rc_util.rc_black)
        self.data_option.config(bg=rc_util.rc_bg_txt_color, borderwidth=0, relief=tk.FLAT)

        if self.data != ():
            self.data_entry_var.set(self.data[0])

        self.data_selector_entry = tk.Entry(self, relief=tk.FLAT, highlightthickness=0, bg=rc_util.rc_bg_txt_color)

        if self.direction == 'IN':
            self.data_option.grid(row=2, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=2, pady=2)
            self.data_selector_entry.grid(row=2, column=1, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)
            self.port_entry.grid(row=2, column=3, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)
            self.port_selector_entry.grid(row=2, column=4, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)
        else:
            self.port_entry.grid(row=2, column=0, sticky=tk.N + tk.E + tk.S + tk.W, padx=2, pady=2)
            self.port_selector_entry.grid(row=2, column=1, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)
            self.data_option.grid(row=2, column=3, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)
            self.data_selector_entry.grid(row=2, column=4, sticky=tk.N + tk.E + tk.S + tk.W, padx=4, pady=4)

        # row 3 Buttons
        self.add_button = tk.Button(self, text='New', bg=rc_util.rc_bg_color,
                                    bd=0, command=self.cmd_new, relief=tk.FLAT)
        self.add_button.grid(row=3, column=0, sticky=tk.E + tk.W, padx=4, pady=8)
        self.del_button = tk.Button(self, text='Delete', bg=rc_util.rc_bg_color,
                                    bd=0, command=self.cmd_del, relief=tk.FLAT)
        self.del_button.grid(row=3, column=1, sticky=tk.E + tk.W, padx=4, pady=8)
        self.cancel_button = tk.Button(self, text='Cancel', bg=rc_util.rc_bg_color,
                                       bd=0, command=self.cmd_cancel, relief=tk.FLAT)
        self.cancel_button.grid(row=3, column=3, sticky=tk.E + tk.W, padx=4, pady=8)
        self.ok_button = tk.Button(self, text='OK', bg=rc_util.rc_bg_color, bd=0, command=self.cmd_ok, relief=tk.FLAT)
        self.ok_button.grid(row=3, column=4, sticky=tk.E + tk.W, padx=4, pady=8)
        return

    def _init_widgets(self):
        self.lst_0_str = ()
        self.lst_1_str = ()
        self.lst_3_str = ()
        self.lst_4_str = ()
        for x in self.io_tab:
            self.lst_0_str += (x[0],)
            self.lst_1_str += (x[1],)
            self.lst_3_str += (x[2],)
            self.lst_4_str += (x[3],)

        self.lst_0_var.set(self.lst_0_str)
        self.lst_1_var.set(self.lst_1_str)
        self.lst_3_var.set(self.lst_3_str)
        self.lst_4_var.set(self.lst_4_str)
        return

    def cmd_new(self):
        # get and check data
        p_name = self.port_entry.get()
        p_name = p_name.strip()
        if not p_name.isalnum() or p_name[0].isdigit():
            tkinter.messagebox.showwarning(title='Get/Set Widget Data', message='Illegal Identifiere')
            return
        if p_name not in self.port_init_tab:
            ret = tkinter.messagebox.askokcancel(master=self, title='Get/Set Widget Data',
                                                 message='Port ' + p_name + ' does not exist')
            if not ret:
                return

        p_sel = self.port_selector_entry.get()
        p_sel = p_sel.strip()
        d_name = self.data_entry_var.get()
        d_sel = self.data_selector_entry.get()
        d_sel = d_sel.strip()

        if self.direction == 'IN':
            self.io_tab.append((d_name, d_sel, p_name, p_sel))
            self.lst_0_widget.insert(tk.END, d_name)
            self.lst_1_widget.insert(tk.END, d_sel)
            self.lst_3_widget.insert(tk.END, p_name)
            self.lst_4_widget.insert(tk.END, p_sel)
        else:
            self.io_tab.append((p_name, p_sel, d_name, d_sel))
            self.lst_0_widget.insert(tk.END, p_name)
            self.lst_1_widget.insert(tk.END, p_sel)
            self.lst_3_widget.insert(tk.END, d_name)
            self.lst_4_widget.insert(tk.END, d_sel)

        self.port_entry.delete(0, last=tk.END)
        self.port_selector_entry.delete(0, last=tk.END)
        ### if self.data != None and self.data != (): self.data_entry_var.set(self.data[0])
        self.data_entry_var.get()
        self.data_selector_entry.delete(0, last=tk.END)
        return

# check a connection ???
# try:
##            p_default_str = self.p_default_entry.get()
##            p_default_val = eval(p_default_str, vars(cmath),{})
##            p_default_str = repr(p_default_val)
# except:
##            error_list = extract_tb(sys.exc_info()[2])
##            error = error_list[len(error_list)-1]
##            error_str = 'Error in Default Value:\n    %s\n   File: %s , Line number: %s ,\n   Method: %s , Statement: %s'\
# %(sys.exc_info()[1], error[0], error[1], error[2], error[3])
##            tkMessageBox.showerror('Port Default Value', error_str)
# return

    def cmd_del(self):
        lst_0_sel = self.lst_0_widget.curselection()
        lst_1_sel = self.lst_1_widget.curselection()
        lst_3_sel = self.lst_3_widget.curselection()
        lst_4_sel = self.lst_4_widget.curselection()

        if lst_0_sel != ():
            index = int(lst_0_sel[0])
        elif lst_1_sel != ():
            index = int(lst_1_sel[0])
        elif lst_3_sel != ():
            index = int(lst_3_sel[0])
        elif lst_4_sel != ():
            index = int(lst_4_sel[0])
        else:
            tkinter.messagebox.showwarning(title='Widget IO', message='No Selection')
            return

        self.lst_0_widget.delete(index)
        self.lst_1_widget.delete(index)
        self.lst_3_widget.delete(index)
        self.lst_4_widget.delete(index)

        del self.io_tab[index]
        return

    def cmd_cancel(self):
        self.destroy()
        self.tk_master.destroy()
        return

    def cmd_ok(self):
        if self.ret_fkt is not None:
            self.ret_fkt(self.io_tab)
        self.destroy()
        self.tk_master.destroy()
        return

    def cmd_exit(self):
        # for test only
        # self.quit()
        self.destroy()
        self.tk_master.destroy()
        return

    def cmd_none(self):
        return

if __name__ == '__main__':
    # Tk()
    io_tab = [('name_1', "['sel_1']", 'name_2', "['sel_2]'")]
    port_tab = (('test_port', ''))  # port_name:(selctor, init_val)
    data_tab = (('test_data_1', ''), ('test_data_2', ''))  # ((selktor, init_val), )
    # test =  RCIODialog(io_tab, port_tab, data_tab, ret_fkt=None, name='test_name', direction='IN')
    test = RCIODialog(None, None, data_tab, ret_fkt=None, name='test_name', direction='IN')
    test.mainloop()
