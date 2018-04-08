from future import standard_library
standard_library.install_aliases()
from builtins import str
from builtins import object
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
import tkinter.simpledialog

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_io_dialog import RCIODialog


class RCTKEntry(object):

    def __init__(self, master=None, init_data=None, widget_in_tab=None, widget_out_tab=None,
                 io_mode='OUT', name='Entry', orient=tk.VERTICAL, entry_type='string',
                 interactive=False, relief=tk.FLAT, **kwd):

        self.tk_master = master
        self.init_data = init_data
        self.default_relief = relief
        self.rc_event = None
        #self.width = width
        #self.height = height

        self.name = name
        self.io_mode = io_mode
        self.orient = orient
        self.entry_type = entry_type

        self.widget_in_tab = []
        if widget_in_tab is not None:
            self.widget_in_tab = widget_in_tab
        self.widget_out_tab = []
        if widget_out_tab is not None:
            self.widget_out_tab = widget_out_tab

        self.get_init_tab = {'get_data': (self.get_data, None)}
        self.set_init_tab = {'set_data': (self.set_data, None)}

        self.label_widget = None
        self.entry_widget = None
        self.out_entry = None

##        self.entry_config = ('I Entry', {'io_mode':io_mode, 'label':'Entry', 'type':'string'}, None)

        if interactive:
            # ggf. weiterer dialog
            name = tkinter.simpledialog.askstring(self.tk_master, prompt='Scaler label?',
                                                  initialvalue=self.name)
            if name != '' and name is not None:
                self.name = name
        self.create_widgets()
        return

    def create_widgets(self):

        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** simple io cmd_slider')

        self.label_widget = tk.Label(self.tk_master, text=self.name, bg=rc_util.rc_bg_color, relief=tk.FLAT)
        self.entry_widget = tk.Entry(self.tk_master)
        #self.entry_widget['relief'] = FLAT
        if self.io_mode == 'OUT':
            self.entry_widget.config(relief=tk.FLAT, highlightthickness=0, disabledbackground=rc_util.rc_output_color,
                                     bg=rc_util.rc_output_color, state="disabled")
        else:
            self.entry_widget.config(relief=tk.FLAT, highlightthickness=0, disabledbackground=rc_util.rc_input_color,
                                     bg=rc_util.rc_input_color)
        self.label_act_color = self.label_widget['bg']
        self.in_act_color = self.entry_widget['bg']

        if self.io_mode == 'INOUT':
            self.out_entry = tk.Entry(self.tk_master)
            self.out_entry.config(relief=tk.FLAT, highlightthickness=0, disabledbackground=rc_util.rc_output_color,
                                  bg=rc_util.rc_output_color, state="disabled")
            #self.out_entry['disabledbackground'] = self.act_color
            self.out_entry['state'] = "disabled"
            self.out_entry['relief'] = tk.FLAT
            self.out_act_color = self.out_entry['bg']

            self.out_entry.bind('<Enter>', self._cmd_set_highlight)
            self.out_entry.bind('<Leave>', self._cmd_reset_highlight)

        self.label_widget.bind('<Enter>', self._cmd_set_highlight)
        self.label_widget.bind('<Leave>', self._cmd_reset_highlight)
        self.entry_widget.bind('<Enter>', self._cmd_set_highlight)
        self.entry_widget.bind('<Leave>', self._cmd_reset_highlight)

        self.popup = tk.Menu(self.label_widget, tearoff=0)
        self.popup.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                          activeforeground=rc_util.rc_black)
        self.popup.add_command(label='Config set', command=self.cmd_set_dialog)
        self.popup.add_command(label='Config get', command=self.cmd_get_dialog)
        self.popup.add_command(label='Config Widget', command=self._cmd_config)
        #self.popup.add_command(label='Delete', command = self._cmd_delete)
        self.label_widget.bind('<Button-3>', self._cmd_popup)
        return

    def get_installed_widgets(self):
        widget_structure = ['I Entry', {'name': self.name,
                                        'io_mode': self.io_mode,
                                        'orient': self.orient,
                                        'entry_type': self.entry_type,
                                        'widget_in_tab': self.widget_in_tab,
                                        'widget_out_tab': self.widget_out_tab}, None]
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        return

    def grid_widgets(self, row=0, column=0, columspan=3, rowspan=3, orient=None):
        if orient is not None:
            self.orient = orient

        col_cnt, row_cnt = 0, 0
        if self.io_mode == 'IN' or self.io_mode == 'OUT':
            self.label_widget.grid_forget()
            self.entry_widget.grid_forget()
            if self.orient == tk.HORIZONTAL:
                self.label_widget.grid(row=row, column=column, sticky=tk.N + tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.entry_widget.grid(row=row, column=column + 1, columnspan=2, sticky=tk.N +
                                       tk.E + tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 3, 1
            else:
                self.label_widget.grid(row=row, column=column, sticky=tk.N + tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.entry_widget.grid(row=row + 1, column=column, sticky=tk.N + tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 1, 3
        else:
            self.label_widget.grid_forget()
            self.entry_widget.grid_forget()
            self.out_entry.grid_forget()
            if self.orient == tk.HORIZONTAL:
                self.label_widget.grid(row=row, column=column, sticky=tk.N + tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.entry_widget.grid(row=row, column=column + 1, sticky=tk.N + tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.out_entry.grid(row=row, column=column + 2, sticky=tk.N + tk.E +
                                    tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 3, 1
            else:
                self.label_widget.grid(row=row, column=column, sticky=tk.N + tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.entry_widget.grid(row=row + 1, column=column, sticky=tk.N + tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.out_entry.grid(row=row + 2, column=column, sticky=tk.N + tk.E +
                                    tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 1, 3
        return (col_cnt, row_cnt)

    def _cmd_popup(self, event):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** frame popup at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.rc_event = event
        # self.label_widget.unbind('<Enter>')
        # self.label_widget.unbind('<Leave>')
        self.popup.post(event.x_root, event.y_root)
        # self.label_widget.bind('<Enter>',self._cmd_set_highlight)
        # self.label_widget.bind('<Leave>',self._cmd_reset_highlight)
        return

    def _cmd_config(self):
        name = tkinter.simpledialog.askstring(self.label_widget, prompt='Entry label?')
        if name is not None and name != '':
            self.name = name
        self.label_widget.config(text=self.name)
        return

    def connect_widgets(self, id_n, get_src, set_src, widget_init_tab=None,
                        init_inport=None, init_outport=None, **kwd):
        # erzeugt den get code fuer die widget anbindung
        local_get_src = '### get for widget %s id: %d\n' % (self.name, id_n)
        if self.widget_in_tab is not None and self.widget_in_tab != []:
            for d_name in self.get_init_tab:
                # create lione: d_name_data_id_n = get_tab[d_name_id_n]()
                local_get_src += "    %s_data_%d = get_tab['%s_%d'][0]()\n" % (d_name, id_n, d_name, id_n)
                widget_init_tab['%s_%d' % (d_name, id_n)] = (
                    self.get_init_tab[d_name][0], self.get_init_tab[d_name][1])
            #local_get_src += '\n'

            for d_name, d_sel, p_name, p_sel in self.widget_in_tab:
                # create line: if p_name in port_tab: port_tab[p_name] p_sel = d_name_data_id_n [d_sel]
                local_get_src += "    if '%s' in port_tab: port_tab['%s'] %s = %s_data_%d %s\n" % (
                    p_name, p_name, p_sel, d_name, id_n, d_sel)
            #local_get_src += '\n'
        get_src += local_get_src
    ###
        # erzeugt den set code fuer die widget anbindung
        local_set_src = '### set for widget %s id: %d\n' % (self.name, id_n)
        if self.widget_out_tab is not None and self.widget_out_tab != []:
            for d_name in self.set_init_tab:
                # create lione: d_name_data_id_n = copy.deepcopy(set_tab[d_name_id_n][1])
                local_set_src += "    %s_data_%d = copy.deepcopy(set_tab['%s_%d'][1])\n" % (d_name, id_n, d_name, id_n)
                widget_init_tab['%s_%d' % (d_name, id_n)] = (
                    self.set_init_tab[d_name][0], self.set_init_tab[d_name][1])
            #local_set_src += '\n'

            for p_name, p_sel, d_name, d_sel in self.widget_out_tab:
                # create line: if p_name in port_tab: d_name_data_id_n d_sel = port_tab[p_name] p_sel
                local_set_src += "    if '%s' in port_tab: %s_data_%d %s = port_tab['%s'] %s\n" % (
                    p_name, d_name, id_n, d_sel, p_name, p_sel)
            #local_set_src += '\n'

            for d_name in self.set_init_tab:
                # create line: set_tab[d_name_id_n](d_name_data_id_n)
                local_set_src += "    set_tab['%s_%d'][0](%s_data_%d)\n" % (d_name, id_n, d_name, id_n)
            #local_set_src += '\n'
        set_src += local_set_src

        return id_n + 1, get_src, set_src

    def get_data(self):
        if self.io_mode == 'IN' or self.io_mode == 'INOUT':
            return self.entry_widget.get()
        return None

    def set_data(self, value):
        # print 'st: ', value
        if self.io_mode == 'OUT':
            self.entry_widget.delete(0, tk.END)
            self.entry_widget.insert(tk.END, str(value))
        if self.io_mode == 'INOUT':
            # print 'st2: ', value
            self.out_entry['state'] = "normal"

            self.out_entry.delete(0, tk.END)
            self.out_entry.insert(tk.END, str(value))
        return

    def set_ret_fkt(self, tab):
        if tab is not None:
            self.widget_out_tab = tab
        return

    def cmd_set_dialog(self):
        RCIODialog(self.widget_out_tab, self.init_data.init_inport, self.set_init_tab,
                   ret_fkt=self.set_ret_fkt, name=self.name, direction='OUT')
        return

    def get_ret_fkt(self, tab):
        if tab is not None:
            self.widget_in_tab = tab
        return

    def cmd_get_dialog(self):
        RCIODialog(self.widget_in_tab, self.init_data.init_outport, self.get_init_tab,
                   ret_fkt=self.get_ret_fkt, name=self.name, direction='IN')
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
        self.label_widget.config(bg=rc_util.rc_hl_color)
        # self.entry_widget.config(bg=rc_hl_color)
        #if self.io_mode == 'INOUT': self.out_entry.config(bg=rc_hl_color)
        # self.config(bg=rc_hl_color)
        return

    def _cmd_reset_highlight(self, event):
        self.pkm_event = event
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** frame reset hightlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_set_highlight()
        self.rc_reset_highlight()
        return

    def rc_reset_highlight(self):
        self.label_widget.config(bg=self.label_act_color)
        # self.entry_widget.config(bg=self.in_act_color)
        #if self.io_mode == 'INOUT': self.out_entry.config(bg=self.out_act_color)
        # self.config(bg=self.act_color)
        return
