from future import standard_library
standard_library.install_aliases()
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


class RCTKSlider(object):

    def __init__(self, master=None, init_data=None, widget_in_tab=None, widget_out_tab=None, io_mode='OUT',
                 orient=tk.VERTICAL, interactive=False, height=rc_util.rc_min_size, width=rc_util.rc_min_size,
                 name='Slider: ', lower_bound=-1, upper_bound=1, resolution=0.01, tickinterval=0.25, **kwd):

        self.tk_master = master
        self.init_data = init_data
        self.default_relief = tk.FLAT
        self.rc_event = None
        #self.width = width
        #self.height = height

        self.name = name

        self.io_mode = io_mode
        self.orient = orient
        self.lower_bound = -1
        self.upper_bound = 1
        self.resolution = 0.01
        self.tickinterval = 0.25
        self.widget_in_tab = []
        if widget_in_tab is not None:
            self.widget_in_tab = widget_in_tab
        self.widget_out_tab = []
        if widget_out_tab is not None:
            self.widget_out_tab = widget_out_tab

        self.get_init_tab = {'get_data': (self.get_data, None)}
        self.set_init_tab = {'set_data': (self.set_data, None), }

# self.slider_config = ('I Scale', {'io_mode':io_mode, 'label':'Slider: ', 'from_':-1, 'to':1, 'resolution':0.01,
# 'tickinterval':0.25, 'bind':({'name':'idefault'},{'name':'out_default_data'})}, None)
##
# self.slider_config = ('I Scale', {'io_mode':io_mode, 'label':'Slider: ', 'from_':-1, 'to':1, 'resolution':0.01,
# 'tickinterval':0.25, 'widget_in_tab': self.widget_in_tab, 'widget_out_tab': self.widget_out_tab}, None)
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

        self.scale_widget = tk.Scale(self.tk_master, orient=self.orient, showvalue=1, relief=tk.FLAT,
                                     sliderrelief=tk.FLAT, width=8, sliderlength=16, bd=1, bg=rc_util.rc_bg_color,
                                     from_=self.lower_bound, to=self.upper_bound, highlightthickness=0,
                                     resolution=self.resolution)
        # resolution=self.resolution, tickinterval=self.tickinterval)
        # self.scale_widget.config(relief=FLAT)

        if self.io_mode == 'OUT':
            self.scale_widget['troughcolor'] = rc_util.rc_output_color
        else:
            self.scale_widget['troughcolor'] = rc_util.rc_input_color

        self.label_act_color = self.label_widget['bg']
        self.i_scale_act_color = self.scale_widget['bg']

        if self.io_mode == 'INOUT':
            self.out_scale = tk.Scale(self.tk_master, orient=self.orient, showvalue=1, relief=tk.FLAT,
                                      sliderrelief=tk.FLAT, width=8, sliderlength=16, bd=1, bg=rc_util.rc_bg_color,
                                      from_=self.lower_bound, to=self.upper_bound, highlightthickness=0,
                                      resolution=self.resolution)
            # resolution=self.resolution, tickinterval=self.tickinterval)

            self.out_scale['troughcolor'] = rc_util.rc_output_color
            self.o_scale_act_color = self.out_scale['bg']

            self.out_scale.bind('<Enter>', self._cmd_set_highlight)
            self.out_scale.bind('<Leave>', self._cmd_reset_highlight)

        self.label_widget.bind('<Enter>', self._cmd_set_highlight)
        self.label_widget.bind('<Leave>', self._cmd_reset_highlight)
        self.scale_widget.bind('<Enter>', self._cmd_set_highlight)
        self.scale_widget.bind('<Leave>', self._cmd_reset_highlight)

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

        # self.slider_config = ('I Scale', {'io_mode':io_mode, 'label':'Slider: ', 'from_':-1, 'to':1, 'resolution':0.01,
        # 'tickinterval':0.25, 'widget_in_tab': self.widget_in_tab, 'widget_out_tab': self.widget_out_tab}, None)
        ##        widget_structure = self.slider_config

        widget_structure = ['I Scale', {'name': self.name,
                                        'io_mode': self.io_mode,
                                        'lower_bound': self.lower_bound,
                                        'orient': self.orient,
                                        'upper_bound': self.upper_bound,
                                        'resolution': self.resolution,
                                        'tickinterval': self.tickinterval,
                                        'widget_in_tab': self.widget_in_tab,
                                        'widget_out_tab': self.widget_out_tab}, None]
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        return

    def grid_widgets(self, row=0, column=0, columspan=3, rowspan=3, orient=None):
        if orient is not None:
            self.orient = orient
            self.scale_widget['orient'] = self.orient
            if self.io_mode == 'INOUT':
                self.out_scale['orient'] = self.orient

        col_cnt, row_cnt = 0, 0

        if self.io_mode == 'IN' or self.io_mode == 'OUT':
            self.label_widget.grid_forget()
            self.scale_widget.grid_forget()
            if self.orient == tk.HORIZONTAL:
                self.label_widget.grid(row=row, column=column, sticky=tk.N + tk.E + tk.W +
                                       tk.S, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.scale_widget.grid(row=row, column=column + 1, columnspan=2, sticky=tk.N +
                                       tk.E + tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 3, 1
            else:
                self.label_widget.grid(row=row, column=column, sticky=tk.N + tk.E + tk.W +
                                       tk.S, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.scale_widget.grid(row=row + 1, column=column, rowspan=2, sticky=tk.N +
                                       tk.S, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 1, 3
        else:
            self.label_widget.grid_forget()
            self.scale_widget.grid_forget()
            self.out_scale.grid_forget()
            if self.orient == tk.HORIZONTAL:
                self.label_widget.grid(row=row, column=column, rowspan=2, sticky=tk.N + tk.E +
                                       tk.W + tk.S, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.scale_widget.grid(row=row, column=column + 1, columnspan=2, sticky=tk.E +
                                       tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.out_scale.grid(row=row + 1, column=column + 1, columnspan=2, sticky=tk.E +
                                    tk.W, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 3, 2
            else:
                self.label_widget.grid(row=row, column=column, columnspan=2, sticky=tk.N +
                                       tk.E + tk.W + tk.S, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.scale_widget.grid(row=row + 1, column=column, rowspan=2, sticky=tk.N +
                                       tk.S, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                self.out_scale.grid(row=row + 1, column=column + 1, rowspan=2, sticky=tk.N +
                                    tk.S, padx=rc_util.rc_padx, pady=rc_util.rc_pady)
                col_cnt, row_cnt = 2, 3
        return (col_cnt, row_cnt)

    def _cmd_popup(self, event):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** frame popup at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.rc_event = event
        self.popup.post(event.x_root, event.y_root)
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
        #local_get_src = ''
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
        #local_set_src = ''
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
            return self.scale_widget.get()
        return None

    def set_data(self, value):
        if self.io_mode == 'OUT':
            self.scale_widget.set(value)
        if self.io_mode == 'INOUT':
            self.out_scale.set(value)
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
        # self.scale_widget.config(bg=rc_hl_color)
        #if self.io_mode == 'INOUT': self.out_scale.config(bg=rc_hl_color)
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
        # self.scale_widget.config(bg=self.i_scale_act_color)
        #if self.io_mode == 'INOUT': self.out_scale.config(bg=self.o_scale_act_color)
        # self.config(bg=self.act_color)
        return
