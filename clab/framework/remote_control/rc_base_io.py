from future import standard_library
standard_library.install_aliases()
################
#
# Modul:        PaderKicker Monitor
# File:         rc_base_io.py
#
# Author:       Bernd Kleinjohann
# Created:      June 2011
# Version:      2.0
# Contents:     this class contains all common data for themonitor thread
#
import tkinter as tk

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_io_dialog import RCIODialog


class RCBaseIO(tk.Frame):

    def __init__(self, master=None, init_data=None, widget_in_tab=None, widget_out_tab=None,
                 height=rc_util.rc_min_size, width=rc_util.rc_min_size, name='Undefined: ', relief=tk.FLAT, **kwd):

        self.tk_master = master
        self.init_data = init_data
        self.default_relief = relief
        self.rc_event = None
        self.width = width
        self.height = height

        self.name = name

        self.widget_in_tab = []
        if widget_in_tab is not None:
            self.widget_in_tab = widget_in_tab
        self.widget_out_tab = []
        if widget_out_tab is not None:
            self.widget_out_tab = widget_out_tab

        self.get_init_tab = {}
        self.set_init_tab = {}

        self.create_io_widgets()

        return

    def create_io_widgets(self):  # create widgets
        tk.Frame.__init__(self, self.tk_master, relief=self.default_relief,
                          bd=rc_util.rc_border, width=self.width, height=self.height)
        self.act_color = self['bg']

        # change bg color iff curser in frame
        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)

        self.popup = tk.Menu(self, tearoff=0)
        self.popup.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                          activeforeground=rc_util.rc_black)
        self.popup.add_command(label='Config set', command=self.cmd_set_dialog)
        self.popup.add_command(label='Config get', command=self.cmd_get_dialog)
        self.bind('<Button-3>', self._cmd_popup)
        return

    def _cmd_popup(self, event):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** frame popup at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.rc_event = event

        self.unbind('<Enter>')
        self.unbind('<Leave>')
        self.popup.post(event.x_root, event.y_root)
        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

    def connect_widgets(self, id_n, get_src, set_src, widget_init_tab=None,
                        init_inport=None, init_outport=None, **kwd):
        # erzeugt den get code fuer die widget anbindung
        local_get_src = '### get for widget %s id: %d\n' % (self.name, id_n)
        if self.widget_in_tab is not None and self.widget_in_tab != []:
            d_get_lst = []
            for d_name, d_sel, p_name, p_sel in self.widget_in_tab:
                if not '%s_data_%d' % (d_name, id_n) in d_get_lst:
                    # create lione: d_name_data_id_n = get_tab[d_name_id_n]()
                    local_get_src += "    %s_data_%d = get_tab['%s_%d'][0]()\n" % (d_name, id_n, d_name, id_n)
                    widget_init_tab['%s_%d' % (d_name, id_n)] = (
                        self.get_init_tab[d_name][0], self.get_init_tab[d_name][1])
                    d_get_lst.append('%s_data_%d' % (d_name, id_n))

                # create line: if p_name in port_tab: port_tab[p_name] p_sel = d_name_data_id_n [d_sel]
                local_get_src += "    if '%s' in port_tab: port_tab['%s'] %s = %s_data_%d %s\n" % (
                    p_name, p_name, p_sel, d_name, id_n, d_sel)
        get_src += local_get_src

        # erzeugt den set code fuer die widget anbindung
        local_set_src = '### set for widget %s id: %d\n' % (self.name, id_n)
        if self.widget_out_tab is not None and self.widget_out_tab != []:
            d_set_lst = []
            for p_name, p_sel, d_name, d_sel in self.widget_out_tab:
                if not '%s_data_%d' % (d_name, id_n) in d_set_lst:
                    # create lione: d_name_data_id_n = copy.deepcopy(set_tab[d_name_id_n][1])
                    local_set_src += "    %s_data_%d = copy.deepcopy(set_tab['%s_%d'][1])\n" % (
                        d_name, id_n, d_name, id_n)
                    widget_init_tab['%s_%d' % (d_name, id_n)] = (
                        self.set_init_tab[d_name][0], self.set_init_tab[d_name][1])
                    d_set_lst.append('%s_data_%d' % (d_name, id_n))

                # create line: if p_name in port_tab: d_name_data_id_n d_sel = port_tab[p_name] p_sel
                local_set_src += "    if '%s' in port_tab: %s_data_%d %s = port_tab['%s'] %s\n" % (
                    p_name, d_name, id_n, d_sel, p_name, p_sel)
            #local_set_src += '\n'
            for d_name in self.set_init_tab:
                if '%s_data_%d' % (d_name, id_n) in d_set_lst:
                    # create line: set_tab[d_name_id_n](d_name_data_id_n)
                    local_set_src += "    set_tab['%s_%d'][0](%s_data_%d)\n" % (d_name, id_n, d_name, id_n)
            #local_set_src += '\n'
        set_src += local_set_src

        return id_n + 1, get_src, set_src

    def set_ret_fkt(self, tab):
        if tab is not None:
            self.widget_out_tab = tab
        return

    def cmd_set_dialog(self):
        RCIODialog(self.widget_out_tab, self.init_data.init_inport, self.set_init_tab,
                   ret_fkt=self.set_ret_fkt, name='Undefined: ', direction='OUT')
        return

    def get_ret_fkt(self, tab):
        if tab is not None:
            self.widget_in_tab = tab
        return

    def cmd_get_dialog(self):
        RCIODialog(self.widget_in_tab, self.init_data.init_outport, self.get_init_tab,
                   ret_fkt=self.get_ret_fkt, name='Undefined: ', direction='IN')
        return

    def _cmd_set_highlight(self, event):
        self.rc_event = event
        # print 'set rc base'
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
        # print 'reset rc base'
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** frame reset hightlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_set_highlight()
        self.rc_reset_highlight()
        return

    def rc_reset_highlight(self):
        self.config(bg=rc_util.rc_bg_color)
        return
