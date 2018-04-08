from future import standard_library
standard_library.install_aliases()
################
#
# Modul:        PaderKicker Monitor
# File:         rc_frame.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2015
# Version:      1.0
# Contents:     empty frame class for rc
#
import tkinter as tk

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_init_node import RCInitTkGui


class RCFrame(tk.Frame):

    def __init__(self, master=None, width=rc_util.rc_min_size,
                 height=rc_util.rc_min_size, relief=tk.FLAT, init_data=None, **kwd):

        self.tk_master = master
        self.init_data = init_data

        self.height = height
        self.width = width
        self.default_relief = relief

        self.rc_event = None
        self.rc_slave_widget = None
        self.rc_slave_widget_cmd = None

        self.create_widgets()
        return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master, width=self.width,
                          height=self.height, relief=self.default_relief)
        self['bg'] = rc_util.rc_bg_empty_color
        self.act_color = self['bg']
        self.columnconfigure(0, weight=1, minsize=rc_util.rc_min_size)
        self.rowconfigure(0, weight=1, minsize=rc_util.rc_min_size)

        # install pop up menu in frame self
        self.popup = tk.Menu(self, tearoff=0)
        self.popup.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                          activeforeground=rc_util.rc_black)
        for widget_cmd in RCInitTkGui.rc_menu_widgets:
            if SMOBaseObject.info:
                SMOBaseObject.debug_handler.out(
                    '*** INFO *** RCFrame: Define menu comand: %s ' % widget_cmd)
            # define cmd handler

            def cmd_handler(self=self, widget_cmd=widget_cmd):
                if SMOBaseObject.info:
                    SMOBaseObject.out(
                        '*** INFO *** RCFrame: Execute layout comand: %s' % widget_cmd)
                if self.rc_slave_widget is not None:
                    self.rc_slave_widget.destroy()
                    self.rc_slave_widget = None
                # cmd/sub is an entry in PKMonitorBaseClass.pkm_content_widgets and
                # has the format {...,sub:{ ... ,label: (classdefinition, {init params}), ...},...}
                self['bg'] = rc_util.rc_bg_color  # may be changed by new_slave
                new_slave = RCInitTkGui.rc_menu_widgets[widget_cmd]
                self.rc_slave_widget = new_slave[0](master=self, init_data=self.init_data, **new_slave[1])
                self.rc_slave_widget_cmd = widget_cmd

                self.rc_slave_widget.grid(row=0, column=0, sticky=tk.N + tk.E + tk.S + tk.W)
                self.config(bd=rc_util.rc_border, relief=tk.FLAT)
                self.rc_event = None
                self.act_color = self['bg']
                return
            self.popup.add_command(label=widget_cmd, command=cmd_handler)

        self.popup.add_command(label='Delete', command=self._cmd_delete)
        self.bind('<Button-3>', self._cmd_popup)

        # change bg color iff curser in frame
        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

    def get_installed_widgets(self):
        if self.rc_slave_widget is None:
            widget_structure = ['Frame', {}, None]
        else:
            widget_structure = ['Frame', {}, {'widget_structure': self.rc_slave_widget.get_installed_widgets()}]
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        if widget_structure is None:
            self['bg'] = rc_util.rc_bg_empty_color
            self.act_color = self['bg']
            return

        if self.rc_slave_widget is not None:
            # achtung aufraeumen
            self.rc_slave_widget.destroy()
            self.rc_slave_widget = None

        self['bg'] = rc_util.rc_bg_color
        self.act_color = self['bg']

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
        # if recursive == False:
        if self.rc_slave_widget is None:
            return id_n, get_src, set_src
        else:
            return self.rc_slave_widget.connect_widgets(id_n, get_src, set_src, **kwd)

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

    def _cmd_set_highlight(self, event):
        self.rc_event = event
        # print 'set rc frame'
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
        # print 'reset rc frame'
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

    def _cmd_delete(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** PKMonitorFrame: Execute content comand: delete ')
        if self.rc_slave_widget is not None:
            self.rc_slave_widget.destroy()
            self.rc_slave_widget = None
            self['bg'] = rc_util.rc_bg_empty_color
            self.act_color = self['bg']
        self.config(relief=tk.FLAT)
        return
