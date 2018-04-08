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
import Tkinter as tk
import tkSimpleDialog

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_init_node import RCInitTkGui
from .rc_frame import RCFrame


class RCTabed(tk.LabelFrame):
    # tab count for unique tab labels (for test use)

    def __init__(self, master=None, init_data=None,
                 width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT, cmd_side='nw', **kwd):

        self.tk_master = master
        self.init_data = init_data
        self.height = height
        self.width = width
        self.default_relief = relief
        self.cmd_side = cmd_side

        self.rc_event = None
        self.rc_slave_widget = []
        self.rc_tab_but = []
        self.rc_tab_but_label = []
        self.act_slave = -1
        self.tab_cnt = 0

        self.create_widgets()
        return

    def create_widgets(self):
        tk.LabelFrame.__init__(self, self.tk_master, width=self.width,
                               height=self.height, relief=self.default_relief)
        self.act_color = self['bg']
        self.rowconfigure(0, weight=1, minsize=rc_util.rc_min_size)
        self.columnconfigure(0, weight=1, minsize=rc_util.rc_min_size)
        if self.cmd_side in ('nw', 'n', 'ne', 'sw', 's', 'se'):
            self.vert = False
        elif self.cmd_side in ('wn', 'w', 'ws', 'en', 'n', 'es'):
            self.vert = True
        else:
            self.cmd_side = 'nw'
            self.vert = False

        # create label Buttons
        self.tab_cmd_frame = tk.Frame(self, bd=rc_util.rc_button_border, padx=0, pady=0, relief=tk.FLAT)
        self.tab_cmd_label = tk.Label(self.tab_cmd_frame, text='Tabs:', relief=tk.FLAT, padx=0, pady=0, bd=0)
        if self.vert:
            self.tab_cmd_label.pack(side=tk.LEFT)
        else:
            self.tab_cmd_label.pack(side=tk.TOP)
        self.config(labelwidget=self.tab_cmd_frame)
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        # install pop up menu in frame self
        self.popup = tk.Menu(self, relief=tk.FLAT, tearoff=0)
        self.popup.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                          activeforeground=rc_util.rc_black)
        self.popup.add_command(label='Add Tab', command=self._cmd_create_tab)
        self.popup.add_command(label='Remove Tab', command=self._cmd_remove_tab)
        self.popup.add_command(label='Change Orient', command=self._cmd_change_side)
        self.popup.add_command(label='Order Left UP', command=self._cmd_order_left_up)
        self.popup.add_command(label='Order Right Down', command=self._cmd_order_right_down)
        self.popup.add_command(label='Delete Tabbed', command=self._cmd_delete_tab)
        self.bind('<Button-3>', self._cmd_popup)

        # change bg color iff curser in frame
        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

    def get_installed_widgets(self):
        widget_lst = self.rc_slave_widget
        if widget_lst is None or widget_lst == []:
            widget_structure = ['Tabbed', {'cmd_side': self.cmd_side}, None]
        else:
            slave_lst = []
            for x in widget_lst:
                slave_lst.append(x.get_installed_widgets())
            widget_structure = ['Tabbed', {'cmd_side': self.cmd_side},
                                {'label_lst': self.rc_tab_but_label, 'widget_structure': slave_lst}]
        return widget_structure

    def install_widgets(self, widget_structure=None, label_lst=None, **kwd):
        if widget_structure is None:
            return
        if label_lst is None:
            return
        if len(label_lst) < len(widget_structure):
            print 'illegal but widget data'
            return

        self.rc_event = None
        if self.rc_tab_but is not None:
            for but in self.rc_tab_but:
                but.destroy()
            self.rc_tab_but = []
        if self.rc_slave_widget is not None:
            for widget in self.rc_slave_widget:
                widget.destroy()
# aufraeumen

        for i, x in enumerate(label_lst):
            self.rc_slave_widget_cmd = widget_structure[i][0]
            widget_cmd = RCInitTkGui.rc_menu_widgets[self.rc_slave_widget_cmd][0]
            new_widget = widget_cmd(master=self, init_data=self.init_data, **widget_structure[i][1])

            cmd_text = label_lst[i]
            new_but = tk.Button(self.tab_cmd_frame, text=cmd_text)
            new_but.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)

            self.rc_slave_widget.append(new_widget)
            self.rc_tab_but.append(new_but)
            self.rc_tab_but_label.append(cmd_text)

            if widget_structure[i][2] is not None:
                new_widget.install_widgets(**widget_structure[i][2])

        self.update_layout()
        return

    def connect_widgets(self, id_n, get_src, set_src, **kwd):
        for wdg in self.rc_slave_widget:
            id_n, get_src, set_src = wdg.connect_widgets(id_n, get_src, set_src, **kwd)
        return id_n, get_src, set_src

    def update_layout(self):
        self.tab_cmd_frame.pack_forget()
        if self.vert:
            self.tab_cmd_label.pack(side=tk.TOP)
        else:
            self.tab_cmd_label.pack(side=tk.LEFT)
        for i, x in enumerate(self.rc_tab_but):
            # print 'update', i
            def cmd_switch_to_tab(self=self, tab=i):
                if SMOBaseObject.info:
                    SMOBaseObject.debug_handler.out('*** INFO *** switch to tab %d cmd' % tab)
                if tab >= 0 and tab < len(self.rc_slave_widget):
                    self.rc_slave_widget[self.act_slave].grid_remove()
                    self.rc_tab_but[self.act_slave].config(bg=self.act_color, relief=tk.FLAT)
                    self.act_slave = tab
                    print 'set act', self.act_slave
                    self.rc_slave_widget[tab].grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
                    self.rc_tab_but[tab].config(bg=rc_util.rc_hl_color, relief=tk.FLAT, padx=0, pady=0)
                return
            x.config(command=cmd_switch_to_tab)
            if i == self.act_slave:
                self.rc_slave_widget[i].grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)
                x.config(bg=rc_util.rc_hl_color, relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)
            else:
                self.rc_slave_widget[i].grid_remove()
                x.config(bg=self.act_color, relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)
            if self.vert:
                x.pack(side=tk.TOP)
            else:
                x.pack(side=tk.LEFT)
        # self.tab_cmd_frame.config(height=10)
        self.config(labelwidget=self.tab_cmd_frame)
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

    def _cmd_create_tab(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** create tab menu')
        new_widget = RCFrame(master=self, init_data=self.init_data,
                             width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
        default_cmd_text = 'Tab: ' + str(self.tab_cnt)
        self.tab_cnt += 1

        cmd_text = tkSimpleDialog.askstring(RCInitTkGui.rc_tk_root, prompt='Tab Label?',
                                            initialvalue=default_cmd_text)
        new_but = tk.Button(self.tab_cmd_frame, text=cmd_text)
        new_but.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color)

        if self.act_slave < 0 or self.act_slave >= len(self.rc_slave_widget):
            # print 'yyyyyyy append', str(self.act_slave)
            self.act_slave = len(self.rc_slave_widget)
            self.rc_slave_widget.append(new_widget)
            self.rc_tab_but.append(new_but)
            self.rc_tab_but_label.append(cmd_text)
        else:
            # print 'zzzzzzz insert', str(self.act_slave)
            self.rc_slave_widget.insert(self.act_slave, new_widget)
            self.rc_tab_but.insert(self.act_slave, new_but)
            self.rc_tab_but_label.insert(self.act_slave, cmd_text)
        self.update_layout()
        return

    def _cmd_remove_tab(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** delete tab menu')
        if self.act_slave >= 0 and self.act_slave < len(self.rc_slave_widget):
            self.rc_slave_widget[self.act_slave].destroy()
            del self.rc_slave_widget[self.act_slave]
            self.rc_tab_but[self.act_slave].destroy()
            del self.rc_tab_but[self.act_slave]
            del self.rc_tab_but_label[self.act_slave]
            if self.act_slave >= len(self.rc_slave_widget):
                self.act_slave = len(self.rc_slave_widget) - 1
            if self.act_slave < 0:
                self.act_slave = -1
            self.update_layout()
        return

    def _cmd_change_side(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** chane side of tab menu')
        return

    def _cmd_order_left_up(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** tab menu change order left up')
        return

    def _cmd_order_right_down(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** tab menu change order right down')
        return

    def _cmd_delete_tab(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** delete tab menu')
        self.tk_master.rc_slave_widget = None
        self.rc_event = None
        self.tk_master.config(relief=tk.FLAT, bd=rc_util.rc_border)
        self.destroy()
        return
