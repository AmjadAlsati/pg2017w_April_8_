################
#
# Modul:        PaderKicker Monitor
# File:         pk_monitor_main_window.py
#
# Author:       Bernd Kleinjohann
# Created:      June 2011
# Version:      2.0
# Contents:     this class contains all common data for the monitor thread
#
import Tkinter as tk
import tkSimpleDialog

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_init_node import RCInitTkGui
from .rc_frame import RCFrame


class RCSimpleGrid(tk.Frame):

    def __init__(self, master=None, init_data=None,
                 width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT,
                 orient=tk.HORIZONTAL, **kwd):

        self.tk_master = master
        self.init_data = init_data

        self.height = height
        self.width = width
        self.default_relief = relief
        self.orient = orient
        self.rc_event = None

        self.rc_slave_grid_lst = []
        self.col_row_cnt = 0

##        self.rc_slave_widget_lst = []
##        self.slave_para_lst = []

        self.create_widgets()
        return

    def create_widgets(self):
        tk.Frame.__init__(self, self.tk_master, width=self.width,
                          height=self.height, relief=self.default_relief)
        self.act_color = self['bg']
        if self. orient == tk.HORIZONTAL:
            self.rowconfigure(0, weight=1)
            self.rowconfigure(1, weight=1)
            self.rowconfigure(2, weight=1)
        else:
            self.columnconfigure(0, weight=1)
            self.columnconfigure(1, weight=1)
            self.columnconfigure(2, weight=1)

        # install pop up menu in frame self

        self.popup.add_command(label='Frame', command=self._cmd_frame)
        self.popup.add_command(label='Label', command=self._cmd_label)
        self.popup.add_command(label='Fill', command=self._cmd_fill)

        for widget_cmd in RCInitTkGui.rc_io_widgets:
            # define cmd handler
            def cmd_handler(self=self, widget_cmd=widget_cmd):
                # if SMOBaseObject.info: SMOBaseObject.debug_handler.out('*** INFO *** simple io cmd_frame')
                if pos < 0:
                    pos = 0
                if pos > len(self.rc_slave_widget):
                    pos = len(self.rc_slave_widget)

                ### pos bestimmen ###

                new_slave = RCInitTkGui.rc_io_widgets[widget_cmd]
                self.rc_slave_widget = new_slave[0](
                    master=self, init_data=self.init_data, orient=self.orient, interactive=True, **new_slave[1])

                col_row_size = new_slave[1]['col_row_cnt']
                self.col_row_cnt += col_row_size

                self.update_layout()
                self.rc_event = None
                return
            self.popup.add_command(label=widget_cmd, command=cmd_handler)

##        self.popup = Menu(self, relief=FLAT, tearoff=0)
##        self.popup.add_command(label='I Scale', command = self._cmd_i_slider)
##        self.popup.add_command(label='O Scale', command = self._cmd_o_slider)
##        self.popup.add_command(label='IO Scale', command = self._cmd_io_slider)
##
##        self.popup.add_command(label='I Entry', command = self._cmd_i_entry)
##        self.popup.add_command(label='O Entry', command = self._cmd_o_entry)
##        self.popup.add_command(label='IO Entry', command = self._cmd_io_entry)

        self.popup.add_command(label='Cange Orient', command=self._cmd_change_orient)
        self.popup.add_command(label='Delete', command=self._cmd_delete)
        self.bind('<Button-3>', self._cmd_popup)

        # change bg color iff curser in frame
        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

    def get_installed_widgets(self):

        if self.rc_slave_config is None or self.rc_slave_config == []:
            widget_structure = ['Grid H', {'orient': self.orient, 'col_row_cnt': self.col_row_cnt}, None]
        else:
            slave_lst = []
            for wi in self.rc_slave_grid_lst:
                if wi[0] == 'Label':
                    slave_lst.append(['Label', {'text': wi[2]}, None])
                elif wi[0] == 'Fill':
                    slave_lst.append(['Fill', {}, None])
                elif wi[0] == 'Frame':
                    slave_lst.append(['Frame', {}, {'widget_structure': self.rc_slave_widget.get_installed_widgets()}])
                else:
                    slave_lst.append(wi[1].get_installed_widgets())
            widget_structure = ['Grid H', {'orient': self.orient, , 'col_row_cnt': self.col_row_cnt}, {'widget_structure': slave_lst}]
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        if widget_structure is None:
            return

        rc_cnt = 0
        self.rc_slave_grid_lst = []
        for wi in widget_structure:
            if wi[0] == 'Label':
                #slave_lst.append( ['Label', {'Label_text': wi[2]}, None] )
                new_widget = tk.Label(self, relief=tk.FLAT, **wi[1])
                self.rc_slave_grid_lst.append(('Label', new_widget, wi[1]['text']))
                self.col_row_cnt += 1
            elif wi[0] == 'Fill':
                self.rc_slave_grid_lst.append(('Fill', None))
                self.col_row_cnt += 1
            elif wi[0] == 'Frame':
                new_widget = RCFrame(master=self, init_data=self.init_data,
                                     width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)
                self.rc_slave_grid_lst.append(('Frame', new_widget))
                rc_cnt += 3
                if wi[2] is not None:
                    new_widget.install_widgets(parent=self, **x[2])
            else:
                widget_cmd = RCInitTkGui.rc_io_widgets[wi[0]]
                new_widget = widget_cmd(master=self, init_data=self.init_data, **wi[1])
                rc_cnt +=
####
####
####
                if wi[2] is not None:
                    new_widget.install_widgets(parent=self, **x[2])
        self.update_layout()
        return

    def update_layout(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** simple io update layout: %d, %d' % (self.col_row_cnt, len(self.rc_slave_widget)))
        if self.orient == tk.HORIZONTAL:
            self.columnconfigure(0, weight=0)
            self.columnconfigure(1, weight=1)
            self.columnconfigure(2, weight=1)

            for i in range(0, self.col_row_cnt + 1):
                # print 'row ', i
                self.rowconfigure(i, weight=0)
        else:
            self.rowconfigure(0, weight=0)
            self.rowconfigure(1, weight=1)
            self.rowconfigure(2, weight=1)

            for i in range(0, self.col_row_cnt):
                # print 'column ', i
                self.columnconfigure(i, weight=0)

        rc_cnt = 0
        for widget_act in self.rc_slave_grid_lst:
            if widget_act[0] == 'Frame':
                if self.orient == tk.HORIZONTAL:
                    self.rowconfigure(i, weight=1)
                    widget_act[1].grid(row=rc_cnt, rowspan=3, column=0, columnspan=3, sticky=tk.N + tk.E + tk.S + tk.W)
                else:
                    self.columnconfigure(i, weight=1)
                    widget_act[1].grid(column=rc_cnt, columnspan=3, row=0, rowspan=3, sticky=tk.N + tk.E + tk.S + tk.W)
                rc_cnt += 3
            elif widget_act[0] == 'Fill':
                if self.orient == tk.HORIZONTAL:
                    self.rowconfigure(rc_cnt, weight=1)
                else:
                    self.columnconfigure(rc_cnt, weight=1)
                rc_cnt += 1
            elif widget_act[0] == 'Label':
                if self.orient == tk.HORIZONTAL:
                    self.rowconfigure(rc_cnt, weight=1)
                    widget_act[1].grid(row=rc_cnt, column=0, columnspan=3, sticky=tk.S + tk.E)
                else:
                    self.columnconfigure(rc_cnt, weight=1)
                    widget_act[1].grid(row=0, column=rc_cnt, rowspan=3, sticky=tk.N + tk.E)
                rc_cnt += 1
            else:
                if self.orient == tk.HORIZONTAL:
                    col_cnt, row_cnt = widget_act[1].grid_widgets(
                        self, row=c_cnt, column=0, columspan=3, rowspan=2, orient=self.orient)
                    c_cnt += row_cnt
                else:
                    col_cnt, row_cnt = widget_act[1].grid_widgets(
                        self, row=0, column=col_cnt, columspan=2, rowspan=3, orient=self.orient)
                    c_cnt += col_cnt
            if self.orient == tk.HORIZONTAL:
                self.col_row_cnt = rc_cnt
            else:
                self.col_row_cnt = rc_cnt
        return

    def _cmd_popup(self, event):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** simple io popup at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
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
                '*** INFO *** simple io set highlight at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
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
                '*** INFO *** simple io reset hightlight at %d,%d,%d,%d' %
                (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_set_highlight()
        self.rc_reset_highlight()

    def rc_reset_highlight(self):
        self.config(bg=self.act_color)
        return

    def _cmd_frame(self, pos=-1):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** simple io cmd_frame')
        new_widget = RCFrame(master=self, init_data=self.init_data,
                             width=rc_util.rc_min_size, height=rc_util.rc_min_size, relief=tk.FLAT)

        if pos < 0:
            pos = 0
        if pos >= len(self.rc_slave_grid_lst):
            self.rc_slave_grid_lst.append(('Frame', new_widget))
        else:
            self.rc_slave_grid_lst.insert(pos, ('Frame', new_widget))

        self.col_row_cnt += 3
        self.update_layout()
        return

    def _cmd_label(self, pos=-1):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** simple io cmd_label')
        # to do:  get pos by selektion
        label_string = 'Label: '
        label_string = tkSimpleDialog.askstring(self.tk_master, prompt='Label string?',
                                                initialvalue=label_string)
        new_widget = tk.Label(self, text=label_string, relief=tk.FLAT)

        if pos < 0:
            pos = 0
        if pos >= len(self.rc_slave_grid_lst):
            self.rc_slave_grid_lst.append(('Label', new_widget))
        else:
            self.rc_slave_grid_lst.insert(pos, ('Label', new_widget, label_string))

        self.col_row_cnt += 1
        self.update_layout()
        return

    def _cmd_fill(self, pos=-1):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO ***  simple io cmd_fill')
        # to do:  get pos by selektion
        if pos < 0:
            pos = 0
        if pos >= len(self.rc_slave_grid_lst):
            self.rc_slave_grid_lst.append(('Fill', None))
        else:
            self.rc_slave_grid_lst.insert(pos, ('Fill', None))

        self.col_row_cnt += 1
        self.update_layout()
        return

    def _cmd_slider(self, io_mode='IN', pos=-1):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO ***  simple io cmd_i_slider')
        new_widget = RCTKSlider(master=self, init_data=self.init_data,
                                io_mode=io_mode, orient=self.orient, interactive=True, **kwd)

        if pos < 0:
            pos = 0
        if pos >= len(self.rc_slave_grid_lst):
            self.rc_slave_grid_lst.append(('I Scale', new_widget))
        else:
            self.rc_slave_grid_lst.insert(pos, ('I Scale', new_widget))

        self.col_row_cnt += 1
        if io_mode = 'INOUT':
            self.col_row_cnt += 1
        self.update_layout()
        return

    def _cmd_i_slider(self, pos=-1):
        return self._cmd_slider(io_mode='IN', pos=pos)

    def _cmd_o_slider(self, pos=-1):
        return self._cmd_slider(io_mode='OUT', pos=pos)

    def _cmd_io_slider(self, pos=-1):
        return self._cmd_slider(io_mode='INOUT', pos=pos)

    def _cmd_entry(self, io_mode='i_entry', pos=-1):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** simple io cmd_entry')
        new_widget = RCTKEntry(master=self, init_data=self.init_data,
                               io_mode=io_mode, orient=self.orient, interactive=True, **kwd)

        if pos < 0:
            pos = 0
        if pos >= len(self.rc_slave_grid_lst):
            self.rc_slave_grid_lst.append(('I Entry', new_widget))
        else:
            self.rc_slave_grid_lst.insert(pos, ('I Entry', new_widget))

        self.col_row_cnt += 1
        self.update_layout()
        return

    def _cmd_i_entry(self, io_mode='i_entry', pos=-1):
        return self._cmd_entry(io_mode=io_mode)

    def _cmd_o_entry(self, io_mode='o_entry', pos=-1):
        return self._cmd_entry(io_mode=io_mode)

    def _cmd_io_entry(self, io_mode='io_entry', pos=-1):
        return self._cmd_entry(io_mode=io_mode)

    def _cmd_change_orient(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** menu cmd_change_orient')
        if self.orient == tk.VERTICAL:
            self.orient = tk.HORIZONTAL
        else:
            self.orient = tk.VERTICAL
        self.update_layout()
        return

    def _cmd_remove(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** menu cmd_change_orient')
        return

    def _cmd_config(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** menu cmd_change_orient')
        return

    def _cmd_delete(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out('*** INFO *** delete cmd_delete_slider menu')
        self.tk_master.rc_slave_widget = None
        self.rc_event = None
        self.tk_master.config(relief=tk.FLAT, bd=rc_util.rc_border)
        self.destroy()
        return

    def mir_val(self, val):
        print 'Slider ', str(val)
        self.out_scale.set(100 - self.in_scale.get())
        return
