################
#
# Modul:        PaderKicker Monitor
# File:         pk_monitor_pane.py
#
# Author:       Bernd Kleinjohann
# Created:      June 2011
# Version:      2.0
# Contents:     this class contains all common data for themonitor thread
#
import Tkinter as tk

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_init_node import RCInitTkGui
from .rc_frame import RCFrame


class RCPaned(tk.PanedWindow):

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
#        self.rc_slave_widget = None
        self.rc_slave_widget = []
#        self.rc_panes = []

        self.create_widgets()
        return

    def create_widgets(self):
        tk.PanedWindow.__init__(self, self.tk_master, orient=self.orient,
                                sashpad=rc_util.rc_sashpad, relief=self.default_relief)

        self.act_color = self['bg']

#        self.rowconfigure(0,weight=1, minsize=pkm_min_size )
#        self.columnconfigure(0,weight=1, minsize=pkm_min_size )

        # install pop up menu in frame self
        self.popup = tk.Menu(self, relief=tk.RAISED, tearoff=0)
        self.popup.config(relief=tk.FLAT, bd=0, activebackground=rc_util.rc_button_color,
                          activeforeground=rc_util.rc_black)
        self.popup.add_command(label='Add Pane', command=self._cmd_create_pane)
        self.popup.add_command(label='Remove Pane', command=self._cmd_remove_pane)
        self.popup.add_command(label='Delete Pane', command=self._cmd_delete_pane)
        self.popup.add_command(label='Change Orient', command=self._cmd_change_orient)
        self.bind('<Button-3>', self._cmd_popup)

        # change bg color iff curser in frame
        self.bind('<Enter>', self._cmd_set_highlight)
        self.bind('<Leave>', self._cmd_reset_highlight)
        return

    def get_installed_widgets(self):
        if self.rc_slave_widget is None or self.rc_slave_widget == []:
            widget_structure = ['H Pane', {'orient': self.orient}, None]
        else:
            slave_lst = []
            for x in self.rc_slave_widget:
                slave_lst.append(x.get_installed_widgets())
            widget_structure = ['H Pane', {'orient': self.orient}, {'widget_structure': slave_lst}]
            # print 'XXX len : ', len(slave_lst)
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        if widget_structure is None:
            return
        self.rc_event = None
        for x in widget_structure:
            self.rc_slave_widget_cmd = x[0]
            widget_cmd = RCInitTkGui.rc_menu_widgets[self.rc_slave_widget_cmd][0]
            new_widget = widget_cmd(master=self, init_data=self.init_data, **x[1])
            self.add(new_widget, padx=rc_util.rc_pad, pady=rc_util.rc_pad, minsize=rc_util.rc_min_size)
            self.config(relief=tk.FLAT, bd=0)
            self.rc_slave_widget.append(new_widget)

            if x[2] is not None:
                new_widget.install_widgets(parent=self, **x[2])
        return

# def update_pane_lst(self):
##        self.rc_slave_widget = []
##        lst = self.panes()
# for x in lst:
##            self.pkm_slave_widget.append((x, self.panecget(x, 'height'), self.panecget(x, 'width')))
# return


# def install_widgets(self, master, pkm_root):
##        self.tk_master = master
# self.create_widgets()
# if self.rc_slave_widget != None:
# for pa in self.rc_slave_widget:
# pa[0].install_widgets(self,self.pk_root)
# self.add(new_widget, padx=pkm_frame_pad, pady=pkm_frame_pad,
# minsize=rc_min_size, height=pa[1], width=pa[2])
##            self.update_layout(relief=FLAT, bd=0)
# return
##
# def update_layout(self):
# self.update_pane_lst()
# if self.rc_slave_widget != None and self.rc_slave_widget != []:
##            self.config(relief=FLAT, bd=0)
# else:
##            self['relief'] = FLAT
# return

    def connect_widgets(self, id_n, get_src, set_src, **kwd):
        for wdg in self.rc_slave_widget:
            id_n, get_src, set_src = wdg.connect_widgets(id_n, get_src, set_src, **kwd)
        return id_n, get_src, set_src

    def _cmd_popup(self, event):
        if SMOBaseObject.info:
            SMOBaseObject.out(
                '*** INFO *** pane popup at %d,%d,%d,%d' % (event.x, event.y, event.x_root, event.y_root))
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
            SMOBaseObject.out(
                '*** INFO *** pane frame set highlight at %d,%d,%d,%d' %
                (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_reset_highlight()
        self.rc_set_highlight()
        return

    def rc_set_highlight(self):
        self.config(bg=rc_util.rc_hl_color)
        return

    def _cmd_reset_highlight(self, event):
        self.rc_event = event
        if SMOBaseObject.info:
            SMOBaseObject.out(
                '*** INFO *** pane frame reset hightlight at %d,%d,%d,%d' %
                (event.x, event.y, event.x_root, event.y_root))
        self.tk_master.rc_set_highlight()
        self.rc_reset_highlight()

    def rc_reset_highlight(self):
        self.config(bg=self.act_color)
        return

    def _cmd_change_orient(self):
        if self.orient == tk.VERTICAL:
            self.orient = tk.HORIZONTAL
            self.config(orient=tk.HORIZONTAL)
        else:
            self.orient = tk.VERTICAL
            self.config(orient=tk.VERTICAL)
        return

    def _cmd_create_pane(self):
        new_widget = RCFrame(master=self, init_data=self.init_data, width=rc_util.rc_min_size,
                             height=rc_util.rc_min_size, relief=tk.FLAT)
        pos = self.identify(self.rc_event.x, self.rc_event.y)
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** menu create pane at: %s' % str(pos))
        widget_lst = []
        widget_lst = self.panes()
        if pos != "" and pos is not None:
            new_pane_pos = pos[0]
            if SMOBaseObject.error:
                SMOBaseObject.debug_handler.out(
                    '*** INFO *** create pane pos %d, at %d, %d', (pos[0], self.rc_event.x, self.rc_event.y))
            self.add(new_widget, after=widget_lst[new_pane_pos],
                     padx=rc_util.rc_pad, pady=rc_util.rc_pad, minsize=rc_util.rc_min_size)
            self.rc_slave_widget.insert(new_pane_pos, new_widget)  # -1 ???

        elif self.rc_event.x < rc_util.rc_pad and self.orient == tk.HORIZONTAL or\
                self.rc_event.y < rc_util.rc_pad and self.orient == tk.VERTICAL:
            if SMOBaseObject.error:
                SMOBaseObject.debug_handler.out(
                    '*** INFO *** create first pane at %d, %d', (self.rc_event.x, self.rc_event.y))
            self.add(new_widget, before=widget_lst[0],
                     padx=rc_util.rc_pad, pady=rc_util.rc_pad, minsize=rc_util.rc_min_size)  # padx=pkm_frame_pad
            self.rc_slave_widget.insert(0, new_widget)  # -1 ???
        else:
            if SMOBaseObject.error:
                SMOBaseObject.debug_handler.out(
                    '*** INFO *** create last pane at %d, %d', (self.rc_event.x, self.rc_event.y))
            self.add(new_widget, padx=rc_util.rc_pad, pady=rc_util.rc_pad, minsize=rc_util.rc_min_size)
            self.rc_slave_widget.append(new_widget)
        self.config(relief=tk.FLAT, bd=0)
        self.rc_event = None
        print 'YYY CREATE len : ', len(self.rc_slave_widget)
        return

    def _cmd_remove_pane(self):
        pos = self.identify(self.rc_event.x, self.rc_event.y)
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** delete pane menu %s' % str(pos))
        widget_lst = []
        widget_lst = self.panes()
        if pos != "" and pos is not None:
            del_pane_pos = pos[0]
        elif self.rc_event.x < rc_util.rc_pad and self.orient == tk.HORIZONTAL or\
                self.rc_event.y < rc_util.rc_pad and self.orient == tk.VERTICAL:
            del_pane_pos = 0
        else:
            del_pane_pos = len(widget_lst) - 1
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** delete pane pos: %d, at %d, %d', (del_pane_pos, self.rc_event.x, self.rc_event.y))
        if del_pane_pos < len(widget_lst) and del_pane_pos >= 0:
            self.forget(widget_lst[del_pane_pos])
            self.rc_slave_widget.destroy(self.rc_slave_widget[del_pane_pos])
            del self.rc_slave_widget[del_pane_pos]

        widget_lst = self.panes()
        if widget_lst is None or len(widget_lst) <= 0:
            self.config(relief=tk.FLAT, bd=rc_bd)

        self.rc_event = None
        print 'YYY DELETE len : ', len(self.rc_slave_widget)
        return

    def _cmd_delete_pane(self):
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** delete pane menu')
        self.tk_master.rc_slave_widget = None
        self.rc_event = None
        self.tk_master.config(relief=tk.FLAT, bd=rc_bd)
        self.destroy()
        return
