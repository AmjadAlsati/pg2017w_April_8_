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

from framework.smart_object import SMOBaseObject
from . import rc_util
from .rc_base_io import RCBaseIO

rc_axis_border = 4
rc_marker_border = 3
rc_marker_size = 8


class RCAxisFrame(RCBaseIO):

    def __init__(self, master=None, init_data=None, widget_in_tab=None, widget_out_tab=None,
                 height=rc_util.rc_min_size, width=rc_util.rc_min_size, name='Axis: ', relief=tk.FLAT, **kwd):

        RCBaseIO.__init__(self, master=master, init_data=init_data, widget_in_tab=widget_in_tab,
                          widget_out_tab=widget_out_tab, height=height, width=width, name=name,
                          relief=tk.FLAT, **kwd)
        self.img_token = 1

        self.get_init_tab = {'get_x': (self.get_x, None),
                             'get_y': (self.get_y, None),
                             'get_pos': (self.get_pos, None)}
        self.set_init_tab = {'set_x': (self.set_x, 0.0),
                             'set_y': (self.set_y, 0.0),
                             'set_pos': (self.set_pos, (0.0, 0.0))}

        self.x_in = 0.0
        self.y_in = 0.0
        self.x_out = 0.0
        self.y_out = 0.0

        self.create_widgets()

        return

    def create_widgets(self):
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        self.width = rc_util.rc_default_image_width
        self.height = rc_util.rc_default_image_height

        self.color = 'red'

        self.rect_size = min(self.width, self.height) - 2 * rc_marker_size
        self.rect_center = (self.width / 2, self.height / 2)

        self.axis_canv = tk.Canvas(self,  # height=self.height, width=self.width,
                                   bg=rc_util.rc_bg_color, relief=tk.FLAT, bd=0, selectborderwidth=0)
        self.axis_canv.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)

        # Flag for stuck input value
        self.stuck = False

        # define canvas tags
        self.center_tag = 'ax_center'  # need a unique name
        self.in_tag = 'ax_in'  # + name
        self.out_tag = 'ax_out'  # + name
        self.all_tag = 'ax_all'  # + name

        # basic grafic boarder and center rect
        self.ax_boarder = self.axis_canv.create_rectangle(self.rect_center[0] - self.rect_size / 2,
                                                          self.rect_center[1] - self.rect_size / 2,
                                                          self.rect_center[0] + self.rect_size / 2,
                                                          self.rect_center[1] + self.rect_size / 2,
                                                          outline=None, fill=rc_util.rc_input_color, width=0,
                                                          tags=(self.center_tag, self.all_tag))
        self.ax_center = self.axis_canv.create_rectangle(self.rect_center[0] - rc_marker_size,
                                                         self.rect_center[1] - rc_marker_size,
                                                         self.rect_center[0] + rc_marker_size,
                                                         self.rect_center[1] + rc_marker_size,
                                                         outline=rc_util.rc_bg_frame_color, width=2,
                                                         tags=(self.center_tag, self.all_tag))
        ### rc_dark_input_color = rc_d_green
        ### rc_dark_output_color = rc_d_blue

        # output marker circle
        self.ax_out = self.axis_canv.create_oval(self.rect_center[0] - rc_marker_size + rc_marker_border,
                                                 self.rect_center[1] - rc_marker_size + rc_marker_border,
                                                 self.rect_center[0] + rc_marker_size - rc_marker_border,
                                                 self.rect_center[1] + rc_marker_size - rc_marker_border,
                                                 outline=rc_util.rc_dark_output_color, width=4, tags=(self.out_tag, self.all_tag))
        # input marker cross
        self.ax_in_lhor = self.axis_canv.create_line(self.rect_center[0] - rc_marker_size - rc_marker_border,
                                                     self.rect_center[1],
                                                     self.rect_center[0] + rc_marker_size + rc_marker_border,
                                                     self.rect_center[1],
                                                     width=4, fill=rc_util.rc_dark_input_color, tags=(self.in_tag, self.all_tag))
        self.ax_in_lvert = self.axis_canv.create_line(self.rect_center[0],
                                                      self.rect_center[1] - rc_marker_size - rc_marker_border,
                                                      self.rect_center[0],
                                                      self.rect_center[1] + rc_marker_size + rc_marker_border,
                                                      width=4, fill=rc_util.rc_dark_input_color, tags=(self.in_tag, self.all_tag))
        # boarder
        self.ax_frame = self.axis_canv.create_rectangle(self.rect_center[0] - self.rect_size / 2 - (rc_marker_size + rc_marker_border) / 2,
                                                        self.rect_center[1] - self.rect_size / 2 -
                                                        (rc_marker_size + rc_marker_border) / 2,
                                                        self.rect_center[0] + self.rect_size / 2 +
                                                        (rc_marker_size + rc_marker_border) / 2,
                                                        self.rect_center[1] + self.rect_size / 2 +
                                                        (rc_marker_size + rc_marker_border) / 2,
                                                        outline=rc_util.rc_bg_frame_color, width=rc_marker_size + rc_marker_border,
                                                        tags=(self.center_tag, self.all_tag))

        self.axis_canv.bind('<Configure>', self.cmd_configure)
        self.axis_canv.bind('<Button-1>', self.cmd_start_move)
        self.axis_canv.bind('<ButtonRelease-1>', self.cmd_stop_move)

        # ????? ERROR in Tk gridder ?????
        # self.axis_canv.bind('<Enter>',self._cmd_set_leaf_highlight)
        # self.axis_canv.bind('<Leave>',self._cmd_reset_leaf_highlight)

        # dummy
        self.popup.add_command(label='Config Widget', command=self._cmd_config)
        return

    def get_installed_widgets(self):
        widget_structure = ['Axis', {'widget_in_tab': self.widget_in_tab,
                                     'widget_out_tab': self.widget_out_tab}, None]
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        # nothing to do
        return

    def _cmd_set_leaf_highlight(self, event):
        print 'leaf X set rc image'
        self.config(bg=rc_util.rc_bg_color)
        self.axis_canv.config(bg=rc_util.rc_bg_color)
        # self.axis_canv.config(bg=rc_m_red)
        return

    def _cmd_reset_leaf_highlight(self, event):
        print 'leaf X reset rc image'
        self.config(bg=rc_util.rc_hl_color)
        self.axis_canv.config(bg=rc_util.rc_bg_color)
        # self.axis_canv.config(bg=rc_m_blue)
        return

    # dummy
    def _cmd_config(self):
        # config axis e. g. joystick colors etc
        return

    # resize event
    def cmd_configure(self, event):
        self.width = event.width  # - 2*rc_marker_size
        self.height = event.height  # - 2*rc_marker_size

        self.rect_size = min(self.width, self.height) - 2 * rc_axis_border - rc_marker_size - rc_marker_border
        self.rect_center = (self.width / 2, self.height / 2)

        # basic grafic boarder and center rect
        self.axis_canv.coords(self.ax_boarder,
                              self.rect_center[0] - self.rect_size / 2,
                              self.rect_center[1] - self.rect_size / 2,
                              self.rect_center[0] + self.rect_size / 2,
                              self.rect_center[1] + self.rect_size / 2)
        self.axis_canv.coords(self.ax_center,
                              self.rect_center[0] - rc_marker_size,
                              self.rect_center[1] - rc_marker_size,
                              self.rect_center[0] + rc_marker_size,
                              self.rect_center[1] + rc_marker_size)

        ### set in marker
        self.set_act_in_pos()

        # set out marker
        self.set_act_out_pos()

        self.axis_canv.coords(self.ax_frame,
                              self.rect_center[0] - self.rect_size / 2 - (rc_marker_size + rc_marker_border) / 2,
                              self.rect_center[1] - self.rect_size / 2 - (rc_marker_size + rc_marker_border) / 2,
                              self.rect_center[0] + self.rect_size / 2 + (rc_marker_size + rc_marker_border) / 2,
                              self.rect_center[1] + self.rect_size / 2 + (rc_marker_size + rc_marker_border) / 2)

        return

    def set_act_out_pos(self):
        # output marker circle
        act_x_out_pos = self.rect_center[0] + int(self.x_out * float(self.rect_size / 2))
        act_y_out_pos = self.rect_center[1] + int(self.y_out * float(self.rect_size / 2))
        self.axis_canv.coords(self.ax_out,
                              act_x_out_pos - rc_marker_size + rc_marker_border,
                              act_y_out_pos - rc_marker_size + rc_marker_border,
                              act_x_out_pos + rc_marker_size - rc_marker_border,
                              act_y_out_pos + rc_marker_size - rc_marker_border)
        return

    def set_act_in_pos(self):
        # input marker cross
        act_x_in_pos = self.rect_center[0] + int(self.x_in * float(self.rect_size / 2))
        act_y_in_pos = self.rect_center[1] + int(self.y_in * float(self.rect_size / 2))
        self.axis_canv.coords(self.ax_in_lhor,
                              act_x_in_pos - rc_marker_size - rc_marker_border,
                              act_y_in_pos,
                              act_x_in_pos + rc_marker_size + rc_marker_border,
                              act_y_in_pos)
        self.axis_canv.coords(self.ax_in_lvert,
                              act_x_in_pos,
                              act_y_in_pos - rc_marker_size - rc_marker_border,
                              act_x_in_pos,
                              act_y_in_pos + rc_marker_size + rc_marker_border)
        return

    def get_act_in_pos(self, x, y):
        x_val = float(2 * (x - self.rect_center[0])) / float(self.rect_size)
        x_val_abs = abs(x_val)
        y_val = float(2 * (y - self.rect_center[1])) / float(self.rect_size)
        y_val_abs = abs(y_val)

        if y_val_abs < x_val > 1:
            y_val = y_val / x_val
            x_val = 1
        elif y_val_abs < -x_val > 1:
            y_val = - y_val / x_val
            x_val = -1
        elif x_val_abs < -y_val > 1:
            x_val = - x_val / y_val
            y_val = -1
        elif x_val_abs < y_val > 1:
            x_val = x_val / y_val
            y_val = 1
        else:
            pass

        self.x_in = x_val
        self.y_in = y_val
        if SMOBaseObject.info:
            SMOBaseObject.debug_handler.out(
                '*** INFO *** RCAxisFrame.get_act_in_pos ', (x_val, y_val))
        return

    def cmd_start_move(self, event):
        self.stuck = False
        self.axis_canv.bind('<B1-Motion>', self.cmd_move)
        self.axis_canv.bind('<Button-3>', self.cmd_stuck_acquire)
        return

    def cmd_stop_move(self, event):
        if not self.stuck:
            self.axis_canv.unbind('<B1-Motion>')
            self.axis_canv.unbind('<Button-3>')
            self.x_in = 0.0
            self.y_in = 0.0
            self.set_act_in_pos()
        return

    def cmd_move(self, event):

        if not self.stuck:
            self.get_act_in_pos(event.x, event.y)
            self.set_act_in_pos()

        return

    def cmd_stuck_acquire(self, event):
        self.stuck = True
        self.axis_canv.unbind('<B1-Motion>')
        self.axis_canv.unbind('<Button-3>')
        return

    def get_x(self):
        ret = self.x_in
        return ret

    def get_y(self):
        ret = self.y_in
        return ret

    def get_pos(self):
        ret = (self.x_in, self.y_in)
        return ret

    def set_x(self, x_out):
        self.x_out = float(x_out)
        if self.x_out > 1.0:
            self.x_out = 1.0
        self.set_act_out_pos()
        return

    def set_y(self, y_out):
        self.y_out = float(y_out)
        if self.y_out > 1.0:
            self.y_out = 1.0
        self.set_act_out_pos()
        return

    def set_pos(self, pos):
        self.x_out = float(pos[0])
        self.y_out = float(pos[1])
        if self.x_out > 1.0:
            self.x_out = 1.0
        if self.y_out > 1.0:
            self.y_out = 1.0
        self.set_act_out_pos()
        return
