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
from PIL import Image, ImageTk

from cStringIO import StringIO

from . import rc_util
from .rc_base_io import RCBaseIO


class RCImageFrame(RCBaseIO):

    def __init__(self, master=None, init_data=None, widget_in_tab=None, widget_out_tab=None,
                 height=rc_util.rc_min_size, width=rc_util.rc_min_size, name='Image: ', relief=tk.FLAT, **kwd):

        RCBaseIO.__init__(self, master=master, init_data=init_data, widget_in_tab=widget_in_tab,
                          widget_out_tab=widget_out_tab, height=height, width=width, name=name,
                          relief=tk.FLAT, **kwd)
        self.img_token = 1

        self.get_init_tab = {'get_image_token': (self.get_image_token, None)}
        self.set_init_tab = {'set_image': (self.set_image, None),
                             'set_jpg_image': (self.set_jpg_image, None),
                             'set_image_data': (self.set_image_data, [0, 0, None]),
                             'set_image_token': (self.set_image_token, 1)}
        self.create_widgets()

        # initial image
        init_img_size = (3, 4)
        init_img_data = [(0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255),
                         (255, 255, 0), (255, 0, 255), (0, 255, 255), (255, 255, 255),
                         (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255)]
        init_img = Image.new("RGB", init_img_size)
        init_img.putdata(init_img_data)
        init_img = init_img.resize((240, 360))
        self.set_image(init_img)
        return

    def create_widgets(self):
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

# canvas setup erzeugt weisse linie ???
        self.img_canv = tk.Canvas(self,  # height=rc_default_image_height, width=rc_default_image_width,
                                  bg=rc_util.rc_bg_color, relief=tk.FLAT, bd=0, selectborderwidth=0)

        self.img_canv.grid(row=0, column=0, sticky=tk.N + tk.S + tk.E + tk.W)

        self.img_obj = self.img_canv.create_image(rc_util.rc_default_image_width / 2, rc_util.rc_default_image_height / 2,
                                                  anchor=tk.CENTER)
        self.img = None
        self.img_width = None
        self.img_height = None
        self.display_img = None

        self.img_canv.bind('<Configure>', self._cmd_img_configure)

        # Achting fehler im Gridder ????
        # self.img_canv.bind('<Enter>',self._cmd_set_leaf_highlight)
        # self.img_canv.bind('<Leave>',self._cmd_reset_leaf_highlight)

        self.popup.add_command(label='Config Widget', command=self._cmd_config)
        return

    def get_installed_widgets(self):
        widget_structure = ['Image', {'widget_in_tab': self.widget_in_tab,
                                      'widget_out_tab': self.widget_out_tab}, None]
        return widget_structure

    def install_widgets(self, widget_structure=None, **kwd):
        # set video stream ???
        return

    def _cmd_set_leaf_highlight(self, event):
        print 'leaf X set rc image'
        self.config(bg=rc_util.rc_bg_color)
        self.img_canv.config(bg=rc_util.rc_bg_color)
        # self.img_canv.config(bg=rc_m_red)
        return

    def _cmd_reset_leaf_highlight(self, event):
        print 'leaf X reset rc image'
        self.config(bg=rc_util.rc_hl_color)
        self.img_canv.config(bg=rc_util.rc_bg_color)
        # self.img_canv.config(bg=rc_m_blue)
        return

    def _cmd_img_configure(self, event):
        self.img_width = event.width
        self.img_height = event.height
        # print 'Image reconf img_size (canvas size) ', self.img_width, self.img_height
        self._set_image()
        return

    def get_image_token(self):
        self.img_token -= 1
        return 1

    def set_image_token(self, val):
        if val is not None:
            self.img_token += val
        return

    def get_image(self):
        return self.img

    def set_jpg_image(self, val):
        if val is not None:
            img_file = StringIO(val)
            img = Image.open(img_file)
            self.set_image(img)
        return

    def set_image_data(self, val):  # ascii img
        if val is not None:
            width, heigt, img_data = val
            if img_data is None:
                return
            # convert img data
            img = Image.new("RGB", (width, heigt))
            img.putdata(img_data)
            self.set_image(img)
        return

    def set_image(self, img):  # pil img
        self.img = img

        # hole Fenster groesse
        if img is not None:
            if self.img_width is None:
                self.img_width = rc_util.rc_default_image_width / 2
            else:
                self.img_width = self.img_canv.winfo_width()
            if self.img_height is None:
                self.img_height = rc_util.rc_default_image_height / 2
            else:
                self.img_height = self.img_canv.winfo_height()
            self._set_image()
        return

    def _set_image(self):
        # hole bild groesse
        img_width = self.img.size[0]
        img_height = self.img.size[1]

        # x/y verhaeltnis des bildes soll beibehalten werden
        if float(self.img_width) / self.img_height > float(img_width) / img_height:
            height = self.img_height - 2 * rc_util.rc_image_border  # 2* Rand
            if height < rc_util.rc_image_min_size:
                height = rc_util.rc_image_min_size
            width = int(round(float(img_width) / img_height * height))
            if width < rc_util.rc_image_min_size:
                width = rc_util.rc_image_min_size
        else:
            width = self.img_width - 2 * rc_util.rc_image_border  # 2* Rand
            if width < rc_util.rc_image_min_size:
                width = rc_util.rc_image_min_size
            height = int(round(float(img_height) / img_width * width))
            if height < rc_util.rc_image_min_size:
                height = rc_util.rc_image_min_size

        # display image on screen
        # print 'Image reconf size: ', width, height
        rezise_img = self.img.resize((width, height))
        self.display_img = ImageTk.PhotoImage(rezise_img)

# config canvas size
        self.img_canv.coords(self.img_obj, self.img_width / 2, self.img_height / 2)
        self.img_canv.itemconfigure(self.img_obj, image=self.display_img)
        return

    def _cmd_config(self):
        return
