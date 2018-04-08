################
#
# Modul:        RC_Control
# File:         __init__.py
#
# Author:       Bernd Kleinjohann
# Created:      October  2016
# Version:      0.1
#
# Contents:

"""
RC-Control contains a gui system for remote robot monitoring and control.
The gui includes a simple gui editor. Based on a set of widgets a
gui can be created. The link to the robots are realized by smo_IP_objects.
"""
import Tkinter as tk

from .rc_init_node import RCInitTkGui
from .rc_axis_frame import RCAxisFrame
from .rc_frame import RCFrame
from .rc_monitor import RCHeader
from .rc_image_frame import RCImageFrame
from .rc_pane import RCPaned
from .rc_send import RCSend
from .rc_simple_grid import RCSimpleGrid
from .rc_tabed import RCTabed
from .rc_tk_entry import RCTKEntry
from .rc_tk_slider import RCTKSlider
from .rc_gamepad_frame import RCGamepadFrame

def init_ui():
    """Initializes global UI state.

    This function initializes global UI state and registers concrete elements.
    It has to be called before using the UI.
    """
    RCInitTkGui.rc_menu_widgets['Axis'] = (RCAxisFrame, {})
    RCInitTkGui.rc_menu_widgets['Frame'] = (RCFrame, {'relief': tk.FLAT})
    RCInitTkGui.rc_menu_widgets['Header'] = (RCHeader, {'relief': tk.FLAT})
    RCInitTkGui.rc_menu_widgets['Image'] = (RCImageFrame, {})
    RCInitTkGui.rc_menu_widgets['H Pane'] = (RCPaned, {'orient': tk.HORIZONTAL})
    RCInitTkGui.rc_menu_widgets['V Pane'] = (RCPaned, {'orient': tk.VERTICAL})
    RCInitTkGui.rc_menu_widgets['Send'] = (RCSend, {})
    RCInitTkGui.rc_menu_widgets['Grid H'] = (RCSimpleGrid, {'orient': tk.HORIZONTAL})
    RCInitTkGui.rc_menu_widgets['Grid V'] = (RCSimpleGrid, {'orient': tk.VERTICAL})
    RCInitTkGui.rc_menu_widgets['Grid H'] = (RCSimpleGrid, {'orient': tk.HORIZONTAL})
    RCInitTkGui.rc_menu_widgets['Grid V'] = (RCSimpleGrid, {'orient': tk.VERTICAL})
    RCInitTkGui.rc_menu_widgets['Tabbed'] = (RCTabed, {})
    RCInitTkGui.rc_menu_widgets['Gamepad']=(RCGamepadFrame,{})
    RCInitTkGui.rc_io_widgets['I Entry'] = (RCTKEntry, {'io_mode': 'IN', 'col_row_cnt': 1})
    RCInitTkGui.rc_io_widgets['O Entry'] = (RCTKEntry, {'io_mode': 'OUT', 'col_row_cnt': 1})
    RCInitTkGui.rc_io_widgets['IO Entry'] = (RCTKEntry, {'io_mode': 'INOUT', 'col_row_cnt': 1})
    RCInitTkGui.rc_io_widgets['I Scale'] = (RCTKSlider, {'io_mode': 'IN', 'col_row_cnt': 1})
    RCInitTkGui.rc_io_widgets['O Scale'] = (RCTKSlider, {'io_mode': 'OUT', 'col_row_cnt': 1})
    RCInitTkGui.rc_io_widgets['IO Scale'] = (RCTKSlider, {'io_mode': 'INOUT', 'col_row_cnt': 2})

