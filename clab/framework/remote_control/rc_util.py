################
#
# Modul:        RC_Control
# File:         rc_control.py
#
# Author:       Bernd Kleinjohann
# Created:      Oct. 2015
# Version:      1.0
#
# Contents:
"""
global setups and configurations for RC_Control
"""

from framework.smart_object import smo_set_debug_options
from framework.smart_node import smo_value as rc_value
from framework.smart_node import smo_token as rc_token
from framework.smart_node import smo_list as rc_list

# define names of environment variables path and model name
env_rc_path_name = 'RC_DATA_PATH'
env_rc_model_name = 'PK_MODEL_NAME'

# define default values
rc_default_model_name = 'rc_model_undef'
rc_default_serve = False
rc_default_server_port = 5555
rc_default_display = False
rc_default_local = True

# define connection related values
#rc_value = 1
#rc_token = 2
#rc_list = 3

# define technical default values in ms
rc_control_sample_rate = 50  # ms
rc_vision_sample_rate = 100  # ms; 4 Frames/s

# initial monitor values for content
rc_window_init_xsize = 560
rc_window_init_ysize = 480

# PKMFrame pad x/y
rc_pad = 4
# borderwidth
rc_border = 4
# borderwidth
rc_button_border = 2
# used for frames, panes
rc_min_size = 40
# paneseparator
rc_sashpad = 1
# grid options
rc_padx = 2
rc_pady = 2
# image fame
rc_default_image_width = 400
rc_default_image_height = 300
rc_image_border = 16
rc_image_min_size = 16
# haircross
rc_hair_cross_size = 10
rc_hair_cross_tics = 16

# for paned windows
# rc_min_col_size = 60 #min pane x
# rc_min_row_size = 40 #min pane y
##
# for tabed windows
##rc_tab_cmd_hight = 20

# some color definitions
rc_l_blue = "#dce6f2"  # rgb 220 230 242
rc_m_blue = "#95b3d7"  # rgb 149 179 215
rc_d_blue = "#376092"  # rgb 55 96 146

rc_l_green = "#e6f1de"  # rgb 235 241 222
rc_m_green = "#d7e4bd"  # rgb 215 228 189
rc_d_green = "#77933c"  # rgb 119 147 60

rc_l_red = "#f2dcdc"  # rgb 242 220 220
rc_m_red = "#d99696"  # rgb 217 150 150
rc_d_red = "#953735"  # rgb 149 55 53

rc_l_yellow = "#fff769"  # rgb 255 247 185
rc_m_yellow = "#e9d903"  # rgb 233 217 3
rc_d_yellow = "#988d02"  # rgb 152 141 2

rc_white = "#ffffff"
rc_grey_0 = rc_white
rc_grey_1 = "#fefefe"
rc_grey_2 = "#fdfdfd"
rc_grey_3 = "#fcfcfc"
rc_grey_4 = "#fbfbfb"
rc_grey_5 = "#fafafa"
rc_grey_6 = "#f9f9f9"
rc_grey_7 = "#f8f8f8"
rc_grey_8 = "#f7f7f7"
rc_grey_9 = "#f6f6f6"
rc_grey_10 = "#f5f5f5"
rc_grey_11 = "#f4f4f4"
rc_grey_12 = "#f3f3f3"
rc_grey_13 = "#f2f2f2"
rc_grey_14 = "#f1f1f1"

rc_grey_15 = "#f0f0f0"
rc_grey_16 = "#e0e0e0"

rc_grey_17 = "#d0d0d0"
rc_grey_18 = "#c0c0c0"
rc_grey_19 = "#b0b0b0"
rc_grey_20 = "#a0a0a0"

rc_grey_21 = "#e7e7e7"

rc_black = "#000000"

# windowcolors
# rc_hl_color = 'Grey' # for scope of the layout popups
# rc_hl_color = 'Grey' # for scope of the layout popups
#

rc_bg_color = rc_grey_10
rc_bg_empty_color = rc_grey_2
rc_bg_frame_color = rc_grey_21
rc_bg_txt_color = rc_grey_2
rc_button_color = rc_grey_21
#rc_bg_color = 'LightGrey'
#rc_bg_color = rc_l_blue

rc_hl_color = rc_grey_16  # for scope of the layout popups
# rc_hl_color = rc_l_blue # for scope of the layout popups

rc_fg_color = 'Grey'

rc_input_color = rc_l_green
rc_output_color = rc_l_blue

rc_dark_input_color = rc_d_green
rc_dark_output_color = rc_m_blue


# control color
rc_cont_bg = rc_grey_16
rc_cont_limit = rc_black
rc_cont_in = rc_input_color
rc_cont_out = rc_output_color

# define debug options
## smo_set_debug_options(application=True, debug=False, trace=False, verbous=False, info=True, warning=True, error=True, fatal=True)
##smo_set_debug_options(application=True, debug=False, trace=False, verbous=False, info=False, warning=False, error=False, fatal=False)
smo_set_debug_options(application=True, debug=False, trace=False, verbous=False,
                      info=False, warning=True, error=True, fatal=True)
