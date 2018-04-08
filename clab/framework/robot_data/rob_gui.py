###########################
###
### SMOBaseObject no. 2 of type RCInitTkGui and name -unknown- 
###
###
{'init_in_pred': {'robby': {'state': 'state',\
'getdrive': 'getdrive',\
'img': 'img'}},\
'internal_outport': {'setdrive': None,\
'instr': None},\
'rc_name': 'rob_gui',\
'init_inport': {'state': (None,\
2),\
'getdrive': ([0.0,\
0.0],\
2),\
'img': (None,\
2)},\
'in_pred': {'robby': ({'state': 'state',\
'getdrive': 'getdrive',\
'img': 'img'},\
{'state': 'state',\
'getdrive': 'getdrive',\
'img': 'img'},\
{'state': None,\
'getdrive': None,\
'img': None},\
{'state': None,\
'getdrive': None,\
'img': None})},\
'internal_inport': {'state': None,\
'getdrive': [0.0,\
0.0],\
'img': None},\
'init_outport': {'setdrive': ([0.0,\
0.0],\
2),\
'instr': (None,\
2)},\
'connection_name': 'rc_model_undef',\
'external_outport': {'setdrive': (0.0,\
0.0),\
'instr': ''},\
'out_succ': {'robby': ({'setdrive': 'setdrive',\
'instr': 'instr'},\
{'setdrive': 'setdrive',\
'instr': 'instr'},\
{'setdrive': None,\
'instr': None},\
{'setdrive': None,\
'instr': None})},\
'rc_path': '/home/max/dev/pg2016w/framework/robot_data',\
'external_inport': {'state': None,\
'getdrive': None,\
'img': None},\
'init_out_succ': {'robby': {'setdrive': 'setdrive',\
'instr': 'instr'}},\
'rc_widget_structure': ['Frame',\
{},\
{'widget_structure': ['H Pane',\
{'orient': 'horizontal'},\
{'widget_structure': [['Frame',\
{},\
{'widget_structure': ['Grid H',\
{'col_row_cnt': 4,\
'orient': 'horizontal'},\
{'widget_structure': [['Frame',\
{},\
{'widget_structure': ['Grid H',\
{'col_row_cnt': 4,\
'orient': 'vertical'},\
{'widget_structure': [['Frame',\
{},\
{'widget_structure': ['Image',\
{'widget_out_tab': [('img',\
'',\
'set_jpg_image',\
'')],\
'widget_in_tab': []},\
None]}],\
['I Scale',\
{'widget_out_tab': [],\
'name': 'tilt',\
'upper_bound': 1,\
'lower_bound': -1,\
'widget_in_tab': [('get_data',\
'',\
'setcampos',\
'[1]')],\
'resolution': 0.01,\
'orient': 'horizontal',\
'tickinterval': 0.25,\
'io_mode': 'IN'},\
None]]}]}],\
['I Scale',\
{'widget_out_tab': [],\
'name': 'pan',\
'upper_bound': 1,\
'lower_bound': -1,\
'widget_in_tab': [('get_data',\
'',\
'setcampos',\
'[0]')],\
'resolution': 0.01,\
'orient': 'vertical',\
'tickinterval': 0.25,\
'io_mode': 'IN'},\
None]]}]}],\
['Frame',\
{},\
{'widget_structure': ['Grid H',\
{'col_row_cnt': 4,\
'orient': 'vertical'},\
{'widget_structure': [['Frame',\
{},\
{'widget_structure': ['Axis',\
{'widget_out_tab': [('getdrive',\
'',\
'set_pos',\
'')],\
'widget_in_tab': [('get_pos',\
'',\
'setdrive',\
'')]},\
None]}],\
['I Entry',\
{'io_mode': 'INOUT',\
'name': 'Entry',\
'entry_type': 'string',\
'widget_out_tab': [('state',\
'',\
'set_data',\
'')],\
'orient': 'horizontal',\
'widget_in_tab': [('get_data',\
'',\
'instr',\
'')]},\
None]]}]}]]}]}]}
###
### SMOBaseObject end
###
###########################
