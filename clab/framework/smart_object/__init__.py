################
#
# Modul:        Smart Object Libary
# File:         __init__.py
#
# Author:       Bernd Kleinjohann
# Created:      May 2014
# Version:      0.1
#
# Contents:
#
#               The classes SMOIPID, SMOIPClient, SMOIPServer support an RPC system by IP sockets
#               SMOIPID is a representative (proxy) of an object which resists in a different
#               process in some thread (SMOSyncObject). The calls may be recursive.
#               The IP connetion must be established by SMOIPClient and SMOIPServer. The internal
#               message format (over IP) is PySource using repr and compile/exec.
#

"""
This file defines the puplic elements of the smart object libary

The class SMOBaseObject offer persistent storage of date in PySource format
and offer some basic debug messaging

The SMOSyncObject support syncronized method calls oacross differemt threads
including recursive calls (worker model)

The classes SMOIPID, SMOIPClient, SMOIPServer
support an RPC system by IP sockets
SMOIPID is a representative (proxy) of an
object which resists in a different
process in some thread (SMOSyncObject).
The calls may be recursive.
The IP connetion must be established by
SMOIPClient and SMOIPServer. The internal
message format (over IP) is PySource
using repr and compile/exec
"""

from .smo_util import *
from .smo_base_object import SMOBaseObject, smo_open, smo_save, smo_set_debug_options
from .smo_sync_object import SMOSyncObject, smo_thr_start, smo_thr_stop
from .smo_ip_id import SMOIPID, get_unique_id_str
from .smo_ip_client import SMOIPClient, SMO_IP_connect, SMO_IP_disconnect
from .smo_ip_client import SMO_IP_disconnect_all, SMO_IP_get_all_connection_adress
from .smo_ip_server import SMOIPServer, SMO_IP_new_server, SMO_IP_delete_server, SMO_IP_delete_all_server
from .smo_ip_server import SMO_IP_get_server, SMO_IP_get_all_server_adress
from .smo_ip_server import SMO_IP_publish, SMO_IP_unpublish
