{% set def_passwd = '$1$OmLKZTkO$Hl4dxmjutI8nicK4Lrf2N0' %}
{% set def_groups = ['users', 'sudo', 'dialout', 'plugdev', 'netdev', 'video', 'i2c', 'gpio'] %}

# naming conventions:
# lastname: ccs staff member
# firstname.lastname: every one else

klingler:
  user.present:
    - fullname: Florian Klingler
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

hagenauer:
  user.present:
    - fullname: Florian Hagenauer
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

sommer:
  user.present:
    - fullname: Christoph Sommer
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/zsh

bloessl:
  user.present:
    - fullname: Bastian Bloessl
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

memedi:
  user.present:
    - fullname: Agon Memedi
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

blobel:
  user.present:
    - fullname: Johannes Blobel
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

nabeel:
  user.present:
    - fullname: Muhammad Nabeel
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

pg2016w:
  user.present:
    - fullname: Project Group 2016w
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

sergej.japs:
  user.present:
    - fullname: Sergej Japs
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

ashish.rohilla:
  user.present:
    - fullname: Ashish Rohilla
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

max.schettler:
  user.present:
    - fullname: Max Schettler
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/zsh

julian.heinovski:
  user.present:
    - fullname: Julian Heinovski
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

srinivas.krishnan:
  user.present:
    - fullname: Srinivas Krishnan
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

daniel.tigges:
  user.present:
    - fullname: Daniel Tigges
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/zsh

patrick.steffens:
  user.present:
    - fullname: Patrick Steffens
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash

bkleinjohann:
  user.present:
    - fullname: Bernd Kleinjohann
    - password: {{ def_passwd }}
    - groups: {{ def_groups }}
    - shell: /bin/bash
