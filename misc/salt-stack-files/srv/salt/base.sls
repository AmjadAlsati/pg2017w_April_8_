pkg.upgrade:
  module.run:
    - refresh: True

base:
  pkg:
    - installed
    - pkgs:
      - sudo
      - rsync
      - htop
      - vim
      - salt-minion
      - git
      - tig
      - zsh
      - etckeeper
      - screen
      - etherwake
      - ethtool
      - wakeonlan
      - tmux
      - python-smbus
      - ntp
      - libdc1394-22-dev
      - libdc1394-utils
      - coriander
      - ipython
      - python-setuptools
      - python-pip
      - libusb-1.0-0-dev
      - xrdp
      - jstest-gtk
      - firmware-atheros
      - iw
      - ccache


salt-minion:
  service.dead:
    - enable: False

/etc/sysctl.d/01-disable-ipv6.conf:
  file.managed:
    - source:
      - salt://managed_files/01-disable-ipv6.conf
    - user: root
    - group: root
    - mode: 755

/etc/modprobe.d/sit-blacklist.conf:
  file.managed:
    - source:
      - salt://managed_files/sit-blacklist.conf
    - user: root
    - group: root
    - mode: 755

/etc/wpa_supplicant/wpa_supplicant.conf:
  file.managed:
    - source:
      - salt://managed_files/wpa_supplicant.conf
    - user: root
    - group: root
    - mode: 755

/etc/dhcp/dhclient-exit-hooks.d/hostname:
  file.managed:
    - source:
      - salt://managed_files/hostname
    - user: root
    - group: root
    - mode: 755

/etc/modules:
  file.managed:
    - source:
      - salt://managed_files/modules
    - user: root
    - group: root
    - mode: 755

/etc/modprobe.d/i2c_bcm2708.conf:
  file.managed:
    - source:
      - salt://managed_files/i2c_bcm2708.conf
    - user: root
    - group: root
    - mode: 755
