#!/bin/bash

set -x

HOSTS=
# TODO read from config file
#HOSTS+=" raspi-lumpi"
#HOSTS+=" raspi-hasso"
#HOSTS+=" raspi-bello"
#HOSTS+=" raspi-wuffi"
#HOSTS+=" raspi-brutus"
#HOSTS+=" raspi-pluto"
#HOSTS+=" raspi-rex"
#HOSTS+=" raspi-waldi"
#HOSTS+=" raspi-fiffi"
#HOSTS+=" raspi-laika"
#HOSTS+=" raspi-julian"

HOSTS+=" 192.168.1.83"  # TODO current master host


do_sync () {
	HOST="$1"

	#echo refreshing host key for ${HOST}
	#ssh-keygen -R

	echo syncing to ${HOST}

	ssh root@${HOST} -- "nohup mkdir -p /srv/salt"
	ssh root@${HOST} -- "nohup mkdir -p /srv/salt_backup"
	rsync -Pax --backup --backup-dir /srv/salt_backup --delete --chown root:root --chmod 0600 srv/salt/ root@${HOST}:/srv/salt

	ssh root@${HOST} -- "nohup salt-call --local state.highstate"

}
export -f do_sync

parallel -j 10 do_sync ::: ${HOSTS}
