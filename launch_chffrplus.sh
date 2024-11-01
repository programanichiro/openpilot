#!/usr/bin/env bash

if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi

source "$BASEDIR/launch_env.sh"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    echo 1 > $DIR/../agnos_update
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The BASEDIR init file has to exist, with a newer modtime than anything in
  #    the BASEDIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  echo 1 > $DIR/../launch_1
  echo 0 > $DIR/../launch_2
  echo 0 > $DIR/../launch_3
  echo 0 > $DIR/../launch_4
  echo 0 > $DIR/../launch_5
  echo 0 > $DIR/../launch_6

  if [ ! -f $DIR/../force_prebuild ]; then
    echo 1 > $DIR/../launch_2
    rm ${BASEDIR}/.overlay_init
  fi
  echo 2 > $DIR/../launch_1

  if [ -f "${BASEDIR}/.overlay_init" ]; then
    echo 1 > $DIR/../launch_3
    find ${BASEDIR}/.git -newer ${BASEDIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo 2 > $DIR/../launch_3
      echo "${BASEDIR} has been modified, skipping overlay update installation"
    else
      echo 3 > $DIR/../launch_3
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        echo 4 > $DIR/../launch_3
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo 5 > $DIR/../launch_3
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $BASEDIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $BASEDIR
          cd $BASEDIR

          echo 6 > $DIR/../launch_3
          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
          echo 7 > $DIR/../launch_3
        else
          echo 8 > $DIR/../launch_3
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi
  echo 3 > $DIR/../launch_1

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  echo 4 > $DIR/../launch_1
  # hardware specific init
  if [ -f /AGNOS ]; then
    echo 1 > $DIR/../launch_4
    agnos_init
    echo 2 > $DIR/../launch_4
  fi

  echo 5 > $DIR/../launch_1
  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  echo 6 > $DIR/../launch_1
  if [ ! -f $DIR/common/params_pyx.so ]; then
    echo 1 > $DIR/../launch_5
    echo 1 > $DIR/../force_prebuild
  fi

  echo 7 > $DIR/../launch_1
  # start manager
  cd system/manager
  if [ -f $DIR/../agnos_update ] || [ ! -f $DIR/prebuilt ] && [ -f $DIR/../force_prebuild ]; then
    echo 1 > $DIR/../launch_6
    ./build.py
    echo 2 > $DIR/../launch_6
  fi
  echo 8 > $DIR/../launch_1
  ./manager.py

  echo 9 > $DIR/../launch_1
  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
