#!/usr/bin/env bash
# Launch the Rider PC-client GUI. The GUI codebase lives in gui/; cd there so its
# package imports (core/ ui/ communication/) and rider_config.ini resolve. Run from anywhere.
cd "$(dirname "$0")/gui" && exec python3 pc_client_standalone.py "$@"
