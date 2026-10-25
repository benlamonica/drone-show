#!/bin/bash
# run a command in the KiCad container with the work dir mounted
docker run --rm -u root -e PYTHONPATH=/w/pylib -v /home/claude/work:/w -w /w kicad/kicad:9.0 "$@"
