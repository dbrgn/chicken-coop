#!/usr/bin/env bash
set -euo pipefail

mplayer tv:// -tv driver=v4l2:width=1920:height=1080:fps=30
