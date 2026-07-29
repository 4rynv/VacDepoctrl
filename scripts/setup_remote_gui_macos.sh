#!/bin/bash
# One-time setup for viewing the VacDepoctrl GUI on a Mac instead of the
# Pi's local display, via SSH X11 forwarding (`ssh -X`). See the "Remote GUI
# Access" section in README.md for why you'd want this and the Windows/Linux
# equivalents.
set -e

echo "==> Checking for Homebrew..."
if ! command -v brew >/dev/null 2>&1; then
	echo "Homebrew not found. Install it first: https://brew.sh"
	exit 1
fi

echo "==> Installing XQuartz (X11 server for macOS)..."
if brew list --cask xquartz >/dev/null 2>&1; then
	echo "    already installed"
else
	brew install --cask xquartz
fi

echo "==> Launching XQuartz..."
open -a XQuartz

echo ""
echo "==> Setup complete."
echo "    Open a NEW terminal window (so it picks up XQuartz's environment), then:"
echo "        ssh -X raspberrypi@<pi-ip>"
echo "        source_vacdep && cd ~/VacDepoctrl && python main.py"
echo "    If the window doesn't appear, try 'ssh -Y' instead of 'ssh -X'."
echo "    Still nothing? As a last resort (weakens XQuartz's network security"
echo "    by allowing raw TCP connections to it, not just the local Unix"
echo "    socket ssh -X normally uses) -- only if you understand that"
echo "    tradeoff -- run:"
echo "        defaults write org.macosforge.xquartz.X11 nolisten_tcp -bool false"
echo "    then quit and relaunch XQuartz."
