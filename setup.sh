# Run once from ~/Sputter_ctrl/
# # ─────────────────────────────────────────────
#
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#
echo "==> Working directory : $SCRIPT_DIR"
#
# # ── 1. System dependencies ───────────────────
echo "==> Installing system packages..."
sudo apt install -y python3-pip python3-venv python3-full
#
# # ── 2. Create venv ───────────────────────────
echo "==> Creating virtual environment..."
python3 -m venv "$SCRIPT_DIR/venv"
#
# # ── 3. Install Python packages ───────────────
echo "==> Installing Python packages..."
"$SCRIPT_DIR/venv/bin/python" -m pip install --upgrade pip
"$SCRIPT_DIR/venv/bin/python" -m pip install \
				adafruit-blinka \
				adafruit-circuitpython-ads1x15 \
				adafruit-extended-bus \
				RPi.GPIO \
				pyserial
# # ── 4. Fix alias in ~/.bashrc ─────────────────
echo "==> Updating source_sputt alias in ~/.bashrc..."
ALIAS_LINE="alias source_sputt='source $SCRIPT_DIR/venv/bin/activate'"
if grep -q "source_sputt" ~/.bashrc; then
	# Replace existing alias
	sed -i "s|alias source_sputt=.*|$ALIAS_LINE|" ~/.bashrc
	echo "    alias updated"
else
	# Add fresh alias
	echo "$ALIAS_LINE" >> ~/.bashrc
	echo "    alias added"
fi

# # ── 5. Verify ─────────────────────────────────
echo "==> Verifying installation..."
"$SCRIPT_DIR/venv/bin/python" -c "import board; print('    board        OK')"
"$SCRIPT_DIR/venv/bin/python" -c "import adafruit_ads1x15; print('    ADS1115       OK')"
"$SCRIPT_DIR/venv/bin/python" -c "import adafruit_extended_bus; print('    ExtendedI2C   OK')"
"$SCRIPT_DIR/venv/bin/python" -c "import RPi.GPIO; print('    RPi.GPIO      OK')"
"$SCRIPT_DIR/venv/bin/python" -c "import serial; print('    pyserial      OK')"

# # ── 6. Desktop launcher icon ──────────────────
echo "==> Installing desktop launcher..."
if [ -d "$HOME/Desktop" ]; then
	cp "$SCRIPT_DIR/Sputter_ctrl.desktop" "$HOME/Desktop/Sputter_ctrl.desktop"
	chmod +x "$HOME/Desktop/Sputter_ctrl.desktop"
	# PCManFM/Nautilus-style file managers refuse to run an untrusted
	# .desktop file until told to; gio marks it trusted automatically where
	# available so double-click works immediately. Not fatal if it's
	# missing (older/other desktop environments) -- chmod +x alone is
	# enough on some setups, and worst case the operator right-clicks ->
	# "Allow Launching" once, same as any downloaded .desktop file.
	# `|| true` matters under set -e: over SSH there is often no D-Bus
	# session, so `gio set` itself fails even where gio exists -- and as
	# the final command of this AND-list, that failure would abort the
	# whole script mid-way without it.
	command -v gio >/dev/null 2>&1 && \
		gio set "$HOME/Desktop/Sputter_ctrl.desktop" metadata::trusted true 2>/dev/null || true
	echo "    icon placed on Desktop"
else
	echo "    ~/Desktop not found -- skipped (no desktop environment here?)"
fi

echo ""
echo "==> Setup complete."
echo "    Run 'source ~/.bashrc' then 'source_sputt' to activate the venv."
echo "    Or just double-click the Sputter Vacuum Controller icon on the Desktop."
