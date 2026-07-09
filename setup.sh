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
"$SCRIPT_DIR/venv/bin/python" -c "import RPi.GPIO; print('    RPi.GPIO      OK')"
"$SCRIPT_DIR/venv/bin/python" -c "import serial; print('    pyserial      OK')"

echo ""
echo "==> Setup complete."
echo "    Run 'source ~/.bashrc' then 'source_sputt' to activate the venv."
