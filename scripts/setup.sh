#!/usr/bin/env bash
# Run once: ~/VacDepoctrl/scripts/setup.sh
# # ─────────────────────────────────────────────
#
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
#
echo "==> Repo directory : $REPO_DIR"
#
# # ── 1. System dependencies ───────────────────
echo "==> Installing system packages..."
sudo apt install -y python3-pip python3-venv python3-full python3-tk
#
# # ── 2. Create venv ───────────────────────────
echo "==> Creating virtual environment..."
python3 -m venv "$REPO_DIR/venv"
#
# # ── 3. Install Python packages ───────────────
echo "==> Installing Python packages..."
"$REPO_DIR/venv/bin/python" -m pip install --upgrade pip
"$REPO_DIR/venv/bin/python" -m pip install \
				adafruit-blinka \
				adafruit-circuitpython-ads1x15 \
				adafruit-extended-bus \
				pyserial
# Blinka currently pulls in legacy RPi.GPIO; replace it after dependency resolution.
# Force reinstall restores the shared RPi.GPIO namespace on repeated setup runs.
"$REPO_DIR/venv/bin/python" -m pip uninstall -y RPi.GPIO
"$REPO_DIR/venv/bin/python" -m pip install --force-reinstall --no-deps rpi-lgpio

# # ── 4. Fix alias in ~/.bashrc ─────────────────
echo "==> Updating source_vacdep alias in ~/.bashrc..."
# Drop the pre-rename alias if present (pointed at ~/Sputter_ctrl)
sed -i "/alias source_sputt=/d" ~/.bashrc
ALIAS_LINE="alias source_vacdep='source $REPO_DIR/venv/bin/activate'"
if grep -q "source_vacdep" ~/.bashrc; then
	# Replace existing alias
	sed -i "s|alias source_vacdep=.*|$ALIAS_LINE|" ~/.bashrc
	echo "    alias updated"
else
	# Add fresh alias
	echo "$ALIAS_LINE" >> ~/.bashrc
	echo "    alias added"
fi

# # ── 5. Verify ─────────────────────────────────
echo "==> Verifying installation..."
"$REPO_DIR/venv/bin/python" -c "import board; print('    board        OK')"
"$REPO_DIR/venv/bin/python" -c "import adafruit_ads1x15; print('    ADS1115       OK')"
"$REPO_DIR/venv/bin/python" -c "import adafruit_extended_bus; print('    ExtendedI2C   OK')"
"$REPO_DIR/venv/bin/python" -c "import RPi.GPIO; print('    RPi.GPIO      OK')"
"$REPO_DIR/venv/bin/python" -c "import serial; print('    pyserial      OK')"

# # ── 6. Desktop launcher icon ──────────────────
echo "==> Installing desktop launcher..."
if [ -d "$HOME/Desktop" ]; then
	"$REPO_DIR/venv/bin/python" - "$REPO_DIR" "$HOME/Desktop/VacDepoctrl.desktop" <<'PYDESKTOP'
from pathlib import Path
import sys
repo = Path(sys.argv[1])
template = (repo / "scripts/VacDepoctrl.desktop").read_text()
Path(sys.argv[2]).write_text(template.replace("@REPO_DIR@", str(repo)))
PYDESKTOP
	chmod +x "$HOME/Desktop/VacDepoctrl.desktop"
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
		gio set "$HOME/Desktop/VacDepoctrl.desktop" metadata::trusted true 2>/dev/null || true
	echo "    icon placed on Desktop"
else
	echo "    ~/Desktop not found -- skipped (no desktop environment here?)"
fi

echo ""
echo "==> Setup complete."
echo "    Run 'source ~/.bashrc' then 'source_vacdep' to activate the venv."
echo "    Or just double-click the VacDepoctrl icon on the Desktop."
