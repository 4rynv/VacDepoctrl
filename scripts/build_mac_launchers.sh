#!/bin/bash
# Builds two double-clickable macOS .app launchers and installs them on the
# Desktop -- the Mac equivalent of VacDepoctrl.desktop on the Pi:
#
#   VacDep Simulation.app   -- runs main.py locally with SPUTTER_SIM=1,
#                               no Pi or hardware needed (see README's
#                               Simulation Mode notes / sim/sim_hardware.py).
#   VacDep Remote.app       -- picks a rig (see RIGS below), SSHes in with
#                               X11 forwarding, and launches the real
#                               main.py there (see README's Remote GUI
#                               Access section).
#
# Both open Terminal.app rather than running silently, since main.py's
# startup self-test prints failures to the console before any GUI window
# exists -- a silently-failing launch with no visible error is a real
# usability problem for this kind of hardware-control app (same reasoning
# as Terminal=true in VacDepoctrl.desktop on the Pi side).
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ICNS="$REPO_DIR/assets/AppIcon.icns"
DESKTOP="$HOME/Desktop"

if [ ! -f "$ICNS" ]; then
	echo "assets/AppIcon.icns not found -- generate it first (see assets/app-icon-1024.png)."
	exit 1
fi

scaffold_app () {
	local app_name="$1"
	local bundle_id="$2"
	local app_dir="$DESKTOP/$app_name.app"

	rm -rf "$app_dir"
	mkdir -p "$app_dir/Contents/MacOS" "$app_dir/Contents/Resources"
	cp "$ICNS" "$app_dir/Contents/Resources/AppIcon.icns"

	cat > "$app_dir/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>CFBundleExecutable</key>
	<string>launcher</string>
	<key>CFBundleIconFile</key>
	<string>AppIcon</string>
	<key>CFBundleIdentifier</key>
	<string>$bundle_id</string>
	<key>CFBundleName</key>
	<string>$app_name</string>
	<key>CFBundlePackageType</key>
	<string>APPL</string>
	<key>CFBundleShortVersionString</key>
	<string>1.0</string>
	<key>LSUIElement</key>
	<false/>
</dict>
</plist>
PLIST

	echo "$app_dir"
}

echo "==> Building VacDep Simulation.app..."
SIM_APP_DIR="$(scaffold_app "VacDep Simulation" "com.aryanv.vacdepoctrl.sim")"
# osascript reads the script from stdin (heredoc) rather than a -e '...'
# argument -- an earlier version embedded the shell command directly into
# a single-quoted -e argument, which silently stripped any single quotes
# the command itself contained (bash parsed them as its own quoting, not
# literal characters). The inner heredoc's quoted delimiter ('APPLESCRIPT')
# writes the AppleScript out completely literally, sidestepping that class
# of bug entirely.
cat > "$SIM_APP_DIR/Contents/MacOS/launcher" <<LAUNCHER
#!/bin/bash
osascript <<'APPLESCRIPT'
tell application "Terminal"
	activate
	do script "cd '$REPO_DIR' && SPUTTER_SIM=1 /opt/homebrew/bin/python3 main.py"
end tell
APPLESCRIPT
LAUNCHER
chmod +x "$SIM_APP_DIR/Contents/MacOS/launcher"
echo "    built: $SIM_APP_DIR"

echo "==> Building VacDep Remote.app..."
REMOTE_APP_DIR="$(scaffold_app "VacDep Remote" "com.aryanv.vacdepoctrl.remote")"
# Two-stage: pick a rig from RIGS (skipped automatically when there's only
# one), then hand the ssh command to Terminal the same way Simulation does.
# The SSH password itself is deliberately left to ssh's own normal
# interactive Terminal prompt rather than something like sshpass -- that
# already works, is secure, and needs no extra tooling/dependencies; piping
# a stored plaintext password through a second tool would be a strictly
# worse tradeoff for no real gain here.
cat > "$REMOTE_APP_DIR/Contents/MacOS/launcher" <<'LAUNCHER'
#!/bin/bash

# Known rigs -- "user@mdns-hostname|Friendly Name". Uses the Pi's mDNS
# hostname (av.local), not a raw IP: DHCP has moved this rig's address
# before, but the mDNS name survives that. To add another rig (SpinCoater,
# litho, ...) once it comes online: add a line here, then re-run
# scripts/build_mac_launchers.sh to rebuild this app -- 1 entry auto-
# connects with no prompt, 2+ shows a choose-from-list dialog.
RIGS=(
	"raspberrypi@av.local|VacDepoctrl (sputter rig)"
)

if [ "${#RIGS[@]}" -eq 1 ]; then
	SELECTED="${RIGS[0]}"
else
	LIST_ITEMS=""
	for r in "${RIGS[@]}"; do
		LABEL="${r#*|}"
		LIST_ITEMS="${LIST_ITEMS}\"${LABEL}\", "
	done
	LIST_ITEMS="${LIST_ITEMS%, }"
	CHOICE_LABEL=$(osascript -e "choose from list {$LIST_ITEMS} with prompt \"Select a rig to control:\" without multiple selections allowed")
	if [ "$CHOICE_LABEL" = "false" ] || [ -z "$CHOICE_LABEL" ]; then
		exit 0
	fi
	for r in "${RIGS[@]}"; do
		LABEL="${r#*|}"
		if [ "$LABEL" = "$CHOICE_LABEL" ]; then
			SELECTED="$r"
			break
		fi
	done
fi

TARGET="${SELECTED%%|*}"

osascript <<APPLESCRIPT
tell application "Terminal"
	activate
	do script "ssh -X $TARGET -t '/home/raspberrypi/VacDepoctrl/venv/bin/python /home/raspberrypi/VacDepoctrl/main.py'"
end tell
APPLESCRIPT
LAUNCHER
chmod +x "$REMOTE_APP_DIR/Contents/MacOS/launcher"
echo "    built: $REMOTE_APP_DIR"

echo ""
echo "==> Done. Both apps are on the Desktop."
echo "    First launch: right-click -> Open (once) to bypass Gatekeeper's"
echo "    'unidentified developer' warning for unsigned apps."
