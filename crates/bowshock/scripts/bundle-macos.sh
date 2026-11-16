#!/bin/sh
# Builds target/bundle/Bowshock.app (release binary + assets/icons/bowshock.icns), ad-hoc signed so it runs locally.
#   crates/bowshock/scripts/bundle-macos.sh            build the release binary, then bundle it
#   BOWSHOCK_BIN=path/to/bowshock crates/bowshock/scripts/bundle-macos.sh   bundle an existing binary
# Run it from anywhere; paths are resolved from the script's location. The icons are regenerated with `bowshock icons` (see
# src/icons.rs), never by hand.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
icns="$here/../assets/icons/bowshock.icns"
version=$(sed -n 's/^version = "\(.*\)"/\1/p' "$root/Cargo.toml" | head -n 1)

if [ -z "${BOWSHOCK_BIN:-}" ]; then
    (cd "$root" && cargo build --release -p bowshock)
    BOWSHOCK_BIN="$root/target/release/bowshock"
fi

app="$root/target/bundle/Bowshock.app"
rm -rf "$app"
mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources"
cp "$BOWSHOCK_BIN" "$app/Contents/MacOS/bowshock"
cp "$icns" "$app/Contents/Resources/bowshock.icns"

cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key><string>Bowshock</string>
    <key>CFBundleDisplayName</key><string>Bowshock</string>
    <key>CFBundleIdentifier</key><string>io.github.sandvoxel.bowshock</string>
    <key>CFBundleExecutable</key><string>bowshock</string>
    <key>CFBundleIconFile</key><string>bowshock</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleShortVersionString</key><string>${version}</string>
    <key>CFBundleVersion</key><string>${version}</string>
    <key>LSApplicationCategoryType</key><string>public.app-category.developer-tools</string>
    <key>NSHighResolutionCapable</key><true/>
    <key>NSPrincipalClass</key><string>NSApplication</string>
</dict>
</plist>
PLIST

codesign --force --deep --sign - "$app" >/dev/null 2>&1 || echo "warning: codesign failed; the app may not open" >&2
echo "$app"
