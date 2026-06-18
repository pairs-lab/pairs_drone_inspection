#!/usr/bin/env bash
# Download selected Isaac Sim 6.0 factory/warehouse assets from the public S3 root
# into a LOCAL mirror (~/isaacsim_assets) so the demo runs offline. We only fetch
# what the cycle-count POC needs (warehouse env + pallet/bin/box/forklift props),
# NOT the full ~75GB assets pack.
#
# Local layout mirrors the cloud root so USD references resolve:
#   {ASSETS_ROOT cloud} = https://.../Assets/Isaac/6.0
#   {local}             = ~/isaacsim_assets   (strip "Assets/Isaac/6.0/")
# Point Isaac at it via carb setting:
#   /persistent/isaac/asset_root/default = ~/isaacsim_assets
#
# Usage: scripts/download_assets.sh
set -e
BASE="https://omniverse-content-production.s3-us-west-2.amazonaws.com"
STRIP="Assets/Isaac/6.0/"
DEST="${ISAAC_ASSETS_LOCAL:-$HOME/isaacsim_assets}"

# S3 key prefixes to mirror (relative to the bucket). Add more as needed.
PREFIXES=(
  "Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/"
  "Assets/Isaac/6.0/Isaac/Props/Pallet/"
  "Assets/Isaac/6.0/Isaac/Props/KLT_Bin/"
  "Assets/Isaac/6.0/Isaac/Props/PackingTable/"
  "Assets/Isaac/6.0/Isaac/Props/Forklift/"
)

urlenc() { python3 -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$1"; }

for PREFIX in "${PREFIXES[@]}"; do
  LIST="/tmp/isaac_keys_$(echo "$PREFIX" | tr '/' '_').txt"; : > "$LIST"; tok=""
  while :; do
    url="$BASE/?list-type=2&prefix=$PREFIX&max-keys=1000"
    [ -n "$tok" ] && url="$url&continuation-token=$(urlenc "$tok")"
    resp=$(curl -s "$url")
    echo "$resp" | grep -oE '<Key>[^<]+</Key>' | sed -E 's#</?Key>##g' >> "$LIST"
    tok=$(echo "$resp" | grep -oE '<NextContinuationToken>[^<]+' | sed 's/<NextContinuationToken>//')
    [ -z "$tok" ] && break
  done
  grep -vE '/\.thumbs/|/$' "$LIST" > "${LIST}.f" && mv "${LIST}.f" "$LIST"
  echo "[$PREFIX] $(wc -l < "$LIST") files"
  cat "$LIST" | xargs -P 12 -I{} bash -c '
    key="{}"; BASE="'"$BASE"'"; STRIP="'"$STRIP"'"; DEST="'"$DEST"'"
    rel="${key#$STRIP}"; out="$DEST/$rel"; mkdir -p "$(dirname "$out")"
    enc=$(python3 -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$key")
    curl -s -f -o "$out" "$BASE/$enc" || { sleep 1; curl -s -f -o "$out" "$BASE/$enc" || echo "FAIL $key"; }
  '
done
echo "ASSETS_DONE total=$(du -sh "$DEST" | cut -f1) at $DEST"
