#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 || ! -x "$1" ]]; then
  echo "usage: $0 FETCH_REPLAY_DATA_SCRIPT" >&2
  exit 2
fi

fetch_script=$1
fixture_root=$(mktemp -d)
cleanup() { rm -rf -- "$fixture_root"; }
trap cleanup EXIT

mkdir -p "$fixture_root/safe/source"
printf 'locked replay fixture\n' > "$fixture_root/safe/source/input.txt"
tar --create --file "$fixture_root/safe.tar" \
  --directory "$fixture_root/safe" source
safe_digest=$(sha256sum "$fixture_root/safe.tar")
safe_digest=${safe_digest%% *}

"$fetch_script" --url "file://$fixture_root/safe.tar" \
  --archive-sha256 "$safe_digest" \
  --destination "$fixture_root/extracted"
test "$(<"$fixture_root/extracted/source/input.txt")" = \
  "locked replay fixture"

if "$fetch_script" --url "file://$fixture_root/safe.tar" \
  --archive-sha256 \
  0000000000000000000000000000000000000000000000000000000000000000 \
  --destination "$fixture_root/checksum-mismatch"; then
  echo "checksum mismatch was accepted" >&2
  exit 1
fi
test ! -e "$fixture_root/checksum-mismatch"

mkdir -p "$fixture_root/unsafe"
ln -s /tmp "$fixture_root/unsafe/escape"
tar --create --file "$fixture_root/unsafe.tar" \
  --directory "$fixture_root/unsafe" escape
unsafe_digest=$(sha256sum "$fixture_root/unsafe.tar")
unsafe_digest=${unsafe_digest%% *}
if "$fetch_script" --url "file://$fixture_root/unsafe.tar" \
  --archive-sha256 "$unsafe_digest" \
  --destination "$fixture_root/link-escape"; then
  echo "archive symlink was accepted" >&2
  exit 1
fi
test ! -e "$fixture_root/link-escape"
