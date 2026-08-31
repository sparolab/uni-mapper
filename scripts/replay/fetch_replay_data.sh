#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "usage: $0 --url IMMUTABLE_URL --archive-sha256 SHA256 --destination DIR" >&2
}

url=""
archive_sha256=""
destination=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --url) url=$2; shift 2 ;;
    --archive-sha256) archive_sha256=${2#sha256:}; shift 2 ;;
    --destination) destination=$2; shift 2 ;;
    --help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

if [[ -z "$url" || ! "$archive_sha256" =~ ^[0-9a-f]{64}$ ||
      -z "$destination" ]]; then
  usage
  exit 2
fi
if [[ -e "$destination" ]]; then
  echo "destination must not already exist: $destination" >&2
  exit 2
fi

temporary_root=$(mktemp -d)
archive="$temporary_root/replay-data.tar"
unpack_root="$temporary_root/unpacked"
cleanup() { rm -rf -- "$temporary_root"; }
trap cleanup EXIT

curl --fail --location --proto '=https,file' --retry 3 \
  --output "$archive" "$url"
printf '%s  %s\n' "$archive_sha256" "$archive" | sha256sum --check --status

if tar --list --file "$archive" | grep -Eq '(^/|(^|/)\.\.(/|$))'; then
  echo "archive contains an unsafe path" >&2
  exit 1
fi
if tar --list --verbose --file "$archive" | grep -Eq '^[lh]'; then
  echo "archive links are not permitted" >&2
  exit 1
fi

mkdir -p "$unpack_root"
tar --extract --file "$archive" --directory "$unpack_root" \
  --no-same-owner --no-same-permissions
mkdir -p "$(dirname "$destination")"
mv -- "$unpack_root" "$destination"
