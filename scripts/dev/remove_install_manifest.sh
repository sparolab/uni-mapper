#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 PREFIX INSTALL_MANIFEST OWNER" >&2
  exit 2
fi

prefix=$(realpath -m "$1")
manifest=$2
owner=$3
if [[ "$prefix" == / || "$prefix" == . || -z "$prefix" ]]; then
  echo "refusing unsafe developer prefix: $prefix" >&2
  exit 1
fi
if [[ ! -f "$manifest" ]]; then
  printf '==> no %s install manifest; no installed paths removed\n' "$owner"
  exit 0
fi

mapfile -t owned_paths < "$manifest"
for owned_path in "${owned_paths[@]}"; do
  if [[ -z "$owned_path" || "$owned_path" != "$prefix/"* ||
        "$owned_path" == "$prefix" || "$owned_path" == */../* ]]; then
    printf 'unsafe %s install manifest path: %s\n' \
      "$owner" "$owned_path" >&2
    exit 1
  fi
done

removed=0
for owned_path in "${owned_paths[@]}"; do
  if [[ -e "$owned_path" || -L "$owned_path" ]]; then
    rm -f -- "$owned_path"
    removed=$((removed + 1))
  fi
done
printf '==> removed %s %s-owned installed paths\n' "$removed" "$owner"
