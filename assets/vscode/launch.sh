#!/bin/bash
set -euo pipefail

# dt-vscode selects the default user or impersonates HOST_UID at startup.
# A new impersonated user is created by dt-launcher-default after this hook.
editor_uid="${HOST_UID:-$(id -u "${DT_USER_NAME:-duckie}")}"
if getent passwd "${editor_uid}" >/dev/null; then
    editor_gid="$(id -g "${editor_uid}")"
else
    editor_gid="${editor_uid}"
fi

# only generated colcon output belongs to this hook; source mounts are untouched
for directory in build install log; do
    output_path="${COLCON_WS:?COLCON_WS must be set}/${directory}"
    mkdir -p "${output_path}"
    chown -R "${editor_uid}:${editor_gid}" "${output_path}"
done

exec dt-launcher-default "$@"
