#!/bin/sh
# User-local vehicle entry point; the Rust executable performs inference.
set -eu
. "${XDG_CONFIG_HOME:-$HOME/.config}/xt-stcar/runtime.env"
: "${XT_STCAR_RELEASE:?missing release directory}"
: "${XT_STCAR_RUNTIME_LIB:?missing runtime library}"
if [ "${1:-}" = infer ]; then
    have_runtime=no
    have_model=no
    have_config=no
    backend=native-ort
    previous=
    for argument in "$@"; do
        if [ "$previous" = --backend ]; then backend=$argument; fi
        case "$argument" in
            --runtime-lib) have_runtime=yes ;;
            --model) have_model=yes ;;
            --config) have_config=yes ;;
        esac
        previous=$argument
    done
    if [ "$backend" = native-ort ] && [ "$have_runtime" = no ]; then
        set -- "$@" --runtime-lib "$XT_STCAR_RUNTIME_LIB"
    fi
    if [ "$have_model" = no ]; then
        set -- "$@" --model "$XT_STCAR_RELEASE/models/yolo26n.onnx"
    fi
    if [ "$have_config" = no ]; then
        set -- "$@" --config "$XT_STCAR_RELEASE/config/yolo26n.json"
    fi
fi
exec "$XT_STCAR_RELEASE/bin/xt-stcar" "$@"
