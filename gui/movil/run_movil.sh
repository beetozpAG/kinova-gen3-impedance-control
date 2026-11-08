#!/usr/bin/env bash
# Control desde el celular (colores, pausar, seguir). Muestra la URL y el PIN.
# Uso:  ./run_movil.sh   (desde cualquier carpeta); opciones: --puerto N, --log ARCHIVO
cd "$(dirname "$(readlink -f "$0")")" || exit 1
exec ../.venv/bin/python servidor.py "$@"
