#!/usr/bin/env bash
# Interfaz grafica del MODULAR. Uso:  ./run_gui.sh   (desde cualquier carpeta)
# Primera vez (crea el entorno con pyqtgraph; usa PyQt5, numpy y VTK del sistema):
#   uv venv --system-site-packages --python /usr/bin/python3 .venv
#   uv pip install --python .venv/bin/python --no-deps "pyqtgraph==0.13.7"
cd "$(dirname "$(readlink -f "$0")")" || exit 1
exec .venv/bin/python -m modular_gui "$@"
