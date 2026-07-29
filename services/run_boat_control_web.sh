#!/bin/bash
source ~/sgw-config

$VENV_PATH/bin/streamlit run $SGW_WS/boat_control_web/app.py --server.port 8080
