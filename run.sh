#!/bin/bash
set -e
cd "$(dirname "$0")"

# If the venv exists but is missing key packages, nuke and rebuild
if [ -d ".venv" ]; then
  if ! .venv/bin/python -c "import plyfile" 2>/dev/null; then
    echo "Rebuilding virtual environment (dependencies changed)..."
    rm -rf .venv
  fi
fi

if [ ! -d ".venv" ]; then
  echo "Creating virtual environment..."
  python3 -m venv .venv
fi

source .venv/bin/activate

echo "Installing dependencies..."
pip install -q --upgrade pip
pip install -q -r requirements.txt

echo ""
echo "  LiDAR Grade Explorer"
echo "  ─────────────────────────────────────"
echo "  Open http://localhost:5050 in your browser"
echo "  Upload a .ply or .e57 scan to get started"
echo "  Ctrl-C to stop"
echo ""

python app.py
