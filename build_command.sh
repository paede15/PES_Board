docker run --rm -it \
  -v "$PWD":/work -w /work \
  python:3.11-slim \
  bash -lc "pip install -U platformio && pio run"

docker run --rm -it \
  --device=/dev/ttyACM0 \
  -v "$PWD":/work -w /work \
  python:3.11-slim \
  bash -lc "pip install -U platformio && pio run -t upload"
