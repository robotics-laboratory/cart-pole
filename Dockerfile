# syntax=docker/dockerfile:1.6
FROM python:3.10-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    SERIAL_PORT=/dev/ttyUSB0 \
    SERIAL_SPEED=500000 \
    PYTHONPATH=/cartpole

WORKDIR /cartpole

RUN apt-get update -q \
 && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      curl \
      protobuf-compiler \
      libjpeg62-turbo \
      libpng16-16 \
 && rm -rf /var/lib/apt/lists/*

COPY requirements.pi.txt /tmp/requirements.pi.txt
RUN pip install --no-cache-dir -U pip \
 && pip install --no-cache-dir -r /tmp/requirements.pi.txt \
 && arch="$(uname -m)" \
 && if [ "$arch" = "aarch64" ]; then \
      pip install --no-cache-dir \
        "https://download.pytorch.org/whl/torch-2.0.1-cp310-cp310-manylinux2014_aarch64.whl"; \
    elif [ "$arch" = "x86_64" ]; then \
      pip install --no-cache-dir torch==2.0.1+cpu \
        --extra-index-url https://download.pytorch.org/whl/cpu; \
    else \
      pip install --no-cache-dir torch==2.0.1; \
    fi

COPY protocol.proto /cartpole/protocol.proto
COPY scripts/update_protobuf.py /cartpole/scripts/update_protobuf.py
RUN python /cartpole/scripts/update_protobuf.py --python-only

COPY . /cartpole
RUN python /cartpole/scripts/update_protobuf.py --python-only \
 && python -c "from cartpole.device import protocol_pb2; print('protobuf OK')" \
 && python -c "from cartpole.simulator import Simulator; print('simulator OK')" \
 && chmod +x /cartpole/scripts/run_pi.sh

ENTRYPOINT ["/cartpole/scripts/run_pi.sh"]
CMD ["help"]
