# Candidate Linux component image; not built/run here.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git python3 python3-venv zlib1g-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv
COPY requirements.lock /tmp/capture-requirements.lock
RUN /opt/venv/bin/python -m pip install --no-cache-dir -r /tmp/capture-requirements.lock
ENV PATH="/opt/venv/bin:$PATH"
ENV HOME=/tmp
WORKDIR /workspace
CMD ["python", "tests/standalone-camera-mock/run.py"]
