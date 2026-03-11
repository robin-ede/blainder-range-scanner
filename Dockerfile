# syntax=docker/dockerfile:1
FROM python:3.11-slim-bookworm

ARG BLENDER_VERSION=5.0.1
ARG BLENDER_SERIES=Blender5.0
ARG BLENDER_ARCHIVE=blender-${BLENDER_VERSION}-linux-x64.tar.xz
ARG BLENDER_DOWNLOAD_URL=https://download.blender.org/release/${BLENDER_SERIES}/${BLENDER_ARCHIVE}

ENV DEBIAN_FRONTEND=noninteractive \
    BLENDER_HOME=/opt/blender \
    BLENDER_BIN=/opt/blender/blender \
    PYTHONUNBUFFERED=1 \
    UV_LINK_MODE=copy

# Grab uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv

# System dependencies with apt caching
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    libdbus-1-3 \
    libegl1 \
    libgl1 \
    libglib2.0-0 \
    libgomp1 \
    libglu1-mesa \
    libice6 \
    libsm6 \
    libx11-6 \
    libxcursor1 \
    libxext6 \
    libxfixes3 \
    libxi6 \
    libxinerama1 \
    libxkbcommon0 \
    libxrender1 \
    libxxf86vm1 \
    xz-utils \
    && rm -rf /var/lib/apt/lists/*

# Download and install Blender
RUN curl -fsSL "${BLENDER_DOWNLOAD_URL}" -o /tmp/${BLENDER_ARCHIVE} \
    && mkdir -p /opt \
    && tar -xJf /tmp/${BLENDER_ARCHIVE} -C /opt \
    && mv /opt/blender-${BLENDER_VERSION}-linux-x64 ${BLENDER_HOME} \
    && ln -s ${BLENDER_BIN} /usr/local/bin/blender \
    && rm -f /tmp/${BLENDER_ARCHIVE}

WORKDIR /workspace

COPY range_scanner/requirements.txt /tmp/range_scanner_requirements.txt

# Install dependencies ONLY to Blender's Python using uv cache
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python ${BLENDER_HOME}/5.0/python/bin/python3.11 \
        -r /tmp/range_scanner_requirements.txt \
        matplotlib \
        numpy \
        scipy

COPY . /workspace

# Compile your scripts using Blender's Python!
RUN chmod +x /workspace/docker/entrypoint.sh \
    && ${BLENDER_HOME}/5.0/python/bin/python3.11 -m py_compile \
        scripts/run_fixed_scene_validation_trial.py \
        scripts/run_fixed_scene_validation_batch.py \
        scripts/run_pose_then_noise_validation.py \
        scripts/run_multi_seed_pose_then_noise_validation.py \
        scripts/run_animated_path_validation.py \
        scripts/visualize_validation_report.py

ENTRYPOINT ["/bin/bash", "/workspace/docker/entrypoint.sh"]
CMD ["help"]
