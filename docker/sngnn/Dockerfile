ARG BASE_IMAGE=cmucal/cabot-navigation:latest
FROM $BASE_IMAGE

USER root

# Install system dependencies if any needed for dgl or opencv (usually in base, but safe to check)
# dgl might need libgomp1
RUN apt-get update && apt-get install -y \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
COPY SNGNN2D-v2/requirements.txt /tmp/requirements.txt

# Force numpy version and install requirements
# dgl 2.0.0+cu121 might install numpy 2.x if not constrained?
# We install numpy first.
RUN pip install "numpy==1.24.4"

# Remove dgl from requirements if we want to install specific version manually or let pip handle it but we need to watch out for numpy upgrade.
# The requirements.txt in SNGNN2D-v2 has: 
# torch~=2.1.2
# dgl==2.0.0+cu121 --find-links https://data.dgl.ai/wheels/cu121/repo.html
# numpy~=1.26.3  <-- This conflicts with user request.

# We will edit requirements on the fly or just ignore the file and run pip install commands.
# Generating a modified requirements file is safer.

RUN sed -i 's/numpy~=1.26.3/numpy==1.24.4/' /tmp/requirements.txt && \
    pip install -r /tmp/requirements.txt

# Ensure numpy 1.24.4 is kept
RUN pip install "numpy==1.24.4"

# USER developer

