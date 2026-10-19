# ============================================================================
# Input: CSV with: t, x, y, yaw
# ============================================================================

FROM pytorch/pytorch:2.4.0-cuda12.1-cudnn9-runtime

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    DATA_DIR=/data \
    OUT_DIR=/output

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Apenas o necessário para o train_gt.py
RUN pip install --upgrade pip && pip install \
        numpy==1.26.4 \
        pandas==2.2.2 \
        tensorboard==2.17.0

COPY train_gt.py /app/train_gt.py

RUN mkdir -p ${DATA_DIR} ${OUT_DIR}

# Uso:
#   docker build -t v2x-regression -f Dockerfile .
#   docker run --gpus all -v $(pwd)/gt.csv:/data/gt.csv -v $(pwd)/runs:/output v2x-regression
# CPU:  docker run -e CUDA_VISIBLE_DEVICES="" -v $(pwd)/gt.csv:/data/gt.csv -v $(pwd)/runs:/output v2x-regression --device cpu
ENTRYPOINT ["python", "train_gt.py", "--poses", "/data/gt.csv", "--out", "/output"]
CMD ["--device", "cuda", "--epochs", "50", "--batch-size", "128"]
