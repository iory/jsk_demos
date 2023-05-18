ARG PYTORCH="1.13.1"
ARG CUDA="11.6"
ARG CUDNN="8"

FROM pytorch/pytorch:${PYTORCH}-cuda${CUDA}-cudnn${CUDNN}-devel

RUN if [ -e "/etc/apt/sources.list.d/cuda.list" ]; then \
        mv /etc/apt/sources.list.d/cuda.list /etc/apt/sources.list.d/cuda.list.save; \
    fi
RUN if [ -e "/etc/apt/sources.list.d/nvidia-ml.list" ]; then \
        mv /etc/apt/sources.list.d/nvidia-ml.list /etc/apt/sources.list.d/nvidia-ml.list.save; \
    fi
RUN apt update && apt install -y curl software-properties-common \
    libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

COPY *.py /workspace/
COPY requirements.txt /workspace/requirements.txt
RUN pip install -U pip \
    && pip install -r requirements.txt

CMD ["bash"]
