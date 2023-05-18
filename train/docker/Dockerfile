ARG PYTORCH="1.13.1"
ARG CUDA="11.6"
ARG CUDNN="8"

FROM pytorch/pytorch:${PYTORCH}-cuda${CUDA}-cudnn${CUDNN}-devel

ENV USER_NAME=user

RUN if [ -e "/etc/apt/sources.list.d/cuda.list" ]; then \
        mv /etc/apt/sources.list.d/cuda.list /etc/apt/sources.list.d/cuda.list.save; \
    fi
RUN if [ -e "/etc/apt/sources.list.d/nvidia-ml.list" ]; then \
        mv /etc/apt/sources.list.d/nvidia-ml.list /etc/apt/sources.list.d/nvidia-ml.list.save; \
    fi
RUN apt update && apt install -y sudo curl software-properties-common \
    libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/${USER_NAME}
RUN chmod u+s /usr/sbin/useradd \
    && chmod u+s /usr/sbin/groupadd

COPY *.py /workspace/
COPY requirements.txt /workspace/requirements.txt
RUN pip install -U pip \
    && pip install -r requirements.txt
# download pretrained model
RUN python -c 'import numpy; from rembg import remove; remove(numpy.ones((10, 10), dtype=numpy.uint8))'

USER $USER_NAME
ENV HOME /home/$USER_NAME

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
