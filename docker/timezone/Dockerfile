ARG FROM_IMAGE
FROM $FROM_IMAGE

ARG TZ_OVERWRITE=UTC
ENV TZ=$TZ_OVERWRITE

RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ | sudo tee /etc/timezone
