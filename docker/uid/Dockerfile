ARG FROM_IMAGE
FROM $FROM_IMAGE

ARG UID

RUN sudo groupmod -g $UID developer && sudo usermod -u $UID developer
