ARG FROM_IMAGE

FROM $FROM_IMAGE

RUN sudo apt update && sudo apt install -y wget gpg apt-transport-https
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee packages.microsoft.gpg
RUN sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
RUN sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
RUN sudo rm -f packages.microsoft.gpg
RUN sudo apt update
RUN sudo apt install -y code # or code-insiders
RUN sudo apt install -y gdb
