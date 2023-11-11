FROM mcr.microsoft.com/devcontainers/base:jammy

# Setup packages needed for `make devbox-install`
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install git ansible python3-pip vim
