# Usa a imagem base do Ubuntu 24.10
# FROM ubuntu:24.10

# Debian
# FROM debian:bookworm
FROM python:3.11

ENV DEBIAN_UPDATE_ALWAYS 1
ENV DEBIAN_FRONTEND noninteractive

# Atualiza os pacotes para compilacao
RUN apt-get update && apt-get install -y \
    libmsgpack-c2 libmosquitto1 libzmq5 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install ufr

# Define o diretório de trabalho
WORKDIR /app

# Copia o código-fonte para o container (substitua pelo seu método preferido)
COPY debian_install/lib/* /usr/lib/

# Define o comando padrão para executar o programa (ajuste conforme necessário)
# CMD ["./build/nome_do_seu_executavel"]