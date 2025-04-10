# Usa a imagem base do Ubuntu 24.10
FROM ubuntu:24.10

# Atualiza os pacotes para compilacao
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Instala as dependências necessárias
RUN apt-get update && apt-get install -y \
    libmsgpack-dev \
    libzmq3-dev \
    libmosquitto-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Define o diretório de trabalho
WORKDIR /app

# Copia o código-fonte para o container (substitua pelo seu método preferido)
COPY . .

# Cria um diretório de build e compila o projeto
RUN mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . -- -j$(nproc)

# Define o comando padrão para executar o programa (ajuste conforme necessário)
# CMD ["./build/nome_do_seu_executavel"]