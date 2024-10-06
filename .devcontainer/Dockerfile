FROM ghcr.io/shc-astra/rover-basestation-docker@sha256:c916b6da52871ab78b33075477e40e5c16de81dc5433e80ad951640bbd286630

RUN apt update

# Install base utilities
#RUN apt install -y build-essential
# Install miniconda
ENV CONDA_DIR /opt/conda
RUN curl 'https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh' --output ~/miniconda.sh \
    && bash ~/miniconda.sh -b -p /opt/conda
# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH
# Make conda activate always run
RUN echo 'conda activate' >> .bashrc

# Install development tools
RUN apt install -y git
RUN conda install -c conda-forge pyarrow
RUN apt install -y black
