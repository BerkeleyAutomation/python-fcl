# NOTE: this docker file is only for local testing of the build
# example of testing a build locally:
# docker build . -t pythonfcl -f build_dependencies/Dockerfile

FROM quay.io/pypa/manylinux2010_x86_64:latest

# build and install libccd and fcl using cmake
COPY build_dependencies/install_linux.sh .
RUN bash install_linux.sh

# manylinux includes a bunch of pythons
# to test with others change this env variable
#ENV PATH=/opt/python/cp27-cp27m/bin:$PATH
ENV PATH=/opt/python/cp39-cp39/bin:$PATH

# we need numpy to build python-fcl
# since we set our path we'll be using the right pip
RUN pip install numpy cython

# build the python-fcl module
COPY . /python_fcl
RUN pip wheel /python_fcl --no-deps -w wheelhouse/
RUN pip install python_fcl --no-index -f /wheelhouse
RUN ls /wheelhouse
RUN auditwheel repair wheelhouse/python_fcl-0.7.0-cp39-cp39-linux_x86_64.whl -w /wheelhouse
