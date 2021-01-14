# Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
# 
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# 
# 
#  Dockerfile
# 
#  Created on: Oct 15, 2020
#       Author: Lyubov Miloserdova
#               miloslubov@gmail.com
#

FROM quay.io/pypa/manylinux2010_x86_64
RUN yum install -y chrpath && yum remove cmake -y && yum clean all && rm -rf /var/lib/{yum,rpm}
RUN set -xueo pipefail \
 && export NUMPROC=$(grep -Ec '^processor\s+:' /proc/cpuinfo) \
 && cd /usr/src \
 && curl -OL https://cmake.org/files/v3.6/cmake-3.6.2.tar.gz \
 && tar -zxvf ./cmake-3.6.2.tar.gz \
 && cd ./cmake-3.6.2 \
 && ./bootstrap --parallel=$NUMPROC --prefix=/usr/local \
 && make -j $NUMPROC \
 && make install/strip -j $NUMPROC \
 && rm -rf /usr/src/cmake-3.6.2*

RUN echo "export PATH=/usr/local/bin:$PATH:$HOME/bin" >> ~/.bashrc
ENV BASH_ENV /etc/profile
