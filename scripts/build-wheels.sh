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
#  build-wheels.sh
# 
#  Created on: Nov 28, 2020
#       Author: Lyubov Miloserdova
#               miloslubov@gmail.com
#

#!/bin/bash
set -euo pipefail
export LC_ALL=C

LATEST=""

cd $(dirname $(readlink -f "${BASH_SOURCE[0]}"))
cd ../ 

mkdir ./build
mkdir ./dist
mkdir ./mrob

cp ./__init__.py ./mrob/__init__.py 

cd ./build

NUMPROC=$(grep -Ec '^processor\s+:' /proc/cpuinfo)

for PYBIN in /opt/python/cp3*/bin/
do
    LATEST=${PYBIN}

    cmake .. -DPYTHON_EXECUTABLE:FILEPATH=${PYBIN}python3
    make -j $NUMPROC
    
    mv ../lib/* ../mrob
done

chrpath -r '$ORIGIN' ../mrob/mrob.*.so
${LATEST}python3 -m pip install --user -q pep517
${LATEST}python3 -m pep517.build ../
auditwheel repair ../dist/*.whl

