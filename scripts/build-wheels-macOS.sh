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
#  Created on: Mar 22, 2021
#       Author: Lyubov Miloserdova
#               miloslubov@gmail.com
#

#!/bin/bash
set -euo pipefail
export LC_ALL=C
export MACOSX_DEPLOYMENT_TARGET=10.9

cd $(dirname $(greadlink -f "${BASH_SOURCE[0]}"))/..
mkdir -p ./build ./dist ./mrob

cp ./__init__.py ./mrob/__init__.py 

cd ./build

NUMPROC=$(sysctl -n hw.ncpu)
echo "Running $NUMPROC parallel jobs"

for PYBIN in /Users/runner/hostedtoolcache/Python/3.*/x64/bin/python3.[5-9]
do
    cmake .. -DPYTHON_EXECUTABLE:FILEPATH=$PYBIN \
             -DCMAKE_MACOSX_RPATH=ON \
             -DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE \
             -DCMAKE_INSTALL_RPATH="@loader_path" 
    make -j $NUMPROC
    
    mv ../lib/* ../mrob
done

cd ../
python3 -m pip install --user -q build
python3 -m build --wheel --outdir dist/ .

