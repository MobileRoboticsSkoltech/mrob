SETLOCAL EnableDelayedExpansion

#Early check for build tools
cmake --version || EXIT /B !ERRORLEVEL!

pushd %~dp0 
cd .. 

mkdir .\build || popd && EXIT /B !ERRORLEVEL!
mkdir .\mrob || popd && EXIT /B !ERRORLEVEL!
cp __init__.py .\mrob\__init__.py || popd && EXIT /B !ERRORLEVEL!


for /D %%P in (C:\hostedtoolcache\windows\Python\3*) do CALL :build %%P\x64\python.exe || popd && EXIT /B !ERRORLEVEL!


python -m pip install --user -q build || popd && EXIT /B !ERRORLEVEL!
python -m build --wheel --outdir dist . || popd && EXIT /B !ERRORLEVEL!

popd
EXIT /B 0

:build
cmake -S . -B build -G "Visual Studio 16 2019" -A "x64" -DPYTHON_EXECUTABLE:FILEPATH=%~1 -DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=%cd%\mrob -DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE=%cd%\mrob || EXIT /B !ERRORLEVEL!
cmake --build build --config Release -j %NUMBER_OF_PROCESSORS% || EXIT /B !ERRORLEVEL!
EXIT /B 0
