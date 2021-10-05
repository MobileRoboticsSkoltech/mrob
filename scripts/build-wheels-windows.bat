set msvc_redist_path='C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Redist\MSVC\14.29.30133\x64\Microsoft.VC142.CRT'
mkdir .\build 
mkdir .\mrob
cp __init__.py .\mrob\__init__.py
cp %msvc_redist_path%\msvcp140.dll .\mrob\msvcp140.dll
cp %msvc_redist_path%\vcruntime140.dll .\mrob\vcruntime140.dll 
cp %msvc_redist_path%\vcruntime140_1.dll .\mrob\vcruntime140_1.dll 

for /D %%P in (C:\hostedtoolcache\windows\Python\3*) do CALL :build %%P\x64\python.exe

python -m pip install --user -q build
python -m build --wheel --outdir dist .

EXIT /B %ERRORLEVEL% 

:build
cmake -S . -B build -G "Visual Studio 16 2019" -A "x64" -DPYTHON_EXECUTABLE:FILEPATH=%~1 -DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=%cd%\mrob -DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE=%cd%\mrob 
cmake --build build --config Release -j %NUMBER_OF_PROCESSORS%
EXIT /B 0