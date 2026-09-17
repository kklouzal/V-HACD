@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
cd /d "%~dp0"
cl /nologo /std:c++20 /permissive- /O2 /W4 /WX /EHsc /MD /DNDEBUG /I..\include /Fecontracttests.exe ContractTests.cpp
