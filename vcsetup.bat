rmdir /q /s build
mkdir build

cd build 
cmake.exe -G "Visual Studio 15 2017" -A x64 ..
