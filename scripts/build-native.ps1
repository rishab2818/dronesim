$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
cmake -S $root -B (Join-Path $root 'build/native') -DCMAKE_BUILD_TYPE=Release
cmake --build (Join-Path $root 'build/native') --config Release

