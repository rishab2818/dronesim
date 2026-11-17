$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
ctest --test-dir (Join-Path $root 'build/native') -C Release --output-on-failure

