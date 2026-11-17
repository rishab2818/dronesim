# Build and run

## Prerequisites

Install Node.js LTS with npm, CMake 3.20+, a C11 compiler (GCC, Clang, or MSVC), and the Emscripten SDK. Activate Emscripten in each new shell. The application needs no network at runtime.

### Windows PowerShell

```powershell
git clone https://github.com/emscripten-core/emsdk.git $env:USERPROFILE/.emsdk
& $env:USERPROFILE/.emsdk/emsdk.bat install latest
& $env:USERPROFILE/.emsdk/emsdk.bat activate latest
& $env:USERPROFILE/.emsdk/emsdk_env.ps1
./scripts/build-native.ps1
./scripts/test.ps1
./scripts/build-wasm.ps1
cd web
npm install
npm test
npm run dev
```

Open the localhost URL printed by Vite. Do not use `file://`.

### Linux/macOS shell

```sh
git clone https://github.com/emscripten-core/emsdk.git "$HOME/.emsdk"
"$HOME/.emsdk/emsdk" install latest
"$HOME/.emsdk/emsdk" activate latest
. "$HOME/.emsdk/emsdk_env.sh"
./scripts/build-native.sh
./scripts/test.sh
./scripts/build-wasm.sh
cd web
npm ci
npm test
npm run dev
```

For a production static bundle, run `npm run build`; output is in `web/dist`. The generated Wasm assets must already exist in `web/src/wasm/generated` before the Vite build.

## Minimal GCC fallback

If CMake is not yet installed, compile the tests directly from the repository root:

```powershell
$src = Get-ChildItem engine/src/*.c | ForEach-Object FullName
gcc -std=c11 -Wall -Wextra -Wpedantic -Iengine/include $src engine/tests/test_suite.c -lm -o drone_tests.exe
./drone_tests.exe
```

