param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
$root=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Push-Location $root
try {
    New-Item -ItemType Directory -Force build | Out-Null
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Itests/mock -ICore/Inc Core/Src/mhz19.c tests/mhz19_test.c -o build/mhz19_test.exe
    if($LASTEXITCODE -ne 0) { throw 'MH-Z19 test build failed.' }
    & ./build/mhz19_test.exe
    if($LASTEXITCODE -ne 0) { throw 'MH-Z19 test failed.' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -DNRF24_HOST_TEST -ICore/Inc Core/Src/NRF24L01.c tests/radio_test.c -o build/radio_test.exe
    if($LASTEXITCODE -ne 0) { throw 'Host test build failed.' }
    & ./build/radio_test.exe
    if($LASTEXITCODE -ne 0) { throw 'Radio simulation failed.' }
    & ./build/radio_test.exe wrap
    if($LASTEXITCODE -ne 0) { throw 'Tick-wrap simulation failed.' }
} finally { Pop-Location }
