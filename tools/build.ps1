param([string]$ToolchainBin='')
$ErrorActionPreference='Stop'
$root=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if(-not $ToolchainBin) {
    $command=Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue
    if($command) { $ToolchainBin=Split-Path $command.Source }
    else {
        $command=Get-ChildItem 'C:/ST' -Recurse -Filter arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue | Select-Object -First 1
        if($command) { $ToolchainBin=$command.DirectoryName }
    }
}
if(-not $ToolchainBin) { throw 'Pass -ToolchainBin with the GNU Arm Embedded bin directory.' }
Push-Location $root
try {
    New-Item -ItemType Directory -Force build | Out-Null
    $cc=Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
    $common=@('-mcpu=cortex-m0','-mthumb','-Os','-g3','-flto','-ffunction-sections','-fdata-sections',
        '-fno-unwind-tables','-fno-asynchronous-unwind-tables','-DUSE_HAL_DRIVER','-DSTM32F030x6',
        '-ICore/Inc','-IDrivers/STM32F0xx_HAL_Driver/Inc','-IDrivers/STM32F0xx_HAL_Driver/Inc/Legacy',
        '-IDrivers/CMSIS/Device/ST/STM32F0xx/Include','-IDrivers/CMSIS/Include')
    $sources=@(Get-ChildItem Core/Src -Filter '*.c')+@(Get-ChildItem Drivers/STM32F0xx_HAL_Driver/Src -Filter '*.c' | Where-Object { $_.Name -notlike '*template*' })
    $objects=@()
    foreach($source in $sources) {
        $object=Join-Path 'build' ($source.BaseName+'.o')
        & $cc @common -std=c11 -Wall -Wextra -c $source.FullName -o $object
        if($LASTEXITCODE -ne 0) { throw "Compilation failed: $($source.Name)" }
        $objects+=$object
    }
    $startup='Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f030x6.s'
    & $cc @common -c $startup -o build/startup.o
    if($LASTEXITCODE -ne 0) { throw 'Startup assembly failed.' }
    $objects+='build/startup.o'
    & $cc @common @objects '-Ttools/stm32f030f4.ld' '--specs=nano.specs' '--specs=nosys.specs' '-Wl,--gc-sections,-Map=build/CO2_Node.map' -o build/CO2_Node.elf
    if($LASTEXITCODE -ne 0) { throw 'Link failed: flash/RAM budgets are 16 KB / 4 KB including 512-byte reserved stack.' }
    & (Join-Path $ToolchainBin arm-none-eabi-size.exe) build/CO2_Node.elf
    & (Join-Path $ToolchainBin arm-none-eabi-objcopy.exe) -O ihex build/CO2_Node.elf build/CO2_Node.hex
    if($LASTEXITCODE -ne 0) { throw 'HEX creation failed.' }
    & (Join-Path $ToolchainBin arm-none-eabi-objcopy.exe) -O binary build/CO2_Node.elf build/CO2_Node.bin
    if($LASTEXITCODE -ne 0) { throw 'BIN creation failed.' }
} finally { Pop-Location }
