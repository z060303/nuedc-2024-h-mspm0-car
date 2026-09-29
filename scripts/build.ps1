param(
    [string]$CcsRoot = 'D:\TI\CCS',
    [string]$SdkRoot = 'D:\TI\CCS\mspm0_sdk_2_10_00_04'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildDir = Join-Path $projectRoot 'build\Debug'
$compiler = Join-Path $CcsRoot 'ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe'
$sysconfig = Join-Path $CcsRoot 'sysconfig_1.26.2\sysconfig_cli.bat'
$product = Join-Path $SdkRoot '.metadata\product.json'
$sdkSource = Join-Path $SdkRoot 'source'
$startup = Join-Path $sdkSource 'ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c'

foreach ($path in @($compiler, $sysconfig, $product, $startup)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Missing tool or SDK file: $path" }
}
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

& $sysconfig -s $product --script (Join-Path $projectRoot '24_H.syscfg') -o $buildDir --compiler ticlang
if ($LASTEXITCODE -ne 0) { throw 'SysConfig generation failed' }

$includes = @(
    $buildDir, $sdkSource, (Join-Path $sdkSource 'third_party\CMSIS\Core\Include'),
    $projectRoot, (Join-Path $projectRoot 'comm1'), (Join-Path $projectRoot 'comm2'),
    (Join-Path $projectRoot 'motor'), (Join-Path $projectRoot 'pid'),
    (Join-Path $projectRoot 'Drivers\MSPM0')
)
$compileArgs = @('-c', '-march=thumbv6m', '-mcpu=cortex-m0plus', '-mfloat-abi=soft',
    '-mlittle-endian', '-mthumb', '-O0', '-Wall', '-D__MSPM0G3507__', '-D__USE_SYSCONFIG__')
foreach ($include in $includes) { $compileArgs += "-I$include" }

$sources = @(
    (Join-Path $projectRoot '24_H.c'),
    (Join-Path $projectRoot 'comm1\hwt101.c'),
    (Join-Path $projectRoot 'comm2\comm2.c'),
    (Join-Path $projectRoot 'motor\motor.c'),
    (Join-Path $projectRoot 'pid\pid.c'),
    (Join-Path $projectRoot 'Drivers\MSPM0\clock.c'),
    (Join-Path $projectRoot 'Drivers\MSPM0\interrupt.c'),
    (Join-Path $buildDir 'ti_msp_dl_config.c'),
    $startup
)
$objects = @()
for ($i = 0; $i -lt $sources.Count; $i++) {
    $object = Join-Path $buildDir ("source_{0:D2}.o" -f $i)
    & $compiler @compileArgs '-o' $object $sources[$i]
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($sources[$i])" }
    $objects += $object
}

$output = Join-Path $buildDir 'nuedc_2024_h_mspm0_car.out'
$toolchainLib = Join-Path $CcsRoot 'ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\lib'
$linkArgs = @('-march=thumbv6m', '-mcpu=cortex-m0plus', '-mfloat-abi=soft', '-mthumb',
    '-Wl,--rom_model', "-Wl,-i$sdkSource", "-Wl,-i$buildDir", "-Wl,-i$toolchainLib",
    '-o', $output) + $objects + @(
    "-Wl,-l$(Join-Path $buildDir 'device_linker.cmd')", '-Wl,-ldevice.cmd.genlibs',
    '-Wl,-llibc.a')
& $compiler @linkArgs
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
Write-Output "Built: $output"
