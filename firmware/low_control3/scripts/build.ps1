param(
    [ValidateSet('Debug','Release')][string]$Configuration='Release',
    [switch]$SkipTests
)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path $PSScriptRoot -Parent
$originalPath=$env:PATH
function Run-Checked([string]$Program,[string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}
try {
    if (-not (Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue)) {
        $toolchainBin=$env:ARM_GNU_TOOLCHAIN
        if (-not $toolchainBin) {
            $bundleRoot=Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\gnu-tools-for-stm32'
            $compiler=Get-ChildItem -LiteralPath $bundleRoot -Filter arm-none-eabi-gcc.exe -Recurse -ErrorAction SilentlyContinue |
                Sort-Object FullName -Descending | Select-Object -First 1
            if ($compiler) { $toolchainBin=$compiler.DirectoryName }
        }
        if (-not $toolchainBin) { throw 'ARM compiler missing. Put arm-none-eabi-gcc in PATH or set ARM_GNU_TOOLCHAIN to its bin directory.' }
        $env:PATH="$toolchainBin;$env:PATH"
    }
    $cmake=(Get-Command cmake -ErrorAction Stop).Source
    Run-Checked $cmake @('--preset',$Configuration,'-S',$projectRoot)
    Run-Checked $cmake @('--build',(Join-Path $projectRoot "build/$Configuration"),'--parallel','2')
    if (-not $SkipTests) {
        $hostCompiler=(Get-Command g++ -ErrorAction Stop).Source
        $hostBuild=Join-Path $projectRoot 'build/host'
        Run-Checked $cmake @('-S',(Join-Path $projectRoot 'tests'),'-B',$hostBuild,'-G','Ninja',"-DCMAKE_CXX_COMPILER=$hostCompiler")
        Run-Checked $cmake @('--build',$hostBuild,'--parallel','2')
        Run-Checked (Get-Command ctest -ErrorAction Stop).Source @('--test-dir',$hostBuild,'--output-on-failure')
    }
    Write-Host "Firmware: $projectRoot/build/$Configuration/low_control3.{elf,hex,bin}"
    Write-Host 'Build only: no board was flashed.'
} finally { $env:PATH=$originalPath }
