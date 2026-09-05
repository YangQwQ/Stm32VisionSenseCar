# Host-side unit tests: build & run UartFrame / Dispatch on PC (no hardware)
# Requires MinGW gcc.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path   # .../tools/host_test
$proj = Split-Path -Parent (Split-Path -Parent $root)      # .../tools then project root
$gcc  = 'C:\mingw64\bin\gcc.exe'
$exe1 = Join-Path $root 'test_uartframe.exe'
$exe2 = Join-Path $root 'test_dispatch.exe'

Write-Host "[build] test_uartframe" -ForegroundColor Cyan
$args1 = @('-std=c99', "-I$root\inc", "-I$proj\Control", "-I$proj\Hardware",
           '-Wall', '-Wextra', "$proj\Control\UartFrame.c", "$root\test_uartframe.c", '-o', $exe1)
& $gcc $args1
if ($LASTEXITCODE -ne 0) { Write-Host "build uartframe FAIL" -ForegroundColor Red; exit 1 }

Write-Host "[build] test_dispatch" -ForegroundColor Cyan
$args2 = @('-std=c99', "-I$root\inc", "-I$proj\Control", "-I$proj\Hardware",
           '-Wall', '-Wextra', "$proj\Control\Dispatch.c", "$root\test_dispatch.c", "$root\stubs.c", '-o', $exe2)
& $gcc $args2
if ($LASTEXITCODE -ne 0) { Write-Host "build dispatch FAIL" -ForegroundColor Red; exit 1 }

$ok = $true
Write-Host "`n[run] test_uartframe" -ForegroundColor Cyan
& $exe1
if ($LASTEXITCODE -ne 0) { $ok = $false }

Write-Host "`n[run] test_dispatch" -ForegroundColor Cyan
& $exe2
if ($LASTEXITCODE -ne 0) { $ok = $false }

Write-Host ""
if ($ok) { Write-Host "ALL EJECTED TESTS PASSED" -ForegroundColor Green } else { Write-Host "SOME TESTS FAILED" -ForegroundColor Red }
exit $(if ($ok) { 0 } else { 1 })