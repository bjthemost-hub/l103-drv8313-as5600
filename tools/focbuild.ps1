param([Parameter(Mandatory=$true)][string]$Proj)
$ErrorActionPreference = "Continue"
$TC   = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC/bin"
$MK   = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/others/Build_Tools/Make/bin"
$env:Path = "$TC;$MK;" + $env:Path
Set-Location -LiteralPath $Proj
& make.exe 2>&1 | Select-Object -Last 40
"MAKE_EXIT=$LASTEXITCODE"
