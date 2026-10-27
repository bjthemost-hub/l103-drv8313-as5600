$ErrorActionPreference = "Continue"
$OCD = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin/openocd.exe"
$BIN = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin"
Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Milliseconds 400
$oa = @("-f","$BIN/wch-riscv.cfg","-s",$BIN,
        "-c","chip_id CH32L10x",
        "-c","init",
        "-c","halt",
        "-c","reset run",
        "-c","exit")
& $OCD @oa 2>&1 | Select-Object -Last 12
"EXIT=$LASTEXITCODE"
