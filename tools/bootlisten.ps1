$ErrorActionPreference="Continue"
$OCD = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin/openocd.exe"
$BIN = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin"
Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Milliseconds 300
$sp = New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
$sp.Encoding = [System.Text.Encoding]::UTF8
$sp.ReadTimeout = 500
$sp.Open()
$sb = New-Object System.Text.StringBuilder
$t0 = Get-Date
# 先读 0.4s 清掉缓存
while (((Get-Date) - $t0).TotalSeconds -lt 0.4) { try { [void]$sp.ReadExisting() } catch {} }
# 复位
$oa = @("-f","$BIN/wch-riscv.cfg","-s",$BIN,"-c","chip_id CH32L10x","-c","init","-c","reset run","-c","exit")
& $OCD @oa 2>&1 | Out-Null
$t0 = Get-Date
while (((Get-Date) - $t0).TotalSeconds -lt 6) {
  try { $line = $sp.ReadLine(); [void]$sb.AppendLine($line) } catch {}
}
$sp.Close()
$sb.ToString()
