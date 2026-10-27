param([string]$Cmd="S", [double]$Secs=4.0, [int]$Repeat=1)
$ErrorActionPreference="Continue"
try{
 $sp=New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
 $sp.Encoding=[System.Text.Encoding]::UTF8
 $sp.ReadTimeout=400
 $sp.Open(); Start-Sleep -Milliseconds 400; $sp.DiscardInBuffer()
 foreach($k in 1..$Repeat){ $sp.Write($Cmd+"`r"); Start-Sleep -Milliseconds 250 }
 $t0=Get-Date
 while(((Get-Date)-$t0).TotalSeconds -lt $Secs){ try{ $l=$sp.ReadLine(); "{0,5:N1}s | {1}" -f ((Get-Date)-$t0).TotalSeconds,$l }catch{} }
 $sp.Close()
}catch{ "failed: $($_.Exception.Message)" }
