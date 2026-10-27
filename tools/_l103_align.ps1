param([string]$Cmd="A", [double]$Pre=2.5, [double]$Secs=12.0)
$ErrorActionPreference="Continue"
try{
 $sp=New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
 $sp.Encoding=[System.Text.Encoding]::UTF8
 $sp.ReadTimeout=300
 $sp.Open(); Start-Sleep -Milliseconds 300
 $t0=Get-Date
 while(((Get-Date)-$t0).TotalSeconds -lt $Pre){ try{ $l=$sp.ReadLine(); if($l.Trim()){ "[boot] $l" } }catch{} }
 $sp.DiscardInBuffer()
 $sp.Write($Cmd+"`r")
 $t1=Get-Date
 while(((Get-Date)-$t1).TotalSeconds -lt $Secs){ try{ $l=$sp.ReadLine()
   if($l -match 'ALIGN|Align|mid_angle|movement|sensor_direction|PP check|Zero elec|Enable driver|FAULT: act|Error'){ "{0,5:N1}s | {1}" -f ((Get-Date)-$t1).TotalSeconds,$l }
 }catch{} }
 $sp.Close()
}catch{ "failed: $($_.Exception.Message)" }
