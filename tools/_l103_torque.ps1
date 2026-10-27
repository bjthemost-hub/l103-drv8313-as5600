param([double]$Volts=2.5, [double]$Secs=6.0)
$ErrorActionPreference="Continue"
try{
 $sp=New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
 $sp.Encoding=[System.Text.Encoding]::UTF8
 $sp.ReadTimeout=400
 $sp.Open(); Start-Sleep -Milliseconds 400; $sp.DiscardInBuffer()
 $sp.Write("M0`r");     Start-Sleep -Milliseconds 250
 $sp.Write("T$Volts`r")
 $t0=Get-Date
 while(((Get-Date)-$t0).TotalSeconds -lt $Secs){
   try{ $l=$sp.ReadLine()
     if($l -match 'CS:|AS5600: raw|TTARGET|TARGET|MODE|FAULT: act|MOT: ') { "{0,5:N1}s | {1}" -f ((Get-Date)-$t0).TotalSeconds,$l }
   }catch{}
 }
 $sp.Write("T0`r")
 $t1=Get-Date
 while(((Get-Date)-$t1).TotalSeconds -lt 1.5){ try{ $l=$sp.ReadLine(); if($l -match 'TARGET|CS:'){ "{0,5:N1}s | {1}" -f ((Get-Date)-$t1).TotalSeconds,$l } }catch{} }
 $sp.Close()
}catch{ "failed: $($_.Exception.Message)" }
