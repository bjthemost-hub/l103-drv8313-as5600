# l103_seq.ps1 -- L103 台架「往返运动」的标准命令
#
#   .\l103_seq.ps1                    往返(默认): 0 -> +1圈 -> +2圈 -> 回机械0, 100rpm
#   .\l103_seq.ps1 -Preset home       直接回机械0
#   .\l103_seq.ps1 -Preset list       列出所有模板
#   .\l103_seq.ps1 -DryRun            只算分段打印出来, 一个字节都不发(先看再发)
#   .\l103_seq.ps1 -Preset nudge      0 -> +30度 -> 回0 (小幅试探)
#   .\l103_seq.ps1 -Segs "3.1416:60,0:60"   自己排段(机械坐标 rad:rpm)
#
# 注意: 段的目标是【机械坐标绝对位置, 单位 rad】, 不是相对量、不是圈。
#       "回机械0" 是回 UART F / CAN 0x112 设过的那个零点。
# 参数一露头就会真动轴 -- 先用 -DryRun 看一眼分段。
param(
  [string]$Preset  = "roundtrip",
  [double]$Dwell   = 0.5,
  [double]$Timeout = 15,
  [string]$Segs    = "",
  [switch]$DryRun
)
$ErrorActionPreference = "Continue"

$py = $null
foreach ($c in @((Get-Command python.exe -ErrorAction SilentlyContinue).Source,
                 "C:\Python314\python.exe",
                 "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe")) {
  if ($c -and (Test-Path -LiteralPath $c)) { $py = $c; break }
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$logDir    = Join-Path (Split-Path -Parent $scriptDir) "日志"
if (-not $py) {
  "找不到 python.exe -- 手工跑: python `"$scriptDir\l103_can.py`" seq --preset $Preset"
  exit 1
}

$argl = @("$scriptDir\l103_can.py", "seq", "--dwell", $Dwell, "--timeout", $Timeout,
          "--save", (Join-Path $logDir ("seq_" + $Preset + ".txt")))
if ($Segs) { $argl += @("--segs",   $Segs)   }
else       { $argl += @("--preset", $Preset) }
if ($DryRun) { $argl += "--dry-run" }

"python = $py"
"python $($argl -join ' ')"
& $py @argl
"EXIT=$LASTEXITCODE"
