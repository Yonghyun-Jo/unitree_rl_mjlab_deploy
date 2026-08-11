<#
.SYNOPSIS
  PICO 를 USB 에 꽂기만 하면 텔레옵 USB 경로를 자동 수립하는 감시자.

.DESCRIPTION
  로그온 시 자동 시작되어 백그라운드에서 adb 채널을 감시한다.
    · PICO 가 붙으면  → pico_session.ps1 실행 (PC-Service + 포트 + adb reverse 터널)
    · 터널이 사라지면 → 다시 수립  (다른 adb 도구가 서버를 재시작한 경우 등)
    · PC-Service 가 죽으면 → 다시 기동 + 새 포트로 터널 재설치
    · PICO 를 뽑았다 꽂으면 → 다시 수립
  즉 "어제는 됐는데 오늘 안 된다" 를 사람이 아니라 이 루프가 처리한다.

  ⚠ 자동화 불가능한 것 (사람 몫으로 남음):
      앱에서 127.0.0.1 로 connect / Full Body / 트래커 / T-pose 캘리브 / Send ON.
      전부 헤드셋 안 UI 라 PC 에서 건드릴 수단이 없다.
      이 스크립트가 하는 일은 "connect 를 누르면 반드시 붙는 상태"를 항상 유지하는 것까지다.

.PARAMETER Install
  로그온 자동 시작 작업으로 등록한다. 관리자 PowerShell 필요.
  등록된 작업은 '가장 높은 수준의 권한'으로 돌기 때문에, 매 부팅마다 -Tune(USB 절전 해제 등)이
  UAC 프롬프트 없이 적용된다. 이게 관리자 등록을 권하는 이유다.

.PARAMETER Uninstall
  등록 해제.

.PARAMETER Status
  등록 상태와 최근 로그를 보여준다.

.PARAMETER Com1
  publisher 대상 주소를 로그에 남겨 두기 위한 값(감시자는 publisher 를 띄우지 않는다).

.EXAMPLE
  # 최초 1회 — 관리자 PowerShell
  .\pico_autostart.ps1 -Install
  # 이후로는 PICO 를 꽂기만 하면 된다. 확인은:
  .\pico_autostart.ps1 -Status
#>

[CmdletBinding()]
param(
    [switch]$Install,
    [switch]$Uninstall,
    [switch]$Status,
    [int]$PollSec   = 3,
    [string]$Com1   = "100.121.81.113",
    [string]$LogPath = "$PSScriptRoot\autostart.log"
)

$ErrorActionPreference = "Continue"
$TaskName = "PICO teleop USB autostart"
$Self     = $PSCommandPath
$Session  = Join-Path $PSScriptRoot "pico_session.ps1"

function Log($msg) {
    $line = "{0}  {1}" -f (Get-Date -Format "MM-dd HH:mm:ss"), $msg
    Write-Host $line
    try { Add-Content -Path $LogPath -Value $line -Encoding UTF8 } catch { }
}

# ══════════════════════════════════════════════════════════════════════════════
# -Status
# ══════════════════════════════════════════════════════════════════════════════
if ($Status) {
    $t = Get-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
    if ($t) {
        $info = Get-ScheduledTaskInfo -TaskName $TaskName
        Write-Host "등록됨 : $($t.State)" -ForegroundColor Green
        Write-Host "마지막 실행 : $($info.LastRunTime)  결과 0x$('{0:X}' -f $info.LastTaskResult)"
    } else {
        Write-Host "등록 안 됨 — 관리자 PowerShell 에서 -Install" -ForegroundColor Yellow
    }
    if (Test-Path $LogPath) {
        Write-Host "`n--- 최근 로그 ($LogPath) ---" -ForegroundColor Cyan
        Get-Content $LogPath -Tail 25
    } else { Write-Host "`n로그 없음 ($LogPath)" -ForegroundColor DarkGray }
    exit 0
}

# ══════════════════════════════════════════════════════════════════════════════
# -Install / -Uninstall
# ══════════════════════════════════════════════════════════════════════════════
if ($Install -or $Uninstall) {
    $isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
               ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
    if (-not $isAdmin) {
        Write-Host "관리자 권한이 필요합니다. PowerShell 을 '관리자 권한으로 실행' 후 다시 돌리세요." -ForegroundColor Red
        exit 1
    }

    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false -ErrorAction SilentlyContinue
    if ($Uninstall) { Write-Host "등록 해제 완료." -ForegroundColor Green; exit 0 }

    if (-not (Test-Path $Session)) {
        Write-Host "pico_session.ps1 이 같은 폴더에 없습니다: $Session" -ForegroundColor Red
        exit 1
    }

    $action = New-ScheduledTaskAction -Execute "powershell.exe" `
        -Argument ("-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File `"{0}`" -Com1 {1}" -f $Self, $Com1)
    $trigger  = New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME
    $principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive -RunLevel Highest
    $settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries `
                 -ExecutionTimeLimit ([TimeSpan]::Zero) -RestartCount 3 -RestartInterval (New-TimeSpan -Minutes 1) `
                 -StartWhenAvailable

    Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $trigger `
        -Principal $principal -Settings $settings -Description "PICO USB 텔레옵 경로 자동 수립" | Out-Null

    Write-Host "등록 완료: '$TaskName'" -ForegroundColor Green
    Write-Host "  · 다음 로그온부터 자동. 지금 바로 시작하려면:" -ForegroundColor DarkGray
    Write-Host "      Start-ScheduledTask -TaskName '$TaskName'"
    Write-Host "  · 상태·로그:  .\pico_autostart.ps1 -Status"
    Write-Host "  · 해제     :  .\pico_autostart.ps1 -Uninstall  (관리자)"
    exit 0
}

# ══════════════════════════════════════════════════════════════════════════════
# 감시 루프 (기본 동작)
# ══════════════════════════════════════════════════════════════════════════════
if (-not (Test-Path $Session)) { Log "FATAL pico_session.ps1 없음: $Session"; exit 1 }

function Find-Adb {
    $c = Get-Command adb.exe -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    foreach ($p in @(
        "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe",
        "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe",
        "C:\dev\pico-capture\platform-tools\adb.exe")) { if (Test-Path $p) { return $p } }
    return $null
}

Log "=== autostart 감시 시작 (poll ${PollSec}s, com1=$Com1) ==="

$adb = Find-Adb
if (-not $adb) { Log "FATAL adb.exe 를 못 찾음"; exit 1 }
Log "adb: $adb"

$state      = "waiting"   # waiting | ready | error
$tunnels    = @()
$svcPid     = $null
$tuned      = $false      # -Tune 은 부팅당 1회면 충분(powercfg 는 영속)
$errBackoff = 0

function Invoke-Setup([bool]$withTune) {
    # ⚠ $args 는 PowerShell 자동 변수라 함수 안에서 덮어쓰지 않는다
    $psArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $Session, "-Com1", $Com1)
    if ($withTune) { $psArgs += "-Tune" }
    $out = & powershell.exe @psArgs 2>&1 | Out-String
    $code = $LASTEXITCODE
    foreach ($l in ($out -split "`r?`n")) {
        if ($l -match 'OK |WARN |FAIL |===') { Log ("  | " + $l.Trim()) }
    }
    return $code
}

while ($true) {
    Start-Sleep -Seconds $PollSec

    # ── PICO 가 붙어 있나
    $devLines = @(& $adb devices 2>$null | Select-Object -Skip 1 | Where-Object { "$_" -match '\S' })
    $present  = @($devLines | Where-Object { "$_" -match '\sdevice(\s|$)' }).Count -gt 0

    if (-not $present) {
        if ($state -ne "waiting") { Log "PICO 분리됨 — 대기 상태로" }
        $state = "waiting"; $tunnels = @(); $svcPid = $null; $errBackoff = 0
        continue
    }

    # ── 붙어 있는데 아직 수립 안 함
    if ($state -ne "ready") {
        if ($state -eq "error") {
            $errBackoff++
            if ($errBackoff % 10 -ne 0) { continue }   # 실패 시 30초 간격으로 재시도
        }
        Log "PICO 감지 → USB 경로 수립"
        Start-Sleep -Seconds 2                          # 열거 직후 adb 가 불안정한 구간 회피
        $code = Invoke-Setup (-not $tuned)
        if ($code -eq 0) {
            $tuned   = $true
            $state   = "ready"
            $tunnels = @((& $adb reverse --list 2>$null) -split "`r?`n" |
                         ForEach-Object { if ("$_" -match 'tcp:(\d+)') { $Matches[1] } })
            $svcPid  = (Get-Process -ErrorAction SilentlyContinue |
                        Where-Object { $_.ProcessName -match "RoboticsServiceProcess|run3D|XRobo" } |
                        Select-Object -First 1).Id
            Log "READY — 앱 IP 칸에 127.0.0.1. 터널 $($tunnels -join ', ') / PC-Service PID $svcPid"
            $errBackoff = 0
        } else {
            $state = "error"
            Log "수립 실패(exit $code) — 30초 후 재시도. 케이블/개발자모드/PC-Service 확인"
        }
        continue
    }

    # ── ready 상태 유지 감시: 터널·서비스가 살아 있나
    $alive = & $adb reverse --list 2>$null | Out-String
    $lost  = @($tunnels | Where-Object { $alive -notmatch "tcp:$_\s" })
    if ($lost.Count -gt 0) { Log "터널 $($lost -join ', ') 소실 → 재수립"; $state = "error"; $errBackoff = 9; continue }

    if ($svcPid -and -not (Get-Process -Id $svcPid -ErrorAction SilentlyContinue)) {
        Log "PC-Service 종료됨 → 재기동 (포트가 바뀌므로 터널도 다시)"
        $state = "error"; $errBackoff = 9; continue
    }
}
