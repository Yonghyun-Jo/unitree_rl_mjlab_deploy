<#
.SYNOPSIS
  PICO 텔레옵 세션 준비 — USB 전용 (윈도우 노트북 쪽).

.DESCRIPTION
  PICO ↔ 노트북 링크를 USB 로 고정하고, 매 세션 손으로 하던 것을 한 번에 처리한다.

    0) USB 링크 품질 사전점검 (adb 디바이스, USB 3.x 여부, 케이블/허브)
    1) PC-Service(RoboticsServiceProcess.exe) 확인 → 없으면 기동 + 리스닝까지 대기
    2) 그 PID 가 실제로 듣는 포트 조회  ⚠ 동적 할당이라 기동할 때마다 바뀐다
    3) 그 포트마다 `adb reverse` 터널 설치 → 앱의 127.0.0.1 이 노트북으로 넘어옴
    4) (-Tune) USB/전원/우선순위 튜닝으로 링크 품질 고정
    5) (-Publish) publisher 기동

  WiFi 폴백은 없다. USB 경로가 안 서면 그 자리에서 멈추고 원인을 출력한다.

  ── 왜 매일 다시 해야 하나 (= 이 스크립트의 존재 이유)
    · PC-Service 는 윈도우 서비스가 아니라 콘솔 앱 → 재부팅하면 안 뜬다
    · 리스닝 포트는 동적 할당 → 어제의 포트 번호는 오늘 무효
    · adb reverse 터널은 재부팅 / USB 재연결 / adb kill-server 로 사라진다
  셋 다 세션마다 재수립이 필요하고, 하나라도 빠지면 앱에서 "TCP failed" 로만 보인다.

  ── USB 경로의 구조적 한계 (알고 있어야 함)
    adb reverse 는 TCP 만 포워딩한다. PC-Service 가 UDP 포트도 열고 있고 pose 스트림이
    그쪽을 타면 USB 로는 안 넘어온다. 증상 = "connect 는 되는데 pose 가 계속 0.0".
    스크립트가 UDP 포트를 발견하면 경고하고, §5 판정표로 구분하게 해 준다.

.PARAMETER Com1
  브릿지가 도는 제어PC 주소. publisher 의 --com1 으로 넘어간다.
    com1 LAN        192.168.50.211
    com1 Tailscale  100.121.81.113
    실로봇 온보드    <온보드 IP>
  ※ 이건 노트북→제어PC 구간이고, USB 고정은 PICO→노트북 구간이다. 서로 무관.

.PARAMETER CheckOnly   아무것도 바꾸지 않고 진단만 출력.
.PARAMETER Restart     떠 있는 PC-Service 를 죽이고 새로 띄운다 (포트가 바뀌므로 최후수단).
.PARAMETER Tune        USB/전원/프로세스 우선순위 튜닝 적용. ⚠ 관리자 권한 필요.
.PARAMETER Publish     준비 후 pico_publisher.py 까지 실행.

.EXAMPLE
  .\pico_session.ps1 -CheckOnly                     # 뭐가 막혔는지만 보기
  .\pico_session.ps1                                # USB 경로 수립 (매 세션 이거 하나)
  .\pico_session.ps1 -Tune                          # 최초 1회 + 재부팅 후 (관리자 PowerShell)
  .\pico_session.ps1 -Com1 192.168.50.211 -Publish
#>

[CmdletBinding()]
param(
    [string]$Com1        = "100.121.81.113",
    [string]$CaptureDir  = "C:\dev\pico-capture",
    [string]$CondaEnv    = "pico",
    [switch]$CheckOnly,
    [switch]$Restart,
    [switch]$Tune,
    [switch]$Publish
)

# ⚠ "Stop" 으로 두지 않는다. PowerShell 7 에서는 native 명령(adb 등)이 stderr 에 쓴 것이
#    2>&1 로 합쳐질 때 ErrorRecord 가 되어 스크립트를 죽인다("* daemon started successfully" 조차).
#    아래는 전부 반환값·존재를 명시적으로 검사하므로 Continue 가 맞다.
$ErrorActionPreference = "Continue"

function Step($n, $msg) { Write-Host "`n[$n] $msg" -ForegroundColor Cyan }
function Ok  ($msg)     { Write-Host "    OK   $msg" -ForegroundColor Green }
function Warn($msg)     { Write-Host "    WARN $msg" -ForegroundColor Yellow }
function Bad ($msg)     { Write-Host "    FAIL $msg" -ForegroundColor Red }

# CheckOnly 일 때는 죽지 않고 계속 진단한다 (뭐가 더 막혀 있는지 한 번에 보려고)
$script:Blocked = $false
function Die ($msg) {
    Bad $msg
    if ($CheckOnly) { $script:Blocked = $true; return }
    Write-Host "`n=== USB 경로를 세우지 못했습니다. 위 FAIL 을 해결하고 다시 실행하세요. ===" -ForegroundColor Red
    exit 1
}

$IsAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
           ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

Write-Host "=== PICO teleop session setup  (USB ONLY) ===" -ForegroundColor White
Write-Host "    capture dir     : $CaptureDir"
Write-Host "    제어PC (publisher 대상) : $Com1"
if ($CheckOnly) { Write-Host "    mode : CHECK-ONLY" -ForegroundColor Yellow }

function Find-Adb {
    $c = Get-Command adb.exe -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    foreach ($p in @(
        "$CaptureDir\platform-tools\adb.exe",
        "$CaptureDir\pcservice\adb.exe",
        "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe",
        "C:\platform-tools\adb.exe")) { if (Test-Path $p) { return $p } }
    $f = Get-ChildItem -Path $CaptureDir -Recurse -Filter adb.exe -ErrorAction SilentlyContinue |
         Select-Object -First 1
    if ($f) { return $f.FullName }
    return $null
}

function Find-Service {
    foreach ($p in @(
        "$CaptureDir\pcservice\RoboticsServiceProcess.exe",
        "$CaptureDir\pcservice\run3D.exe")) { if (Test-Path $p) { return $p } }
    $f = Get-ChildItem -Path $CaptureDir -Recurse -Filter RoboticsServiceProcess.exe -ErrorAction SilentlyContinue |
         Select-Object -First 1
    if ($f) { return $f.FullName }
    return $null
}

function Get-ServiceProcs {
    Get-Process -ErrorAction SilentlyContinue |
        Where-Object { $_.ProcessName -match "RoboticsServiceProcess|run3D|XRobo" }
}

# ══════════════════════════════════════════════════════════════════════════════
# 0. USB 링크 (여기가 실패하면 그 뒤는 전부 무의미하므로 먼저 본다)
# ══════════════════════════════════════════════════════════════════════════════
Step 0 "USB 링크"

$adb       = Find-Adb
$devSerial = $null

if (-not $adb) {
    Die @"
adb.exe 를 못 찾았습니다. USB 경로는 adb 없이는 불가능합니다.
  · Android platform-tools 를 받아 $CaptureDir\platform-tools\ 에 풀거나
  · -CaptureDir 로 실제 위치를 지정하세요.
"@
} else {
    Write-Host "    adb: $adb"

    # adb 서버를 미리 띄워 첫 명령의 지연/타임아웃을 없앤다
    & $adb start-server 2>&1 | Out-Null

    $devLines = @(& $adb devices -l 2>&1 | Select-Object -Skip 1 | Where-Object { "$_" -match '\S' })
    $devices  = @($devLines | Where-Object { "$_" -match '\sdevice(\s|$)' })
    $unauth   = @($devLines | Where-Object { "$_" -match 'unauthorized' })
    $offline  = @($devLines | Where-Object { "$_" -match 'offline' })

    if ($unauth.Count -gt 0) {
        Die @"
헤드셋이 'unauthorized' 입니다.
  → 헤드셋을 착용하고 'USB 디버깅을 허용하시겠습니까?' 팝업에서 [항상 허용] + [확인].
    팝업이 안 보이면 케이블을 뽑았다 다시 꽂으세요.
"@
    } elseif ($offline.Count -gt 0) {
        Die "헤드셋이 'offline' 입니다. → adb kill-server 후 케이블 재연결, 그리고 다시 실행."
    } elseif ($devices.Count -eq 0) {
        Die @"
adb 에 PICO 가 안 보입니다. USB 전용이므로 여기서 진행 불가.
  체크리스트 (위에서부터):
   1. 케이블이 '충전 전용'이 아닌가 — PICO 동봉 케이블 또는 USB 3.x 데이터 케이블
   2. USB 허브/도킹을 거치지 않는가 — 노트북 본체 포트에 직결
   3. 헤드셋: 설정 > 일반 > 개발자 모드 ON, USB 디버깅 ON
   4. 헤드셋 USB 모드가 '충전만'으로 잡히지 않았는가 (알림에서 파일전송/MTP 로 변경)
   5. adb kill-server 후 재시도
"@
    } else {
        $devSerial = ("$($devices[0])" -split '\s+')[0]
        Ok "PICO 연결됨: $devSerial"

        # USB 3.x 여부 — 노트북 본체 직결/3.0 포트인지 간접 확인 (실패해도 무해)
        try {
            $usbDev = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
                      Where-Object { $_.InstanceId -match [regex]::Escape($devSerial) } | Select-Object -First 1
            if ($usbDev) {
                $parentId = (Get-PnpDeviceProperty -InstanceId $usbDev.InstanceId `
                             -KeyName 'DEVPKEY_Device_Parent' -ErrorAction SilentlyContinue).Data
                if     ($parentId -match 'USB4|XHCI|ROOT_HUB30') { Ok   "USB 3.x 컨트롤러에 물려 있습니다 ($parentId)" }
                elseif ($parentId)                               { Warn "상위 허브: $parentId — USB 2.0 이거나 외장 허브일 수 있습니다. 본체 USB 3.x 포트 직결 권장." }
            }
        } catch { }
    }
}

# ══════════════════════════════════════════════════════════════════════════════
# 1. PC-Service
# ══════════════════════════════════════════════════════════════════════════════
Step 1 "PC-Service"

$procs = Get-ServiceProcs

if ($procs -and $Restart -and -not $CheckOnly) {
    Warn "-Restart → 기존 PC-Service 종료 (포트가 바뀌므로 터널도 다시 겁니다)"
    $procs | Stop-Process -Force
    Start-Sleep -Seconds 2
    $procs = $null
}

if (-not $procs) {
    if ($CheckOnly) {
        Bad "PC-Service 미실행 → 이 상태면 앱에 어떤 IP 를 넣어도 TCP failed."
        Write-Host "        -CheckOnly 를 빼고 실행하면 자동 기동합니다." -ForegroundColor Yellow
        exit 1
    }
    $exe = Find-Service
    if (-not $exe) { Die "RoboticsServiceProcess.exe 를 못 찾았습니다 (검색 루트: $CaptureDir). -CaptureDir 로 지정하세요." }
    Write-Host "    기동: $exe"
    Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe) | Out-Null

    $deadline = (Get-Date).AddSeconds(20); $listening = $false
    do {
        Start-Sleep -Milliseconds 500
        $procs = Get-ServiceProcs
        if ($procs) {
            foreach ($p in $procs) {
                if (Get-NetTCPConnection -State Listen -OwningProcess $p.Id -ErrorAction SilentlyContinue) {
                    $listening = $true; break
                }
            }
        }
    } while (-not $listening -and (Get-Date) -lt $deadline)

    if (-not $listening) {
        Die "기동은 했으나 20초 안에 리스닝 소켓이 안 열렸습니다. PC-Service 창의 에러를 확인하세요 (VC_redist 미설치 등)."
    }
    Ok "기동 완료 (PID $($procs.Id -join ', '))"
} else {
    Ok "이미 실행 중 (PID $($procs.Id -join ', ')) — 재기동 안 함(포트 유지)"
}

# ══════════════════════════════════════════════════════════════════════════════
# 2. 리스닝 포트 (동적 할당)
# ══════════════════════════════════════════════════════════════════════════════
Step 2 "리스닝 포트"

$tcp = @(); $udp = @()
foreach ($p in $procs) {
    $tcp += @(Get-NetTCPConnection -State Listen -OwningProcess $p.Id -ErrorAction SilentlyContinue)
    $udp += @(Get-NetUDPEndpoint -OwningProcess $p.Id -ErrorAction SilentlyContinue)
}
foreach ($c in $tcp) { Write-Host ("    TCP  {0,-22} :{1}" -f $c.LocalAddress, $c.LocalPort) }
foreach ($c in $udp) { Write-Host ("    UDP  {0,-22} :{1}" -f $c.LocalAddress, $c.LocalPort) }

$tcpPorts = @($tcp | ForEach-Object { $_.LocalPort } | Sort-Object -Unique)
$udpPorts = @($udp | ForEach-Object { $_.LocalPort } | Sort-Object -Unique)

if ($tcpPorts.Count -eq 0) { Die "TCP 리스닝 포트가 없습니다. 서비스 초기화 실패." }
Ok "TCP $($tcpPorts -join ', ')"
if ($udpPorts.Count -gt 0) {
    Warn "UDP $($udpPorts -join ', ') 도 열려 있습니다. adb reverse 는 TCP 전용이라 USB 로는 안 넘어갑니다."
    Warn "→ 앱 connect 는 되는데 pose 가 계속 0.0 이면 이게 원인입니다 (§5 판정표)."
}

# ══════════════════════════════════════════════════════════════════════════════
# 3. adb reverse 터널  ← USB 경로의 본체
# ══════════════════════════════════════════════════════════════════════════════
Step 3 "adb reverse 터널 (앱의 127.0.0.1 → 노트북)"

if ($CheckOnly) {
    if (-not $devSerial) {
        Bad "USB 디바이스가 없어 터널 상태를 볼 수 없습니다."
        $script:Blocked = $true
    } else {
        $existing = & $adb reverse --list 2>&1 | Out-String
        if ($existing -match '\S') {
            Write-Host $existing
            $missing = @($tcpPorts | Where-Object { $existing -notmatch "tcp:$_\s" })
            if ($missing.Count -gt 0) {
                Bad "포트 $($missing -join ', ') 에 터널이 없습니다 (서비스 재기동으로 포트가 바뀐 상태)."
                $script:Blocked = $true
            } else { Ok "모든 TCP 포트에 터널이 있습니다." }
        } else {
            Bad "reverse 터널이 하나도 없습니다 → 지금 앱에 127.0.0.1 을 넣으면 TCP failed."
            $script:Blocked = $true
        }
    }
    Write-Host ""
    if ($script:Blocked) {
        Write-Host "=== 막힘. -CheckOnly 를 빼고 실행하면 대부분 자동으로 복구됩니다. ===" -ForegroundColor Red
        exit 1
    }
    Write-Host "=== USB 경로 정상. 앱 IP 칸 = 127.0.0.1 ===" -ForegroundColor Green
    exit 0
}

# 포트가 바뀌었을 수 있으니 전부 지우고 새로 건다 (stale 터널이 오히려 오연결을 만든다)
& $adb reverse --remove-all 2>&1 | Out-Null
$done = @()
foreach ($port in $tcpPorts) {
    $r = & $adb reverse "tcp:$port" "tcp:$port" 2>&1
    if ($LASTEXITCODE -eq 0) { $done += $port } else { Warn "reverse tcp:$port 실패 — $r" }
}
if ($done.Count -eq 0) { Die "터널을 하나도 못 걸었습니다. adb kill-server 후 케이블 재연결하고 다시 실행하세요." }

$verify = & $adb reverse --list 2>&1 | Out-String
$lost = @($done | Where-Object { $verify -notmatch "tcp:$_\s" })
if ($lost.Count -gt 0) { Die "터널 설치 후 확인에서 $($lost -join ', ') 가 사라졌습니다. USB 링크가 불안정합니다 (케이블/포트 교체)." }
Ok "터널 $($done -join ', ') 설치·확인 완료"

# ══════════════════════════════════════════════════════════════════════════════
# 4. 링크 품질 튜닝 (-Tune, 관리자)
# ══════════════════════════════════════════════════════════════════════════════
Step 4 "링크 품질"

if ($Tune) {
    if (-not $IsAdmin) {
        Warn "-Tune 은 관리자 권한이 필요합니다. PowerShell 을 '관리자 권한으로 실행' 후 다시 돌리세요."
        Warn "(터널은 이미 설치됐으니 이대로 세션은 진행 가능합니다.)"
    } else {
        # 4-1. USB selective suspend OFF — 윈도우가 유휴 판단으로 USB 를 재우면 프레임이 툭툭 끊긴다
        try {
            $sub = "2a737441-1930-4402-8d77-b2bebba308a3"; $set = "48e6b7a6-50f5-4782-a5d4-53bb8f07e226"
            powercfg /setacvalueindex SCHEME_CURRENT $sub $set 0 | Out-Null
            powercfg /setdcvalueindex SCHEME_CURRENT $sub $set 0 | Out-Null
            powercfg /setactive SCHEME_CURRENT | Out-Null
            Ok "USB selective suspend 비활성화 (AC/DC)"
        } catch { Warn "USB selective suspend 설정 실패: $_" }

        # 4-2. USB 허브 절전 해제 — '전원 절약을 위해 장치를 끌 수 있음' 체크 해제와 동일
        try {
            $n = 0
            Get-CimInstance -Namespace root\wmi -ClassName MSPower_DeviceEnable -ErrorAction Stop |
                Where-Object { $_.InstanceName -match 'USB' } | ForEach-Object {
                    if ($_.Enable) { Set-CimInstance -InputObject $_ -Property @{ Enable = $false }; $n++ }
                }
            Ok "USB 장치 절전 해제 ($n 개)"
        } catch { Warn "USB 절전 해제 실패(무시 가능): $($_.Exception.Message)" }

        # 4-3. 고성능 전원 계획 — 절전 계획은 USB/CPU 를 같이 조인다
        try {
            $hp = "8c5e7fda-e8bf-4a96-9a85-a6e23a8c635c"
            if ((powercfg /list) -match $hp) { powercfg /setactive $hp | Out-Null; Ok "전원 계획: 고성능" }
            else { Warn "고성능 계획이 없습니다 → 윈도우 설정 > 전원 에서 '최고의 성능' 으로 두세요." }
        } catch { Warn "전원 계획 변경 실패: $_" }

        # 4-4. PC-Service 우선순위 — 50Hz 스트림이 백그라운드 작업에 밀리지 않게
        try {
            foreach ($p in Get-ServiceProcs) { $p.PriorityClass = 'High' }
            Ok "PC-Service 우선순위: High"
        } catch { Warn "우선순위 변경 실패: $_" }
    }
} else {
    Write-Host "    (튜닝 생략. 최초 1회 + 재부팅 후에는 관리자 PowerShell 에서 -Tune 을 권장)" -ForegroundColor DarkGray
}

# 세션 중 반드시 지킬 것 — 자동화 불가라 안내로 남긴다
Write-Host "`n    세션 내내 지킬 것:" -ForegroundColor White
Write-Host "      · 케이블·포트 건드리지 않기 (재연결 = 터널 소멸 = 이 스크립트 재실행)"
Write-Host "      · PC-Service 창 닫지 않기 (닫으면 포트가 바뀝니다)"
Write-Host "      · adb kill-server / 다른 adb 도구(Android Studio·scrcpy) 실행 금지 — 터널이 날아갑니다"
Write-Host "      · 헤드셋 벗지 않기 (절전 진입 시 pose 가 0 으로 정지)"

# ══════════════════════════════════════════════════════════════════════════════
# 5. 앱 조작 + 판정
# ══════════════════════════════════════════════════════════════════════════════
Step 5 "PICO 앱"

Write-Host ""
Write-Host "  ┌──────────────────────────────────────────────┐" -ForegroundColor Green
Write-Host "  │   앱 IP 칸에 입력:   127.0.0.1                │" -ForegroundColor Green
Write-Host "  └──────────────────────────────────────────────┘" -ForegroundColor Green
Write-Host "     앱은 plain XRoboToolkit 을 쓸 것 (ROS1/ROS2 판은 PC-Service 경로를 안 탑니다)"
Write-Host ""
Write-Host "  connect → WORKING 확인 후, mode3 전신 텔레옵이면:" -ForegroundColor White
Write-Host "     1) Full Body tracking       2) Pico Swift 발 트래커 2개"
Write-Host "     3) T-pose 캘리브레이션       4) Send 토글 ON"
Write-Host "     ⚠ 'Switch w/ A Button' 이 켜져 있으면 A 버튼으로 Send 가 꺼집니다."
Write-Host ""
Write-Host "  검증:  conda activate $CondaEnv"
Write-Host "         python $CaptureDir\scripts\test_pico_pose.py"
Write-Host ""
Write-Host "     결과 판정" -ForegroundColor White
Write-Host "       값이 변하고 body=True      → 정상. publisher 로."
Write-Host "       값이 변하는데 body=False   → 앱 설정 (Full Body / 트래커 / T-pose / Send)"
Write-Host "       connect 는 됐는데 값이 0.0 → pose 가 UDP 를 타는 경우. USB 로는 못 넘김." -ForegroundColor Yellow
Write-Host "                                     (§2 에서 UDP 포트 경고가 떴다면 이 케이스)"
Write-Host "       TCP failed                 → 이 스크립트를 다시 실행 (터널/포트가 어긋난 것)"

if (-not $Publish) {
    Write-Host "`n  publisher:  python $CaptureDir\pico_publisher.py --com1 $Com1" -ForegroundColor White
    Write-Host "  (또는 이 스크립트를 -Publish 로 다시 실행)" -ForegroundColor DarkGray
    exit 0
}

# ══════════════════════════════════════════════════════════════════════════════
# 6. publisher
# ══════════════════════════════════════════════════════════════════════════════
Step 6 "publisher"
Write-Host "    앱 connect + Send ON 이 끝났으면 Enter, 아니면 Ctrl+C" -ForegroundColor Yellow
Read-Host | Out-Null

$py = "$env:USERPROFILE\miniconda3\envs\$CondaEnv\python.exe"
if (-not (Test-Path $py)) { $py = "$env:USERPROFILE\anaconda3\envs\$CondaEnv\python.exe" }
if (-not (Test-Path $py)) {
    Warn "conda env '$CondaEnv' 의 python.exe 를 못 찾아 PATH 의 python 을 씁니다."
    Warn "'Python' 한 줄만 찍고 끝나면 Microsoft Store 스텁입니다 → conda activate $CondaEnv 후 수동 실행."
    $py = "python"
}

$pub = Start-Process -FilePath $py -ArgumentList @("$CaptureDir\pico_publisher.py", "--com1", $Com1) `
                     -NoNewWindow -PassThru
try { $pub.PriorityClass = 'High' } catch { }
Write-Host "    publisher PID $($pub.Id) — [stat] ... body=True 가 나오면 정상" -ForegroundColor DarkGray
$pub.WaitForExit()
