# Windows 地上PC (既定: 192.168.2.98) で BlueROV カメラ映像を録画する。
#
# 事前準備:
#   1. FFmpeg をインストールし PATH に通す
#      https://www.gyan.dev/ffmpeg/builds/ などから ffmpeg.exe を取得
#   2. ロボット側で camera_driver を起動する
#   3. Windows ファイアウォールで UDP 5601 の受信を許可する
#
# Usage (PowerShell):
#   .\record_camera_client.ps1
#   .\record_camera_client.ps1 -OutputDir "C:\bluerov_videos" -UdpPort 5601
#
# 停止: Ctrl+C

param(
    [string]$OutputDir = "$env:USERPROFILE\bluerov_videos",
    [int]$UdpPort = 5601
)

$ErrorActionPreference = "Stop"

$ffmpeg = Get-Command ffmpeg -ErrorAction SilentlyContinue
if (-not $ffmpeg) {
    Write-Error @"
ffmpeg が見つかりません。PATH に ffmpeg.exe を追加してください。
例: https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip
"@
}

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outputFile = Join-Path $OutputDir "bluerov_$timestamp.mp4"
$sdpFile = Join-Path $env:TEMP "bluerov_camera_$UdpPort.sdp"

$sdp = @"
v=0
o=- 0 0 IN IP4 127.0.0.1
s=BlueROV Camera
c=IN IP4 0.0.0.0
t=0 0
m=video $UdpPort RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1
"@

Set-Content -Path $sdpFile -Value $sdp -Encoding ascii

Write-Host "UDP $UdpPort で RTP/H264 を受信し、次のファイルに録画します:"
Write-Host "  $outputFile"
Write-Host "停止するには Ctrl+C を押してください。"

try {
    & ffmpeg `
        -hide_banner `
        -loglevel warning `
        -protocol_whitelist "file,udp,rtp" `
        -i $sdpFile `
        -c copy `
        -f mp4 `
        $outputFile
}
finally {
    if (Test-Path $sdpFile) {
        Remove-Item $sdpFile -Force
    }
    Write-Host ""
    Write-Host "録画を停止しました: $outputFile"
}
