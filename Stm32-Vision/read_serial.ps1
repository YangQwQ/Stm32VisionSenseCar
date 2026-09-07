param(
    [string]$Port = 'COM7',
    [int]$Seconds = 12,
    [switch]$Reset = $false
)
$ser = [System.IO.Ports.SerialPort]::new($Port, 115200)
$ser.Parity = [System.IO.Ports.Parity]::None
$ser.DataBits = 8
$ser.StopBits = [System.IO.Ports.StopBits]::One
$ser.ReadTimeout = 300
try {
    $ser.Open()
} catch {
    Write-Output "OPEN_ERR: $_"
    exit 1
}
if ($Reset) {
    # 触发一次复位（auto-reset：RTS 控制 EN）
    $ser.DtrEnable = $true
    $ser.RtsEnable = $false
    Start-Sleep -Milliseconds 100
    $ser.RtsEnable = $true
    Start-Sleep -Milliseconds 200
    $ser.RtsEnable = $false
}
$stop = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $stop) {
    try {
        $line = $ser.ReadLine()
        Write-Output $line
    } catch {
        # timeout
    }
}
$ser.Close()