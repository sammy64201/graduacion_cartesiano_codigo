[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$PuertoESP,
    [Parameter(Mandatory = $true)][string]$PuertoPortenta,
    [string]$DirectorioSalida,
    [int]$LimiteSegundos = 2400
)
Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
if ($PuertoESP -eq $PuertoPortenta) { throw 'Los puertos de las placas deben ser distintos.' }
if ($LimiteSegundos -lt 10) { throw 'LimiteSegundos debe ser al menos 10.' }
if ([string]::IsNullOrWhiteSpace($DirectorioSalida)) {
    $DirectorioSalida = Join-Path $PSScriptRoot 'registros'
}
New-Item -ItemType Directory -Path $DirectorioSalida -Force | Out-Null
$taskArchivo = Join-Path $DirectorioSalida ('final_rs485_' + (Get-Date -Format 'yyyy-MM-dd_HH-mm-ss') + '.log')
$taskWriter = New-Object System.IO.StreamWriter($taskArchivo, $false, (New-Object System.Text.UTF8Encoding($false)))
$taskWriter.AutoFlush = $true
$taskPuertos = @()
$taskCronometro = [System.Diagnostics.Stopwatch]::StartNew()
$taskIniciado = $false
$taskResultado = $null
$taskFinMs = 0
try {
    foreach ($taskDef in @(@('ESP', $PuertoESP), @('PORTENTA', $PuertoPortenta))) {
        $taskSerial = New-Object System.IO.Ports.SerialPort($taskDef[1], 115200, 'None', 8, 'One')
        $taskPuertos += [pscustomobject]@{ Nombre = $taskDef[0]; Puerto = $taskSerial; Pendiente = '' }
        $taskSerial.DtrEnable = ($taskDef[0] -eq 'PORTENTA')
        $taskSerial.RtsEnable = $false
        $taskSerial.Open()
    }
    Write-Host 'Registrando ambos USB a 115200. Se enviara T a Portenta tras 3 segundos.'
    Write-Host ('Log: ' + [System.IO.Path]::GetFullPath($taskArchivo))
    while ($taskCronometro.Elapsed.TotalSeconds -lt $LimiteSegundos) {
        foreach ($taskCanal in $taskPuertos) {
            $taskCanal.Pendiente += $taskCanal.Puerto.ReadExisting()
            while ($taskCanal.Pendiente.Contains("`n")) {
                $taskIndice = $taskCanal.Pendiente.IndexOf("`n")
                $taskLinea = $taskCanal.Pendiente.Substring(0, $taskIndice).TrimEnd("`r")
                $taskCanal.Pendiente = $taskCanal.Pendiente.Substring($taskIndice + 1)
                $taskEntrada = (Get-Date).ToUniversalTime().ToString('o') + ' [' + $taskCanal.Nombre + '] ' + $taskLinea
                $taskWriter.WriteLine($taskEntrada)
                Write-Host $taskEntrada
                if ($taskCanal.Nombre -eq 'PORTENTA' -and $taskLinea -match '\[FINAL PORTENTA\] RESULTADO=(PASS|FAIL)') {
                    $taskResultado = $Matches[1]
                    $taskFinMs = $taskCronometro.ElapsedMilliseconds
                }
            }
        }
        if (-not $taskIniciado -and $taskCronometro.ElapsedMilliseconds -ge 3000) {
            $taskPuertos[1].Puerto.Write('T')
            $taskWriter.WriteLine((Get-Date).ToUniversalTime().ToString('o') + ' [PC] T enviado a Portenta')
            $taskIniciado = $true
        }
        # Capturar tambien el ultimo resumen ESP despues del resultado Portenta.
        if ($null -ne $taskResultado -and $taskCronometro.ElapsedMilliseconds - $taskFinMs -ge 2000) { break }
        Start-Sleep -Milliseconds 20
    }
    if ($null -eq $taskResultado) { throw 'No se recibio resultado final; la ronda no esta aprobada. Revisar el log.' }
    Write-Host ('Resultado Portenta: ' + $taskResultado + '. Revisar tambien los contadores ESP del README.')
} finally {
    foreach ($taskCanal in $taskPuertos) {
        if ($taskCanal.Puerto.IsOpen) { $taskCanal.Puerto.Close() }
        $taskCanal.Puerto.Dispose()
    }
    $taskWriter.Dispose()
    $taskCronometro.Stop()
}
