$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0
$appRoot = Join-Path $PSScriptRoot 'HUSKYLENS2_SEGMENTACION\registrador_un_com'
foreach ($ruta in @((Join-Path $appRoot 'registrar_un_com.ps1'), (Join-Path $appRoot 'RegistroCsv.psm1'))) {
    $tokens = $null
    $errores = $null
    $null = [System.Management.Automation.Language.Parser]::ParseFile($ruta, [ref]$tokens, [ref]$errores)
    if ($errores.Count -gt 0) { throw ($errores | Out-String) }
}
Import-Module (Join-Path $appRoot 'RegistroCsv.psm1') -Force

$originalCulture = [System.Threading.Thread]::CurrentThread.CurrentCulture
$temporal = Join-Path ([System.IO.Path]::GetTempPath()) ('registro-com-test-' + [Guid]::NewGuid().ToString('N'))
$null = [System.IO.Directory]::CreateDirectory($temporal)
$escritor = $null
try {
    # El CSV debe conservar punto decimal aun en una PC configurada en espanol.
    [System.Threading.Thread]::CurrentThread.CurrentCulture = [System.Globalization.CultureInfo]::GetCultureInfo('es-ES')
    $archivo = Join-Path $temporal 'prueba.csv'
    $escritor = New-EscritorRegistroCsv -Ruta $archivo
    $segmentacion = [ordered]@{
        tipo='segmentacion'; frame=12; ms=25000; algoritmo=129; indice=0; id=0
        nombre=('pieza, "' + [char]0x00F1 + '"'); contenido="linea uno`nlinea dos"
        tipo_resultado_raw=28; level_raw=-1; u_px=350; v_px=220; ancho_px=60; alto_px=35
        x_mm=20.5; y_mm=-15.2; coordenadas_validas=$true; en_calibracion=$true; en_banda=$false
    } | ConvertTo-Json -Compress
    $frame = '{"tipo":"frame","frame":12,"ms":25000,"algoritmo":129,"resultados":1}'
    $sinObjetos = '{"tipo":"frame","frame":13,"ms":25200,"algoritmo":129,"resultados":0}'
    $nulos = '{"tipo":"segmentacion","id":1,"x_mm":null,"y_mm":null,"en_banda":false}'
    $jsonMalo = '{"tipo":roto}'
    $texto = "[CAL] Muestras 25/25`r`n$frame`n$segmentacion`r`n$sinObjetos`n$nulos`n$jsonMalo`nultima linea incompleta"
    $pendiente = ''
    # Simular lecturas seriales cortadas dentro de JSON, CRLF y mensajes.
    for ($offset = 0; $offset -lt $texto.Length; $offset += 7) {
        $chunk = $texto.Substring($offset, [Math]::Min(7, $texto.Length - $offset))
        $separadas = Split-LineasRegistro -Texto ($pendiente + $chunk)
        foreach ($linea in $separadas.Lineas) {
            $fila = ConvertTo-FilaRegistro -Linea $linea -Puerto 'COM5' -Baud 115200
            Write-FilaRegistroCsv -Escritor $escritor -Fila $fila
        }
        $pendiente = $separadas.Pendiente
    }
    if ($pendiente -ne 'ultima linea incompleta') { throw 'Se perdio la ultima linea parcial.' }
    Write-FilaRegistroCsv -Escritor $escritor -Fila (ConvertTo-FilaRegistro -Linea $pendiente -Puerto 'COM5' -Fragmento)
    $escritor.Dispose()
    $escritor = $null
    $filas = @(Import-Csv -LiteralPath $archivo -Encoding UTF8)
    if ($filas.Count -ne 7) { throw ('Filas perdidas o duplicadas: ' + $filas.Count) }
    if (($filas.tipo -join ',') -ne 'mensaje,frame,segmentacion,frame,segmentacion,error_json,fragmento') {
        throw 'No se preservaron los distintos tipos de linea.'
    }
    if ($filas[2].nombre -ne ('pieza, "' + [char]0x00F1 + '"') -or
        $filas[2].contenido -ne "linea uno`nlinea dos" -or $filas[2].raw -ne $segmentacion -or
        $filas[2].x_mm -ne '20.5' -or $filas[2].y_mm -ne '-15.2' -or
        $filas[2].id -ne '0' -or $filas[2].en_banda -ne 'false' -or
        $filas[2].coordenadas_validas -ne 'true' -or $filas[2].level_raw -ne '-1') {
        throw 'Campos de deteccion alterados en el CSV.'
    }
    if ($filas[3].resultados -ne '0' -or $filas[4].x_mm -ne '' -or $filas[4].y_mm -ne '' -or
        $filas[5].raw -ne $jsonMalo -or [string]::IsNullOrWhiteSpace($filas[5].error_parseo) -or
        $filas[6].raw -ne $pendiente) { throw 'Se perdieron ceros, nulos, errores o el fragmento final.' }
    foreach ($fila in $filas) {
        if ($fila.puerto_com -ne 'COM5' -or $fila.baud -ne '115200' -or !$fila.pc_utc) {
            throw 'Faltan datos de origen/tiempo.'
        }
    }
    $bytes = [System.IO.File]::ReadAllBytes($archivo)
    if ($bytes[0] -ne 0xEF -or $bytes[1] -ne 0xBB -or $bytes[2] -ne 0xBF) {
        throw 'El archivo no tiene BOM UTF8 para Excel.'
    }
    $bloqueo = $false
    try { $otro = New-EscritorRegistroCsv -Ruta $archivo; $otro.Dispose() } catch { $bloqueo = $true }
    if (-not $bloqueo) { throw 'Se sobrescribio una captura existente.' }
    Write-Output 'PASS: sintaxis, JSON serial fragmentado, CSV UTF8, caracteres especiales, decimales, nulos, ceros, errores y cierre parcial.'
} finally {
    [System.Threading.Thread]::CurrentThread.CurrentCulture = $originalCulture
    if ($null -ne $escritor) { $escritor.Dispose() }
    # Eliminar solo los archivos temporales explicitamente creados por esta prueba.
    $archivoPrueba = Join-Path $temporal 'prueba.csv'
    if (Test-Path -LiteralPath $archivoPrueba) { Remove-Item -LiteralPath $archivoPrueba }
    if (Test-Path -LiteralPath $temporal) { Remove-Item -LiteralPath $temporal }
}
