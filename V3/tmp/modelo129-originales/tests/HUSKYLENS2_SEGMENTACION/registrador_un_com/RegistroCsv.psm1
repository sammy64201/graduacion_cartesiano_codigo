Set-StrictMode -Version 2.0

$script:Columnas = @(
    'pc_utc', 'puerto_com', 'baud', 'tipo', 'frame', 'ms', 'algoritmo',
    'resultados', 'indice', 'id', 'nombre', 'contenido', 'tipo_resultado_raw',
    'level_raw', 'u_px', 'v_px', 'ancho_px', 'alto_px', 'x_mm', 'y_mm',
    'coordenadas_validas', 'en_calibracion', 'en_banda', 'error_parseo', 'raw'
)

function Get-ColumnasRegistro {
    return $script:Columnas
}

function ConvertTo-CampoRegistroCsv {
    param([AllowNull()][object]$Valor)
    if ($null -eq $Valor) { return '""' }
    if ($Valor -is [bool]) {
        $texto = $Valor.ToString().ToLowerInvariant()
    } elseif ($Valor -is [System.IFormattable]) {
        $texto = $Valor.ToString($null, [System.Globalization.CultureInfo]::InvariantCulture)
    } else {
        $texto = [string]$Valor
    }
    return '"' + $texto.Replace('"', '""') + '"'
}

function ConvertTo-FilaRegistro {
    param(
        [Parameter(Mandatory)][AllowEmptyString()][string]$Linea,
        [Parameter(Mandatory)][string]$Puerto,
        [int]$Baud = 115200,
        [DateTime]$Utc = [DateTime]::UtcNow,
        [switch]$Fragmento
    )
    $fila = [ordered]@{}
    foreach ($columna in $script:Columnas) { $fila[$columna] = $null }
    $fila.pc_utc = $Utc.ToString('o', [System.Globalization.CultureInfo]::InvariantCulture)
    $fila.puerto_com = $Puerto
    $fila.baud = $Baud
    $fila.tipo = 'mensaje'
    $fila.raw = $Linea
    if ($Fragmento) {
        $fila.tipo = 'fragmento'
    } elseif ($Linea.TrimStart().StartsWith('{')) {
        try {
            $objeto = ConvertFrom-Json -InputObject $Linea -ErrorAction Stop
            if ($null -eq $objeto -or $objeto -isnot [pscustomobject]) {
                throw 'Se esperaba un objeto JSON.'
            }
            $fila.tipo = 'json'
            foreach ($columna in $script:Columnas) {
                if ($columna -in @('pc_utc', 'puerto_com', 'baud', 'error_parseo', 'raw')) { continue }
                $propiedad = $objeto.PSObject.Properties[$columna]
                if ($null -ne $propiedad) { $fila[$columna] = $propiedad.Value }
            }
            if ([string]::IsNullOrWhiteSpace([string]$fila.tipo)) { $fila.tipo = 'json' }
        } catch {
            $fila.tipo = 'error_json'
            $fila.error_parseo = $_.Exception.Message
        }
    }
    return [pscustomobject]$fila
}

function Write-FilaRegistroCsv {
    param(
        [Parameter(Mandatory)][System.IO.TextWriter]$Escritor,
        [Parameter(Mandatory)][psobject]$Fila
    )
    $campos = foreach ($columna in $script:Columnas) {
        ConvertTo-CampoRegistroCsv $Fila.$columna
    }
    $Escritor.WriteLine(($campos -join ','))
}

function New-EscritorRegistroCsv {
    param([Parameter(Mandatory)][string]$Ruta)
    # CreateNew impide reemplazar una captura existente por accidente.
    $flujo = New-Object System.IO.FileStream($Ruta, [System.IO.FileMode]::CreateNew,
        [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
    try {
        $escritor = New-Object System.IO.StreamWriter($flujo, (New-Object System.Text.UTF8Encoding($true)))
        $escritor.AutoFlush = $true
        $encabezado = foreach ($columna in $script:Columnas) { ConvertTo-CampoRegistroCsv $columna }
        $escritor.WriteLine(($encabezado -join ','))
        return $escritor
    } catch {
        $flujo.Dispose()
        throw
    }
}

function Split-LineasRegistro {
    param([Parameter(Mandatory)][AllowEmptyString()][string]$Texto)
    # ReadExisting puede entregar medio JSON. Solo separar lineas completas.
    $partes = $Texto.Split([char]10)
    $lineas = New-Object 'System.Collections.Generic.List[string]'
    for ($i = 0; $i -lt $partes.Length - 1; $i++) {
        $linea = $partes[$i]
        if ($linea.EndsWith("`r")) { $linea = $linea.Substring(0, $linea.Length - 1) }
        $lineas.Add($linea)
    }
    return [pscustomobject]@{ Lineas = $lineas.ToArray(); Pendiente = $partes[-1] }
}

Export-ModuleMember -Function Get-ColumnasRegistro, ConvertTo-FilaRegistro,
    Write-FilaRegistroCsv, New-EscritorRegistroCsv, Split-LineasRegistro
