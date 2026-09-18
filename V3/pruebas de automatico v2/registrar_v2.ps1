[CmdletBinding()]
param(
    [string]$PuertoESP,
    [string]$PuertoPortenta,
    [string]$DirectorioSalida
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

# En Windows PowerShell 5.1 los valores predeterminados de param() pueden
# evaluarse antes de que $PSScriptRoot tenga contenido. Se resuelve la ruta
# una vez que el cuerpo del script ya esta en ejecucion.
if ([string]::IsNullOrWhiteSpace($DirectorioSalida)) {
    $DirectorioSalida = Join-Path $PSScriptRoot 'registros_v2'
}

$script:Columnas = @(
    'pc_utc', 'source_name_es', 'event', 'description_es', 'phase_name_es',
    'pc_elapsed_ms', 'source', 'board_ms', 'session_esp', 'cause', 'query_id',
    'objective_seq', 'encoder_seq',
    'encoder_count', 'objective_encoder_count', 'encoder_age_ms',
    'encoder_velocity_mm_s', 'encoder_flags', 'husky_results',
    'candidates_valid', 'candidates_unique', 'duplicates',
    'rejected_homography', 'rejected_belt', 'candidate_index',
    'candidate_group', 'selected_group', 'class', 'confidence',
    'pixel_x', 'pixel_y', 'width_px', 'height_px', 'x_mm', 'y_mm',
    'x_compensated_mm', 'y_compensated_mm', 'dispersion_x_mm',
    'dispersion_y_mm', 'travel_mm',
    'relation_y_encoder', 'consecutive_detections', 'query_duration_ms',
    'travel_during_query_mm', 'encoder_before', 'encoder_after', 'phase',
    'arm_x_mm', 'arm_y_mm', 'z_steps', 'target_x_mm', 'target_y_mm',
    'error_x_mm', 'error_y_mm', 'movement_x', 'movement_y', 'movement_z',
    'ack_seq', 'ack_code', 'error_code', 'result', 'lost_records',
    'message', 'raw'
)

$script:MapaCampos = @{
    'ms'          = 'board_ms'
    'session'     = 'session_esp'
    'event'       = 'event'
    'cause'       = 'cause'
    'query'       = 'query_id'
    'obj'         = 'objective_seq'
    'encseq'      = 'encoder_seq'
    'enc'         = 'encoder_count'
    'ref_enc'     = 'objective_encoder_count'
    'age'         = 'encoder_age_ms'
    'vel'         = 'encoder_velocity_mm_s'
    'flags'       = 'encoder_flags'
    'husky'       = 'husky_results'
    'valid'       = 'candidates_valid'
    'unique'      = 'candidates_unique'
    'dup'         = 'duplicates'
    'rej_h'       = 'rejected_homography'
    'rej_b'       = 'rejected_belt'
    'candidate'   = 'candidate_index'
    'group'       = 'candidate_group'
    'selected'    = 'selected_group'
    'class'       = 'class'
    'confidence'  = 'confidence'
    'px'          = 'pixel_x'
    'py'          = 'pixel_y'
    'width'       = 'width_px'
    'height'      = 'height_px'
    'x'           = 'x_mm'
    'y'           = 'y_mm'
    'xc'          = 'x_compensated_mm'
    'yc'          = 'y_compensated_mm'
    'dx'          = 'dispersion_x_mm'
    'dy'          = 'dispersion_y_mm'
    'travel'      = 'travel_mm'
    'rel_y'       = 'relation_y_encoder'
    'n'           = 'consecutive_detections'
    'query_ms'    = 'query_duration_ms'
    'query_travel'= 'travel_during_query_mm'
    'enc_before'  = 'encoder_before'
    'enc_after'   = 'encoder_after'
    'phase'       = 'phase'
    'arm_x'       = 'arm_x_mm'
    'arm_y'       = 'arm_y_mm'
    'z_steps'     = 'z_steps'
    'target_x'    = 'target_x_mm'
    'target_y'    = 'target_y_mm'
    'error_x'     = 'error_x_mm'
    'error_y'     = 'error_y_mm'
    'mov_x'       = 'movement_x'
    'mov_y'       = 'movement_y'
    'mov_z'       = 'movement_z'
    'ack'         = 'ack_seq'
    'ack_code'    = 'ack_code'
    'error'       = 'error_code'
    'lost'        = 'lost_records'
    'message'     = 'message'
}

$script:Reloj = [System.Diagnostics.Stopwatch]::StartNew()
$script:EscritorSesion = $null
$script:ArchivoSesion = $null
$script:ContadorNombre = 0
$script:Buffers = @{ E = ''; P = '' }
$script:BloqueadoHastaSalirV2 = $false
$script:UltimaMuestraEnVivo = @{
    ESP_IDLE = [DateTime]::MinValue
    PORTENTA_TELEMETRY = [DateTime]::MinValue
}
$prebuffer = New-Object 'System.Collections.Generic.Queue[object]'

function Pedir-Puerto([string]$Nombre, [string]$ValorActual) {
    if (-not [string]::IsNullOrWhiteSpace($ValorActual)) {
        return $ValorActual.Trim().ToUpperInvariant()
    }
    $disponibles = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    Write-Host ('Puertos disponibles: ' + ($disponibles -join ', '))
    return (Read-Host "Puerto COM de $Nombre").Trim().ToUpperInvariant()
}

function Validar-Puerto([string]$Puerto, [string]$Nombre) {
    if ($Puerto -notmatch '^COM\d+$') {
        throw "Puerto invalido para ${Nombre}: '$Puerto'. Use un nombre como COM8."
    }
}

function Abrir-Puerto([string]$Nombre, [int]$Baudios, [bool]$ActivarDtr) {
    $puerto = New-Object System.IO.Ports.SerialPort
    $puerto.PortName = $Nombre
    $puerto.BaudRate = $Baudios
    $puerto.Parity = [System.IO.Ports.Parity]::None
    $puerto.DataBits = 8
    $puerto.StopBits = [System.IO.Ports.StopBits]::One
    $puerto.Handshake = [System.IO.Ports.Handshake]::None
    # La Portenta expone Serial mediante USB CDC y necesita DTR para considerar
    # al host conectado. En la ESP32 se mantiene apagado para evitar alterar
    # sus lineas de auto-reset del conversor USB-UART.
    $puerto.DtrEnable = $ActivarDtr
    $puerto.RtsEnable = $false
    $puerto.ReadTimeout = 50
    $puerto.WriteTimeout = 500
    $puerto.Open()
    return $puerto
}

function Convertir-CampoCsv([object]$Valor) {
    if ($null -eq $Valor) { return '""' }
    $texto = [string]$Valor
    return '"' + $texto.Replace('"', '""') + '"'
}

function Analizar-Linea([string]$Linea, [string]$FuentePredeterminada) {
    $datos = @{}
    $fuente = $FuentePredeterminada
    $estructurada = $false
    if ($Linea.StartsWith('V2LOG|')) {
        $partes = $Linea.Split('|')
        if ($partes.Count -ge 3) {
            $fuente = $partes[1]
            for ($i = 2; $i -lt $partes.Count; $i++) {
                $posicion = $partes[$i].IndexOf('=')
                if ($posicion -le 0) { continue }
                $clave = $partes[$i].Substring(0, $posicion)
                $valor = $partes[$i].Substring($posicion + 1)
                $datos[$clave] = $valor
            }
            $estructurada = $datos.ContainsKey('event')
        }
    }
    return [pscustomobject]@{
        Structured = $estructurada
        Source = $fuente
        Data = $datos
    }
}

function Obtener-Dato([object]$Analizada, [string]$Clave) {
    if ($Analizada.Structured -and $Analizada.Data.ContainsKey($Clave)) {
        return [string]$Analizada.Data[$Clave]
    }
    return ''
}

function Nombre-FaseV2([string]$Fase) {
    switch ($Fase) {
        '0' { return 'ESPERANDO_PIEZA' }
        '1' { return 'PREPOSICIONANDO' }
        '2' { return 'PREPARANDO_DESCENSO' }
        '3' { return 'DISPARANDO_CATCH' }
        '4' { return 'BAJANDO_Z' }
        '5' { return 'SUBIENDO_Z' }
        '6' { return 'ESPERANDO_CATCH_AUTOMATICO' }
        '7' { return 'COMPLETADO' }
        '8' { return 'CANCELANDO' }
        default { return '' }
    }
}

function Nombre-AckV2([string]$Codigo) {
    switch ($Codigo) {
        '0' { return 'NINGUNO' }
        '1' { return 'ACEPTADO' }
        '2' { return 'RECHAZADO_RANGO' }
        '3' { return 'CANCELADO' }
        '4' { return 'COMPLETADO' }
        default { return $Codigo }
    }
}

function Describir-Evento([object]$Analizada, [string]$Linea) {
    if (-not $Analizada.Structured) { return $Linea }
    $evento = Obtener-Dato $Analizada 'event'
    $causa = Obtener-Dato $Analizada 'cause'
    $fase = Nombre-FaseV2 (Obtener-Dato $Analizada 'phase')
    $mensaje = Obtener-Dato $Analizada 'message'
    $obj = Obtener-Dato $Analizada 'obj'
    $enc = Obtener-Dato $Analizada 'enc'
    $vel = Obtener-Dato $Analizada 'vel'

    switch ($evento) {
        'SESSION_START' { return 'ENTRADA A AUTOMATICO V2' }
        'SESSION_END' { return 'SALIDA DE AUTOMATICO V2: ' + $mensaje }
        'QUERY' {
            $q = Obtener-Dato $Analizada 'query'
            $n = Obtener-Dato $Analizada 'n'
            $travel = Obtener-Dato $Analizada 'travel'
            if ($causa -eq 'SIN_PIEZA') {
                return "Camara sin pieza | consulta=$q encoder=$enc velocidad=$vel mm/s"
            }
            if ($causa -eq 'PUBLICADO') {
                $clase = Obtener-Dato $Analizada 'class'
                $x = Obtener-Dato $Analizada 'x'
                $y = Obtener-Dato $Analizada 'yc'
                return "OBJETIVO PUBLICADO #$obj | clase=$clase pieza=($x,$y) mm encoder=$enc"
            }
            return "Consulta ${q}: $causa | detecciones=$n recorrido=$travel mm encoder=$enc velocidad=$vel mm/s"
        }
        'CANDIDATE' {
            $indice = Obtener-Dato $Analizada 'candidate'
            $clase = Obtener-Dato $Analizada 'class'
            $conf = Obtener-Dato $Analizada 'confidence'
            $x = Obtener-Dato $Analizada 'x'
            $y = Obtener-Dato $Analizada 'y'
            return "Candidato ${indice}: causa=$causa clase=$clase confianza=$conf posicion=($x,$y) mm"
        }
        'TELEMETRY' {
            $ax = Obtener-Dato $Analizada 'arm_x'
            $ay = Obtener-Dato $Analizada 'arm_y'
            $z = Obtener-Dato $Analizada 'z_steps'
            $tx = Obtener-Dato $Analizada 'target_x'
            $ty = Obtener-Dato $Analizada 'target_y'
            $ex = Obtener-Dato $Analizada 'error_x'
            $ey = Obtener-Dato $Analizada 'error_y'
            return "$fase | brazo=($ax,$ay,Z:$z) objetivo=($tx,$ty) error=($ex,$ey) velocidad=$vel mm/s encoder=$enc"
        }
        'PHASE' { return "FASE -> $fase | objetivo #$obj" }
        'TRIGGER' { return "PREBAJADA DE Z | objetivo #$obj | preposicion terminada, Z baja para esperar la pieza" }
        'BUTTON_X' { return "X RECIBIDA E IGNORADA | Automatico V2 no requiere confirmacion | objetivo #$obj fase=$fase" }
        'ACCEPT' { return "OBJETIVO #$obj ACEPTADO | encoder=$enc velocidad=$vel mm/s" }
        'REJECT' { return "OBJETIVO #$obj RECHAZADO: $mensaje" }
        'ACK' {
            $ack = Nombre-AckV2 (Obtener-Dato $Analizada 'ack_code')
            return "ACK objetivo #${obj}: $ack"
        }
        'READY_CATCH' {
            $piezaY = Obtener-Dato $Analizada 'target_y'
            return "Z ABAJO EN DIN04 | objetivo #$obj | pieza estimada Y=$piezaY mm | esperando cruce automatico Y=0 | encoder=$enc"
        }
        'CAPTURE' {
            if ($mensaje -eq 'CATCH AUTOMATICO POR ENCODER') {
                $piezaY = Obtener-Dato $Analizada 'target_y'
                return "CATCH AUTOMATICO POR ENCODER | objetivo #$obj pieza estimada Y=$piezaY mm encoder=$enc velocidad=$vel mm/s"
            }
            return "CAPTURA | objetivo #$obj fase=$fase encoder=$enc velocidad=$vel mm/s"
        }
        'RESULT' {
            if ($mensaje -eq 'CATCH_AUTOMATICO') {
                return "CICLO DE CATCH AUTOMATICO COMPLETADO | objetivo #$obj"
            }
            return "RESULTADO objetivo #${obj}: $mensaje"
        }
        'CANCEL' { return "CANCELACION objetivo #${obj}: $mensaje | fase=$fase velocidad=$vel mm/s" }
        'ERROR' { return "ERROR V2: $mensaje" }
        'DROP' { return 'REGISTROS PERDIDOS: ' + (Obtener-Dato $Analizada 'lost') }
        default { return "$evento $mensaje".Trim() }
    }
}

function Mostrar-EventoEnVivo([object]$Envoltura, [object]$Analizada) {
    $ahora = $Envoltura.Utc
    $evento = Obtener-Dato $Analizada 'event'
    $causa = Obtener-Dato $Analizada 'cause'
    if ($Analizada.Structured -and $evento -eq 'QUERY' -and
        $causa -in @('SIN_PIEZA', 'V2_INACTIVO', 'BRAZO_OCUPADO',
                     'OBJETIVO_ACTIVO', 'ESPERA_DESAPARICION')) {
        if ($ahora.Subtract($script:UltimaMuestraEnVivo['ESP_IDLE']).TotalMilliseconds -lt 1000) {
            return
        }
        $script:UltimaMuestraEnVivo['ESP_IDLE'] = $ahora
    }
    if ($Analizada.Structured -and $evento -eq 'TELEMETRY') {
        if ($ahora.Subtract($script:UltimaMuestraEnVivo['PORTENTA_TELEMETRY']).TotalMilliseconds -lt 250) {
            return
        }
        $script:UltimaMuestraEnVivo['PORTENTA_TELEMETRY'] = $ahora
    }
    if (-not $Analizada.Structured -and
        $Envoltura.Line -notmatch '\[AUTO V2\]|\[V2\]\[DIAG\]|\[ERROR\]') {
        return
    }

    $hora = $Envoltura.Utc.ToLocalTime().ToString('HH:mm:ss.fff')
    $nombre = if ($Analizada.Source -eq 'E') { 'ESP32   ' }
              elseif ($Analizada.Source -eq 'P') { 'PORTENTA' }
              else { 'PC      ' }
    $descripcion = Describir-Evento $Analizada $Envoltura.Line
    $color = if ($evento -in @('CANCEL', 'ERROR', 'DROP')) { 'Red' }
             elseif ($Analizada.Source -eq 'E') { 'Cyan' }
             elseif ($Analizada.Source -eq 'P') { 'Yellow' }
             else { 'Gray' }
    Write-Host "[$hora] [$nombre] $descripcion" -ForegroundColor $color
}

function Nueva-Envoltura([string]$Fuente, [string]$Linea) {
    return [pscustomobject]@{
        Utc = [DateTime]::UtcNow
        ElapsedMs = [Math]::Round($script:Reloj.Elapsed.TotalMilliseconds, 3)
        Source = $Fuente
        Line = $Linea
    }
}

function Leer-LineasDisponibles(
    [System.IO.Ports.SerialPort]$Puerto,
    [string]$Fuente
) {
    $salida = @()
    $fragmento = $Puerto.ReadExisting()
    if ([string]::IsNullOrEmpty($fragmento)) { return $salida }

    $texto = ($script:Buffers[$Fuente] + $fragmento).Replace("`r", '')
    $partes = $texto.Split("`n")
    $script:Buffers[$Fuente] = $partes[$partes.Count - 1]
    for ($i = 0; $i -lt $partes.Count - 1; $i++) {
        if ($partes[$i].Length -gt 0) {
            $salida += Nueva-Envoltura $Fuente $partes[$i]
        }
    }
    return $salida
}

function Agregar-Prebuffer([object]$Envoltura) {
    $prebuffer.Enqueue($Envoltura)
    $limite = $Envoltura.Utc.AddSeconds(-2)
    while ($prebuffer.Count -gt 0 -and $prebuffer.Peek().Utc -lt $limite) {
        [void]$prebuffer.Dequeue()
    }
}

function Obtener-NombreSesion() {
    if (-not (Test-Path -LiteralPath $DirectorioSalida)) {
        [void](New-Item -ItemType Directory -Path $DirectorioSalida)
    }
    $base = 'v2_' + (Get-Date).ToString('yyyy-MM-dd_HH-mm-ss')
    $ruta = Join-Path $DirectorioSalida ($base + '.csv')
    $sufijo = 1
    while (Test-Path -LiteralPath $ruta) {
        $ruta = Join-Path $DirectorioSalida ('{0}_{1:D2}.csv' -f $base, $sufijo)
        $sufijo++
    }
    return $ruta
}

function Iniciar-Sesion() {
    if ($null -ne $script:EscritorSesion) {
        $script:EscritorSesion.Dispose()
    }
    $script:ArchivoSesion = Obtener-NombreSesion
    $utf8ConBom = New-Object System.Text.UTF8Encoding($true)
    $script:EscritorSesion = New-Object System.IO.StreamWriter(
        $script:ArchivoSesion, $false, $utf8ConBom
    )
    $script:EscritorSesion.AutoFlush = $true
    # Coma: coincide con la configuracion de Excel observada en el equipo de
    # pruebas. Todos los valores se escapan entre comillas.
    $script:EscritorSesion.WriteLine(($script:Columnas -join ','))
    Write-Host ('Archivo: ' + $script:ArchivoSesion)
}

function Cerrar-Sesion([string]$Motivo) {
    if ($null -eq $script:EscritorSesion) { return }
    $script:EscritorSesion.Flush()
    $script:EscritorSesion.Dispose()
    $script:EscritorSesion = $null
    $script:ArchivoSesion = $null
    Write-Host ('Estado: sesion cerrada (' + $Motivo + ')')
}

function Escribir-Fila([object]$Envoltura, [object]$Analizada) {
    if ($null -eq $script:EscritorSesion) { return }
    $fila = @{}
    foreach ($columna in $script:Columnas) { $fila[$columna] = '' }
    $fila['pc_utc'] = $Envoltura.Utc.ToString(
        'yyyy-MM-ddTHH:mm:ss.fffZ',
        [System.Globalization.CultureInfo]::InvariantCulture
    )
    $fila['pc_elapsed_ms'] = $Envoltura.ElapsedMs
    $fila['source'] = $Analizada.Source
    $fila['source_name_es'] = if ($Analizada.Source -eq 'E') { 'ESP32' }
        elseif ($Analizada.Source -eq 'P') { 'PORTENTA' }
        else { 'PC' }
    $fila['raw'] = $Envoltura.Line

    if ($Analizada.Structured) {
        foreach ($clave in $Analizada.Data.Keys) {
            if ($script:MapaCampos.ContainsKey($clave)) {
                $fila[$script:MapaCampos[$clave]] = $Analizada.Data[$clave]
            }
        }
        if ($fila['event'] -eq 'RESULT') {
            $fila['result'] = $fila['message']
        }
    } else {
        $fila['event'] = 'RAW'
        $fila['message'] = $Envoltura.Line
    }
    $fila['description_es'] = Describir-Evento $Analizada $Envoltura.Line
    $fila['phase_name_es'] = Nombre-FaseV2 $fila['phase']

    $campos = foreach ($columna in $script:Columnas) {
        Convertir-CampoCsv $fila[$columna]
    }
    $script:EscritorSesion.WriteLine(($campos -join ','))
}

function Escribir-EventoHost([string]$Evento, [string]$Mensaje) {
    if ($null -eq $script:EscritorSesion) { return }
    $linea = 'V2LOG|H|event=' + $Evento + '|message=' + $Mensaje.Replace('|', '/')
    $envoltura = Nueva-Envoltura 'H' $linea
    $analizada = Analizar-Linea $linea 'H'
    Escribir-Fila $envoltura $analizada
}

$PuertoESP = Pedir-Puerto 'ESP32' $PuertoESP
$PuertoPortenta = Pedir-Puerto 'Portenta' $PuertoPortenta
Validar-Puerto $PuertoESP 'ESP32'
Validar-Puerto $PuertoPortenta 'Portenta'
if ($PuertoESP -eq $PuertoPortenta) {
    throw 'La ESP32 y la Portenta deben usar puertos COM distintos.'
}

$serialESP = $null
$serialPortenta = $null
$lecturaIniciada = $false
$datosVistos = @{ E = $false; P = $false }
$avisoSinDatosMostrado = $false
$inicioLecturaUtc = [DateTime]::UtcNow
try {
    $serialESP = Abrir-Puerto $PuertoESP 460800 $false
    Write-Host "Estado: ESP32 conectada en $PuertoESP a 460800"
    $serialPortenta = Abrir-Puerto $PuertoPortenta 115200 $true
    Write-Host "Estado: Portenta conectada en $PuertoPortenta a 115200"
    Write-Host 'Estado: esperando entrada a Automatico V2'
    $lecturaIniciada = $true

    while ($true) {
        $entradas = @()
        $entradas += Leer-LineasDisponibles $serialESP 'E'
        $entradas += Leer-LineasDisponibles $serialPortenta 'P'

        foreach ($envoltura in ($entradas | Sort-Object Utc)) {
            if (-not $datosVistos[$envoltura.Source]) {
                $datosVistos[$envoltura.Source] = $true
                $nombreFuente = if ($envoltura.Source -eq 'P') {
                    'Portenta'
                } else { 'ESP32' }
                Write-Host ('Estado: recibiendo datos de ' + $nombreFuente)
            }
            Agregar-Prebuffer $envoltura
            $analizada = Analizar-Linea $envoltura.Line $envoltura.Source
            $evento = if ($analizada.Data.ContainsKey('event')) {
                $analizada.Data['event']
            } else { '' }
            # SESSION_START es la referencia normal. TELEMETRY/PHASE y los
            # demas eventos permiten conectar el registrador cuando la placa
            # ya estaba dentro de V2 y la linea de entrada ocurrio antes.
            $eventosQueConfirmanV2 = @(
                'SESSION_START', 'TELEMETRY', 'PHASE', 'ACCEPT', 'REJECT',
                'ACK', 'TRIGGER', 'READY_CATCH', 'BUTTON_X', 'CAPTURE',
                'RESULT', 'CANCEL', 'ERROR'
            )
            $eventoConfirmaV2 = $eventosQueConfirmanV2 -contains $evento
            $estadoGeneralLegado = $null
            if ($analizada.Source -eq 'P' -and
                $envoltura.Line -match '\[BOOT\]\s+Estado general ->\s*(\d+)') {
                $estadoGeneralLegado = [int]$Matches[1]
            }
            if ($analizada.Source -eq 'P' -and
                (($analizada.Structured -and $evento -eq 'SESSION_END') -or
                 ($null -ne $estadoGeneralLegado -and $estadoGeneralLegado -ne 11))) {
                $script:BloqueadoHastaSalirV2 = $false
            }
            $esInicioEstructurado = $null -eq $script:EscritorSesion -and
                $analizada.Structured -and $analizada.Source -eq 'P' -and
                $eventoConfirmaV2 -and
                (-not $script:BloqueadoHastaSalirV2 -or
                 $evento -eq 'SESSION_START')
            # El estado Wire 11 identifica Automatico V2. Esta linea se emite
            # siempre, incluso si la banda ya estaba en movimiento y no se
            # imprime el aviso tradicional de espera.
            $esEntradaEstadoLegado = $null -eq $script:EscritorSesion -and
                -not $script:BloqueadoHastaSalirV2 -and
                $estadoGeneralLegado -eq 11
            # Compatibilidad con una Portenta que aun no tenga cargados los
            # eventos V2LOG: la primera linea tradicional de Automatico V2
            # tambien abre una sesion y el prebuffer conserva lo anterior.
            $esInicioLegado = $null -eq $script:EscritorSesion -and
                -not $script:BloqueadoHastaSalirV2 -and
                $analizada.Source -eq 'P' -and
                $envoltura.Line -match '\[AUTO V2\]'
            $esInicio = $esInicioEstructurado -or $esEntradaEstadoLegado -or
                $esInicioLegado
            $esSalidaEstadoLegado = $null -ne $script:EscritorSesion -and
                $null -ne $estadoGeneralLegado -and
                $estadoGeneralLegado -ne 11
            $esCancelacionLegada = $analizada.Source -eq 'P' -and
                $envoltura.Line -match '\[AUTO V2\]\s+Cancelando:'

            if ($null -ne $script:EscritorSesion -or $esInicio) {
                Mostrar-EventoEnVivo $envoltura $analizada
            }

            if ($esInicio) {
                if ($null -ne $script:EscritorSesion) {
                    Escribir-EventoHost 'SESSION_RESTART' 'nuevo inicio sin cierre previo'
                    Cerrar-Sesion 'reinicio inesperado'
                }
                Iniciar-Sesion
                foreach ($anterior in $prebuffer.ToArray()) {
                    Escribir-Fila $anterior (Analizar-Linea $anterior.Line $anterior.Source)
                }
                if ($analizada.Structured -and $evento -eq 'CANCEL') {
                    $script:BloqueadoHastaSalirV2 = $true
                    Cerrar-Sesion 'CANCEL'
                } elseif ($esCancelacionLegada) {
                    $script:BloqueadoHastaSalirV2 = $true
                    Cerrar-Sesion 'CANCEL legado'
                }
                continue
            }

            if ($null -ne $script:EscritorSesion) {
                Escribir-Fila $envoltura $analizada
                if ($analizada.Structured -and $analizada.Source -eq 'P' -and
                    ($evento -eq 'SESSION_END' -or $evento -eq 'CANCEL')) {
                    if ($evento -eq 'CANCEL') {
                        $script:BloqueadoHastaSalirV2 = $true
                    }
                    Cerrar-Sesion $evento
                } elseif ($esCancelacionLegada) {
                    $script:BloqueadoHastaSalirV2 = $true
                    Cerrar-Sesion 'CANCEL legado'
                } elseif ($esSalidaEstadoLegado) {
                    Cerrar-Sesion ('salida a estado ' + $estadoGeneralLegado)
                }
            }
        }
        if (-not $avisoSinDatosMostrado -and
            [DateTime]::UtcNow.Subtract($inicioLecturaUtc).TotalSeconds -ge 5 -and
            (-not $datosVistos['E'] -or -not $datosVistos['P'])) {
            $faltantes = @()
            if (-not $datosVistos['E']) { $faltantes += 'ESP32' }
            if (-not $datosVistos['P']) { $faltantes += 'Portenta' }
            Write-Host ('ERROR: no se reciben datos de: ' +
                ($faltantes -join ', ') +
                '. Cierre los monitores seriales de Arduino.') -ForegroundColor Red
            $avisoSinDatosMostrado = $true
        }
        Start-Sleep -Milliseconds 5
    }
}
catch {
    $mensaje = $_.Exception.Message
    Write-Host ('ERROR: ' + $mensaje) -ForegroundColor Red
    # Si el USB se pierde antes de observar SESSION_START, se conserva de
    # todas formas el prebuffer y la causa. Esto permite diagnosticar reinicios
    # o ruido USB que antes terminaban sin crear ningun archivo.
    if ($lecturaIniciada -and $null -eq $script:EscritorSesion) {
        try {
            Iniciar-Sesion
            foreach ($anterior in $prebuffer.ToArray()) {
                Escribir-Fila $anterior (Analizar-Linea $anterior.Line $anterior.Source)
            }
        } catch {
            Write-Host ('ERROR: no se pudo crear el CSV de emergencia: ' +
                $_.Exception.Message) -ForegroundColor Red
        }
    }
    Escribir-EventoHost 'CONNECTION_LOST' $mensaje
    Cerrar-Sesion 'perdida de comunicacion'
}
finally {
    if ($null -ne $script:EscritorSesion) {
        Escribir-EventoHost 'LOGGER_STOP' 'registrador detenido'
        Cerrar-Sesion 'registrador detenido'
    }
    if ($null -ne $serialESP) {
        try { if ($serialESP.IsOpen) { $serialESP.Close() } } catch {}
        $serialESP.Dispose()
    }
    if ($null -ne $serialPortenta) {
        try { if ($serialPortenta.IsOpen) { $serialPortenta.Close() } } catch {}
        $serialPortenta.Dispose()
    }
}
