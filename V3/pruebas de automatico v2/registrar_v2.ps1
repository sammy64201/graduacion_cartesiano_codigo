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
    'catch_type', 'trigger', 'servo_rot_deg', 'label_rot_deg',
    'approx_rot_deg', 'orientation_votes_x', 'orientation_votes_y',
    'suggested_rot_deg', 'correction_deg', 'aspect_ratio',
    'suggestion_source',
    'physical_result', 'camera_x_mm', 'camera_y_mm', 'piece_y_mm',
    'error_disparo_mm',
    'error_disparo_ms', 'mode', 'trial', 'trial_elapsed_ms',
    'scale_mm_count', 'encoder_sign', 'camera_ref_valid',
    'camera_ref_x_mm', 'camera_ref_y_mm', 'camera_ref_encoder',
    'predicted_y_mm', 'camera_sign_y', 'camera_swap_xy',
    'camera_distance_mm', 'camera_offset_y_mm', 'mark_index',
    'observation_encoder', 'observed_arm_y_mm', 'predicted_observation_y_mm',
    'camera_encoder_error_y_mm', 'message', 'raw',
    'auto_x_mm', 'auto_y_mm', 'label_x_mm', 'label_y_mm',
    'delta_x_mm', 'delta_y_mm', 'catch_y_mm', 'close_threshold_y_mm',
    'angle_origin', 'angle_before_deg', 'angle_after_deg',
    'adjust_start_ms', 'adjust_end_ms', 'catch_command_ms',
    'detection_to_command_ms', 'close_elapsed_ms', 'advance_mm',
    'estimated_close_error_mm', 'anticipation_ms',
    'predicted_close_y_mm', 'suggested_advance_extra_ms',
    'physical_error_measured', 'catch_button_ms', 'catch_button_encoder',
    'catch_button_piece_y_mm', 'z_bottom_ms', 'button_to_grip_ms',
    'camera_speed_valid', 'camera_speed_mm_s', 'camera_vy_mm_s',
    'encoder_window_mm_s', 'speed_error_mm_s', 'camera_travel_mm',
    'encoder_travel_mm', 'camera_interval_ms', 'camera_y_first_mm',
    'camera_y_last_mm', 'camera_unique_y',
    'track_arm_y_button_mm', 'track_error_button_mm',
    'track_arm_y_bottom_mm', 'track_error_bottom_mm',
    'track_arm_y_close_mm', 'track_error_close_mm',
    'track_before_button_ms', 'timing_result', 'auto_reference_ms',
    'reference_status', 'catch_start_ms', 'delta_reference_ms',
    'trigger_error_y_mm', 'trigger_speed_mm_s', 'trigger_encoder',
    'trigger_arm_y_mm', 'trigger_piece_y_mm',
    'scale_ratio_camera_encoder', 'scale_camera_suggested_mm_count',
    'tested_offset_ms', 'next_offset_ms', 'adjust_step_ms', 'adjust_trials', 'adjust_success_streak', 'adjust_confirmed', 'adjust_limit'
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
    'encoder_velocity_mm_s' = 'encoder_velocity_mm_s'
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
    'catch_type'  = 'catch_type'
    'trigger'     = 'trigger'
    'servo_rot_deg' = 'servo_rot_deg'
    'label_rot' = 'label_rot_deg'
    'approx_rot' = 'approx_rot_deg'
    'votes_x' = 'orientation_votes_x'
    'votes_y' = 'orientation_votes_y'
    'suggested_rot' = 'suggested_rot_deg'
    'correction_deg' = 'correction_deg'
    'aspect_ratio' = 'aspect_ratio'
    'suggestion_source' = 'suggestion_source'
    'physical_result' = 'physical_result'
    'cam_x' = 'camera_x_mm'
    'cam_y' = 'camera_y_mm'
    'piece_y' = 'piece_y_mm'
    'error_disparo_mm' = 'error_disparo_mm'
    'error_disparo_ms' = 'error_disparo_ms'
    'mode' = 'mode'
    'trial' = 'trial'
    'elapsed_ms' = 'trial_elapsed_ms'
    'scale_mm_count' = 'scale_mm_count'
    'encoder_sign' = 'encoder_sign'
    'camera_ref_valid' = 'camera_ref_valid'
    'camera_ref_x' = 'camera_ref_x_mm'
    'camera_ref_y' = 'camera_ref_y_mm'
    'camera_ref_encoder' = 'camera_ref_encoder'
    'predicted_y' = 'predicted_y_mm'
    'camera_sign_y' = 'camera_sign_y'
    'camera_swap_xy' = 'camera_swap_xy'
    'camera_distance_mm' = 'camera_distance_mm'
    'camera_offset_y_mm' = 'camera_offset_y_mm'
    'mark' = 'mark_index'
    'observation_encoder' = 'observation_encoder'
    'observed_arm_y' = 'observed_arm_y_mm'
    'predicted_observation_y' = 'predicted_observation_y_mm'
    'camera_encoder_error_y' = 'camera_encoder_error_y_mm'
    'auto_x' = 'auto_x_mm'
    'auto_y' = 'auto_y_mm'
    'label_x' = 'label_x_mm'
    'label_y' = 'label_y_mm'
    'delta_x' = 'delta_x_mm'
    'delta_y' = 'delta_y_mm'
    'catch_y' = 'catch_y_mm'
    'close_threshold_y' = 'close_threshold_y_mm'
    'angle_origin' = 'angle_origin'
    'angle_before' = 'angle_before_deg'
    'angle_after' = 'angle_after_deg'
    'adjust_start_ms' = 'adjust_start_ms'
    'adjust_end_ms' = 'adjust_end_ms'
    'catch_command_ms' = 'catch_command_ms'
    'detection_to_command_ms' = 'detection_to_command_ms'
    'close_elapsed_ms' = 'close_elapsed_ms'
    'advance_mm' = 'advance_mm'
    'error_y_estimado_mm' = 'estimated_close_error_mm'
    'anticipacion_ms' = 'anticipation_ms'
    'cierre_y_predicha' = 'predicted_close_y_mm'
    'adelanto_extra_sugerido_ms' = 'suggested_advance_extra_ms'
    'error_fisico_medido' = 'physical_error_measured'
    'catch_button_ms' = 'catch_button_ms'
    'catch_button_encoder' = 'catch_button_encoder'
    'catch_button_piece_y' = 'catch_button_piece_y_mm'
    'z_bottom_ms' = 'z_bottom_ms'
    'button_to_grip_ms' = 'button_to_grip_ms'
    'camera_speed_valid' = 'camera_speed_valid'
    'camera_speed_mm_s' = 'camera_speed_mm_s'
    'camera_vy_mm_s' = 'camera_vy_mm_s'
    'encoder_window_mm_s' = 'encoder_window_mm_s'
    'speed_error_mm_s' = 'speed_error_mm_s'
    'camera_travel_mm' = 'camera_travel_mm'
    'encoder_travel_mm' = 'encoder_travel_mm'
    'camera_interval_ms' = 'camera_interval_ms'
    'camera_y_first_mm' = 'camera_y_first_mm'
    'camera_y_last_mm' = 'camera_y_last_mm'
    'camera_unique_y' = 'camera_unique_y'
    'track_arm_y_button' = 'track_arm_y_button_mm'
    'track_error_button' = 'track_error_button_mm'
    'track_arm_y_bottom' = 'track_arm_y_bottom_mm'
    'track_error_bottom' = 'track_error_bottom_mm'
    'track_arm_y_close' = 'track_arm_y_close_mm'
    'track_error_close' = 'track_error_close_mm'
    'track_before_button_ms' = 'track_before_button_ms'
    'timing_result' = 'timing_result'
    'tested_offset_ms' = 'tested_offset_ms'
    'next_offset_ms' = 'next_offset_ms'
    'adjust_step_ms' = 'adjust_step_ms'
    'adjust_trials' = 'adjust_trials'
    'adjust_success_streak' = 'adjust_success_streak'
    'adjust_confirmed' = 'adjust_confirmed'
    'adjust_limit' = 'adjust_limit'
    'auto_reference_ms' = 'auto_reference_ms'
    'reference_status' = 'reference_status'
    'catch_start_ms' = 'catch_start_ms'
    'delta_reference_ms' = 'delta_reference_ms'
    'trigger_error_y' = 'trigger_error_y_mm'
    'trigger_speed_mm_s' = 'trigger_speed_mm_s'
    'trigger_encoder' = 'trigger_encoder'
    'trigger_arm_y' = 'trigger_arm_y_mm'
    'trigger_piece_y' = 'trigger_piece_y_mm'
    'camera_x' = 'camera_x_mm'
    'camera_y' = 'camera_y_mm'
    'scale_ratio_camera_encoder' = 'scale_ratio_camera_encoder'
    'scale_camera_suggested_mm_count' = 'scale_camera_suggested_mm_count'
}

$script:Reloj = [System.Diagnostics.Stopwatch]::StartNew()
$script:EscritorSesion = $null
$script:ArchivoSesion = $null
$script:EscritorDiagnostico = $null
$script:ArchivoDiagnostico = $null
$script:ContadorNombre = 0
$script:Buffers = @{ E = ''; P = '' }
$script:BloqueadoHastaSalirV2 = $false
$script:TipoSesion = 'v2'
$script:UltimaMuestraEnVivo = @{
    ESP_IDLE = [DateTime]::MinValue
    PORTENTA_TELEMETRY = [DateTime]::MinValue
    ENCODER_TEST_SAMPLE = [DateTime]::MinValue
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
    # ML_SAMPLE tambien se reconoce con firmware anterior (trigger=X/ENCODER).
    if ($Linea.StartsWith('ML_SAMPLE|')) {
        foreach ($parte in $Linea.Split('|') | Select-Object -Skip 1) {
            $posicion = $parte.IndexOf('=')
            if ($posicion -le 0) { continue }
            $datos[$parte.Substring(0, $posicion)] = $parte.Substring($posicion + 1)
        }
        $datos['event'] = 'ML_SAMPLE'
        if ($datos.ContainsKey('seq')) { $datos['obj'] = $datos['seq'] }
        if ($datos.ContainsKey('belt_mm_s')) { $datos['vel'] = $datos['belt_mm_s'] }
        if ($datos.ContainsKey('trigger')) {
            $datos['catch_type'] = if ($datos['trigger'] -eq 'X') { 'MANUAL' }
                elseif ($datos['trigger'] -eq 'ENCODER') { 'AUTOMATICO' }
                else { 'DESCONOCIDO' }
        }
        $estructurada = $true
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
        '4' { return 'BAJANDO_PRECAPTURA' }
        '5' { return 'SUBIENDO_Z' }
        '6' { return 'ESPERANDO_CATCH_AUTOMATICO' }
        '7' { return 'COMPLETADO' }
        '8' { return 'CANCELANDO' }
        '9' { return 'PREPARANDO_ESPERA' }
        '10' { return 'CERRANDO_PINZA' }
        '11' { return 'BAJANDO_CATCH' }
        '12' { return 'MOVIENDO_ENTREGA' }
        '13' { return 'BAJANDO_ENTREGA' }
        '14' { return 'ABRIENDO_PINZA' }
        '15' { return 'SUBIENDO_FINAL' }
        '16' { return 'SIGUIENDO_PIEZA_EN_Y' }
        '17' { return 'EVALUANDO_CATCH' }
        default { return '' }
    }
}

function Nombre-FaseML([string]$Fase) {
    switch ($Fase) {
        '0' { return 'ESPERANDO_PIEZA' }
        '1' { return 'PREPOSICIONANDO' }
        '2' { return 'BAJANDO_PRECAPTURA' }
        '3' { return 'ALINEACION_MANUAL' }
        '4' { return 'CERRANDO_PINZA' }
        '5' { return 'SUBIENDO_CON_PIEZA' }
        '6' { return 'MOVIENDO_ENTREGA' }
        '7' { return 'BAJANDO_ENTREGA' }
        '8' { return 'ABRIENDO_PINZA' }
        '9' { return 'SUBIENDO_FINAL' }
        '10' { return 'LISTO' }
        '11' { return 'CANCELANDO' }
        '12' { return 'PREPARANDO_ESPERA' }
        '13' { return 'BAJANDO_CATCH' }
        '14' { return 'ESPERANDO_CONFIRMACION' }
        '15' { return 'SIGUIENDO_PIEZA_EN_Y' }
        default { return '' }
    }
}

function Nombre-FaseAngulo([string]$Fase) {
    switch ($Fase) {
        '0' { return 'PREPARANDO_Z' }
        '1' { return 'ESPERANDO_PIEZA' }
        '2' { return 'MOVIENDO_XY' }
        '3' { return 'AJUSTANDO_GIRO' }
        '4' { return 'BAJANDO_Z' }
        '5' { return 'CERRANDO_PINZA' }
        '6' { return 'SUBIENDO_Z' }
        '7' { return 'CANCELANDO' }
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
        '5' { return 'CERRAR_PINZA' }
        '6' { return 'ABRIR_PINZA' }
        default { return $Codigo }
    }
}

function Describir-Evento([object]$Analizada, [string]$Linea) {
    if (-not $Analizada.Structured) { return $Linea }
    $evento = Obtener-Dato $Analizada 'event'
    $causa = Obtener-Dato $Analizada 'cause'
    $fase = if ((Obtener-Dato $Analizada 'mode') -in @('ML', 'ML_V2', 'ML_TRACK')) {
        Nombre-FaseML (Obtener-Dato $Analizada 'phase')
    } elseif ((Obtener-Dato $Analizada 'mode') -eq 'ANGLE_LABEL') {
        Nombre-FaseAngulo (Obtener-Dato $Analizada 'phase')
    } else { Nombre-FaseV2 (Obtener-Dato $Analizada 'phase') }
    $mensaje = Obtener-Dato $Analizada 'message'
    $obj = Obtener-Dato $Analizada 'obj'
    $enc = Obtener-Dato $Analizada 'enc'
    $vel = Obtener-Dato $Analizada 'vel'

    switch ($evento) {
        'CAL_REFERENCE' { return "AJUSTE CATCH #$obj | referencia nominal de alineacion (300 ms)" }
        'CAL_TRIGGER' { return "AJUSTE CATCH #$obj | descenso automatico | diferencia=" + (Obtener-Dato $Analizada 'delta_reference_ms') + ' ms | referencia=' + (Obtener-Dato $Analizada 'reference_status') }
        'CAL_GRIP' { return "AJUSTE CATCH #$obj | DIN04 confirmado; orden de cierre" }
        'CAL_GRIP_APPLIED' { return "AJUSTE CATCH #$obj | ESP aplico el pulso de cierre" }
        'CAL_CLOSE' { return "AJUSTE CATCH #$obj | tiempo de cierre completado; agarre sin verificar" }
        'CAL_AWAIT_FEEDBACK' { return "AJUSTE CATCH #$obj | X=LA AGARRO; cuadrado=ANTES; circulo=DESPUES" }
        'CAL_SAMPLE' { return "AJUSTE CATCH #$obj | resultado=" + (Obtener-Dato $Analizada 'timing_result') + ' | probado=' + (Obtener-Dato $Analizada 'tested_offset_ms') + ' ms | proximo=' + (Obtener-Dato $Analizada 'next_offset_ms') + ' ms | agarres=' + (Obtener-Dato $Analizada 'adjust_success_streak') + '/3' }
        'CAL_REJECT' { return "AJUSTE CATCH #$obj | X ignorada: todavia no esta alineado X/Y" }
        'ENCODER_TEST_START' { return 'PRUEBA DE ENCODER LISTA | X: iniciar/detener; circulo: cero; triangulo: salir | sin catch ni control de banda' }
        'ENCODER_TEST_END' { return 'PRUEBA DE ENCODER TERMINADA | CSV guardado' }
        'ENCODER_TEST_RESET' { return 'CONTADOR A CERO | X: iniciar otra medicion' }
        'ENCODER_TEST_RUN_START' { return 'MEDICION #' + (Obtener-Dato $Analizada 'trial') + ' INICIADA CON X | pulsos=0 | tiempo=0 s' }
        'ENCODER_TEST_REFERENCE' { return 'REFERENCIA CAMARA FIJADA | una sola pieza; referencia conservada hasta circulo' }
        'ENCODER_TEST_OBSERVATION' {
            return 'CAMARA VS ENCODER | medicion #' + (Obtener-Dato $Analizada 'trial') +
                ' | diferencia Y=' + (Obtener-Dato $Analizada 'camera_encoder_error_y') +
                ' mm (misma pieza; no es error fisico medido)'
        }
        { $_ -in @('ENCODER_TEST_SAMPLE', 'ENCODER_TEST_MARK') } {
            $titulo = if ($evento -eq 'ENCODER_TEST_MARK') { 'MEDICION DETENIDA CON X' }
                else { 'MIDIENDO' }
            $tiempoMs = Obtener-Dato $Analizada 'elapsed_ms'
            return $titulo + ' | pulsos=' + (Obtener-Dato $Analizada 'pulses') +
                ' | tiempo=' + $tiempoMs + ' ms'
        }
        'ML_SAMPLE' {
            $tipo = Obtener-Dato $Analizada 'catch_type'
            $disparador = Obtener-Dato $Analizada 'trigger'
            $servo = Obtener-Dato $Analizada 'servo_rot_deg'
            $errorMs = Obtener-Dato $Analizada 'error_disparo_ms'
            $resultado = Obtener-Dato $Analizada 'physical_result'
            if ($resultado -eq '') { $resultado = 'SIN CONFIRMAR' }
            return "CATCH $tipo ($disparador) | objetivo #$obj | velocidad=$vel mm/s | orientacion=$servo grados | diferencia con prediccion=$errorMs ms | agarre=$resultado"
        }
        'ML_BUTTON_CATCH' {
            return "ML X CATCH #$obj | Portenta ms=" +
                (Obtener-Dato $Analizada 'board_ms') + ' | encoder=' + $enc +
                ' | error seguimiento=' + (Obtener-Dato $Analizada 'error_y') + ' mm'
        }
        'ML_TRACK' {
            return "SEGUIMIENTO #$obj | pieza Y=" +
                (Obtener-Dato $Analizada 'piece_y') + ' mm | brazo Y=' +
                (Obtener-Dato $Analizada 'arm_y') + ' mm | error=' +
                (Obtener-Dato $Analizada 'error_y') + ' mm'
        }
        'ML_TRACK_REJECT' { return "X IGNORADA #$obj | falta recorrido Y para terminar el catch" }
        'ML_AWAIT_FEEDBACK' { return "ML V2 #$obj | confirme agarre: X=si, cuadrado=no" }
        'ML_RESULT' {
            return "ML V2 #$obj | resultado fisico=" +
                (Obtener-Dato $Analizada 'physical_result')
        }
        'CAMERA_SPEED' {
            return "CAMARA VS ENCODER #$obj | camara=" +
                (Obtener-Dato $Analizada 'camera_speed_mm_s') +
                ' mm/s | encoder=' + (Obtener-Dato $Analizada 'encoder_window_mm_s') +
                ' mm/s | valida=' + (Obtener-Dato $Analizada 'camera_speed_valid')
        }
        { $_ -in @('ML_ANGLE_SUGGESTION', 'AUTO_ANGLE_SUGGESTION') } {
            return "PIEZA #$obj | angulo sugerido=" +
                (Obtener-Dato $Analizada 'suggested_rot') +
                ' grados | servo=' + (Obtener-Dato $Analizada 'servo_rot_deg') +
                ' grados'
        }
        'ML_ANGLE_ADJUST' {
            return "ML AJUSTE MANUAL #$obj | angulo " +
                (Obtener-Dato $Analizada 'angle_before') + ' -> ' +
                (Obtener-Dato $Analizada 'angle_after') +
                ' grados | ESP ms=' + (Obtener-Dato $Analizada 'adjust_end_ms')
        }
        'ML_CATCH_TRIGGER' {
            return "ML ORDEN CATCH #$obj | " +
                (Obtener-Dato $Analizada 'catch_type') + ' (' +
                (Obtener-Dato $Analizada 'trigger') + ') | Portenta ms=' +
                (Obtener-Dato $Analizada 'ms') + ' | angulo=' +
                (Obtener-Dato $Analizada 'servo_rot_deg') + ' grados'
        }
        'ML_GRIP_APPLIED' {
            return "ML SERVO CERRADO #$obj | ESP ms=" +
                (Obtener-Dato $Analizada 'ms') + ' | angulo=' +
                (Obtener-Dato $Analizada 'servo_rot_deg') + ' grados'
        }
        'ML_CLOSE' {
            return "ML CIERRE ESTIMADO #$obj | tipo=" +
                (Obtener-Dato $Analizada 'catch_type') + ' | error Y estimado=' +
                (Obtener-Dato $Analizada 'error_y_estimado_mm') + ' mm'
        }
        'ANGLE_SAMPLE' {
            $servo = Obtener-Dato $Analizada 'servo_rot_deg'
            return "ANGULO GUARDADO | objetivo #$obj | servo=$servo grados | catch con banda detenida; agarre fisico no verificado"
        }
        'ANGLE_DETECTION' { return "PIEZA DETECTADA | objetivo #$obj | caja guardada; confianza si disponible" }
        'ANGLE_FEEDBACK' {
            return "GIRO APLICADO | objetivo #$obj | sugerido=" +
                (Obtener-Dato $Analizada 'suggested_rot') +
                ' grados | final=' + (Obtener-Dato $Analizada 'servo_rot_deg') +
                ' grados | correccion=' + (Obtener-Dato $Analizada 'correction_deg')
        }
        'READY_ANGLE' { return "BRAZO POSICIONADO | ajustar el giro con joystick derecho" }
        'SESSION_START' {
            if ((Obtener-Dato $Analizada 'mode') -eq 'ML') {
                return 'ENTRADA A ENSENANZA ML | catch con X o encoder; giro ajustable'
            }
            if ((Obtener-Dato $Analizada 'mode') -eq 'ML_TRACK') {
                return 'ENTRADA A SEGUIMIENTO Y | X inicia catch; giro ajustable'
            }
            if ((Obtener-Dato $Analizada 'mode') -eq 'ML_V2') {
                return 'ENTRADA A ENSENANZA ML V2 | catch con X'
            }
            if ((Obtener-Dato $Analizada 'mode') -eq 'ANGLE_LABEL') {
                return 'ENTRADA A REGISTRAR ANGULO | banda detenida'
            }
            return 'ENTRADA A AUTOMATICO V2'
        }
        'SESSION_END' { return 'SALIDA DE MODO: ' + $mensaje }
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
        'TRIGGER' { return "DESCENSO DE Z | objetivo #${obj}: $mensaje" }
        'AUTO_TRACK' { return "SEGUIMIENTO AUTOMATICO #$obj | brazo Y=" + (Obtener-Dato $Analizada 'arm_y') + ' mm | error=' + (Obtener-Dato $Analizada 'error_y') + ' mm' }
        'RELEASE' { return "ENTREGA DERECHA #$obj | apertura de pinza; falta retirada final de Z" }
        'BUTTON_X' {
            if ((Obtener-Dato $Analizada 'mode') -eq 'CATCH_CAL') { return "X RECIBIDA | ajuste de catch #$obj fase=$fase" }
            return "X RECIBIDA E IGNORADA | Automatico V2 no requiere confirmacion | objetivo #$obj fase=$fase"
        }
        'ACCEPT' { return "OBJETIVO #$obj ACEPTADO | encoder=$enc velocidad=$vel mm/s" }
        'REJECT' { return "OBJETIVO #$obj RECHAZADO: $mensaje" }
        'ACK' {
            $ack = Nombre-AckV2 (Obtener-Dato $Analizada 'ack_code')
            return "ACK objetivo #${obj}: $ack"
        }
        'READY_CATCH' {
            $piezaY = Obtener-Dato $Analizada 'target_y'
            return "Z EN PRECAPTURA | objetivo #$obj | pieza estimada Y=$piezaY mm | catch automatico | encoder=$enc"
        }
        'GRIP_COMMAND' {
            $piezaY = Obtener-Dato $Analizada 'target_y'
            return "ORDEN CERRAR GARRA | objetivo #$obj pieza estimada Y=$piezaY mm encoder=$enc velocidad=$vel mm/s"
        }
        'CAPTURE' {
            if ($mensaje -eq 'PINZA CERRADA POR PREDICCION DE ENCODER') {
                $piezaY = Obtener-Dato $Analizada 'target_y'
                return "CIERRE ORDENADO COMPLETADO | objetivo #$obj pieza estimada Y=$piezaY mm encoder=$enc velocidad=$vel mm/s | agarre fisico no verificado"
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
    if ($evento -in @('ENCODER_TEST_SAMPLE', 'ENCODER_TEST_OBSERVATION')) {
        if ($ahora.Subtract($script:UltimaMuestraEnVivo['ENCODER_TEST_SAMPLE']).TotalMilliseconds -lt 500) {
            return
        }
        $script:UltimaMuestraEnVivo['ENCODER_TEST_SAMPLE'] = $ahora
    }
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
    $base = $script:TipoSesion + '_' + (Get-Date).ToString('yyyy-MM-dd_HH-mm-ss')
    $ruta = Join-Path $DirectorioSalida ($base + '.csv')
    $sufijo = 1
    while (Test-Path -LiteralPath $ruta) {
        $ruta = Join-Path $DirectorioSalida ('{0}_{1:D2}.csv' -f $base, $sufijo)
        $sufijo++
    }
    return $ruta
}

function Iniciar-DiagnosticoContinuo() {
    if (-not (Test-Path -LiteralPath $DirectorioSalida)) {
        [void](New-Item -ItemType Directory -Path $DirectorioSalida)
    }
    $base = 'i2c_' + (Get-Date).ToString('yyyy-MM-dd_HH-mm-ss')
    $ruta = Join-Path $DirectorioSalida ($base + '.log')
    $sufijo = 1
    while (Test-Path -LiteralPath $ruta) {
        $ruta = Join-Path $DirectorioSalida ('{0}_{1:D2}.log' -f $base, $sufijo)
        $sufijo++
    }
    $utf8ConBom = New-Object System.Text.UTF8Encoding($true)
    $script:EscritorDiagnostico = New-Object System.IO.StreamWriter(
        $ruta, $false, $utf8ConBom
    )
    $script:EscritorDiagnostico.AutoFlush = $true
    $script:ArchivoDiagnostico = $ruta
    $script:EscritorDiagnostico.WriteLine('pc_utc|pc_elapsed_ms|source|raw')
    Write-Host ('Diagnostico I2C continuo: ' + $ruta)
}

function Escribir-DiagnosticoContinuo([object]$Envoltura) {
    if ($null -eq $script:EscritorDiagnostico) { return }
    $lineaSegura = $Envoltura.Line.Replace("`r", '').Replace("`n", ' ')
    $script:EscritorDiagnostico.WriteLine(('{0}|{1}|{2}|{3}' -f
        $Envoltura.Utc.ToString('yyyy-MM-ddTHH:mm:ss.fffZ'),
        $Envoltura.ElapsedMs,
        $Envoltura.Source,
        $lineaSegura))
}

function Mostrar-DiagnosticoContinuo([object]$Envoltura) {
    if ($Envoltura.Line -notmatch '^\[(I2CDBG|RESET)\]') { return }
    $hora = $Envoltura.Utc.ToLocalTime().ToString('HH:mm:ss.fff')
    $nombre = if ($Envoltura.Source -eq 'E') { 'ESP32   ' } else { 'PORTENTA' }
    $color = if ($Envoltura.Source -eq 'E') { 'Cyan' } else { 'Yellow' }
    Write-Host "[$hora] [$nombre] $($Envoltura.Line)" -ForegroundColor $color
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
    if ($script:TipoSesion -eq 'encoder') {
        $script:EscritorSesion.WriteLine('ensayo,contador_pulsos,tiempo_s')
    } else {
        $script:EscritorSesion.WriteLine(($script:Columnas -join ','))
    }
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
    # Una fila por medicion completa; diagnosticos y muestras no son ensayos.
    if ($script:TipoSesion -eq 'encoder') {
        if ($Analizada.Source -eq 'P' -and
            (Obtener-Dato $Analizada 'mode') -eq 'ENCODER_TEST' -and
            (Obtener-Dato $Analizada 'event') -eq 'ENCODER_TEST_MARK') {
            $ensayo = Obtener-Dato $Analizada 'trial'
            $pulsos = Obtener-Dato $Analizada 'pulses'
            $tiempoMs = Obtener-Dato $Analizada 'elapsed_ms'
            if ($ensayo -match '^\d+$' -and $pulsos -match '^\d+$' -and
                $tiempoMs -match '^\d+$') {
                $segundos = ([double]::Parse($tiempoMs,
                    [System.Globalization.CultureInfo]::InvariantCulture) / 1000.0).ToString(
                    '0.000', [System.Globalization.CultureInfo]::InvariantCulture)
                $script:EscritorSesion.WriteLine("$ensayo,$pulsos,$segundos")
            }
        }
        return
    }
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
    $fila['phase_name_es'] = if ($fila['mode'] -eq 'ML') {
        Nombre-FaseML $fila['phase']
    } elseif ($fila['mode'] -eq 'ANGLE_LABEL') {
        Nombre-FaseAngulo $fila['phase']
    } else { Nombre-FaseV2 $fila['phase'] }

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
    Iniciar-DiagnosticoContinuo
    Write-Host 'Estado: esperando Automatico V2, Ensenanza ML o Seguimiento Y'
    $lecturaIniciada = $true

    while ($true) {
        $entradas = @()
        $entradas += Leer-LineasDisponibles $serialESP 'E'
        $entradas += Leer-LineasDisponibles $serialPortenta 'P'

        foreach ($envoltura in ($entradas | Sort-Object Utc)) {
            Escribir-DiagnosticoContinuo $envoltura
            Mostrar-DiagnosticoContinuo $envoltura
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
                'ACK', 'TRIGGER', 'READY_CATCH', 'GRIP_COMMAND', 'BUTTON_X', 'CAPTURE',
                'RESULT', 'CANCEL', 'ERROR', 'READY_ANGLE', 'ANGLE_SAMPLE',
                'ML_ACCEPT', 'ML_CATCH_TRIGGER', 'ML_CLOSE', 'ML_SAMPLE',
                'ML_BUTTON_CATCH', 'ML_AWAIT_FEEDBACK', 'ML_RESULT',
                'ML_TRACK', 'ML_TRACK_REJECT', 'CAL_REFERENCE', 'CAL_TRIGGER',
                'CAL_GRIP', 'CAL_CLOSE', 'CAL_AWAIT_FEEDBACK', 'CAL_SAMPLE'
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
            # SESSION_START incluye el modo elegido. Esperar ese evento evita
            # crear un CSV V2 cuando la pantalla 11 corresponde a registrar angulo.
            # Las muestras permiten iniciar tambien si el registrador se abre
            # cuando la prueba de encoder ya estaba en marcha.
            $esEventoEncoder = $analizada.Structured -and
                $analizada.Source -eq 'P' -and
                $evento -in @('ENCODER_TEST_START', 'ENCODER_TEST_RUN_START', 'ENCODER_TEST_SAMPLE',
                    'ENCODER_TEST_RESET', 'ENCODER_TEST_REFERENCE', 'ENCODER_TEST_MARK',
                    'ENCODER_TEST_OBSERVATION')
            $esInicioEncoder = $esEventoEncoder -and
                ($null -eq $script:EscritorSesion -or $script:TipoSesion -ne 'encoder')
            $esInicio = $esInicioEstructurado -or $esInicioEncoder
            $esSalidaEstadoLegado = $null -ne $script:EscritorSesion -and
                $null -ne $estadoGeneralLegado -and
                $estadoGeneralLegado -ne $(if ($script:TipoSesion -eq 'encoder') { 17 }
                    elseif ($script:TipoSesion -eq 'angulo') { 9 }
                    elseif ($script:TipoSesion -in @('ml', 'ml_v2', 'ml_track')) { 16 } else { 11 })
            $esCancelacionLegada = $analizada.Source -eq 'P' -and
                $envoltura.Line -match '\[AUTO V2\]\s+Cancelando:'

            if ($null -ne $script:EscritorSesion -or $esInicio -or
                $evento -eq 'ML_SAMPLE') {
                Mostrar-EventoEnVivo $envoltura $analizada
            }

            if ($esInicio) {
                if ($null -ne $script:EscritorSesion) {
                    Escribir-EventoHost 'SESSION_RESTART' 'nuevo inicio sin cierre previo'
                    Cerrar-Sesion 'reinicio inesperado'
                }
                $script:TipoSesion = if ($esInicioEncoder) { 'encoder' }
                    elseif ((Obtener-Dato $analizada 'mode') -eq 'ANGLE_LABEL') { 'angulo' }
                    elseif ((Obtener-Dato $analizada 'mode') -eq 'ML_V2') { 'ml_v2' }
                    elseif ((Obtener-Dato $analizada 'mode') -eq 'ML_TRACK') { 'ml_track' }
                    elseif ((Obtener-Dato $analizada 'mode') -eq 'ML') { 'ml' }
                    elseif ((Obtener-Dato $analizada 'mode') -eq 'CATCH_CAL') { 'catch_cal' }
                    else { 'v2' }
                Iniciar-Sesion
                foreach ($anterior in $prebuffer.ToArray()) {
                    Escribir-Fila $anterior (Analizar-Linea $anterior.Line $anterior.Source)
                }
                if ($analizada.Structured -and $evento -eq 'CANCEL' -and
                     $script:TipoSesion -notin @('angulo', 'ml', 'ml_v2', 'ml_track', 'catch_cal')) {
                    $script:BloqueadoHastaSalirV2 = $true
                    Cerrar-Sesion 'CANCEL'
                } elseif ($esCancelacionLegada -and $script:TipoSesion -notin @('angulo', 'ml', 'ml_v2', 'ml_track', 'catch_cal')) {
                    $script:BloqueadoHastaSalirV2 = $true
                    Cerrar-Sesion 'CANCEL legado'
                }
                continue
            }

            if ($null -ne $script:EscritorSesion) {
                Escribir-Fila $envoltura $analizada
                if ($analizada.Structured -and $analizada.Source -eq 'P' -and
                    ($evento -eq 'SESSION_END' -or
                       ($evento -eq 'CANCEL' -and $script:TipoSesion -notin @('angulo', 'ml', 'ml_v2', 'ml_track', 'catch_cal')) -or
                     $evento -eq 'ENCODER_TEST_END')) {
                    if ($evento -eq 'CANCEL') {
                        $script:BloqueadoHastaSalirV2 = $true
                    }
                    Cerrar-Sesion $evento
                } elseif ($esCancelacionLegada -and $script:TipoSesion -notin @('angulo', 'ml', 'ml_v2', 'ml_track', 'catch_cal')) {
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
    if ($null -ne $script:EscritorDiagnostico) {
        $script:EscritorDiagnostico.Flush()
        $script:EscritorDiagnostico.Dispose()
        $script:EscritorDiagnostico = $null
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
