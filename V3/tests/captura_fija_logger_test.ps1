$ErrorActionPreference = 'Stop'
# Cargar solo funciones/constantes de los registradores; nunca abrir puertos.
foreach ($carpeta in @('pruebas de automatico v2', 'automatico v2 rs485')) {
    $ruta = Join-Path $PSScriptRoot "../$carpeta/registrar_v2.ps1"
    $tokens = $null; $errores = $null
    $ast = [System.Management.Automation.Language.Parser]::ParseFile(
        $ruta, [ref]$tokens, [ref]$errores)
    if ($errores.Count) { throw ($errores | Out-String) }
    foreach ($nodo in $ast.FindAll({ param($n)
        $n -is [System.Management.Automation.Language.AssignmentStatementAst] -and
        $n.Left.Extent.Text -in @('$script:Columnas', '$script:MapaCampos')
    }, $false)) { Invoke-Expression $nodo.Extent.Text }
    foreach ($nodo in $ast.FindAll({ param($n)
        $n -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $n.Name -in @('Analizar-Linea', 'Obtener-Dato', 'Nombre-FaseV2', 'Nombre-AckV2',
            'Describir-Evento', 'Convertir-CampoCsv', 'Escribir-Fila')
    }, $false)) { Invoke-Expression $nodo.Extent.Text }
    if (($script:Columnas | Select-Object -Unique).Count -ne $script:Columnas.Count) {
        throw 'Columnas duplicadas'
    }
    $script:TipoSesion = 'v2'
    $script:EscritorSesion = New-Object System.IO.StringWriter
    $linea = 'V2LOG|E|event=OBJECTIVE_REFERENCE|obj=42|objective_flags=7|reference_encoder=-100|reference_query_age_ms=15|reference_source=QUERY_MIDPOINT_APPROX|capture_timestamp=NA|image_age_known=0|camera_delay_cal_ms=NA|camera_jitter_cal_ms=NA|image_reference_uncertainty_mm=NA|camera_delay_compensated=0|closure_axis_minor=Y|servo_rot_deg=59|rotation_command_applied=1|rotation_physically_verified=0|rej_crop=2'
    $analizada = Analizar-Linea $linea 'E'
    Escribir-Fila ([pscustomobject]@{Utc=[DateTime]::UtcNow;ElapsedMs=0;
        Source='E';Line=$linea}) $analizada
    $filas = @($script:EscritorSesion.ToString() | ConvertFrom-Csv -Header $script:Columnas)
    if ($filas.Count -ne 1) { throw 'Referencia perdida' }
    $fila = $filas[0]
    if ($fila.objective_flags -ne '7' -or $fila.objective_encoder_count -ne '-100' -or
        $fila.reference_source -ne 'QUERY_MIDPOINT_APPROX' -or
        $fila.capture_timestamp -ne 'NA' -or $fila.image_age_known -ne '0' -or
        $fila.rotation_physically_verified -ne '0' -or
        $fila.closure_axis_minor -ne 'Y' -or $fila.rejected_cropped -ne '2' -or
        $fila.camera_delay_compensated -ne '0' -or $fila.camera_delay_cal_ms -ne 'NA') {
        throw "Metadatos de referencia perdidos en $carpeta"
    }
    $script:EscritorSesion.Dispose()
    $script:EscritorSesion = New-Object System.IO.StringWriter
    $linea = 'V2LOG|P|event=FIXED_PREDICTION|obj=42|physical_validated=1|stage=DESCEND|remaining_mm=20|velocity_mm_s=30|acceleration_mm_s2=10|horizon_min_s=0.5|horizon_max_s=0.6|pred_y_min_mm=-2|pred_y_max_mm=3|position_error_mm=1|window_decision=2'
    $analizada = Analizar-Linea $linea 'P'
    Escribir-Fila ([pscustomobject]@{Utc=[DateTime]::UtcNow;ElapsedMs=0;
        Source='P';Line=$linea}) $analizada
    $fila = @($script:EscritorSesion.ToString() | ConvertFrom-Csv -Header $script:Columnas)[0]
    if ($fila.stage -ne 'DESCEND' -or $fila.physical_validated -ne '1' -or
        $fila.pred_y_min_mm -ne '-2' -or $fila.pred_y_max_mm -ne '3' -or
        $fila.position_error_mm -ne '1' -or $fila.acceleration_mm_s2 -ne '10' -or
        $fila.window_decision -ne '2' -or $fila.description_es -notmatch 'no verificado') {
        throw "Prediccion/intervalo de captura perdidos en $carpeta"
    }
    $script:EscritorSesion.Dispose()
    if ($carpeta -eq 'automatico v2 rs485') {
        $script:EscritorSesion = New-Object System.IO.StringWriter
        $linea = 'V2LOG|P|event=FIXED_PREDICTION|obj=43|physical_validated=0|test_mode=1|stage=CLOSE|pred_y_min_mm=-20|pred_y_max_mm=20|window_decision=4|nominal_horizon_s=0.308|nominal_y_mm=0.2|nominal_window_decision=2|catch_offset_ms=-100|configured_catch_offset_ms=100|nominal_contact_s=0.508|nominal_acceleration_mm_s2=0'
        $analizada = Analizar-Linea $linea 'P'
        Escribir-Fila ([pscustomobject]@{Utc=[DateTime]::UtcNow;ElapsedMs=0;
            Source='P';Line=$linea}) $analizada
        $fila = @($script:EscritorSesion.ToString() | ConvertFrom-Csv -Header $script:Columnas)[0]
        if ($fila.physical_validated -ne '0' -or $fila.test_mode -ne '1' -or
            $fila.nominal_horizon_s -ne '0.308' -or $fila.nominal_y_mm -ne '0.2' -or
            $fila.nominal_window_decision -ne '2' -or $fila.window_decision -ne '4' -or
            $fila.catch_offset_ms -ne '-100' -or $fila.configured_catch_offset_ms -ne '100' -or
            $fila.nominal_contact_s -ne '0.508' -or $fila.nominal_acceleration_mm_s2 -ne '0' -or
            $fila.pred_y_min_mm -ne '-20' -or $fila.pred_y_max_mm -ne '20') {
            throw 'CSV RS485 mezcla ensayo nominal con validacion/incertidumbre fisica'
        }
        $script:EscritorSesion.Dispose()
        $script:EscritorSesion = New-Object System.IO.StringWriter
        $linea = 'V2LOG|E|event=QUERY|query=169|cause=FUERA_CALIBRACION|rej_cal=1|rej_b=0'
        $analizada = Analizar-Linea $linea 'E'
        Escribir-Fila ([pscustomobject]@{Utc=[DateTime]::UtcNow;ElapsedMs=0;
            Source='E';Line=$linea}) $analizada
        $fila = @($script:EscritorSesion.ToString() | ConvertFrom-Csv -Header $script:Columnas)[0]
        if ($fila.rejected_calibration -ne '1' -or $fila.rejected_belt -ne '0' -or
            $fila.cause -ne 'FUERA_CALIBRACION' -or $fila.description_es -notmatch 'tags') {
            throw 'CSV RS485 confunde banda con area calibrada'
        }
        $script:EscritorSesion.Dispose()
        $script:EscritorSesion = New-Object System.IO.StringWriter
        $linea = 'V2LOG|P|event=FIXED_NOT_READY|test_mode=1|velocity_mm_s=151|velocity_min_mm_s=1|admission_velocity_max_mm_s=150|trial_velocity_max_mm_s=150|velocity_sample_age_ms=20'
        $analizada = Analizar-Linea $linea 'P'
        Escribir-Fila ([pscustomobject]@{Utc=[DateTime]::UtcNow;ElapsedMs=0;
            Source='P';Line=$linea}) $analizada
        $fila = @($script:EscritorSesion.ToString() | ConvertFrom-Csv -Header $script:Columnas)[0]
        if ($fila.velocity_mm_s -ne '151' -or $fila.admission_velocity_max_mm_s -ne '150' -or
            $fila.trial_velocity_max_mm_s -ne '150' -or $fila.velocity_sample_age_ms -ne '20') {
            throw 'CSV RS485 pierde rango o frescura de velocidad'
        }
        $script:EscritorSesion.Dispose()
    }
}
Write-Output 'PASS: ambos CSV conservan referencia aproximada/giro; RS485 separa ensayo nominal de perfil e incertidumbre fisica'
