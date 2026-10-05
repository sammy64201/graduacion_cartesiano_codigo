[CmdletBinding()]
param(
    [string]$Puerto,
    [int]$Baud = 115200,
    [string]$DirectorioSalida,
    [string]$CapturaInterfaz
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()
Import-Module (Join-Path $PSScriptRoot 'RegistroCsv.psm1') -Force

if ([string]::IsNullOrWhiteSpace($DirectorioSalida)) {
    $DirectorioSalida = Join-Path $PSScriptRoot 'registros'
}

$script:Serie = $null
$script:Escritor = $null
$script:ArchivoCsv = $null
$script:Buffer = ''
$script:TotalLineas = 0
$script:TotalObjetos = 0
$script:TotalFrames = 0
$script:TotalErroresJson = 0
$script:PuertoActivo = ''
$script:BaudActivo = 115200
$script:Procesando = $false
$script:Cerrando = $false

$form = New-Object System.Windows.Forms.Form
$form.Text = 'Registrador CSV - un puerto COM'
$form.ClientSize = New-Object System.Drawing.Size(1020, 710)
$form.MinimumSize = New-Object System.Drawing.Size(1036, 640)
$form.StartPosition = 'CenterScreen'
$form.Font = New-Object System.Drawing.Font('Segoe UI', 10)
$form.BackColor = [System.Drawing.Color]::FromArgb(245, 247, 250)

function New-Etiqueta {
    param([string]$Texto, [int]$X, [int]$Y, [int]$Ancho = 200)
    $control = New-Object System.Windows.Forms.Label
    $control.Text = $Texto
    $control.Location = New-Object System.Drawing.Point($X, $Y)
    $control.Size = New-Object System.Drawing.Size($Ancho, 24)
    $form.Controls.Add($control)
    return $control
}

function New-Boton {
    param([string]$Texto, [int]$X, [int]$Y, [int]$Ancho = 130)
    $control = New-Object System.Windows.Forms.Button
    $control.Text = $Texto
    $control.Location = New-Object System.Drawing.Point($X, $Y)
    $control.Size = New-Object System.Drawing.Size($Ancho, 34)
    $form.Controls.Add($control)
    return $control
}

$titulo = New-Etiqueta 'Captura de segmentacion' 20 18 750
$titulo.Font = New-Object System.Drawing.Font('Segoe UI', 18, [System.Drawing.FontStyle]::Bold)
$titulo.Height = 38
$subtitulo = New-Etiqueta 'Un COM, terminal en vivo y CSV con los datos completos de cada lectura.' 22 61 960

$null = New-Etiqueta 'Puerto COM' 22 101 135
$com = New-Object System.Windows.Forms.ComboBox
$com.Location = New-Object System.Drawing.Point(22, 128)
$com.Size = New-Object System.Drawing.Size(135, 30)
$com.DropDownStyle = 'DropDownList'
$form.Controls.Add($com)
$actualizar = New-Boton 'Actualizar' 170 124 110
$null = New-Etiqueta 'Baudios' 297 101 120
$baudControl = New-Object System.Windows.Forms.ComboBox
$baudControl.Location = New-Object System.Drawing.Point(297, 128)
$baudControl.Size = New-Object System.Drawing.Size(125, 30)
$baudControl.DropDownStyle = 'DropDown'
$baudControl.Items.AddRange([object[]]@('9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600'))
$baudControl.Text = [string]$Baud
$form.Controls.Add($baudControl)
$iniciar = New-Boton 'Iniciar captura' 444 124 160
$iniciar.BackColor = [System.Drawing.Color]::FromArgb(26, 104, 172)
$iniciar.ForeColor = [System.Drawing.Color]::White
$iniciar.FlatStyle = 'Flat'
$detener = New-Boton 'Detener' 616 124 125
$detener.Enabled = $false
$abrirCarpeta = New-Boton 'Abrir carpeta' 754 124 145

$null = New-Etiqueta 'Carpeta de los CSV' 22 176 300
$carpeta = New-Object System.Windows.Forms.TextBox
$carpeta.Location = New-Object System.Drawing.Point(22, 204)
$carpeta.Size = New-Object System.Drawing.Size(850, 28)
$carpeta.Anchor = 'Top, Left, Right'
$carpeta.Text = $DirectorioSalida
$form.Controls.Add($carpeta)
$elegirCarpeta = New-Boton 'Elegir...' 886 199 110
$elegirCarpeta.Anchor = 'Top, Right'
$estado = New-Etiqueta 'Desconectado. Seleccione el COM de la ESP32 e inicie la captura.' 22 249 970
$estado.ForeColor = [System.Drawing.Color]::FromArgb(35, 67, 93)
$estado.Anchor = 'Top, Left, Right'
$archivo = New-Etiqueta 'CSV: se crea un archivo nuevo en cada captura.' 22 279 975
$archivo.AutoEllipsis = $true
$archivo.Anchor = 'Top, Left, Right'
$contadores = New-Etiqueta 'Lineas: 0   |   Detecciones: 0   |   Frames: 0   |   JSON invalidos: 0' 22 309 970
$contadores.Anchor = 'Top, Left, Right'

$terminal = New-Object System.Windows.Forms.TextBox
$terminal.Multiline = $true
$terminal.ScrollBars = 'Both'
$terminal.Location = New-Object System.Drawing.Point(22, 346)
$terminal.Size = New-Object System.Drawing.Size(974, 262)
$terminal.Anchor = 'Top, Bottom, Left, Right'
$terminal.ReadOnly = $true
$terminal.WordWrap = $false
$terminal.Font = New-Object System.Drawing.Font('Consolas', 9)
$terminal.BackColor = [System.Drawing.Color]::FromArgb(19, 31, 45)
$terminal.ForeColor = [System.Drawing.Color]::FromArgb(217, 231, 243)
$form.Controls.Add($terminal)

$recalibrar = New-Boton 'Calibrar (C)' 22 625 140
$modelo = New-Boton 'Abrir modelo (M)' 172 625 165
$pausar = New-Boton 'Pausa salida (P)' 347 625 165
$ayuda = New-Boton 'Ayuda (H)' 522 625 125
$limpiar = New-Boton 'Limpiar pantalla' 657 625 155
$comando = New-Object System.Windows.Forms.TextBox
$comando.Location = New-Object System.Drawing.Point(822, 629)
$comando.Size = New-Object System.Drawing.Size(50, 28)
$comando.MaxLength = 1
$form.Controls.Add($comando)
$enviar = New-Boton 'Enviar' 882 625 114
foreach ($control in @($recalibrar, $modelo, $pausar, $ayuda, $limpiar, $comando, $enviar)) {
    $control.Anchor = 'Bottom, Left'
}
$nota = New-Etiqueta 'Cierre el monitor de Arduino antes de conectar. Se guardan tambien mensajes y lecturas sin detecciones.' 22 674 975
$nota.Font = New-Object System.Drawing.Font('Segoe UI', 9)
$nota.Anchor = 'Bottom, Left, Right'

function Add-Terminal {
    param([string]$Texto)
    $terminal.AppendText($Texto + [Environment]::NewLine)
    # Limite solo visual: el CSV conserva todo lo recibido.
    if ($terminal.TextLength -gt 120000) {
        $terminal.Select(0, 30000)
        $terminal.SelectedText = ''
    }
    $terminal.SelectionStart = $terminal.TextLength
    $terminal.ScrollToCaret()
}

function Update-Contadores {
    $contadores.Text = 'Lineas: {0}   |   Detecciones: {1}   |   Frames: {2}   |   JSON invalidos: {3}' -f
        $script:TotalLineas, $script:TotalObjetos, $script:TotalFrames, $script:TotalErroresJson
}

function Set-ControlesCaptura {
    param([bool]$Activa)
    foreach ($control in @($com, $baudControl, $actualizar, $carpeta, $elegirCarpeta, $iniciar)) {
        $control.Enabled = -not $Activa
    }
    foreach ($control in @($detener, $recalibrar, $modelo, $pausar, $ayuda, $comando, $enviar)) {
        $control.Enabled = $Activa
    }
}

function Update-Puertos {
    $seleccion = $com.Text
    $com.Items.Clear()
    $puertos = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object {
        if ($_ -match '^COM(\d+)$') { [int]$Matches[1] } else { [int]::MaxValue }
    })
    foreach ($nombre in $puertos) { $null = $com.Items.Add($nombre) }
    if ($com.Items.Contains($seleccion)) { $com.SelectedItem = $seleccion }
    elseif ($puertos.Count -gt 0) { $com.SelectedIndex = 0 }
    else { $estado.Text = 'No se detectan puertos COM. Conecte la ESP32 y pulse Actualizar.' }
}

function Save-Linea {
    param([AllowEmptyString()][string]$Linea, [switch]$Fragmento)
    if ($null -eq $script:Escritor) { return }
    $fila = ConvertTo-FilaRegistro -Linea $Linea -Puerto $script:PuertoActivo -Baud $script:BaudActivo -Fragmento:$Fragmento
    Write-FilaRegistroCsv -Escritor $script:Escritor -Fila $fila
    $script:TotalLineas++
    switch ($fila.tipo) {
        'segmentacion' { $script:TotalObjetos++ }
        'frame' { $script:TotalFrames++ }
        'error_json' { $script:TotalErroresJson++ }
    }
    Add-Terminal $Linea
}

function Read-Serie {
    if ($null -eq $script:Serie -or -not $script:Serie.IsOpen) { return }
    # No ReadLine: una linea incompleta no debe bloquear la interfaz.
    $script:Buffer += $script:Serie.ReadExisting()
    $partes = Split-LineasRegistro -Texto $script:Buffer
    # Actualizar pendiente despues de escribir: ante fallo de disco se informa
    # el error y se detiene; no se continua una captura aparentemente correcta.
    foreach ($linea in $partes.Lineas) { Save-Linea -Linea $linea }
    $script:Buffer = $partes.Pendiente
    if ($script:Buffer.Length -gt 65536) {
        Save-Linea -Linea $script:Buffer -Fragmento
        $script:Buffer = ''
    }
    Update-Contadores
}

function Stop-Captura {
    param([string]$Motivo = 'Captura detenida', [switch]$OmitirLectura)
    $timer.Stop()
    $problemas = New-Object 'System.Collections.Generic.List[string]'
    try {
        if (-not $OmitirLectura) { Read-Serie }
        if ($script:Buffer.Length -gt 0 -and $null -ne $script:Escritor) {
            Save-Linea -Linea $script:Buffer -Fragmento
        }
    } catch { $problemas.Add($_.Exception.Message) }
    finally {
        if ($null -ne $script:Serie) {
            try { $script:Serie.Dispose() } catch { $problemas.Add($_.Exception.Message) }
            $script:Serie = $null
        }
        if ($null -ne $script:Escritor) {
            try { $script:Escritor.Dispose() } catch { $problemas.Add($_.Exception.Message) }
            $script:Escritor = $null
        }
        $script:Buffer = ''
        Set-ControlesCaptura $false
        Update-Contadores
    }
    $estado.Text = $Motivo + '. Puerto liberado; CSV guardado.'
    if ($problemas.Count -gt 0) {
        $estado.Text = $Motivo + '. Hubo un error al cerrar la captura.'
        Add-Terminal ('[APP] Error de cierre: ' + ($problemas -join '; '))
    }
}

function Show-Error {
    param([string]$Texto)
    $null = [System.Windows.Forms.MessageBox]::Show($form, $Texto, 'Registrador de un COM', 'OK', 'Error')
}

function Start-Captura {
    if ($null -ne $script:Serie) { return }
    $rutaCreada = $null
    try {
        if ([string]::IsNullOrWhiteSpace($com.Text)) { throw 'Seleccione un puerto COM.' }
        $velocidad = 0
        if (-not [int]::TryParse($baudControl.Text, [ref]$velocidad) -or $velocidad -le 0) {
            throw 'Ingrese una velocidad de baudios valida.'
        }
        if ([string]::IsNullOrWhiteSpace($carpeta.Text)) { throw 'Seleccione una carpeta para el CSV.' }
        $destino = [System.IO.Path]::GetFullPath($carpeta.Text)
        $null = [System.IO.Directory]::CreateDirectory($destino)
        $script:PuertoActivo = $com.Text
        $script:BaudActivo = $velocidad
        $nombre = 'captura_{0}_{1}_{2}.csv' -f $com.Text, [DateTime]::Now.ToString('yyyyMMdd_HHmmss'),
            [Guid]::NewGuid().ToString('N').Substring(0, 6)
        $rutaCreada = Join-Path $destino $nombre
        $script:Escritor = New-EscritorRegistroCsv -Ruta $rutaCreada
        $script:Serie = New-Object System.IO.Ports.SerialPort($com.Text, $velocidad,
            [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
        $script:Serie.Encoding = New-Object System.Text.UTF8Encoding($false)
        $script:Serie.Handshake = [System.IO.Ports.Handshake]::None
        $script:Serie.DtrEnable = $false
        $script:Serie.RtsEnable = $false
        $script:Serie.ReadBufferSize = 65536
        $script:Serie.ReadTimeout = 200
        $script:Serie.WriteTimeout = 500
        $script:Serie.Open()
        $script:ArchivoCsv = $rutaCreada
        $script:Buffer = ''
        $script:TotalLineas = $script:TotalObjetos = $script:TotalFrames = $script:TotalErroresJson = 0
        $terminal.Clear()
        $archivo.Text = 'CSV: ' + $script:ArchivoCsv
        $carpeta.Text = $destino
        $estado.Text = 'Capturando {0} a {1} baudios. Cada linea recibida se guarda en el CSV.' -f $com.Text, $velocidad
        Set-ControlesCaptura $true
        Update-Contadores
        Add-Terminal '[APP] Conectado. Puede pulsar Calibrar (C) para repetir tags y abrir el modelo.'
        $timer.Start()
    } catch {
        $mensaje = $_.Exception.Message
        Stop-Captura -Motivo 'No se pudo iniciar' -OmitirLectura
        $estado.Text = 'No se pudo iniciar la captura.'
        Show-Error ($mensaje + "`n`nCierre el monitor serie de Arduino y cualquier otra app que use ese COM.")
    }
}

function Send-Comando {
    param([string]$Texto)
    if ($null -eq $script:Serie -or -not $script:Serie.IsOpen) { return }
    if ($Texto -cnotmatch '^[CcMmPpHh012]$') {
        Show-Error 'Para este sketch use C, M, P, H, 0, 1 o 2.'
        return
    }
    try {
        $script:Serie.Write($Texto)
        Add-Terminal ('[APP] Comando enviado: ' + $Texto)
    } catch {
        $mensaje = $_.Exception.Message
        Stop-Captura -Motivo 'Error al enviar comando'
        Show-Error $mensaje
    }
}

$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 50
$timer.Add_Tick({
    if ($script:Procesando) { return }
    $script:Procesando = $true
    try {
        Read-Serie
        if ($null -ne $script:Serie -and -not $script:Serie.IsOpen) {
            throw 'El puerto serie se cerro inesperadamente.'
        }
    } catch {
        $mensaje = $_.Exception.Message
        Stop-Captura -Motivo 'Captura interrumpida' -OmitirLectura
        $estado.Text = 'Error de captura: ' + $mensaje
        Add-Terminal ('[APP] ERROR: ' + $mensaje)
        Show-Error ($mensaje + "`n`nLa captura se detuvo. Revise conexion/carpeta y vuelva a iniciar.")
    } finally { $script:Procesando = $false }
})

$actualizar.Add_Click({ Update-Puertos })
$iniciar.Add_Click({ Start-Captura })
$detener.Add_Click({ Stop-Captura })
$recalibrar.Add_Click({ Send-Comando 'C' })
$modelo.Add_Click({ Send-Comando 'M' })
$pausar.Add_Click({ Send-Comando 'P' })
$ayuda.Add_Click({ Send-Comando 'H' })
$enviar.Add_Click({ Send-Comando $comando.Text })
$comando.Add_KeyDown({ if ($_.KeyCode -eq 'Enter') { $_.SuppressKeyPress = $true; Send-Comando $comando.Text } })
$limpiar.Add_Click({ $terminal.Clear() })
$elegirCarpeta.Add_Click({
    $dialogo = New-Object System.Windows.Forms.FolderBrowserDialog
    try {
        $dialogo.Description = 'Carpeta para las capturas CSV'
        if ([System.IO.Directory]::Exists($carpeta.Text)) { $dialogo.SelectedPath = $carpeta.Text }
        if ($dialogo.ShowDialog($form) -eq 'OK') { $carpeta.Text = $dialogo.SelectedPath }
    } finally { $dialogo.Dispose() }
})
$abrirCarpeta.Add_Click({
    try {
        $destino = [System.IO.Path]::GetFullPath($carpeta.Text)
        if (-not [System.IO.Directory]::Exists($destino)) {
            throw 'La carpeta aun no existe. Inicie una captura para crearla.'
        }
        $null = Start-Process -FilePath 'explorer.exe' -ArgumentList ('"' + $destino + '"') -WindowStyle Normal
    } catch { Show-Error $_.Exception.Message }
})
$form.Add_FormClosing({
    if (-not $script:Cerrando) {
        $script:Cerrando = $true
        Stop-Captura -Motivo 'Aplicacion cerrada'
    }
})

Set-ControlesCaptura $false
Update-Puertos
if (-not [string]::IsNullOrWhiteSpace($Puerto) -and $com.Items.Contains($Puerto)) {
    $com.SelectedItem = $Puerto
}

try {
    if (-not [string]::IsNullOrWhiteSpace($CapturaInterfaz)) {
        # Verificacion visual sin abrir COM ni iniciar captura.
        $com.Items.Clear()
        $null = $com.Items.Add('COM5')
        $com.SelectedIndex = 0
        $estado.Text = 'Vista de prueba - sin conexion a hardware.'
        Add-Terminal '[CAL] Muestras tag0=25/25 tag1=25/25 tag2=25/25 tag3=25/25'
        Add-Terminal '[MODELO] Abriendo personalizado 129; espera de carga 8 s.'
        Add-Terminal '{"tipo":"frame","frame":1,"ms":15000,"algoritmo":129,"resultados":1}'
        Add-Terminal '{"tipo":"segmentacion","id":1,"nombre":"pieza","x_mm":20.50,"y_mm":-15.20,"en_banda":true}'
        $form.Show()
        [System.Windows.Forms.Application]::DoEvents()
        $form.PerformLayout()
        $bitmap = New-Object System.Drawing.Bitmap($form.Width, $form.Height)
        try {
            $form.DrawToBitmap($bitmap, (New-Object System.Drawing.Rectangle(0, 0, $form.Width, $form.Height)))
            $bitmap.Save([System.IO.Path]::GetFullPath($CapturaInterfaz), [System.Drawing.Imaging.ImageFormat]::Png)
        } finally { $bitmap.Dispose(); $form.Hide() }
        Write-Output ('PASS: interfaz creada; captura: ' + $CapturaInterfaz)
    } else {
        $null = $form.ShowDialog()
    }
} finally {
    if ($null -ne $script:Serie -or $null -ne $script:Escritor) { Stop-Captura }
    $timer.Dispose()
    $form.Dispose()
}
