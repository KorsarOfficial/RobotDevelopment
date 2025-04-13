# Скрипт для установки Webots
Write-Host "Устанавливаем Webots..."

# Создаем директорию для ключей
$keyringsDir = "C:\etc\apt\keyrings"
if (-not (Test-Path $keyringsDir)) {
    New-Item -Path $keyringsDir -ItemType Directory -Force
}

# Скачиваем ключ
$keyPath = Join-Path $keyringsDir "Cyberbotics.asc"
Invoke-WebRequest -Uri "https://cyberbotics.com/Cyberbotics.asc" -OutFile $keyPath

# Скачиваем Webots
$webotsUrl = "https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b_setup.exe"
$webotsSetup = Join-Path $env:TEMP "webots_setup.exe"
Write-Host "Скачиваем установщик Webots..."
Invoke-WebRequest -Uri $webotsUrl -OutFile $webotsSetup

# Запускаем установку
Write-Host "Запускаем установку Webots..."
Start-Process -FilePath $webotsSetup -ArgumentList "/S" -Wait

# Устанавливаем переменную окружения
[Environment]::SetEnvironmentVariable("WEBOTS_HOME", "C:\Program Files\Webots", "User")
Write-Host "Переменная окружения WEBOTS_HOME установлена"

Write-Host "Установка Webots завершена!" 