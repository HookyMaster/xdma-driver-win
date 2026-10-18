@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title XDMA driver portable install

net session >nul 2>&1
if errorlevel 1 (
    echo [*] requesting administrator privileges ...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

echo ============================================================
echo  XDMA driver portable install   (Win10/11 x64, test-signed)
echo ============================================================
echo.

for %%F in (XDMA.sys XDMA.inf XDMA.cat XDMA.cer) do (
    if not exist "%%F" (
        echo INSTALL_FAILED missing_%%F
        pause
        exit /b 1
    )
)

echo [1/5] environment check
set "SBPATH=HKLM\SYSTEM\CurrentControlSet\Control\SecureBoot\State"
set "SB=none"
for /f "tokens=3" %%a in ('reg query "%SBPATH%" /v UEFISecureBootEnabled 2^>nul ^| findstr /i "UEFISecureBootEnabled"') do set "SB=%%a"
echo     UEFISecureBootEnabled = %SB%
if /i "%SB%"=="0x1" (
    echo.
    echo     !! Secure Boot is ON. A test-signed kernel driver will NOT load.
    echo     !! Disable Secure Boot in BIOS/UEFI setup, then run this again.
    echo.
    echo INSTALL_FAILED secureboot_on
    pause
    exit /b 2
)

bcdedit /enum > "%TEMP%\xdma_bcd.txt" 2>&1
echo     current testsigning state:
findstr /i "testsigning" "%TEMP%\xdma_bcd.txt"
if errorlevel 1 echo         (not set - will be enabled in step 3)

echo.
echo [2/5] import test certificate
certutil -addstore -f Root "%~dp0XDMA.cer" >nul 2>&1
certutil -addstore -f TrustedPublisher "%~dp0XDMA.cer" >nul 2>&1
certutil -store Root 2>nul | findstr /i "WDKTestCert" >nul
if errorlevel 1 (
    echo     !! certificate not found in Root store
    echo INSTALL_FAILED cert_import
    pause
    exit /b 3
)
echo     CERT_IMPORT_OK

echo.
echo [3/5] enable test signing
bcdedit /set testsigning on >nul 2>&1
if errorlevel 1 (
    echo     !! bcdedit failed - Secure Boot or BitLocker is blocking it
    echo INSTALL_FAILED bcdedit
    pause
    exit /b 4
)
bcdedit /enum 2>nul | findstr /i "testsigning"
echo     TESTSIGNING_SET

echo.
echo [4/5] add driver package
pnputil /add-driver "%~dp0XDMA.inf" /install
set "RC=%errorlevel%"
echo     pnputil rc=%RC%   (0 = ok, 3010 = reboot required)
if not "%RC%"=="0" if not "%RC%"=="3010" (
    echo INSTALL_FAILED pnputil_%RC%
    pause
    exit /b 5
)

echo.
echo [5/5] device status
powershell -NoProfile -Command "Get-PnpDevice -PresentOnly | Where-Object {$_.InstanceId -like '*VEN_10EE*'} | Select-Object Status,FriendlyName,InstanceId | Format-List"
powershell -NoProfile -Command "$d = Get-CimInstance Win32_PnPSignedDriver | Where-Object {$_.DeviceID -match 'VEN_10EE'}; if ($d) { $d | Select-Object DriverVersion,DriverProviderName,InfName | Format-List } else { '    no VEN_10EE device bound yet' }"

echo.
echo ============================================================
echo  If test signing was just turned on, REBOOT NOW.
echo  After reboot verify with:   python xdma.py list
echo ============================================================
pause
exit /b 0
