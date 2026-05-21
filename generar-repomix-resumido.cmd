@echo off
setlocal
cd /d "%~dp0"

echo Generando Repomix resumido...
echo Salida: repomix-output-resumido.xml
echo.

call npx --yes repomix@latest --config "repomix-resumido.config.json" --no-security-check
if errorlevel 1 (
  echo.
  echo Error: no se pudo generar el Repomix resumido.
  echo Verifica que Node.js/npm esten instalados y que tengas conexion para descargar Repomix la primera vez.
  pause
  exit /b 1
)

echo.
echo Listo: repomix-output-resumido.xml
pause
