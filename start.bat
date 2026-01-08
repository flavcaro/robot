@echo off
echo ============================================
echo   Courier Robot - Quick Start
echo ============================================
echo.
echo Container will be available at: http://localhost:6080
echo.

docker image inspect courier-robot:nav2 >nul 2>&1
if errorlevel 1 (
    echo [WARNING] Image not found! Build first.
    pause
    exit /b 1
)

docker run -it --rm ^
    -p 6080:80 ^
    -v "C:\robot\ros2_ws:/home/ubuntu/ros2_ws" ^
    --name courier-robot ^
    courier-robot:nav2
