@echo off
echo ============================================
echo   Courier Robot - Quick Start
echo ============================================
echo.
echo Container will be available at: http://localhost:6080
echo.
REM Prefer image with apriltags preinstalled if available
docker image inspect courier-robot:nav2-apriltags >nul 2>&1
if errorlevel 1 (
    docker image inspect courier-robot:nav2 >nul 2>&1
    if errorlevel 1 (
        echo [WARNING] Image not found! Build first.
        pause
        exit /b 1
    ) else (
        set IMAGE=courier-robot:nav2
    )
) else (
    set IMAGE=courier-robot:nav2-apriltags
)

docker run -it --rm ^
    -p 6080:80 ^
    -v "C:\robot\ros2_ws:/home/ubuntu/ros2_ws" ^
    --name courier-robot ^
    %IMAGE%
