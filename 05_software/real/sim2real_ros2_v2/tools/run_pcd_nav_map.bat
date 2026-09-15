@echo off
setlocal
cd /d "%~dp0\.."

python .\tools\pcd_nav_pygame.py --pcd .\map\map_b.pcd --overlay-json .\tools\semantic_overlay_example.json

endlocal
