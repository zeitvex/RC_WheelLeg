@echo off
setlocal
cd /d "%~dp0\.."

python .\tools\pcd_nav_pygame.py --pcd .\map\A_min.pcd --route-name A_min_route --route-map A_min --route-obstacle slalom --save-route .\map\routes\A_min\A_min_route.json --default-speed 0.35 --default-policy rough --default-tolerance 0.15

endlocal
