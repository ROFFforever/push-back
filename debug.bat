@echo off
powershell -NoProfile -ExecutionPolicy Bypass -Command "pros build; pros upload; clear; pros terminal | Tee-Object -FilePath 'C:\Users\User\robotics\newer-release\LemLib-0.5.6\visualizers\logs.txt'"
