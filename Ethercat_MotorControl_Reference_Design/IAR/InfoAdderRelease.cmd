@ echo off
del %1\..\For_TwinCAT\Upgrade\*.efw
%1\FileInfoAdder.exe %1\Release\Exe\MotorControl.bin %1\..\For_TwinCAT\Upgrade\AX58200_MotorControl.efw -sg ASIX -ft 0
%1\FileInfoAdder.exe %1\..\For_TwinCAT\Upgrade\Bl60_Mode3.txt %1\..\For_TwinCAT\Upgrade\AX58200_Bl60_Mode3.efw -sg ASIX -ft 1

