@ echo off
del ..\For_TwinCAT\Upgrade\*.efw
.\FileInfoAdder.exe .\Release\Exe\AX58200_MotorControl.bin ..\For_TwinCAT\Upgrade\AX58200_MotorControl.efw -sg ASIX -ft 0
.\FileInfoAdder.exe ..\For_TwinCAT\Upgrade\BL60M24D8E00430080_MTMBL60.txt ..\For_TwinCAT\Upgrade\AX58200_BL60M24D8E00430080_MTMBL60.efw -sg ASIX -ft 1
