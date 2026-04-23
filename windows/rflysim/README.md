## Windows RflySim Launch Assets

This directory contains the Windows-side startup assets used to launch the RflySim simulation environment referenced by the paper.

- `SITLRun.bat`: starts multi-vehicle PX4 SITL in RflySim/CopterSim
- `HITLRun.bat`: starts PX4 HITL and binds simulator instances to Pixhawk COM ports
- `FX150_H_Model.dll`: DLL-based dynamics model loaded by CopterSim

Both batch scripts assume that the RflySim toolchain is installed under `C:\PX4PSP` and that the corresponding WSL-side path is `/mnt/c/PX4PSP`. Update `PSP_PATH` and `PSP_PATH_LINUX` in the scripts if your local installation differs.
