# Release policy

`open-lmm-iridescence` is released at the same version as the OpenLMM Python
SDK and requires that exact SDK version. Version 3.0.0 supports CPython 3.10 on
Ubuntu 22.04 x86_64. The C++ Iridescence GUI remains a separate, supported
application until a future compatibility review explicitly changes the default.

The release artifact must pin `pyridescence==1.0.3`, preserve installed license
metadata, and pass unit plus Xvfb OpenGL smoke tests.
