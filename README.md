# Pixhawk SIL connector for Simulink

Simulink C++ S-function for software-in-the-loop simulation with Pixhawk.

[![View pixhawk-sitl-connector on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)]()

Requirements
- MATLAB & Simulink (MATLAB R2022a or earlier)
- MinGW-w64 or MSVC C/C++ Compiler
- QGroundControl
- PX4-Autopilot source code

Files

[pixhawk_sil_connector.cpp](https://github.com/aviumtechnologies/pixhawk-sil-connector/blob/master/pixhawk_sil_connector.cpp)
<div style="height:1px; background-color:rgba(0,0,0,0.12);"></div>

[make.m](https://github.com/aviumtechnologies/pixhawk-sil-connector/blob/master/make.m)
<div style="height:1px; background-color:rgba(0,0,0,0.12);"></div>

[includes.zip](https://github.com/aviumtechnologies/pixhawk-sil-connector/blob/master/includes.zip) (contains the Asio C++ and MAVLink C libraries)
<div style="height:1px; background-color:rgba(0,0,0,0.12);"></div>

Build instructions

-  Install MATLAB-supported compiler  
https://mathworks.com/support/requirements/supported-compilers.html.
-  Download the "pixhawk_sil_connector.cpp" and "make.m" files and the "includes.zip" archive.
-  Unzip the "includes.zip archive".
-  Run "make.m" to create a "pixhawk_sil_connector.mexw64" (Windows), "pixhawk_sil_connector.mexa64" (Linux), "pixhawk_sil_connector.mexmaci64" (macOS) file.

Use instructions

- Download and install QGroundControl  [https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).
- Create a new "Comm Link" in QGroundControl via the "Application Settings" page. The type of the link must be UDP, the port 18570, and the server address 127.0.0.1.
- Open and run "pixhawk_sil_connector_example.slx".
- Build the PX4-Autopilot source code using the following command:  <pre>make px4_sitl none_plane</pre>  [https://docs.px4.io/master/en/dev_setup/building_px4.html](https://docs.px4.io/master/en/dev_setup/building_px4.html).

[![Example use of the Pixhawk SIL connector](https://i.ytimg.com/vi/b7P1-UgXS7Q/maxresdefault.jpg)](https://youtu.be/b7P1-UgXS7Q)

<p align="center">Example use of the Pixhawk SIL connector</p>

![Pixhawk SIL connector example](https://github.com/KBoychev/pixhawk-sil-connector/blob/master/pixhawk_sil_connector_example.png)

<p align="center">Pixhawk SIL connector example</p>

Additional information available at

https://fst.aviumtechnologies.com/pixhawk-sil-connector