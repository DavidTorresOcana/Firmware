dbx_control Pixhawk project app
-------------------------------

This is an application for the pixhawk project compatible with most of the 
    suported hardware and boards of the project.

* Name: dbx_control
* Description: It includes wrapper funtions  and files to integrate code generated (FCS software) from Simulink models 
    of control development project (TBD: add the link to the repo)
* It is  a Daemon app in NuttX OS so, it is used as follow:
    - To start the app: ´´´dbx_control start ´´´
    - To kill the app:  ´´´dbx_control stop´´´
* License (TO BE DONE): Copyright to DBX Drones 

* Installation: In the Pixhawk Firmware project, follow these steps:
    - Add this entire folder in ~/Firmware/src/modules
    - Modify the correspondent Cmake file, adding the line: ´´´modules/dbx_control´´´
    - Generate code from Simulink control model and place it in this folder






* Changelog:
 - 21/01/2016: Modification on the Cmake file to compile with Pixhawk Firmware release 1.1.2