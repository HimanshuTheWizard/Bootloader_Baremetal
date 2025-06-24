=== Flash is divided into three sections ====
=============================================
1. Bootloader       -  First two sectors are reserved for bootloader (Sector 0 and Sector 1)
2. Application-1    -  4 sectors are reserved for application-1 (Sector 2 to Sector 5)
3. Application-2    -  5 sectors are reserved for application-2 (Sector 6 to Sector 10)
4. NVRAM            -  Used to store variable for boot_flag, active_app

===========================================

==== Initial Steps after factory Reset ====
===========================================
1. Erase all the flash sectors
2. Flash the bootloader Firmware
3. Get into debug mode
4. Keep the User Button pressed to get into bootloader mode
5. Open and run User Bootloader App (Python Program)
6. Make sure that User Bootloader App has selected "Application.bin"
7. Choose memory address (0x08008000) of section-1
8. Flash image of Application-1 into memory section-1

10. Reset the system
11. This time dont press the user button and let the system boot normally.
12. If application-1 runs successfully all four LED on board shall glow.

===================================

===== Steps to update Firmware ====
===================================
1. If you press user button, while application is running, boot_flag gets set and
  system goes to reset
2. After reset, system boots up in bootloader mode.
3. Open User Bootloader App (Python Program) and it shall show currently which slot is occupied
   also, it shows the another memory slot where you need to flash the new image
4. Enter the correct address and and flash the appropriate image (update application.bin in python program)
5. After flashing is done, reset the board, it shall boot with new Firmware
6. Also, the system shall keep track which image is currently active

==================================

