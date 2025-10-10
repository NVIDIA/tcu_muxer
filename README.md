--------------------------------------------------------------------------------

ABOUT

Tegra Combined UART generates a pseudoterminal for communication for each UART
client on Tegra. It handles the tag protocol used to communicate with specific
UART clients. This is handled in tcu_com.c.

--------------------------------------------------------------------------------

DESCRIPTION

tcu_com.c      (tegra combined uart com)
This is the main program. It takes the argument -d /dev/ttyUSB3 and creates
pseudoterminals for each combined UART client. It handles the tag communication
protocol below:

   CLIENT       TAG
=======================
    RCE       | 0xe5
    BPMP      | 0xe2
    CCPLEX    | 0xe1
    SCE       | 0xe3
    SPE       | 0xe0
    TZ        | 0xe4
