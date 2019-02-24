# Communicate with STIM300 over terminal

Install minicom: 

    sudo apt-get install minicom

Look for device:

    dmesg | grep tty

Open minicom with setings:

    sudo minicom -s

Setup minicom for stim300:

Serial port setup:

    A - Serial Device: /dev/ttyUSB0
    E - 921600 8N1
    F - Disable hardware flow control

Modem and dialing: (Clear option A...I)

    A - 
    * -
    * -
    * -
    I -

Screen and keyboard:

    P - Add linefeed

Save setup as dfl

Exit Setup

The stim300 will send the standard datagram, which will appera as a lot of gipperish.
In order to enter servicemode you must frist set the terminal emulation to ANSI:

    Ctr-a t
    A - terminal emulation: ANSI

Make sure the Backspace key send stays as BS
Enter service mode: write "SERVICEMODE" and press enter

    SERVICEMODE

Clear the screen:

    Ctr-A c

Write ? and press enter, and the STIM300 should send info about available commands

    ?

If minicom dont add linebreak on the output from the stim300, restart minicom.
To exit Minicom when in terminal mode press 'Ctrl-A' to get a message bar at the bottom of the terminal window and then press 'X'.
