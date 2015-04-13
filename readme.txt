This is a library for Arduino for decoding dcc signals.
Uses Arduino 1.6 IDE. 

Installation
--------------------------------------------------------------------------------

To install this library, just place this entire folder as a subfolder in your
Arduino/lib/targets/libraries folder.

When installed, this library should look like:

libraries/DCC_Decoder              		(this library's folder)
libraries/DCC_Decoder/DCC_Decoder.cpp       	(the library implementation file)
libraries/DCC_Decoder/DCC_Decoder.h    	        (the library header file)
libraries/DCC_Decoder/keywords.txt 		(the syntax coloring file)
libraries/DCC_Decoder/examples     		(the examples in the "open" menu)
libraries/DCC_Decoder/readme.txt   		(this file)

Building
--------------------------------------------------------------------------------

After this library is installed, you just have to start the Arduino application.

To use this library in a sketch, go to the Sketch | Import Library menu and
select DCC_Decoder.  This will add a corresponding line to the top of your sketch:

#include <DCC_Decoder.h>

To stop using this library, delete that line from your sketch.
