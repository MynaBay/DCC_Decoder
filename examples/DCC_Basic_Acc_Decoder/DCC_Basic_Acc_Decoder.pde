#include <DCC_Decoder.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines and structures
//
#define kDCC_INTERRUPT            0

typedef struct
{
    int               address;                // Address to respond to
    byte              output;                 // State of output 1=on, 0=off
    int               outputPin;              // Arduino output pin to drive
    boolean           isDigital;              // true=digital, false=analog. If analog must also set analogValue field
    boolean           isFlasher;              // true=flash output, false=no time, no flash.
    byte              analogValue;            // Value to use with analog type.
    int               durationMilli;          // Milliseconds to leave output on for.  0 means don't auto off
    
    unsigned long     onMilli;                // Used internally for timing
    unsigned long     offMilli;               // 
} DCCAccessoryAddress;

DCCAccessoryAddress gAddresses[8];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Decoder Init 
//
void ConfigureDecoder()
{
    gAddresses[0].address = 714;
    gAddresses[0].output = 0;
    gAddresses[0].outputPin = 5;
    gAddresses[0].isDigital = false;
    gAddresses[0].isFlasher = false;
    gAddresses[0].analogValue = 250;
    gAddresses[0].durationMilli = 500;
    
    gAddresses[1].address = 715;
    gAddresses[1].output = 0;
    gAddresses[1].outputPin = 6;
    gAddresses[1].isDigital = true;
    gAddresses[1].isFlasher = false;
    gAddresses[1].analogValue = 0;
    gAddresses[1].durationMilli = 500;
        
    gAddresses[2].address = 814;
    gAddresses[2].output = 0;
    gAddresses[2].outputPin = 5;
    gAddresses[2].isDigital = false;
    gAddresses[2].isFlasher = true;
    gAddresses[2].analogValue = 250;
    gAddresses[2].durationMilli = 500;
    
    gAddresses[3].address = 815;
    gAddresses[3].output = 0;
    gAddresses[3].outputPin = 6;
    gAddresses[3].isDigital = true;
    gAddresses[3].isFlasher = true;
    gAddresses[3].analogValue = 0;
    gAddresses[3].durationMilli = 500;
    
    gAddresses[4].address = 914;
    gAddresses[4].output = 0;
    gAddresses[4].outputPin = 5;
    gAddresses[4].isDigital = false;
    gAddresses[4].isFlasher = false;
    gAddresses[4].analogValue = 250;
    gAddresses[4].durationMilli = 0;
    
    gAddresses[5].address = 915;
    gAddresses[5].output = 0;
    gAddresses[5].outputPin = 6;
    gAddresses[5].isDigital = true;
    gAddresses[5].isFlasher = false;
    gAddresses[5].analogValue = 0;
    gAddresses[5].durationMilli = 0;
    
    gAddresses[6].address = 0;
    gAddresses[6].output = 0;
    gAddresses[6].outputPin = 0;
    gAddresses[6].isDigital = false;
    gAddresses[6].isFlasher = false;
    gAddresses[6].analogValue = 0;
    gAddresses[6].durationMilli = 0;
    
    gAddresses[7].address = 0;
    gAddresses[7].output = 0;
    gAddresses[7].outputPin = 0;
    gAddresses[7].isDigital = false;
    gAddresses[7].isFlasher = false;
    gAddresses[7].analogValue = 0;
    gAddresses[7].durationMilli = 0;
    
        // Setup output pins
    for(int i=0; i<(int)(sizeof(gAddresses)/sizeof(gAddresses[0])); i++)
    {
        if( gAddresses[i].outputPin )
        {
            pinMode( gAddresses[i].outputPin, OUTPUT );
        }
        gAddresses[i].onMilli = 0;
        gAddresses[i].offMilli = 0;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Basic accessory packet handler 
//
void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)
{
        // Convert NMRA packet address format to human address
    address -= 1;
    address *= 4;
    address += 1;
    address += (data & 0x06) >> 1;
    
    boolean enable = (data & 0x01) ? 1 : 0;
    
    for(int i=0; i<(int)(sizeof(gAddresses)/sizeof(gAddresses[0])); i++)
    {
        if( address == gAddresses[i].address )
        {
            Serial.print("Basic addr: ");
            Serial.print(address,DEC);
            Serial.print("   activate: ");
            Serial.println(enable,DEC);
            
            if( enable )
            {
                gAddresses[i].output = 1;
                gAddresses[i].onMilli = millis();
                gAddresses[i].offMilli = 0;
            }else{
                gAddresses[i].output = 0;
                gAddresses[i].onMilli = 0;
                gAddresses[i].offMilli = millis();
            }
        }
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup
//
void setup() 
{ 
   Serial.begin(9600);
   DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
   ConfigureDecoder();
   DCC.SetupDecoder( 0x00, 0x00, kDCC_INTERRUPT );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
void loop()
{
    static int addr = 0;
    
        ////////////////////////////////////////////////////////////////
        // Loop DCC library
    DCC.loop();
    
        ////////////////////////////////////////////////////////////////
        // Bump to next address to test
    if( ++addr >= (int)(sizeof(gAddresses)/sizeof(gAddresses[0])) )
    {
        addr = 0;
    }
    
        ////////////////////////////////////////////////////////////////
        // Turn off output?
    if( gAddresses[addr].offMilli && gAddresses[addr].offMilli<millis() )
    {
            // Clear off time
        gAddresses[addr].offMilli = 0;
        
            // Disable output
        if( gAddresses[addr].isDigital )
        {
            digitalWrite( gAddresses[addr].outputPin, LOW);
        }else{
            analogWrite( gAddresses[addr].outputPin, 0);
        }
        
            // If still enabled and a flash type, set on time
        if( gAddresses[addr].output && gAddresses[addr].isFlasher)
        {
            gAddresses[addr].onMilli = millis() + gAddresses[addr].durationMilli;
        }else{
            gAddresses[addr].output = 0;
        }
        
        return;
    }
        
        ////////////////////////////////////////////////////////////////
        // Turn on output?
    if( gAddresses[addr].onMilli && gAddresses[addr].onMilli<=millis() )
    {
            // Clear off time
        gAddresses[addr].onMilli = 0;
        
            // Enable output
        if( gAddresses[addr].isDigital )
        {
            digitalWrite( gAddresses[addr].outputPin, HIGH);
        }else{
            analogWrite( gAddresses[addr].outputPin, gAddresses[addr].analogValue);
        }
        
            // If still enabled and a flash type, set off time
        if( gAddresses[addr].durationMilli )
        {
            gAddresses[addr].offMilli = millis() + gAddresses[addr].durationMilli;
        }
        
        return;
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

