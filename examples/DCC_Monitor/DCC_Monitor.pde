#include <DCC_Decoder.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines and structures
//
#define kDCC_INTERRUPT            0

typedef struct
{
    int count;
    byte validBytes;
    byte data[6];
} DCCPacket;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The dcc decoder object and global data
//
int gPacketCount = 0;
int gIdlePacketCount = 0;
int gLongestPreamble = 0;

DCCPacket gPackets[25];

static unsigned long lastMillis = millis();
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Packet handlers
//

// ALL packets are sent to the RawPacket handler. Returning true indicates that packet was handled. DCC library starts watching for 
// next preamble. Returning false and library continue parsing packet and finds another handler to call.
boolean RawPacket_Handler(byte byteCount, byte* packetBytes)
{
        // Bump global packet count
    ++gPacketCount;
    
    int thisPreamble = DCC.LastPreambleBitCount();
    if( thisPreamble > gLongestPreamble )
    {
        gLongestPreamble = thisPreamble;
    }
    
        // Walk table and look for a matching packet
    for( int i=0; i<(int)(sizeof(gPackets)/sizeof(gPackets[0])); ++i )
    {
        if( gPackets[i].validBytes )
        {
                // Not an empty slot. Does this slot match this packet? If so, bump count.
            if( gPackets[i].validBytes==byteCount )
            {
                char isPacket = true;
                for( int j=0; j<byteCount; j++)
                {
                    if( gPackets[i].data[j] != packetBytes[j] )
                    {
                        isPacket = false;
                        break;
                    } 
                }
                if( isPacket )
                {
                   gPackets[i].count++;
                   return false;
                }
            }
        }else{
                // Empty slot, just copy over data
            gPackets[i].count++;
            gPackets[i].validBytes = byteCount;
            for( int j=0; j<byteCount; j++)
            {
                gPackets[i].data[j] = packetBytes[j];
            }
            return false;
        }
    }    
    
    return false;
}

// Idle packets are sent here (unless handled in rawpacket handler). 
void IdlePacket_Handler(byte byteCount, byte* packetBytes)
{
    ++gIdlePacketCount;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup
//
void setup() 
{ 
   Serial.begin(9600);
    
   DCC.SetRawPacketHandler(RawPacket_Handler);   
   DCC.SetIdlePacketHandler(IdlePacket_Handler);
            
   DCC.SetupMonitor( kDCC_INTERRUPT );   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DumpAndResetTable()
{
    char buffer60Bytes[60];
    
    Serial.print("Total Packet Count: ");
    Serial.println(gPacketCount, DEC);
    
    Serial.print("Idle Packet Count:  ");
    Serial.println(gIdlePacketCount, DEC);
        
    Serial.print("Longest Preamble:  ");
    Serial.println(gLongestPreamble, DEC);
    
    Serial.println("Count    Packet_Data");
    for( int i=0; i<(int)(sizeof(gPackets)/sizeof(gPackets[0])); ++i )
    {
        if( gPackets[i].validBytes > 0 )
        {
            Serial.print(gPackets[i].count, DEC);
            if( gPackets[i].count < 10 )
            {
                Serial.print("        ");
            }else{
                if( gPackets[i].count < 100 )
                {
                    Serial.print("       ");
                }else{
                    Serial.print("      ");
                }
            }
            Serial.println( DCC.MakePacketString(buffer60Bytes, gPackets[i].validBytes, &gPackets[i].data[0]) );
        }
        gPackets[i].validBytes = 0;
        gPackets[i].count = 0;
    }
    Serial.println("============================================");
    
    gPacketCount = 0;
    gIdlePacketCount = 0;
    gLongestPreamble = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
void loop()
{
    DCC.loop();
    
    if( millis()-lastMillis > 2000 )
    {
        DumpAndResetTable();
        lastMillis = millis();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

