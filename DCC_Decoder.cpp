//
// DCC_Decoder.cpp - Arduino library for NMRA DCC Decoding.
// Written by Kevin Snow, MynaBay.com, November, 2011. 
// Questions: dcc@mynabay.com
// Released into the public domain.
//

#include "Arduino.h"
#include "DCC_Decoder.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Global Decoder object
//

DCC_Decoder DCC;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// NMRA DCC Definitions
//
    // Microsecond 0 & 1 timings 
#define    kONE_Min         52
#define    kONE_Max         64

#define    kZERO_Min        90
#define    kZERO_Max        10000

    // Minimum preamble length
#define    kPREAMBLE_MIN    10

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Interrupt handling
//
unsigned long          DCC_Decoder::gInterruptMicros = 0;
byte                   DCC_Decoder::gInterruptTimeIndex = 0;
volatile unsigned int  DCC_Decoder::gInterruptTime[2];
volatile unsigned int  DCC_Decoder::gInterruptChaos;

///////////////////////////////////////////////////

void DCC_Decoder::DCC_Interrupt()
{
    unsigned long ms = micros();
    gInterruptTime[gInterruptTimeIndex] = ms - gInterruptMicros;
    gInterruptMicros = ms;
    gInterruptChaos += gInterruptTimeIndex;
    gInterruptTimeIndex ^= 0x01;    
}

///////////////////////////////////////////////////

void DCC_Decoder::ShiftInterruptAlignment()
{
    noInterrupts();
    gInterruptTime[0] = gInterruptTime[1];
    gInterruptTimeIndex = 1;
    interrupts();
}

///////////////////////////////////////////////////

void DCC_Decoder::StartInterrupt(byte interrupt)
{
    gInterruptTimeIndex = 0;
    gInterruptTime[0] = gInterruptTime[1] = 0;
    gInterruptChaos = 0;
    gInterruptMicros = micros();
    
    attachInterrupt( interrupt, DCC_Interrupt, CHANGE );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Globals
//
typedef void(*StateFunc)();

    // Current state function pointer
StateFunc       DCC_Decoder::gState;                   // Current state function pointer

    // Timing data from last interrupt
unsigned int    DCC_Decoder::gLastChaos;                  // Interrupt chaos count we processed

    // Preamble bit count
int             DCC_Decoder::gPreambleCount;              // Bit count for reading preamble

    // Reset reason 
byte            DCC_Decoder::gResetReason;                // Result code of last reason decoder was reset
boolean         DCC_Decoder::gHandledAsRawPacket;

    // Packet data
byte            DCC_Decoder::gPacket[kPACKET_LEN_MAX];    // The packet data.
byte            DCC_Decoder::gPacketIndex;                // Byte index to write to.
byte            DCC_Decoder::gPacketMask;                 // Bit index to write to. 0x80,0x40,0x20,...0x01
boolean         DCC_Decoder::gPacketEndedWith1;           // Set true if packet ended on 1. Spec requires that the 
                                                          // packet end bit can count as a bit in next preamble. 
    // CV Storage
byte            DCC_Decoder::gCV[kCV_MAX];                // CV Storage (TODO - Move to PROGMEM)

    // Packet arrival timing
unsigned long   DCC_Decoder::gThisPacketMS;               // Milliseconds of this packet being parsed
boolean         DCC_Decoder::gLastPacketToThisAddress;    // Was last pack processed to this decoder's address?

unsigned long   DCC_Decoder::gLastValidPacketMS;          // Milliseconds of last valid packet
unsigned long   DCC_Decoder::gLastValidPacketToAddressMS; // Milliseconds of last valid packet to this decoder
unsigned long   DCC_Decoder::gLastValidIdlePacketMS;      // Milliseconds of last valid idle packet
unsigned long   DCC_Decoder::gLastValidResetPacketMS;     // Milliseconds of last valid reset packet

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Packet Timing Support
//
unsigned long DCC_Decoder::MillisecondsSinceLastValidPacket()
{
    return millis() - gLastValidPacketMS;
}

unsigned long DCC_Decoder::MillisecondsSinceLastPacketToThisDecoder()
{
    return millis() - gLastValidPacketToAddressMS;
}

unsigned long DCC_Decoder::MillisecondsSinceLastIdlePacket()
{
    return millis() - gLastValidIdlePacketMS;
}

unsigned long DCC_Decoder::MillisecondsSinceLastResetPacket()
{
    return millis() - gLastValidResetPacketMS;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CV Support
//
byte DCC_Decoder::ReadCV(int cv)
{
    if( cv>=kCV_PrimaryAddress && cv<kCV_MAX )
    {
        return gCV[cv];
    }
    return -1;        
}

void DCC_Decoder::WriteCV(int cv, byte data)
{
    if( cv>=kCV_PrimaryAddress && cv<kCV_MAX && cv!=kCV_ManufacturerVersionNo && cv!=kCV_ManufacturerVersionNo )
    {
        gCV[cv] = data;
    }
}

int DCC_Decoder::Address()
{
    int address;

    byte cv29 = DCC_Decoder::ReadCV(kCV_ConfigurationData1);

    if( cv29 & 0x80 )   // Is this an accessory decoder?
    {
        address = DCC_Decoder::ReadCV(kCV_AddressMSB)<<6 | DCC_Decoder::ReadCV(kCV_AddressMSB);        
    }else{
        if( cv29 & 0x20 )   // Multifunction using extended addresses?
        {
            address = DCC_Decoder::ReadCV(kCV_ExtendedAddress1)<<8 | DCC_Decoder::ReadCV(kCV_ExtendedAddress2);        
        }else{
            address = DCC_Decoder::ReadCV(kCV_PrimaryAddress);
        }
    }

    return address;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Handlers
//
BaselineControlPacket DCC_Decoder::func_BaselineControlPacket = NULL;
boolean               DCC_Decoder::func_BaselineControlPacket_All_Packets = false;

void DCC_Decoder::SetBaselineControlPacketHandler(BaselineControlPacket func, boolean allPackets)
{
    func_BaselineControlPacket = func;
    func_BaselineControlPacket_All_Packets = allPackets;
}

//////////////////////////////////////////////////////////////

RawPacket DCC_Decoder::func_RawPacket = NULL;

void DCC_Decoder::SetRawPacketHandler(RawPacket func)
{
    func_RawPacket = func;
}

//////////////////////////////////////////////////////////////

BasicAccDecoderPacket DCC_Decoder::func_BasicAccPacket = NULL;
boolean               DCC_Decoder::func_BasicAccPacket_All_Packets = false;

void DCC_Decoder::SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket func, boolean allPackets)
{
    func_BasicAccPacket = func;
    func_BasicAccPacket_All_Packets = allPackets;
}

//////////////////////////////////////////////////////////////

ExtendedAccDecoderPacket DCC_Decoder::func_ExtdAccPacket = NULL;
boolean                  DCC_Decoder::func_ExtdAccPacket_All_Packets = false;

void DCC_Decoder::SetExtendedAccessoryDecoderPacketHandler(ExtendedAccDecoderPacket func, boolean allPackets)
{
    func_ExtdAccPacket = func;
    func_ExtdAccPacket_All_Packets = allPackets;
}

//////////////////////////////////////////////////////////////

IdleResetPacket DCC_Decoder::func_IdlePacket = NULL;

void DCC_Decoder::SetIdlePacketHandler(IdleResetPacket func)
{
    func_IdlePacket = func;
}

//////////////////////////////////////////////////////////////

IdleResetPacket DCC_Decoder::func_ResetPacket = NULL;

void DCC_Decoder::SetResetPacketHandler(IdleResetPacket func)
{
    func_ResetPacket = func;
}

//////////////////////////////////////////////////////////////

DecodingEngineCompletion DCC_Decoder::func_DecodingEngineCompletion = NULL;

void DCC_Decoder::SetDecodingEngineCompletionStatusHandler(DecodingEngineCompletion func)
{
    func_DecodingEngineCompletion = func;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// State Change Macros
//
#define GOTO_DecoderReset(reason) { gState = DCC_Decoder::State_Reset; gResetReason = reason; return; }
#define GOTO_ExecutePacket()      { gState = DCC_Decoder::State_Execute; return; }
#define GOTO_ReadPacketState()    { gState = DCC_Decoder::State_ReadPacket; return; }
#define GOTO_PreambleState()      { gState = DCC_Decoder::State_ReadPreamble; return; }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Execute packet
//
void DCC_Decoder::State_Execute()
{
    int address;
    
        ///////////////////////////////////////////////////////////
        // Test error dectection
    byte errorDectection = gPacket[0] ^ gPacket[1];
    if( gPacketIndex > 3 ) errorDectection ^= gPacket[2];
    if( gPacketIndex > 4 ) errorDectection ^= gPacket[3];
    if( gPacketIndex > 5 ) errorDectection ^= gPacket[4];
    if( errorDectection != gPacket[gPacketIndex-1] )
    {
        GOTO_DecoderReset( kDCC_ERR_DETECTION_FAILED );
    }
    
        // Save off milliseconds of this valid packet
    gThisPacketMS = millis();
    gLastPacketToThisAddress = false;
    
        ///////////////////////////////////////////////////////////
        // Dispatch to RawPacketHandler - All packets go to raw (except idle and reset above)
        // 
        // gHandledAsRawPacket cleared in Reset. If packet is handled here this flag avoids
        // sending to another dispatch routine. We don't just return here because we need to 
        // figure out packet type and update time fields.
    if( func_RawPacket )
    {
        gHandledAsRawPacket = (func_RawPacket)(gPacketIndex,gPacket);
    }

        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
        // Handle 3 byte packets
    if( gPacketIndex == 3 )
    {
            ///////////////////////////////////////////////////////////
            // Decoder idle & reset packets as defined in 9.2.
        if( gPacket[1]==0x00 )
        {
                // Broadcast idle packet
            if( gPacket[0]==0xFF ) 
            {
                if( !gHandledAsRawPacket && func_IdlePacket )
                {
                    (func_IdlePacket)(gPacketIndex,gPacket);
                }
                GOTO_DecoderReset( kDCC_OK_IDLE );
            }else{
                // Broadcast reset packet
                if( gPacket[0]==0x00 )
                {
                    if( !gHandledAsRawPacket && func_ResetPacket )
                    {
                        (func_ResetPacket)(gPacketIndex,gPacket);
                    }
                    GOTO_DecoderReset( kDCC_OK_RESET );
                }
            }
        }
        
            ///////////////////////////////////////////////////////////
            // Handle as a basic accessory decoder packet
        if( ((gPacket[0] & 0xC0) == 0x80)  &&  ((gPacket[1] & 0x80) == 0x80) )
        {
            address = ~gPacket[1] & 0x70;
            address = (address<<2) + (gPacket[0] & 0x3F);
            gLastPacketToThisAddress = (address==DCC.Address());
            if( gLastPacketToThisAddress || address == 0x003F || func_BasicAccPacket_All_Packets )    // 0x003F is broadcast packet
            {
                if( !gHandledAsRawPacket && func_BasicAccPacket )
                {
                        // Call BasicAccHandler           Activate bit                         data bits
                    (func_BasicAccPacket)( address, ((gPacket[1] & 0x08) ? true : false), (gPacket[1] & 0x07));
                }
            }
            GOTO_DecoderReset( kDCC_OK_BASIC_ACCESSORY );
        }
            
            ///////////////////////////////////////////////////////////
            // Handle as a baseline packet
       
            // What decoder is this addressed to?
        if( gPacket[0] & 0x80 )
        {
            GOTO_DecoderReset( kDCC_ERR_BASELINE_ADDR );
        }
        
            // Baseline instruction packet?
        if( (gPacket[1] & 0xC0) != 0x40 )
        {
            GOTO_DecoderReset( kDCC_ERR_BASELINE_INSTR );
        }
        
        // bits as defined in 9.2
        byte addressByte =  gPacket[0] & 0x7F;
        byte directionBit = gPacket[1] & 0x20;
        byte cBit =         gPacket[1] & 0x10;
        byte speedBits =    gPacket[1] & 0x0F;
        
        // Stop or estop??
        if( speedBits==0 )
        {
            speedBits = kDCC_STOP_SPEED;    
        }else{
            if( speedBits== 1 )
            {
                speedBits = kDCC_ESTOP_SPEED;
            }else{            
                if( gCV[kCV_ConfigurationData1] & 0x02 )  // Bit 1 of CV29: 0=14speeds, 1=28Speeds
                {
                    speedBits = ((speedBits << 1 ) & (cBit ? 1 : 0)) - 3;   // speedBits = 1..28
                }else{
                    speedBits -= 1;                                         // speedBits = 1..14
                }
            }
        }
    
            // Make callback
        gLastPacketToThisAddress = (addressByte==DCC.ReadCV(kCV_PrimaryAddress));
        if( func_BaselineControlPacket_All_Packets || gLastPacketToThisAddress )
        {
            if( !gHandledAsRawPacket && func_BaselineControlPacket )
            {
                (*func_BaselineControlPacket)(addressByte,speedBits,directionBit);
            }
        }
        GOTO_DecoderReset( kDCC_OK_BASELINE );      
    }
    
        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
        // Handle 4 byte packets
    if( gPacketIndex == 4 )
    {
            ///////////////////////////////////////////////////////////
            // Handle as a extd accessory decoder packet  (4 bytes)
        if( ((gPacket[0] & 0xC0) == 0x80)  &&  ((gPacket[1] & 0x85) == 0x01) )
        {
            int msb = (gPacket[1] & 0x06);            
            address = (gPacket[1] & 0x70);
            address = (msb<<8) + (address<<2) + (gPacket[0] & 0x3F);
            gLastPacketToThisAddress = (address==DCC.Address());
            if( gLastPacketToThisAddress || address == 0x033F || func_ExtdAccPacket_All_Packets )    // 0x033F is broadcast packet
            {
                if( !gHandledAsRawPacket && func_ExtdAccPacket )
                {
                        // Call ExtAccHandler             data bits
                    (*func_ExtdAccPacket)( address, gPacket[2] & 0x1F);
                }
            }
            GOTO_DecoderReset( kDCC_OK_EXTENDED_ACCESSORY );
        }
    }
    
        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
        // Handle 5 byte packets
    if( gPacketIndex == 5  )
    {
        // TODO - Implement
    }          
        
        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
        // Handle 6 byte packets
    if( gPacketIndex == 6 )
    {
        // TODO - Implement
    }    
    
        ///////////////////////////////////////////////////////////
        // Done!
    GOTO_DecoderReset( kDCC_OK );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Standard interrupt reader - If a complete bit has been read it places timing in periodA & periodB and flows out bottom.
//
#define StandardInterruptHeader(behalfOf)                                   \
            noInterrupts();                                                 \
            if( gInterruptChaos == gLastChaos )                             \
            {                                                               \
                interrupts();                                               \
                return;                                                     \
            }                                                               \
            if( gInterruptChaos-gLastChaos > 1 )                            \
            {                                                               \
                interrupts();                                               \
                GOTO_DecoderReset( kDCC_ERR_MISSED_BITS );                  \
            }                                                               \
            unsigned int periodA = gInterruptTime[0];                       \
            unsigned int periodB = gInterruptTime[1];                       \
            gLastChaos = gInterruptChaos;                                   \
            interrupts();                                                   \
            boolean aIs1 = ( periodA >= kONE_Min && periodA <= kONE_Max );  \
            if( !aIs1 && (periodA < kZERO_Min || periodA > kZERO_Max) )     \
            {                                                               \
                GOTO_DecoderReset( kDCC_ERR_NOT_0_OR_1 );                   \
            }                                                               \
            boolean bIs1 = ( periodB >= kONE_Min && periodB <= kONE_Max );  \
            if( !bIs1 && (periodB < kZERO_Min || periodB > kZERO_Max) )     \
            {                                                               \
                GOTO_DecoderReset( kDCC_ERR_NOT_0_OR_1 );                   \
            }                                                               \

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Read packet bytes
//
void DCC_Decoder::State_ReadPacket()
{
    // Interrupt header
    StandardInterruptHeader();
    
    // Normally the two halves match. If not, reset
    if( aIs1 == bIs1 )
    {
        // 8 out of 9 times through we'll have a mask and be writing bits
        if( gPacketMask )
        {
            // Write the bit.
            if( aIs1 )
            {
                gPacket[gPacketIndex] |= gPacketMask;
            }
            // advance the bit mask
            gPacketMask = gPacketMask >> 1;
            
        }else{        
            // Getting here is the 9th time and the it's the data start bit between bytes. 
            // Zero indicates more data, 1 indicates end of packet
            
            // Advance index and reset mask
            gPacketIndex++;
            gPacketMask = 0x80;
            
            // Data start bit is a 1, that's the end of packet! Execute.
            if( aIs1 )
            {
                gPacketEndedWith1 = true;
                if( gPacketIndex>=kPACKET_LEN_MIN && gPacketIndex<=kPACKET_LEN_MAX )
                {
                    GOTO_ExecutePacket();
                }
                GOTO_DecoderReset( kDCC_ERR_INVALID_LENGTH );
            }else{
                // Data start bit is a 0. Do we have room for more data?
                if( gPacketIndex >= kPACKET_LEN_MAX )
                {
                    GOTO_DecoderReset( kDCC_ERR_MISSING_END_BIT );
                }
            }
        }
    }else{
        GOTO_DecoderReset( kDCC_ERR_NOT_0_OR_1 );
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Watch for Preamble
//
void DCC_Decoder::State_ReadPreamble()
{     
    // Interrupt header
    StandardInterruptHeader();
    
    // If we get here, booleans aIs1 and bIs1 are set to the two halves of the next bit.
    
    // If both are 1, it's a 1 bit.
    if( aIs1 && bIs1 )
    {
        // Increment preamble bit count
        ++gPreambleCount;
    }else{
        // If they equal it's a 0.
        if( aIs1 == bIs1 )
        {    
            if( gPreambleCount >= kPREAMBLE_MIN )
            { 
                // BANG! Read preamble plus trailing 0. Go read the packet.
                GOTO_ReadPacketState();
            }
        }else{
            // One is 0 the other 1. Shift alignment.
            ShiftInterruptAlignment();  
        }
        // Not enough bits in preamble or shifted alignment. Start over at zero preamble.
        gPreambleCount = 0;
    }  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Reset handling (Part 2)
//
void DCC_Decoder::State_Reset()
{    
        // EngineReset Handler  (Debugging)
    if( func_DecodingEngineCompletion )
    {
        (func_DecodingEngineCompletion)(gHandledAsRawPacket ? kDCC_OK_MAX : gResetReason);
    }
    gHandledAsRawPacket = false;
    
        // If reset with an OK code, this was a valid packet. Save off times
    if( gResetReason < kDCC_OK_MAX )
    {
        // Save MS of last valid packet
        gLastValidPacketMS = gThisPacketMS;
        
        // Save off other times
        switch( gResetReason )
        {
            case kDCC_OK_IDLE:
                gLastValidIdlePacketMS = gThisPacketMS;
                break;
            case kDCC_OK_RESET:
                gLastValidResetPacketMS = gThisPacketMS;
                break;
            case kDCC_OK_BASELINE:
            case kDCC_OK_BASIC_ACCESSORY:
            case kDCC_OK_EXTENDED_ACCESSORY:
                if(gLastPacketToThisAddress)
                {
                    gLastValidPacketToAddressMS = gThisPacketMS;
                }
                break;
            default:
                break;
        }
    }
    
        // Reset packet data
    gPacket[0] = gPacket[1] = gPacket[2] = gPacket[3] = gPacket[4] = gPacket[5] = 0;
    gPacketIndex = 0;
    gPacketMask = 0x80;
    
        // Copy last time and reset chaos
    noInterrupts();
    gPreambleCount = (gPacketEndedWith1 && gLastChaos==gInterruptChaos) ? 1 : 0;
    gLastChaos = gInterruptChaos = 0;
    interrupts();
    
        // Clear packet ended 1 flag
    gPacketEndedWith1 = false;
    
        // Go find preamble 
    GOTO_PreambleState();
}

void DCC_Decoder::State_Boot()
{   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SetupDecoder
//
void DCC_Decoder::SetupDecoder(byte mfgID, byte mfgVers, byte interrupt)  
{
    if( gInterruptMicros == 0 )
    {        
            // Save mfg info
        gCV[kCV_ManufacturerVersionNo] = mfgID;
        gCV[kCV_ManufacturedID] = mfgVers;
        
            // Attach the DCC interrupt
        StartInterrupt(interrupt);
    
            // Start decoder in reset state
        GOTO_DecoderReset( kDCC_OK_BOOT );
    }
}

void DCC_Decoder::SetupMonitor(byte interrupt)
{
    if( gInterruptMicros == 0 )
    {        
            // Attach the DCC interrupt
        StartInterrupt(interrupt);
        
            // Start decoder in reset state
        GOTO_DecoderReset( kDCC_OK_BOOT );    
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Hearbeat function. Dispatch the dcc_decoder library state machine.
//
void DCC_Decoder::loop()
{
    (gState)();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constructor (Not really).
//
DCC_Decoder::DCC_Decoder() 
{
    gState = DCC_Decoder::State_Boot;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Human readable error strings
//

const char PROGMEM*
DCC_Decoder::ResultString(byte resultCode)
{
    static const char PROGMEM* const gResults[] =
    {
        "OK",
        "OK - Unhandled",
        "OK - Boot",
        "OK - Idle packet",
        "OK - Reset packet",
        "OK - Handled raw",
        "OK - Handled baseline",
        "OK - Handled basic accessory",
        "OK - Handled extended accessory",
    };

    static const char PROGMEM* const gErrors[] =
    {
        "ERROR - Detection failed",
        "ERROR - Baseline address",
        "ERROR - Baseline instruction",
        "ERROR - Missed bits",
        "ERROR - Not 0 or 1",
        "ERROR - Invalid packet length",
        "ERROR - Missing packet end bits",
    };

    static const char PROGMEM* const gErrorsBadCode = "ERROR - Bad result code";

    if( resultCode>=0 && resultCode<(sizeof(gResults)/sizeof(gResults[0])) )
    {
        return gResults[resultCode];
    }
    if( resultCode>=100 && (resultCode-100)<(byte)(sizeof(gErrors)/sizeof(gErrors[0])) )
    {
        return gErrors[resultCode-100];
    }
    return gErrorsBadCode;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Helper to make packet strings
//
char* DCC_Decoder::MakePacketString(char* buffer60Bytes, byte byteCount, byte* packet)
{
    buffer60Bytes[0] = 0;
    if( byteCount>=kPACKET_LEN_MIN && byteCount<=kPACKET_LEN_MAX )
    {
        int i = 0;
        for(byte byt=0; byt<byteCount; ++byt)
        {
            byte bit=0x80;
            while(bit)
            {
                buffer60Bytes[i++] = (packet[byt] & bit) ? '1' : '0';
                bit=bit>>1;
            }
            buffer60Bytes[i++] = ' ';
        }
        buffer60Bytes[--i] = 0;
    }
    return buffer60Bytes;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Helper to return preamble length
//
int DCC_Decoder::LastPreambleBitCount()
{
    return gPreambleCount;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
