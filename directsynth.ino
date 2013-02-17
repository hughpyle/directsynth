/* ---- TODO ---- */
/*
  Moar performance
  
  Finish & test the Note LRU in note_on()
  
  Sustain: don't sustain for the first while, then do, then stop.  <= test
  
  Better control over how many partials are rendered.  Give the renderer a number.
  
  Dither? Drive from a lookup table for speed??
  
  MIDI channel number: listen to channel selection dip switches on four digital in pins (to ground -- use internal pullups)
  
  Read timbre from hardware pots
  
*/


/*
  Teensy pin assignments:
  
*/

// Teensy 3.0 using ARM Cortex M4 math routines and SIMD macros
#define ARM_MATH_CM4
#include <arm_math.h>

// Note synthesizer class
#include "Note.h"

// MIDI input.
// Note: need to edit the midi library header for the serial port
// For Teensy3, the UART pins Rx=0 and Tx=1 (Serial1)
//                            Rx=9 and Tx=10 (Serial2)
//                            Rx=7 and Tx=8  (Serial3)
// Note: need to uncomment "typedef uint16_t..." in the midi library header.
#include <MIDI.h>

// MIDI input channel default to "omni" (listen to everything)
byte channel = MIDI_CHANNEL_OMNI;

/* I2S digital audio */
#define I2S_CLOCK_TYPE    I2S_CLOCK_EXTERNAL
#define I2S_PIN_PATTERN   I2S_TX_PIN_PATTERN_1
#include <i2s.h>
void dma_fill( int16_t *pBuf, uint16_t len );

/* Wolfson audio codec controlled by I2C */
/* Library here: https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731 */
#include <Wire.h>
#include <WM8731.h>

float timbre1 = 1.00;    // Inharmonic power law
float timbre2 = 0.335;     // Position on the surface
float timbre3 = 1.8;     // Harmonic decay power law
int8_t drum = 0;


/* ---- Notes LRU ---- */

// Polyphony:
#define MAXNOTES 7

// Some Notes.
// Notes are constructed at start and re-used (no need to realloc their buffers).
Note * note[MAXNOTES];

// List of the *pNote indexes for each of the MIDI note numbers
int8_t midinote[128];  // stores index into note[]. so note[ midinotes[0..127] ] is the note object

int8_t testnotes[10];

void init_notes()
{
    // Initialize arrays of notes
    for( int8_t i=0; i<MAXNOTES; i++ )
    {
        note[i] = new Note();
    }
    for( int16_t i=0; i<=127; i++ )
    {
        midinote[i] = -1;
    }
    for(int8_t i=0; i<10; i++)
    {
        testnotes[i]=0;
    }
}


/* ---- MIDI ---- */

void note_on( byte n, byte v )
{
    Note * pN;
    int8_t i = midinote[n];
    Serial.println( String("midinote:") + i );

    if( i==-1 )
    {
        i = n % (MAXNOTES-1);     // super-simple-hash-function, designed to not collide on octaves (i.e. %12 or %24 or %36 would be bad)
        Serial.println( String("midinote ") + i );
    }
    pN = note[i];
    
    // Is this note already active but the wrong note?  In that case we need to find another slot.
    if( pN->getActive() )
    {
        Serial.println( "Active" );
        if( pN->getMidiNote() != n )
        {
            // Uh-oh.  Wrong-frequency note is in this slot, let's find another slot.
            // Walk until we find an inactive slot.
            Serial.println( "Oops" );
            int8_t j = i+1;
            while( j != i )
            {
                pN = note[j];
                if( !pN->getActive() ) break;
                Serial.println( String("Can't use ") + j );
                j++;
                if( j==MAXNOTES ) j=0;
            }
            i=j;
            Serial.println( String("midinote ") + i );  // TODO
            // Fell out of that loop.  Either pN is inactive,
            // or there are no inactive notes and we'll evict the note that's in our rightful place.
        }
        else
        {
            // Same note, already active.  Oh well.  Just re-initialize it.
        }
    }
    midinote[n]=i;
    pN->on( n, v, timbre1, timbre2, timbre3, drum );

    int8_t na = 0;
    for( int8_t i=0; i<MAXNOTES; i++ )
    {
        if( note[i]->getActive() ) { na++; }
    }
    Serial.println( String("Now ") + na );
}

void note_off( byte n )
{
    int8_t i = midinote[n];
    if( i!=-1 )
    {
        Serial.println( String("midinote:") + i );
        Note *pN = note[i];
        pN->off();
    }
}


void note_atp( byte n, byte v )
{
  // After Touch Poly just reset the volume of the existing note - don't restart the note attack
}


void handle_midi_message()
{
  byte d1, d2, note, velocity, channel;
  byte type = MIDI.getType();
  switch(type)
  {
    case NoteOn:
      note = MIDI.getData1();
      velocity = MIDI.getData2();
      channel = MIDI.getChannel();
      if (velocity > 0) {
        Serial.println(String("Note On:  ch=") + channel + ", note=" + note + ", velocity=" + velocity);
        note_on( note, velocity );
      } else {
        Serial.println(String("Note Off: ch=") + channel + ", note=" + note);
        note_off( note );
      }
      break;

    case NoteOff:
      note = MIDI.getData1();
      velocity = MIDI.getData2();    // we ignore
      channel = MIDI.getChannel();
      Serial.println(String("Note Off: ch=") + channel + ", note=" + note);
      note_off( note );
      break;

    case AfterTouchChannel:
      velocity = MIDI.getData1();
      channel = MIDI.getChannel();
      Serial.println(String("After Touch Channel: ch=") + channel + ", velocity=" + velocity);
      break;

    case AfterTouchPoly:
      note = MIDI.getData1();
      velocity = MIDI.getData2();
      channel = MIDI.getChannel();
      Serial.println(String("After Touch Poly: ch=") + channel + ", note=" + note + ", velocity=" + velocity);
      note_atp( note, velocity );
      break;
 
    case ControlChange:
      d1 = MIDI.getData1();
      d2 = MIDI.getData2();
      Serial.println(String("Control Change: cc=") + d1 + ", value=" + d2);
      control_change( d1, d2 );
      break;

    case PitchBend:
      // ignore d1 despite spec saying value is 14 bits.
      // Korg 707 only uses 7 bits, center value 64
      d1 = MIDI.getData1();
      d2 = MIDI.getData2();
      Serial.println(String("Pitch bend: ") + d2);
      break;

    case ActiveSensing:
      // Yes, it's alive
//    Serial.println( String("CPU ") + (int)(100*((float)callback_dur / (callback_spare + callback_dur))) );
      break;

    default:
      d1 = MIDI.getData1();
      d2 = MIDI.getData2();
      Serial.println(String("Message, type=") + (int)type + ", data = " + d1 + " " + d2);
  }
}

void control_change( byte d1, byte d2 )
{
  // CC 1 = master volume
  // CC 64 = damper pedal (sustain) on/off (>64 on)
  // CC 67 = soft pedal on/off
  // CC 74 = cutoff frequency
  // CC 71 = resonance
  // CC 72 = release time
  // CC 91 = reverb level
  // CC 74 = brightness
  switch(d1)
  {
    case 1:
      WM8731.setOutputVolume( d2 );
      break;
    case 2:
    default:
      break;
  }
}


/* ---- DMA callback ---- */

// The DMA process sends I2S data to the codec from two buffers and ping-pongs between them.
// When it switches to buffer A, it asks us to fill buffer B with data; and vice versa.
// The buffer should contain alternating left-channel and right-channel Q15 values.

// TODO figure a "mono" DMA scheme?

void dma_fill( int16_t *pBuf, uint16_t len )
{
    // Note: 'len' is the number of WORDS in the buffer.  Each sample consumes two (left, right).
    uint16_t nSamples = len>>1;

    // Zero the buffer
    memset( pBuf, 0x0000, len*sizeof(int16_t) );

    // Sum all the notes into the buffer
    for(int8_t j=0; j<MAXNOTES; j++)
    {
        if( note[j] )
        {
            note[j]->addSamplesToBuffer( (int32_t *)pBuf, nSamples );
        }
    }
}


/* ---- Arduino setup & loop ---- */

void setup() 
{
  // Delay startup for debugging purposes
  for( int8_t i=0; i<3; i++ )
  {
    delay(1000);
    Serial.println( String("# initializing...") + i );
  }
  
  // Initialize the array of notes
  init_notes();
  Serial.println( "Notes initialized" );
    
  // Initialize MIDI
  MIDI.begin( channel );
  Serial.println( "MIDI initialized" );
  
  // Initialize the codec for audio output
  uint8_t interface = WM8731_INTERFACE_FORMAT(I2S)
                    | WM8731_INTERFACE_WORDLEN(bits16)
                    | WM8731_INTERFACE_MASTER;
  WM8731.begin( low, WM8731_SAMPLING_RATE(hz48000), interface );
  WM8731.setActive();
  WM8731.setOutputVolume(0xFF);
  Serial.println( "Codec initialized" );
  
  // Initialize I2S and DMA, using the dma_fill callback handler
  I2STx0.begin( dma_fill );
  I2STx0.start();
}


uint32_t nloops = 0;
void loop()
{
  nloops++;
  // TODO read timbre from hardware pots
  // (may be overridden by incoming midi, but reset to hardware values if they change)

  // Is there an incoming MIDI message?
  // TODO implement serial-port interrupts to respond faster
  if (MIDI.read() ) handle_midi_message();
  
  delay(10);
  // delay( rand() % 20 );
  
  // Randomly play some notes
  if( nloops % 200 == 0 )
  {
    int i = rand() % 3;
    int t = testnotes[i];
    if( t==0 )
    {
        t = 32 + ( rand() % 48 );
        Serial.print("====ON====");
        Serial.println(t,DEC);
        note_on( t, 127 );
        testnotes[i] = t;
    }
    else
    {
        Serial.print("====OFF====");
        Serial.println(t,DEC);
        note_off( t );
        testnotes[i] = 0;
    }
  }
}


