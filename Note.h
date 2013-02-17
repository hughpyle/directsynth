#ifndef NOTEH
#define NOTEH

//#include "arduino.h"
#include <arm_math.h>

#ifdef __MK20DX128__
#define sine arm_sin_f32
#else
#define sine sin
#define __PKHTB(ARG1,ARG2,ARG3)          ( ((((uint32_t)(ARG1))          ) & 0xFFFF0000UL) |  \
                                           ((((uint32_t)(ARG2)) >> (ARG3)) & 0x0000FFFFUL)  )
#endif

#include "midi_notes.h"
#include <stdlib.h>

#define SAMPLERATE   48000

/*
    Note:  represents a single note playing.

    Constructor:  provide a note frequency, volume, timbre.
    Subsequently: get samples, until you want to stop.
*/


/* Globals */


/* Number of overtones that could be provisioned. Not more than 127. */
#define MAXPARTIALS 16


// resonant frequencies of a circular drumhead are at zeros of the Bessel functions
// http://cose.math.bas.bg/webMathematica/webComputing/BesselZeros.jsp
// We normalize these, so the "fundamental" is at the requested note-pitch
// Also stiff material may push higher modes to slightly higher frequencies
#define NBESS 6
static const float besselJZeros[6][6] = {
    { 2.404825557695773, 5.520078110286311, 8.653727912911011, 11.79153443901428, 14.93091770848779, 18.07106396791092 }, /* s=0 */
    { 3.831705970207512, 7.015586669815613, 10.17346813506272, 13.32369193631422, 16.47063005087763, 19.61585851046824 }, /* s=1 */
    { 5.135622301840683, 8.417244140399855, 11.61984117214906, 14.79595178235126, 17.95981949498783, 21.11699705302185 }, /* s=2 */
    { 6.380161895923984, 9.761023129981667, 13.01520072169843, 16.22346616031877, 19.40941522643501, 22.58272959310444 }, /* s=3 */
    { 7.588342434503804, 11.06470948850119, 14.37253667161759, 17.61596604980483, 20.82693295696239, 24.01901952477111 }, /* s=4 */
    { 8.771483815959954, 12.33860419746694, 15.70017407971167, 18.98013387517992, 22.21779989656127, 25.43034115422270 }  /* s=5 */
    };

#define Q15MAXF (32767.0/32768.0)

/* Comparable & Comparator */

// After all the partials are generated, we sort them descending by amplitude.
// That allows rendering to process as many or as few as it needs, within its time constraints.
struct tmp_nh {
    q15_t re;       // 'real' component (sine)
    q15_t im;       // 'imaginary' component (cosine)
    q15_t d;        // delta-per-iteration
    q15_t g;        // gain
};

// Compare by abs(gain)
int tmp_cmp( const void *a, const void *b )
{
    const tmp_nh *ia = (const tmp_nh *)a;
    const tmp_nh *ib = (const tmp_nh *)b;
    return (abs(ib->g) - abs(ia->g));
};


/* Class */

class Note
{
    private:
        uint8_t  midinote;
        uint8_t  nActivePartials;
        int8_t   sustaining;            // 0=sustaining, 1=not, -1=off (therefore not sustaining)
        int8_t   spare;
        int32_t  nFills;                // Number of times the addSamplesToBuffer() has been called
        uint32_t a[MAXPARTIALS];        // each is a packed pair of Q15's [const 0.5f, delta]
        uint32_t b[MAXPARTIALS];        // each is a packed pair of Q15's [real, imaginary]
        q15_t    g[MAXPARTIALS];        // each is a packed pair of Q15's [gain, gain]

        int newPartial( tmp_nh * tmp, int8_t * ph, int i, float freq, float volume, float timbre1, float timbre2, float timbre3, int drumlike );
        void setSustain( int8_t nSustaining );

    public:
        Note();
        void on( uint8_t note, uint8_t volume, float timbre1, float timbre2, float timbre3, float drumness );
        void off();
        
        uint32_t getSample();
        inline void addSamplesToBuffer( q31_t *pBuf, uint8_t nSamples );
        inline uint8_t getMidiNote() { return midinote; }
        inline int8_t getActive() { return nActivePartials; }

        static float midi_note_freq( uint8_t midinote );
};


float Note::midi_note_freq( uint8_t note )
{
    if( note<0 ) note=0;
    if( note>127 ) note=127;
    return midi_notes[note];
}


int Note::newPartial( tmp_nh * tmp, int8_t * ph, int i, float freq, float volume, float timbre1, float timbre2, float timbre3, int drumlike )
{
    int h = *ph;
    int j;
    int k;
    double a;
    int ok = 1;
    int done = 0;
    float base = 0;
    float mult = 0;
    float f = 0;
    if( drumlike )
    {
        // circular drum, resonant modes are zeros of the Bessel functions
        j = i % NBESS;
        k = i / NBESS;
        if( k<NBESS )
            base = ( besselJZeros[j][k] / besselJZeros[0][0] ) - 1;
        else
            done = 1;
        j += k;
    }
    else
    {
        // string, resonant frequencies are partial multiples of the fundamental
        j = i;
        base = i;
    }

    // timbre1 drives "inharmonicness": power greater than 1.0 will make the overtones slightly sharp, and vice versa.
    mult = pow( base, timbre1 ) + 1;
    f = 2 * sine(freq*mult*PI/SAMPLERATE);

    // Amplitude of each partial = from "timbre2" position along the string or surface.
    // We "naturally" reduce these by 1/j, but timbre3 controls how fast that happens.
    a = sine( (j+1) * timbre2 * PI );
    if( j>0 && timbre3==0.0 )
        a = 0;
    else
        a = a / pow( j+1, timbre3 );

    if( f>Q15MAXF ) { done=1; ok=0; }       // frequency is too high to cope with
    if( a>Q15MAXF ) a=Q15MAXF;

    //printf( "%f %f %f %d\n", f, a, volume, ok  );
    if( ok )
    {
        tmp[h].d = f * 16384;               // Delta for iteration calc.  Halved because we double in the iter.
        tmp[h].g = a * volume * 16384;      // Gain for this overtone
        tmp[h].re = 0;                      // Starting real value is zero (no starting-glitch please!)
        tmp[h].im = 30000;                  // Starting imaginary value
        h++;
    }

    *ph = h;
    return done;
};

/*
    Constructor
*/
Note::Note()
{
    nActivePartials = 0;
}

/*
    Initialize
        note: midi-note.
        volume:  range 0 to 127.
        timbre1: tunedness of partials.  Usually in the range 0.8 to 1.1 (1.0 is exact).
        timbre2: position along the surface.  Range 0 to 1.
        timbre3: overtone damping powerlaw.  Range from 1.0 (harsh) to 2.0 (dull).  Zero to suppress all overtones.
*/
void Note::on( uint8_t note, uint8_t vol, float timbre1, float timbre2, float timbre3, float drumness )
{
    // Construct the root and overtones.
    // First construct all the string-like partials.  Then drum-like after that.
    // Then sort by descending amplitude - then we can skip processing the tail end if we need more performance.

    float freq = midi_note_freq( note );
    float volume = vol/127.0;
    int8_t h = 0;
    int8_t i = 0;
    tmp_nh tmp[MAXPARTIALS];
    
    midinote = note;
    nFills = 0;

    // while(playing) sleep(0);
    nActivePartials = 0;
    memset( tmp, 0, MAXPARTIALS * sizeof(tmp_nh) );
    memset( a, 0, MAXPARTIALS * sizeof(uint32_t) );		// zero this.a[]
    memset( b, 0, MAXPARTIALS * sizeof(uint32_t) );		// zero this.b[]
    memset( g, 0, MAXPARTIALS * sizeof(q15_t) );		// zero this.g[]

    if( volume>0.0 )
    {
        int8_t done = 0;
		float v = volume * (1.0-drumness);
        for( i=0; v>0.0 && h<MAXPARTIALS && !done; i++ )
        {
            done = newPartial( tmp, &h, i, freq, v, timbre1, timbre2, timbre3, false );   /* stringlike */
            //printf( "string - %d %f\n", h, tmp[h-1].g );
        }
        done = 0;
		v = volume * drumness;
        for( i=0; v>0.0 && h<MAXPARTIALS && !done; i++ )
        {
            done = newPartial( tmp, &h, i, freq, v, timbre1, timbre2, timbre3, true );  /* drumlike */
            //printf( "drum - %d %f\n", h, tmp[h-1].g );
        }

        // Sort the partials by descending absolute amplitude
        qsort( tmp, MAXPARTIALS, sizeof(tmp_nh), tmp_cmp );

        for( i=0; i<MAXPARTIALS; i++ )
        {
            // g[] is the gain
            g[i] = tmp[i].g;
			if(g[i]<0)
			{
				// Make abs(), because having always-positive g is useful later
				g[i] = -g[i];
				tmp[i].re = -tmp[i].re;
				tmp[i].im = -tmp[i].im;
			}
//            if( g[i]<=0 )
//            {
//				// Small enough to ignore
//                break;
//            }
            nActivePartials++;
            // a[lo,hi] is [const 0.5, delta]
            // b[lo,hi] is [re,im].  But for odd-numbered partials we swap them cos of the gain summing thing later
            a[i] = __PKHBT( 16384, tmp[i].d, 16 );
            if( i & 1 )
            {
                b[i] = __PKHBT( tmp[i].re, -tmp[i].im, 16 );
            }
            else
            {
                b[i] = __PKHBT( tmp[i].im, tmp[i].re, 16 );
            }
        }
        setSustain(1);
    }
}


void Note::off()
{
    setSustain(-1);
}


void Note::setSustain( int8_t nSustaining )
{
    // 0=sustaining, 1=normal, -1=note-off (therefore not sustaining)
    sustaining = nSustaining;
    
    // Set gains of all the partials
    if( nActivePartials==0 ) return;
    for( int8_t i=0; i<MAXPARTIALS; i++ )
    {
        // a[lo,hi] is [const 0.5, delta]
        uint16_t g; // = 16384;
		g = 16383;
//        if( nSustaining==0 ) g = 16383 + (i & 1);	// let the fundamental and odd harmonics decay
//		if( nSustaining==0 ) g = 16384 - (i & 1);	// let the even harmonics decay
		if( nSustaining==0 ) g = (i & 2) ? 16383 : 16384;
        else if( nSustaining==-1 ) g = 16377;
        a[i] &= 0xFFFF0000u;
        a[i] |= g;
    }
}


uint32_t Note::getSample()
{
    uint32_t p, q;
    // TODO make very efficient dither or go home
    // r = ( rand()<<6 ) % 0x30000;

    // a[hi,lo] = [delta,0x4000]
    // b[hi,lo] = [y,x]

    // x' = [0.5]*[x] + [delta]*[y], as Q30
    // b[hi,lo] = [y,x'].  Shift x by 15 (Q30 to Q15) and multiply by 2.
    // y' = [0.5*y] - [delta*x'] as Q30
    // b[hi,lo] = [y',x'].  Take high-word of y (Q30 to Q15) and multiply by 2.

    uint8_t i = 0;
    // Calculate all the partials
    p = __SMUAD(a[i],b[i]);
    q = __PKHTB(b[i],p,14);
    p = __SMUSDX(a[i],q);
    b[i] = __PKHBT(q,p,2);

    i++;
    p = __SMUAD(a[i],b[i]);
    q = __PKHTB(b[i],p,14);
    p = __SMUSDX(a[i],q);
    b[i] = __PKHBT(q,p,2);

	i++;
    p = __SMUAD(a[i],b[i]);
    q = __PKHTB(b[i],p,14);
    p = __SMUSDX(a[i],q);
    b[i] = __PKHBT(q,p,2);

    i++;
    p = __SMUAD(a[i],b[i]);
    q = __PKHTB(b[i],p,14);
    p = __SMUSDX(a[i],q);
    b[i] = __PKHBT(q,p,2);

    i++;
#if MAXPARTIALS>4
    if(nActivePartials>4)
    {
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
    }
#endif
#if MAXPARTIALS>8
    if(nActivePartials>8)
    {
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
    }
#endif
#if MAXPARTIALS>12
    if(nActivePartials>12)
    {
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
        p = __SMUAD(a[i],b[i]);
        q = __PKHTB(b[i],p,14);
        p = __SMUSDX(a[i],q);
        b[i] = __PKHBT(q,p,2);
        i++;
    }
#endif

    // Sum value*gain for each partial.
    // The partials b[] are [q15_t re][q15_t im]
    // We multiply by their corresponding gains a pair at a time, like this
    //   b[1]re b[1]im b[2]re b[2]im b[3]re b[3]im b[4]re b[4]im
    // *        g[i]   g[2]   (...skip...)  g[3]   g[4]   (skip...)
    // so actually odd-numbered b[] have their im & re swapped.

    uint16_t * pp = (uint16_t *)&b[0] + 1;
    uint32_t * pB = (uint32_t *)pp;         // pointer to b0-and-a-half
    uint32_t * pG = (uint32_t *)g;          // pointer to g[0-and-1]
    int32_t r = 0;
    
    // SMLAD = (lo*lo)+(hi*hi)+accumulator
    r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
    r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
#if MAXPARTIALS>4
    if(nActivePartials>4)
    {
        r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
        r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
    }
#endif
#if MAXPARTIALS>8
    if(nActivePartials>8)
    {
        r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
        r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
    }
#endif
#if MAXPARTIALS>12
    if(nActivePartials>12)
    {
        r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
        r = __SMLAD( *pB, *pG, r );  pB++; pB++; pG++;
    }
#endif

    // return r>>16; //__SMUAD(r,zz)>>15;
	return __PKHTB(r,r,16);
}


void Note::addSamplesToBuffer( q31_t *pBuf, uint8_t nSamples )
{
    if( nActivePartials==0 ) return;
    
    for( int16_t i=0; i<nSamples; i++ )
    {
        uint32_t v = getSample();
        uint32_t p = *pBuf;
        *pBuf++ = __QADD16( p, v );
    }

	// Recalculate the number of active partials
	// (or, the highest partial-number that we care about -- they're handled in blocks of 4)

	int8_t j = nActivePartials-1;
	uint32_t t=(__SMUAD(b[j],b[j])>>16) * g[j];	// x^2+y^2 * gain
	if(t<0x1000)
	{
		nActivePartials--;
	}

    // set sustain on after a short decay
    nFills++;
    if( nFills==50 && sustaining==1 ) setSustain(0);
}

#endif




