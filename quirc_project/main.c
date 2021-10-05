//-----------------------------------------------------------------------------
//         Headers
//-----------------------------------------------------------------------------

#include <board.h>
#include <string.h>
#include <stdio.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <dbgu/dbgu.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <utility/assert.h>
#include <utility/video.h>
#include <isi/isi.h>
#include <utility/bmp.h>
#include <drivers/twi/twid.h>
#include <twi/twi.h>
#include <components/omnivision/omnivision.h>
#include <components/omnivision/ov9655/ov9655.h>
#include <quirc.h>

//-----------------------------------------------------------------------------
//         Local Define
//-----------------------------------------------------------------------------

/// Number of preview buffer
#define AT91C_ISI_MAX_PREV_BUFFER    1

struct quirc *q = NULL;
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;

/// TWI clock frequency in Hz (400KHz)
#define TWCK            400000

/// List of ISI pins that must be configured for use by the application.
#define PINS_ISI BOARD_ISI_PIO_CTRL1,\
                 BOARD_ISI_PIO_CTRL2,\
                 BOARD_ISI_TWCK,\
                 BOARD_ISI_TWD,\
                 BOARD_ISI_MCK,\
                 BOARD_ISI_VSYNC,\
                 BOARD_ISI_HSYNC,\
                 BOARD_ISI_PCK,\
                 BOARD_ISI_PINS_DATA

/// TWI peripheral redefinition if needed
#if !defined(AT91C_BASE_TWI) && defined(AT91C_BASE_TWI0)
    #define AT91C_BASE_TWI      AT91C_BASE_TWI0
    #define AT91C_ID_TWI        AT91C_ID_TWI0
    #define PINS_TWI            PINS_TWI0
#endif

//-----------------------------------------------------------------------------
//         Local variables
//-----------------------------------------------------------------------------

/// Video driver instance
AT91S_VIDEO S_Video;

/// TWI driver instance.
static Twid twid;

#define BASE_ADRESSE_BUFFER AT91C_EBI_SDRAM

/// Base adresse in SDRAM used for ISI and LCD
unsigned char *BuffCam    = (unsigned char *) (BASE_ADRESSE_BUFFER + 0x00300100);
/// Base adresse in SDRAM used for BMP header corresponding at BuffCam
unsigned char *BuffCamBMP = (unsigned char *) (BASE_ADRESSE_BUFFER + 0x00300000);
/// Base adresse in SDRAM used for Luma transformation
unsigned char *BuffTrans    = (unsigned char *) (BASE_ADRESSE_BUFFER + 0x00500100);

static const Pin pins[] = {PINS_TWI, PINS_ISI};


/// Frame Buffer Descriptors
ISI_FrameBufferDescriptors  FbList[AT91C_ISI_MAX_PREV_BUFFER+1];
ISI_Descriptors IsiDescriptors;

//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Frame Buffer Descriptors (FBD)
/// \param pVideo: codec vsize, codec hsize and lcd_nbpp
//-----------------------------------------------------------------------------
void AllocateFBD(AT91PS_VIDEO pVideo)
{
    unsigned int i;
    unsigned int Fb_offset;

    Fb_offset = ((pVideo->codec_vsize)*(pVideo->codec_hsize)*(pVideo->lcd_nbpp))/8;

    for(i = 0; i <= AT91C_ISI_MAX_PREV_BUFFER; i++) {
        FbList[i].Current = (unsigned int)BuffCam + (i*Fb_offset);
        FbList[i].Next    = (int)&FbList[i+1];
    }
    // Wrapping to first FBD
    FbList[i-1].Next = (int)&FbList[0];

    TRACE_DEBUG("FbList[0].Current = 0x%X\n\r", FbList[0].Current);
    TRACE_DEBUG("FbList[0].Next = 0x%X\n\r", FbList[0].Next);
    TRACE_DEBUG("FbList[1].Current = 0x%X\n\r", FbList[1].Current);
    TRACE_DEBUG("FbList[1].Next = 0x%X\n\r", FbList[1].Next);
}

//-----------------------------------------------------------------------------
/// ISI interrupt handler
/// On "Fifo Preview Empty" interrupt, update LCD Frame Buffer Address
/// Generate statistics for "Fifo Codec Empty", "Fifo Codec Overflow" and
/// Fifo Preview Overflow
//-----------------------------------------------------------------------------

void ISR_IsiHandler(void)
{
    unsigned int status = ISI_StatusRegister();

    //TRACE_DEBUG("I:0x%X", status);
    // Fifo Preview Empty
    if(status & AT91C_ISI_FO_P_EMP) {
        AIC_DisableIT(AT91C_ID_SYS);
        unsigned int offset = 0;
        if( S_Video.IsiPrevBuffIndex < AT91C_ISI_MAX_PREV_BUFFER) {
            S_Video.lcd_fb_addr = FbList[S_Video.IsiPrevBuffIndex].Current;
            S_Video.IsiPrevBuffIndex++;
        }
        else {
            offset = ((S_Video.codec_vsize)*(S_Video.codec_hsize)*(S_Video.lcd_nbpp))/8;
            S_Video.lcd_fb_addr = FbList[0].Current;
            S_Video.IsiPrevBuffIndex = 0;
            IsiDescriptors.CurrentLcdIndex = 0;
        }

                VIDEO_Ycc2Luminance((unsigned char *)BuffCam,
                               (unsigned short *)BuffTrans,
                               (S_Video.codec_hsize*S_Video.codec_vsize), offset);

                if (q == NULL)
                {


                } else {
                     quirc_end(q);
                        int count = quirc_count(q);
                        for (int i = 0; i < count; i++)
                        {
                          quirc_extract(q, i, &code);
                          printf("Point 1: %d, %d, \n", code.corners[0].x, code.corners[0].y);
                          printf("Point 2: %d, %d, \n", code.corners[1].x, code.corners[1].y);
                          printf("Point 3: %d, %d, \n", code.corners[2].x, code.corners[2].y);
                          printf("Point 4: %d, %d, \n", code.corners[3].x, code.corners[3].y);
                          err = quirc_decode(&code, &data);
                          if (err) {

                          } else {
                           printf("Version: %d\n", data.version);
                           printf("ECC level: %c\n", "MLHQ"[data.ecc_level]);
                           printf("Mask: %d\n", data.mask);
                           printf("Length: %d\n", data.payload_len);
                           printf("Payload: %s\n", data.payload);
                          }
                        }
                }
                quirc_destroy(q);
                AIC_EnableIT(AT91C_ID_SYS);
    }
    // Fifo Codec Empty
    if(status & AT91C_ISI_FO_C_EMP) {
        TRACE_DEBUG("Fifo Codec Empty\n\r");
        IsiDescriptors.DisplayCodec = 1;
    }
    // Fifo Codec Overflow
    if(status & AT91C_ISI_FO_C_OVF) {
        TRACE_DEBUG("Fifo Codec Overflow\n\r");
        IsiDescriptors.nb_codec_ovf++;
    }
    // Fifo Preview Overflow
    if(status & AT91C_ISI_FO_P_OVF) {
        TRACE_DEBUG("Fifo Preview Overflow\n\r");
        IsiDescriptors.nb_prev_ovf++;
    }
}


//------------------------------------------------------------------------------
/// TWI interrupt handler. Forwards the interrupt to the TWI driver handler
//------------------------------------------------------------------------------
void ISR_Twi(void)
{
    TWID_Handler(&twid);
}

//------------------------------------------------------------------------------
/// Generic initialisation of test
/// \param pVideo video instance
//-----------------------------------------------------------------------------
void main_Test(AT91PS_VIDEO pVideo)
{
    TRACE_DEBUG("main_Test\n\r");
    S_Video.Hblank = 0;
    S_Video.Vblank = 0;
    S_Video.lcd_fb_addr = (unsigned int)BuffCam;
    // base address of the descriptor list for preview DMA
    S_Video.Isi_fbd_base = (int)&FbList[0];
    S_Video.codec_fb_addr = 0;
    S_Video.rgb_or_yuv = YUV;


    ISI_DisableInterrupt( AT91C_ISI_FO_P_EMP | AT91C_ISI_FO_C_EMP
                        | AT91C_ISI_FO_C_OVF | AT91C_ISI_FO_P_OVF);
    ISI_Init(&S_Video);

    ISI_EnableInterrupt( AT91C_ISI_FO_P_EMP | AT91C_ISI_FO_C_EMP
                       | AT91C_ISI_FO_C_OVF | AT91C_ISI_FO_P_OVF);

    ISI_Reset();
    ISI_Enable();

    ov965x_configure(&twid, S_Video.codec_hsize, S_Video.codec_vsize);
   // ov95x_DumpRegisters(&twid); 
}

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// Application entry point. 
/// Configures the DBGU, TWI, ISI, LCD if present, display test to be launch
/// \return Unused (ANSI-C compatibility)
//-----------------------------------------------------------------------------
int main(void)
{

    q = quirc_new((uint8_t *)BuffTrans);

    PIO_Configure(pins, PIO_LISTSIZE(pins));
    PIO_InitializeInterrupts(0);
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- Quirc Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
    

    S_Video.codec_hsize = 320;
    S_Video.codec_vsize = 240;
    S_Video.lcd_hsize = S_Video.codec_hsize;
    S_Video.lcd_vsize = S_Video.codec_vsize;
    S_Video.lcd_nbpp = 16; // 16-bit format of the LCD controller.
    // Allocate the frame buffers for the preview datapath
    AllocateFBD((AT91PS_VIDEO)&S_Video);

    IsiDescriptors.CurrentLcdIndex = 0;
    IsiDescriptors.DisplayCodec = 0;
    IsiDescriptors.nb_codec_ovf = 0;
    IsiDescriptors.nb_prev_ovf = 0;

    // Configure TWI
    // In IRQ mode: to avoid problems, the priority of the TWI IRQ must be max.
    // In polling mode: try to disable all IRQs if possible.
    // (in this example it does not matter, there is only the TWI IRQ active)
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_ISI;

    // ISI_MCK is mapped in PCK1: 24Mhz Sensor clock
    AT91C_BASE_PMC->PMC_PCKR[1] = AT91C_PMC_CSS_PLLA_CLK | AT91C_PMC_PRES_CLK_32;
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_PCK1;
    while ((AT91C_BASE_PMC->PMC_SR & AT91C_PMC_PCK1RDY) == 0);

    TWI_ConfigureMaster(AT91C_BASE_TWI, TWCK, BOARD_MCK);
    TWID_Initialize(&twid, AT91C_BASE_TWI);
    AIC_ConfigureIT(AT91C_ID_TWI, 0, ISR_Twi);
    AIC_EnableIT(AT91C_ID_TWI);

    S_Video.Hblank = 0;
    S_Video.Vblank = 0;
    S_Video.lcd_hsize = S_Video.codec_hsize;
    S_Video.lcd_vsize = S_Video.codec_vsize;
    // base address of the descriptor list for preview DMA
    S_Video.Isi_fbd_base = (int)&FbList[0];
    S_Video.codec_fb_addr = 0;
    S_Video.rgb_or_yuv = YUV;
    ISI_Init(&S_Video);

    S_Video.IsiPrevBuffIndex = 0;

    AIC_ConfigureIT(AT91C_ID_ISI, AT91C_AIC_PRIOR_HIGHEST, ISR_IsiHandler);
    AIC_EnableIT(AT91C_ID_ISI);

    while( ov965x_init(&twid) == 0 ) {
        printf("-I- Retry init\n\r");
    }
    printf("-I- Init passed\n\r");

    printf("Source Size = [%d,%d]", S_Video.codec_hsize, S_Video.codec_vsize);
    if ( S_Video.rgb_or_yuv == RGB) {
        printf(" RGB\n\r");
    }
    else {
        printf(" YUV\n\r");
    }
    main_Test(&S_Video);
    while(1)
    {
     
    }
}

