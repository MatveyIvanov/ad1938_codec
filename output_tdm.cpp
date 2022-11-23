/* Audio Library for Teensy 3.X
 * Copyright (c) 2017, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "output_tdm.h"
#include "memcpy_audio.h"



#if defined(__IMXRT1062__) || 1

audio_block_t * AudioOutputTDM::block_input[MAX_CHANNELS] = {
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
bool AudioOutputTDM::update_responsibility = false;
static uint32_t zeros[AUDIO_BLOCK_SAMPLES/2];
DMAMEM static uint32_t tdm_tx_buffer[AUDIO_BLOCK_SAMPLES*MAX_CHANNELS];
DMAChannel AudioOutputTDM::dma(false);

void AudioOutputTDM::begin(void)
{
	Serial.print("\n AudioOutputTDM:begin ");
	dma.begin(true); // Allocate the DMA channel first

	for (int i=0; i < MAX_CHANNELS; i++) {
		block_input[i] = NULL;
	}
	// memset(zeros, 0, sizeof(zeros));
	// memset(tdm_tx_buffer, 0, sizeof(tdm_tx_buffer));
	// TODO: should we set & clear the I2S_TCSR_SR bit here?
	config_tdm();

	CORE_PIN7_CONFIG  = 3;  //1:TX_DATA0

	dma.TCD->SADDR = tdm_tx_buffer;
	dma.TCD->SOFF = 4;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = 4;
	dma.TCD->SLAST = -sizeof(tdm_tx_buffer);
	dma.TCD->DADDR = &I2S1_TDR0;
	dma.TCD->DOFF = 0;
	dma.TCD->CITER_ELINKNO = sizeof(tdm_tx_buffer) / 4;
	dma.TCD->DLASTSGA = 0;
	dma.TCD->BITER_ELINKNO = sizeof(tdm_tx_buffer) / 4;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_TX);

	update_responsibility = update_setup();
	dma.enable();

	I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE;
	I2S1_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE;

	dma.attachInterrupt(isr);
}

// TODO: needs optimization...
static void memcpy_tdm_tx(uint32_t *dest, const uint32_t *src1, const uint32_t *src2)
{
	uint32_t i, in1, in2, out1, out2;
	
	//Serial.print(*src2,HEX);
	for (i=0; i < AUDIO_BLOCK_SAMPLES/2; i++) { // MAY BE /4
		in1 = *src1++;
		in2 = *src2++;
	    //Serial.print("\n");
	    //Serial.print(in1,HEX);
		out1 = (in1 << 16) | (in2 & 0xFFFF);
		out2 = (in1 & 0xFFFF0000) | (in2 >> 16);
		//out1=((in1 & 0xFFFF)<<16);
		//out2=((in1 & 0xFFFF0000));

		// !!!ПРОВЕРИТЬ!!!
		*dest = out1;
		//Serial.print("\n");
		//Serial.print(out1,HEX);
		*(dest + (MAX_CHANNELS>>1)) = out2;
		dest += MAX_CHANNELS;
	}
	
	//Serial.print(*src,HEX);
}

void AudioOutputTDM::isr(void)
{
	uint32_t *dest;
	const uint32_t *src1, *src2;
	uint32_t i, saddr;
    //Serial.print("\n AudioOutputTDM:isr ");
	///digitalWriteFast(35, HIGH);
	saddr = (uint32_t)(dma.TCD->SADDR);
	dma.clearInterrupt();
	if (saddr < (uint32_t)tdm_tx_buffer + sizeof(tdm_tx_buffer) / 2) {
		// DMA is transmitting the first half of the buffer
		// so we must fill the second half
		dest = tdm_tx_buffer + AUDIO_BLOCK_SAMPLES*(MAX_CHANNELS>>1);
	} else {
		// DMA is transmitting the second half of the buffer
		// so we must fill the first half
		dest = tdm_tx_buffer;
	}
	if (update_responsibility) AudioStream::update_all();

	// #if IMXRT_CACHE_ENABLED >= 2
	// uint32_t *dc = dest;
	// #endif

	for (i=0; i < MAX_CHANNELS; i += 2) {
		src1 = block_input[i] ? (uint32_t *)(block_input[i]->data) : zeros;
		src2 = block_input[i+1] ? (uint32_t *)(block_input[i+1]->data) : zeros;
		memcpy_tdm_tx(dest, src1, src2);
		dest++;
	}

	// #if IMXRT_CACHE_ENABLED >= 2
	// arm_dcache_flush_delete(dc, sizeof(tdm_tx_buffer) / 2 );
	// #endif

	for (i=0; i < MAX_CHANNELS; i++) {
		if (block_input[i]) {
			release(block_input[i]);
			block_input[i] = NULL;
		}
	}
	//digitalWriteFast(35, LOW);
}


void AudioOutputTDM::update(void)
{
	audio_block_t *prev[MAX_CHANNELS];
	unsigned int i;

	__disable_irq();
	for (i=0; i < MAX_CHANNELS; i++) {
		prev[i] = block_input[i];
		block_input[i] = receiveReadOnly(i);
	}
	__enable_irq();
	for (i=0; i < MAX_CHANNELS; i++) {
		if (prev[i]) release(prev[i]);
	}
}

void AudioOutputTDM::config_tdm(void)
{
	CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

	// if either transmitter or receiver is enabled, do nothing
	if (I2S1_TCSR & I2S_TCSR_TE) return;
	if (I2S1_RCSR & I2S_RCSR_RE) return;
//PLL:
	int fs = AUDIO_SAMPLE_RATE_EXACT;
	// PLL between 27*24 = 648MHz und 54*24=1296MHz
	int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
	int n2 = 1 + (24000000 * 27) / (fs * 256 * n1);

	double C = ((double)fs * 256 * n1 * n2) / 24000000;
	int c0 = C;
	int c2 = 10000;
	int c1 = C * c2 - (c0 * c2);
	set_audioClock(c0, c1, c2);
	// clear SAI1_CLK register locations
	CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
		   | CCM_CSCMR1_SAI1_CLK_SEL(2); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4

	n1 = n1 / 2; //Double Speed for TDM

	CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
		   | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
		   | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f

	IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
			| (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));	//Select MCLK

	// configure transmitter
	int rsync = 0;
	int tsync = 1;

	I2S1_TMR = 0;
	I2S1_TCR1 = I2S_TCR1_RFW(4);
	I2S1_TCR2 = I2S_TCR2_SYNC(tsync) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1)
		| I2S_TCR2_BCD | I2S_TCR2_DIV(0);
	I2S1_TCR3 = I2S_TCR3_TCE;
	I2S1_TCR4 = I2S_TCR4_FRSZ(7) | I2S_TCR4_SYWD(0) | I2S_TCR4_MF
		| I2S_TCR4_FSE | I2S_TCR4_FSD;
	I2S1_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

	I2S1_RMR = 0;
	I2S1_RCR1 = I2S_RCR1_RFW(4);
	I2S1_RCR2 = I2S_RCR2_SYNC(rsync) | I2S_TCR2_BCP | I2S_RCR2_MSEL(1)
		| I2S_RCR2_BCD | I2S_RCR2_DIV(0);
	I2S1_RCR3 = I2S_RCR3_RCE;
	I2S1_RCR4 = I2S_RCR4_FRSZ(7) | I2S_RCR4_SYWD(0) | I2S_RCR4_MF
		| I2S_RCR4_FSE | I2S_RCR4_FSD;
	I2S1_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

	CORE_PIN23_CONFIG = 3;  //1:MCLK
	CORE_PIN21_CONFIG = 3;  //1:RX_BCLK
	CORE_PIN20_CONFIG = 3;  //1:RX_SYNC
}

// !!!TODO!!!
void AudioOutputTDMslave::config_tdm(void)
{
	Serial.print("\n AudioOutputTDMslave:config_tdm ");
	CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

	// if either transmitter or receiver is enabled, do nothing
	if (I2S1_TCSR & I2S_TCSR_TE) return;
	if (I2S1_RCSR & I2S_RCSR_RE) return;

	// Select input clock 0
	// Configure to input the bit-clock from pin, bypasses the MCLK divider
	
	//I2S0_MCR = I2S_MCR_MICS(0);
	//I2S0_MDR = 0;

	//PLL:
	int fs = AUDIO_SAMPLE_RATE_EXACT;
	// PLL between 27*24 = 648MHz und 54*24=1296MHz
	int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
	int n2 = 1 + (24000000 * 27) / (fs * 256 * n1);

	double C = ((double)fs * 256 * n1 * n2) / 24000000;
	int c0 = C;
	int c2 = 10000;
	int c1 = C * c2 - (c0 * c2);
	set_audioClock(c0, c1, c2); // May be here do the trick
	// clear SAI1_CLK register locations
	CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
		   | CCM_CSCMR1_SAI1_CLK_SEL(2); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4

	n1 = n1 / 2; //Double Speed for TDM

	CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
		   | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
		   | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f

	IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
			| (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));	//Select MCLK

	// TEENSY 4.0

	// Transmitter
	int rsync = 0;
	int tsync = 1;

	I2S1_TMR = 0;
	I2S1_TCR1 = I2S_TCR1_RFW(8); // 4 in Audio lib
	I2S1_TCR2 = I2S_TCR2_SYNC(tsync) | I2S_TCR2_BCP; // | I2S_TCR2_MSEL(1) | I2S_TCR2_BCD | I2S_TCR2_DIV(0);
	I2S1_TCR3 = I2S_TCR3_TCE;
	I2S1_TCR4 = I2S_TCR4_FRSZ(15) /*7*/ | I2S_TCR4_SYWD(0) | I2S_TCR4_MF
		| I2S_TCR4_FSE | I2S_TCR4_FSP; // I2S_TCR4_FSD;
	I2S1_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

	// Reciever
	I2S1_RMR = 0;
	I2S1_RCR1 = I2S_RCR1_RFW(8); // 4
	I2S1_RCR2 = I2S_RCR2_SYNC(rsync) | I2S_TCR2_BCP; // | I2S_RCR2_MSEL(1) | I2S_RCR2_BCD | I2S_RCR2_DIV(0);
	I2S1_RCR3 = I2S_RCR3_RCE;
	I2S1_RCR4 = I2S_RCR4_FRSZ(15) /*7*/ | I2S_RCR4_SYWD(0) | I2S_RCR4_MF
		| I2S_RCR4_FSE | I2S_RCR4_FSP| I2S_RCR4_FSD; // No I2S_RCR4_FSP
	I2S1_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

	CORE_PIN23_CONFIG = 3;  //1:MCLK
	CORE_PIN21_CONFIG = 3;  //1:RX_BCLK
	CORE_PIN20_CONFIG = 3;  //1:RX_SYNC

	// TEENSY 4.0

	/* TEENSY 3.6
	// configure transmitter
	I2S0_TMR = 0;
	I2S0_TCR1 = I2S_TCR1_TFW(8);
	I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP;
	I2S0_TCR3 = I2S_TCR3_TCE;
	I2S0_TCR4 = I2S_TCR4_FRSZ(15) | I2S_TCR4_SYWD(0) | I2S_TCR4_MF
		| I2S_TCR4_FSE | I2S_TCR4_FSP;//|I2S_TCR4_FSD;//FSD
	I2S0_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

	// configure receiver (sync'd to transmitter clocks)
	I2S0_RMR = 0;
	I2S0_RCR1 = I2S_RCR1_RFW(8);
	I2S0_RCR2 = I2S_RCR2_SYNC(1) | I2S_TCR2_BCP ;
	I2S0_RCR3 = I2S_RCR3_RCE;
	I2S0_RCR4 = I2S_RCR4_FRSZ(15) | I2S_RCR4_SYWD(0) | I2S_RCR4_MF
		| I2S_RCR4_FSE | I2S_RCR4_FSP| I2S_RCR4_FSD;
	I2S0_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

	// configure pin mux for 3 clock signals
	CORE_PIN23_CONFIG = PORT_PCR_MUX(6); // pin 23, PTC2, I2S0_TX_FS (LRCLK)
	CORE_PIN9_CONFIG  = PORT_PCR_MUX(6); // pin  9, PTC3, I2S0_TX_BCLK
	CORE_PIN11_CONFIG = PORT_PCR_MUX(6); // pin 11, PTC6, I2S0_MCLK
	*/
}

void AudioOutputTDMslave::begin(void)
{
	Serial.print("\n AudioOutputTDMslave:begin ");
	dma.begin(true); // Allocate the DMA channel first

	for (int i=0; i < MAX_CHANNELS; i++) {
		block_input[i] = NULL;
	}
	// memset(zeros, 0, sizeof(zeros));
	// memset(tdm_tx_buffer, 0, sizeof(tdm_tx_buffer));

	// TODO: should we set & clear the I2S_TCSR_SR bit here?
	AudioOutputTDMslave::config_tdm();

	CORE_PIN7_CONFIG  = 3;  //1:TX_DATA0

	dma.TCD->SADDR = tdm_tx_buffer;
	dma.TCD->SOFF = 4;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = 4;
	dma.TCD->SLAST = -sizeof(tdm_tx_buffer);
	dma.TCD->DADDR = &I2S1_TDR0;
	dma.TCD->DOFF = 0;
	dma.TCD->CITER_ELINKNO = sizeof(tdm_tx_buffer) / 4;
	dma.TCD->DLASTSGA = 0;
	dma.TCD->BITER_ELINKNO = sizeof(tdm_tx_buffer) / 4;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_TX);

	update_responsibility = update_setup();
	dma.enable();

	I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE;
	I2S1_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE;

	dma.attachInterrupt(isr);
}

#endif
