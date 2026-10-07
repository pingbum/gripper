// Test the production ISR/API with simulated register state, not SPI timing.
#include <cassert>
#include <iostream>
#include "../../Application/Sensor/Src/MA732.cpp"

void cs_rise(uint32_t remaining, uint16_t word=0x8000) {
  gpio.IDR=GPIO_PIN_6;
  channel.CNDTR=remaining;
  rx_words[0]=word;
  spi.SR=0;
  dma.ISR=0;
  pending=GPIO_PIN_6;
  timer.CNT+=17000;
  EXTI9_5_IRQHandler();
}
int main() {
  MA732_t encoder;
  MA732_t::Sample sample{};
  // Boot in the middle of a transfer: discard until the first CS boundary.
  gpio.IDR=0;
  assert(encoder.start()==HAL_OK);
  assert(encoder.readSample(sample)==HAL_BUSY);
  cs_rise(1);
  assert(encoder.readSample(sample)==HAL_BUSY);
  cs_rise(1,0x1234);
  assert(encoder.readSample(sample)==HAL_OK && sample.word==0x1234);
  const auto sequence=sample.sequence;
  assert(channel.CNDTR==2 && (channel.CCR & DMA_CCR_EN));
  assert((spi.CR1 & SPI_CR1_SSI)==0);
  // Missing/partial word and extra complete word never replace the sample.
  cs_rise(2,0xFFFF);
  cs_rise(0,0xFFFF);
  assert(encoder.readSample(sample)==HAL_OK && sample.sequence==sequence);
  // DMA and SPI errors discard the frame.
  channel.CNDTR=1; dma.ISR=DMA_ISR_TEIF2; pending=GPIO_PIN_6;
  EXTI9_5_IRQHandler();
  channel.CNDTR=1; dma.ISR=0; spi.SR=SPI_SR_OVR; pending=GPIO_PIN_6;
  EXTI9_5_IRQHandler();
  assert(encoder.readSample(sample)==HAL_OK && sample.sequence==sequence);
  // Late ISR after the next CS assertion discards and does not arm mid-frame.
  gpio.IDR=0; channel.CNDTR=1; spi.SR=0; pending=GPIO_PIN_6;
  EXTI9_5_IRQHandler();
  assert(!armed && !(spi.CR1 & SPI_CR1_SPE));
  cs_rise(1);
  assert(encoder.readSample(sample)==HAL_OK && sample.sequence==sequence);
  cs_rise(1,0xFFFC);
  assert(encoder.readAngleRaw()==16383);
  assert(encoder.readSample(sample)==HAL_OK && sample.sequence==sequence+1);
  tick=4;
  MA732_SlaveWatchdog();
  assert(errors==0);
  tick=5;
  assert(encoder.readSample(sample)==HAL_TIMEOUT);
  MA732_SlaveWatchdog();
  assert(errors==ERROR_SPI && gpio.BSRR==DRVOFF_Pin);
  // Tick rollover and callers already running with interrupts masked.
  tick=0xFFFFFFFEU; cs_rise(1);
  tick=1; primask=1;
  assert(encoder.readSample(sample)==HAL_OK && primask==1);
  tick=3;
  assert(encoder.readSample(sample)==HAL_TIMEOUT);
  hspi3.Init.Mode=1;
  assert(encoder.start()==HAL_ERROR);
  std::cout << "MA732 slave receiver tests passed\n";
}
