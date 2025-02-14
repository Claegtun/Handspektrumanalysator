#include "stm32h7xx_hal.h"
#include "peripheral/register.hpp"
#include <array>

namespace device {

    /// @brief  However many samples are in a downsampled frame.
    static constexpr uint16_t frame_length = 1024*2;
    /// @brief  The oversampling factor.
    /// @note   It is also the factor by which the filtered data is downsampled.
    static constexpr uint8_t oversampling = 8;

    /// @brief  Außern-analog-digital-umsetzer.
    class AADU {
        public:

            /// @brief  Constructs the object.
            AADU() {
                // ~~~ Timer ~~~

                // This timer both outputs the CNVST signal to tell the AADU to do a conversion, 
                // and triggers the DMA to carry bytes to the SPI.

                // Turn the clock on for TIM8.
                SET_BIT(RCC->APB2ENR, 1);

                // Set the timer up so that a pulse is output, and DMA is requested on the falling 
                // edge.
                CLEAR_BIT(TIM8->CR1, 4);                // Count upwards.
                CLEAR_BIT(TIM8->CR1, 3);                // Keep counting.
                SET_BIT(TIM8->DIER, 9);                 // Turn the DMA request from CC1 on.
                CLEAR_FIELD(TIM8->CCMR1, 0, 0b11);      // Set CH1 to an output.
                WRITE_FIELD(TIM8->CCMR1, 4, 0xF, 0x6);  // Set to PWM Mode-1.
                CLEAR_BIT(TIM8->CCER, 1);               // Set the polarity to active high.
                SET_BIT(TIM8->CCER, 0);                 // Enable the capture.
                SET_BIT(TIM8->BDTR, 15);                // Turn the main output on.

                // The frequency must be around 384 kHz, which is eightfold the digital sampling 
                // frequency 48 kHz. Given that the bus clock is 64 MHz on reset, 388 kHz is chosen.
                // In Table 1-2 in the datasheet, the conversion takes at least 750 ns. Therefore, 
                // a duty-cycle of 40 % is chosen such that the on-period is at least 1 μs. 
                TIM8->PSC = 32; // Set the prescaler.
                TIM8->ARR = 4;   // Set the period.
                TIM8->CCR1 = 2;  // Set the duty-cycle.

                // Set the pins up for the timer's PWM output.
                WRITE_FIELD(GPIOC->MODER, 6*2, 0b11, 0b10); // Set PC6 as an alternate function.
                WRITE_FIELD(GPIOC->AFR[0], 6*4, 0xF, 0x3);  // Use Alternate Function 5.

                // ~~~ SPI ~~~

                // Given that the AADU outputs serial data, a SPI is used. 

                // Set the pins up for the SPI.
                // Only a MISO and an SCLK are needed.
                WRITE_FIELD(GPIOB->MODER, 13*2, 0b11, 0b10);    // Set PB13 as the SCLK.
                WRITE_FIELD(GPIOB->MODER, 14*2, 0b11, 0b10);    // Set PB14 as the MISO.
                WRITE_FIELD(GPIOB->AFR[1], 5*4, 0xF, 0x5);      // Use Alternate Function 5.
                WRITE_FIELD(GPIOB->AFR[1], 6*4, 0xF, 0x5);      // Use Alternate Function 5.
                WRITE_FIELD(GPIOB->OSPEEDR, 26, 0b11, 0b11);    // Set PB13 to very fast.
                WRITE_FIELD(GPIOB->OSPEEDR, 28, 0b11, 0b11);    // Set PB14 to very fast.

                // Turn the clock on for SPI2.
                SET_BIT(RCC->APB1LENR, 14);

                // Given that about 1.5 μs is available after conversion and that the frame has 
                // sixteen bits, the clock-frequency must be at least 10.3 MHz. Given that the 
                // bus clock is 64 MHz, the clock-frequency is chosen as 16 MHz. 
                WRITE_FIELD(SPI2->CFG1, 28, 0b111, 0b000);  // Prescale the master clock by 1/4.

                // Turn the DMA request on for RX. Set the rest up.
                SET_BIT(SPI2->CFG1, 14);                // Turn the RX DMA on.
                WRITE_FIELD(SPI2->CFG1, 0, 0x1F, 0xF);  // Make the frame a half-word.
                WRITE_FIELD(SPI2->CFG2, 24, 0b11, 0b10); // Take data on the falling edge of SCLK.

                // Effectively, ignore the chip-select since this chip doesn't have one, and the 
                // SPI is always running. Importantly, the SSI must be set before choosing master 
                // lest a mode-fail is raised.
                SET_BIT(SPI2->CFG2, 26);    // Use software-driven SS.
                SET_BIT(SPI2->CR1, 12);     // Set the SSI.
                SET_BIT(SPI2->CFG2, 22);    // Set as a master.

                // ~~~ TX DMA ~~~

                // Set a DMA up to carry dummy half-words to the SPI to transmit. These are not 
                // actually transmitted since there is no MOSI, but are needed so that the SCLK is 
                // output.

                // The stream must be disabled before anything else.
                CLEAR_BIT(DMA1_Stream1->CR, 0); // Disable the stream.
                // Wait for the stream to shut off as recommended on page-631 in the RM.
                while (DMA1_Stream1->CR & (0b1 << 0));

                // Set the destination and the source respectively.
                DMA1_Stream1->PAR = reinterpret_cast<uint32_t>(&(SPI2->TXDR));
                DMA1_Stream1->M0AR = reinterpret_cast<uint32_t>(&tx_buffer);

                // Set the DMA MUX Channel-1 for TIM8_CH1 as given in Table 118 in the RM.
                WRITE_FIELD(DMAMUX1_Channel1->CCR, 0, 0x7F, 0x2F);

                // Let the SPI control the flow such that NDTR is not needed. Instead, 
                // TSIZE of the SPI controls how much is carried.
                SET_BIT(DMA1_Stream1->CR, 5); // Let the peripheral control the flow.

                // Set the rest up.
                WRITE_FIELD(DMA1_Stream1->CR, 13, 0b11, 0b01);  // Take a half-word in.
                WRITE_FIELD(DMA1_Stream1->CR, 11, 0b11, 0b01);  // Put a half-word out.
                CLEAR_BIT(DMA1_Stream1->CR, 10);                // Do not increment the memory.
                CLEAR_BIT(DMA1_Stream1->CR, 9);                 // Do not increment the peripheral.
                CLEAR_BIT(DMA1_Stream1->CR, 8);                 // Do not repeat the transfer.
                WRITE_FIELD(DMA1_Stream1->CR, 6, 0b11, 0b01);   // Carry data to the peripheral. 
                SET_BIT(DMA1_Stream1->CR, 4);                   // Enable the TC interrupt.

                // ~~~ RX DMA ~~~

                // Set a DMA up to carry data from the AADU.

                // Initialise the buffers to half-scale so that the filter has consistency.  
                std::fill(rx_buffer_0.begin(), rx_buffer_0.end(), oversampling*frame_length);
                std::fill(rx_buffer_1.begin(), rx_buffer_1.end(), oversampling*frame_length);

                // The stream must be disabled before anything else.
                CLEAR_BIT(DMA1_Stream0->CR, 0); // Disable the stream.
                // Wait for the stream to shut off as recommended on page-631 in the RM.
                while (DMA1_Stream0->CR & (0b1 << 0));

                // Set the source and the destination respectively.
                DMA1_Stream0->PAR = reinterpret_cast<uint32_t>(&(SPI2->RXDR));
                DMA1_Stream0->M0AR = reinterpret_cast<uint32_t>(&rx_buffer_0);

                // Set the DMA MUX Channel-1 for spi2_rx_dma as given in Table 118 in the RM.
                WRITE_FIELD(DMAMUX1_Channel0->CCR, 0, 0x7F, 0x27);

                // Let the SPI control the flow such that NDTR is not needed. Instead, 
                // TSIZE of the SPI controls how much is carried.
                SET_BIT(DMA1_Stream0->CR, 5); // Let the peripheral control the flow.
                
                // Set the rest up.
                WRITE_FIELD(DMA1_Stream0->CR, 13, 0b11, 0b01);  // Put a half-word out.
                WRITE_FIELD(DMA1_Stream0->CR, 11, 0b11, 0b01);  // Take a half-word in.
                CLEAR_BIT(DMA1_Stream0->CR, 19);                // Use the first buffer.
                CLEAR_BIT(DMA1_Stream0->CR, 18);                // Use only one buffer.
                SET_BIT(DMA1_Stream0->CR, 10);                  // Increment the memory.
                CLEAR_BIT(DMA1_Stream0->CR, 9);                 // Do not increment the peripheral.
                CLEAR_BIT(DMA1_Stream0->CR, 8);                 // Do not repeat the transfer.
                WRITE_FIELD(DMA1_Stream0->CR, 6, 0b11, 0b00);   // Carry data from the peripheral. 
                SET_BIT(DMA1_Stream0->CR, 4);                   // Enable the TC interrupt.
            }

            /// @brief  Begins to get the next frame. Sets the TSIZE, turns the DMA streams on, and 
            ///         starts the timer so that dummy half-words are sent.
            void begin_frame(void) {
                // Turn the DMA streams on s
                SET_BIT(DMA1_Stream0->CR, 0); // Enable the stream.
                SET_BIT(DMA1_Stream1->CR, 0); // Enable the stream.

                // Get the SPI ready to send however many frames.
                SPI2->CR2 = uint16_t(oversampling*frame_length);    // Send 8192 frames.
                SET_BIT(SPI2->CR1, 0);                              // Enable the peripheral.
                SET_BIT(SPI2->CR1, 9);                              // Begin the transmission.

                // Frames will not be sent until the timer begins which requests the DMA.
                SET_BIT(TIM8->CR1, 0); // Begin counting.
            }

            /// @brief  Sees whether the current frame is ready. That is if the SPI is done.
            /// @return Whether the frame is ready.
            inline bool check_frame(void) {
                return (SPI2->SR & (0b1 << 3));
            }

            /// @brief  It stops the SPI, the DMA, and swaps the buffers for the next frame to 
            ///         begin.
            /// @return Whether the frame is ready.
            void ready_next_frame(void) {
                // All flags must be cleared lest the SPI would not begin again.
                WRITE_FIELD(SPI2->IFCR, 3, 0x1FF, 0x1FF); // Clear all SPI2 flags.

                // Stop the timer.
                CLEAR_BIT(TIM8->CR1, 0);

                // Turn the SPI off so that TSIZE can be set later.
                CLEAR_BIT(SPI2->CR1, 0);

                // Turn all the streams off so that the buffer's can be swapped.
                CLEAR_BIT(DMA1_Stream0->CR, 0);
                while (DMA1_Stream0->CR & (0b1 << 0));
                CLEAR_BIT(DMA1_Stream1->CR, 0);
                while (DMA1_Stream1->CR & (0b1 << 0));

                // After both streams are turned off, the TCIF of each is set by hardware as said 
                // on page-629 in the RM. Therefore, it must be cleared. Otherwise, the stream 
                // would not begin again.
                SET_BIT(DMA1->LIFCR, 5);    // Clear the TCIF0.
                SET_BIT(DMA1->LIFCR, 11);   // Clear the TCIF1.

                is_filling_first = !is_filling_first;

                // Swap the filling buffer over to the other one.
                if (is_filling_first) {
                    DMA1_Stream0->M0AR = reinterpret_cast<uint32_t>(&rx_buffer_0);
                } else {
                    DMA1_Stream0->M0AR = reinterpret_cast<uint32_t>(&rx_buffer_1);
                }
            }

            /// @brief  Gets the full buffer.
            /// @return Whichever buffer the SPI  is not filling up.
            const std::array<volatile uint16_t, oversampling*frame_length>& get_full_buffer(void) const {
                // If the SPI is filling the first up, then return the second since it is untouched.
                // Otherwise, return the first.
                if (is_filling_first) {
                    return rx_buffer_1;
                } else {
                    return rx_buffer_0;
                }
            }

        private:
            /// @brief  The dummy data for the SPI to transmit.
            volatile uint16_t tx_buffer = 0xAAAA;
            /// @brief  The first RX buffer.
            std::array<volatile uint16_t, oversampling*frame_length> rx_buffer_0 = {};
            /// @brief  The second RX buffer.
            std::array<volatile uint16_t, oversampling*frame_length> rx_buffer_1 = {};
            /// @brief  Whether the SPI is filtering the first buffer.
            bool is_filling_first = true;
    };

} // namespace device