#include "stm32h7xx_hal.h"
#include "peripheral/register.hpp"
#include <array>

namespace device {

    /// @brief  Digital Filter.
    /// @tparam  frame_length However many samples are in a downsampled frame.
    /// @tparam  oversampling The oversampling factor.
    /// @note   It is also the factor by which the filtered data is downsampled.
    //template<uint16_t frame_length, uint8_t oversampling>
    class DigitalFilter {
        /**
        The filter deals with four buffers, two as inputs, and two as outputs. Four are needed so 
        that the CPU can overwrite an input and access an output whilst the filter is still running.
        Each set is made up of a raw input buffer, raw_n, and of a filtered output buffer, 
        fitlered_n. At a time, one set is being processed by the FMAC whilst the other is available 
        to the CPU. After a frame is filtered, the sets swap around such that the other is being 
        processed by the FMAC. This is all done with DMA such that little CPU involvement is needed.

                reading raw_0                  overwriting filtered_0
                +-----+                        +-----+
                |     |                        |-----|
                |  |  |        +------+        |-----|
                |--v--|------->| FMAC |------->|--|--|
                |-----|        +------+        |  v  |
                |-----|                        |     |
                +-----+                        +-----+

                overwriting raw_1              reading filtered_1
                +-----+                        +-----+
                |-----|                        |-----|
                |-----|                        |-----|
        CPU---->|--|--|                        |-----|----->CPU
                |  v  |                        |-----|
                |     |                        |-----|
                +-----+                        +-----+
         */

        public:

            /// @brief  Constructs the object.
            DigitalFilter() {
                // ~~~ Filter Maths Accelerator (FMAC) ~~~

                // Turn the FMAC's clock on.
                SET_BIT(RCC->AHB2ENR, 16);

                // X2 holds the coefficients, i.e. h. Let the length be N = 64, which is the number 
                // taps.
                WRITE_FIELD(FMAC->X2BUFCFG, 0, 0xFF, 0x00); // Set the base at 0.
                WRITE_FIELD(FMAC->X2BUFCFG, 8, 0xFF, 0x40); // Set the length to be 64.

                // X1 is the input to the filter, i.e. x. It is packed straight after X2. The 
                // length is N + d = 64 + 4 where N is the number of taps and d is some headroom to 
                // ease the throughput.
                WRITE_FIELD(FMAC->X1BUFCFG, 0, 0xFF, 0x40);         // Set the base at 64.
                WRITE_FIELD(FMAC->X1BUFCFG, 8, 0xFF, 0x40+0x04);    // Set the length to be 64 + 4.

                // Y is the output from the filter, i.e. y. It is packed straight after X1. The 
                // length is d = 4, which is some headroom to ease the throughput.
                WRITE_FIELD(FMAC->YBUFCFG, 0, 0xFF, 0x80+0x04); // Set the base at 64+64+8.
                WRITE_FIELD(FMAC->YBUFCFG, 8, 0xFF, 0x04);      // Set the length is 4.

                // Load the 68 noughts into X1 as the initial condition.
                WRITE_FIELD(FMAC->PARAM, 24, 0x7F, 0x01);       // Set the function to load X1.
                WRITE_FIELD(FMAC->PARAM, 0, 0xFF, 0x40+0x04);   // Set P to be 68.
                SET_BIT(FMAC->PARAM, 31);                       // Start loading.

                for (int i = 0; i < 64+4; i++) {
                    *reinterpret_cast<volatile int16_t*>(&FMAC->WDATA) = 0x0000;
                }

                CLEAR_BIT(FMAC->PARAM, 31); // Stop loading.

                // Load the 64 coefficients into the X2 buffer.
                WRITE_FIELD(FMAC->PARAM, 24, 0x7F, 0x02);    // Set the function to load X2.
                WRITE_FIELD(FMAC->PARAM, 0, 0xFF, 0x40);     // Set P to be 64.
                WRITE_FIELD(FMAC->PARAM, 8, 0xFF, 0x00);     // Set Q to be 0.
                SET_BIT(FMAC->PARAM, 31);                    // Start loading.

                for (int i = 0; i < 64; i++) {
                    *reinterpret_cast<volatile int16_t*>(&FMAC->WDATA) = coefficients[i];
                }

                CLEAR_BIT(FMAC->PARAM, 31); // Stop loading.

                // DMA is used to both write and read. The state of both X1 and Y control the flow 
                // of the DMA.
                SET_BIT(FMAC->CR, 8); // Turn the DMA request for RDATA on.
                SET_BIT(FMAC->CR, 9); // Turn the DMA request for WDATA on.

                // Begin the filter. It will not actually begin to filter until the DMA begins.
                WRITE_FIELD(FMAC->PARAM, 24, 0x7F, 0x08);   // Set the function to FIR.
                WRITE_FIELD(FMAC->PARAM, 0, 0xFF, 0x40);    // Set P to be 64, i.e. the number of taps.
                WRITE_FIELD(FMAC->PARAM, 8, 0xFF, 0x00);    // Set Q to be 0.
                WRITE_FIELD(FMAC->PARAM, 16, 0xFF, 0x00);   // Set R to be 0.
                SET_BIT(FMAC->PARAM, 31);                   // Start filtering.

                // ~~~ WDATA DMA ~~~

                // Set a DMA up to carry unfiltered data to the FMAC. It will also control the 
                // overall flow.

                // The stream must be disabled before anything else.
                CLEAR_BIT(DMA1_Stream2->CR, 0); // Disable the stream.
                // Wait for the stream to shut off as recommended on page-631 in the RM.
                while (DMA1_Stream2->CR & (0b1 << 0));

                // Set the destination and the source respectively.
                DMA1_Stream2->PAR = reinterpret_cast<uint32_t>(&(FMAC->WDATA));
                DMA1_Stream2->M0AR = reinterpret_cast<uint32_t>(&raw_0);

                DMA1_Stream2->NDTR = 0x2000; // Send 8192 frames.

                // Set the DMA MUX Channel-1 for FMAC_WR as given in Table 118 in the RM.
                WRITE_FIELD(DMAMUX1_Channel2->CCR, 0, 0x7F, 0x79);

                // Set the rest up.
                WRITE_FIELD(DMA1_Stream2->CR, 13, 0b11, 0b01);  // Take a half-word in.
                WRITE_FIELD(DMA1_Stream2->CR, 11, 0b11, 0b01);  // Put a half-word out.
                CLEAR_BIT(DMA1_Stream2->CR, 19);                // Use the first buffer.
                CLEAR_BIT(DMA1_Stream2->CR, 18);                // Use only one buffer.
                SET_BIT(DMA1_Stream2->CR, 10);                  // Increment the memory.
                CLEAR_BIT(DMA1_Stream2->CR, 9);                 // Do not increment the peripheral.
                CLEAR_BIT(DMA1_Stream2->CR, 8);                 // Do not repeat the transfer.
                WRITE_FIELD(DMA1_Stream2->CR, 6, 0b11, 0b01);   // Carry data to the peripheral. 
                SET_BIT(DMA1_Stream2->CR, 4);                   // Enable the TC interrupt.

                // ~~~ RDATA DMA ~~~

                // Set a DMA up to carry filtered data from the FMAC.
                
                // Initialise the buffer.  
                std::fill(filtered_0.begin(), filtered_0.end(), 0x0000);
                std::fill(filtered_1.begin(), filtered_1.end(), 0x0000);

                // The stream must be disabled before anything else.
                CLEAR_BIT(DMA1_Stream3->CR, 0); // Disable the stream.
                // Wait for the stream to shut off as recommended on page-631 in the RM.
                while (DMA1_Stream3->CR & (0b1 << 0));

                // Set the source and the destination respectively.
                DMA1_Stream3->PAR = reinterpret_cast<uint32_t>(&(FMAC->RDATA));
                DMA1_Stream3->M0AR = reinterpret_cast<uint32_t>(&filtered_0);

                // Set the DMA MUX Channel-1 for FMAC_RD as given in Table 118 in the RM.
                WRITE_FIELD(DMAMUX1_Channel3->CCR, 0, 0x7F, 0x78);

                // Let the FMAC control the flow such that NDTR is not needed. Instead, 
                // this DMA will react to whenever a new output data is available.
                SET_BIT(DMA1_Stream3->CR, 5); // Let the peripheral control the flow.

                // Set the rest up.
                WRITE_FIELD(DMA1_Stream3->CR, 13, 0b11, 0b01);  // Put a half-word out.
                WRITE_FIELD(DMA1_Stream3->CR, 11, 0b11, 0b01);  // Take a half-word in.
                CLEAR_BIT(DMA1_Stream3->CR, 19);                // Use the first buffer.
                CLEAR_BIT(DMA1_Stream3->CR, 18);                // Use only one buffer.
                SET_BIT(DMA1_Stream3->CR, 10);                  // Increment the memory.
                CLEAR_BIT(DMA1_Stream3->CR, 9);                 // Do not increment the peripheral.
                SET_BIT(DMA1_Stream3->CR, 8);                   // Repeat the transfer.
                WRITE_FIELD(DMA1_Stream3->CR, 6, 0b11, 0b00);   // Carry data from the peripheral.
                SET_BIT(DMA1_Stream3->CR, 4);                   // Enable the TC interrupt.
            }

            /// @brief  Begins to filter the next frame. Turns both streams on so that the filter 
            ///         begins.
            void begin_frame(void) {
                DMA1_Stream2->NDTR = oversampling*frame_length; // Send 8192 frames.
                SET_BIT(DMA1_Stream2->CR, 0);                   // Enable the stream.
                SET_BIT(DMA1_Stream3->CR, 0);                   // Enable the stream.
            }

            /// @brief  Sees whether the filter is done. That is if both the writing DMA is done, 
            ///         and there is no new output data from the FMAC.
            /// @return Whether the frame is ready.
            bool check_frame(void) {
                // Unless the WDATA DMA is done, return false.
                if (!(DMA1->LISR & (0b1 << 21))) {
                    return false;
                }

                // Unless there is new output data, return early. This makes sure that the filter 
                // is fully done, i.e. it is not still processing the last byte. This needs not be 
                // cleared since it is a state, not a latch.
                if (!(FMAC->SR & (0b1 << 0))) {
                    return false;
                }

                return true;
            }

            /// @brief  It stops the DMA and swaps the buffers for the next frame to begin.
            /// @return Whether the frame is ready.
            void ready_next_frame(void) {
                // All flags must be cleared lest the DMA would not begin again.
                WRITE_FIELD(DMA1->LIFCR, 18, 0xF, 0xF);
                SET_BIT(DMA1->LIFCR, 16);

                // Turn all the streams off so that the buffer's can be swapped.
                CLEAR_BIT(DMA1_Stream2->CR, 0);
                while (DMA1_Stream2->CR & (0b1 << 0));
                CLEAR_BIT(DMA1_Stream3->CR, 0);
                while (DMA1_Stream3->CR & (0b1 << 0));

                // After Stream-3 is turned off, the TCIF of each is set by hardware as said on 
                // page-629 in the RM. Therefore, it must be cleared. Otherwise, the stream would 
                // not begin again.
                SET_BIT(DMA1->LIFCR, 27); // Clear the TCIF3.

                is_filtering_first = !is_filtering_first;

                // Swap to the other two buffers.
                if (is_filtering_first) {
                    DMA1_Stream2->M0AR = reinterpret_cast<uint32_t>(&raw_0);
                    DMA1_Stream3->M0AR = reinterpret_cast<uint32_t>(&filtered_0);
                } else {
                    DMA1_Stream2->M0AR = reinterpret_cast<uint32_t>(&raw_1);
                    DMA1_Stream3->M0AR = reinterpret_cast<uint32_t>(&filtered_1);
                }
            }

            /// @brief  Gets the raw buffer.
            /// @return Whichever buffer the filter is not accessing up.
            std::array<volatile int16_t, oversampling*frame_length>& get_raw_buffer(void) {
                // If the filter is filling the first up, then return the second since it is 
                // untouched. Otherwise, return the first.
                if (is_filtering_first) {
                    return raw_1;
                } else {
                    return raw_0;
                }
            }

            /// @brief  Gets the full buffer.
            /// @return Whichever buffer the filter is not filling up.
            const std::array<volatile int16_t, oversampling*frame_length>& get_filtered_buffer(
                void
            ) const {
                // If the filter is filling the first up, then return the second since it is 
                // untouched. Otherwise, return the first.
                if (is_filtering_first) {
                    return filtered_1;
                } else {
                    return filtered_0;
                }
            }

        private:
            /// @brief  The first raw buffer.
            std::array<volatile int16_t, oversampling*frame_length> raw_0 = {};
            /// @brief  The second raw buffer.
            std::array<volatile int16_t, oversampling*frame_length> raw_1 = {};
            /// @brief  The first filtered buffer.
            std::array<volatile int16_t, oversampling*frame_length> filtered_0 = {};
            /// @brief  The second filtered buffer.
            std::array<volatile int16_t, oversampling*frame_length> filtered_1 = {};
            /// @brief  Whether the filter is filtering the first set of buffers.
            bool is_filtering_first = true;
    };

} // namespace device