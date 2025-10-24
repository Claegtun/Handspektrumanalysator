#include "stm32h7xx_hal.h"
#include "peripheral/register.hpp"
#include <array>
#include <cstdint>

namespace device {

    /// @brief  Drehkopf / Rotary Encoder
    /// @tparam K whether the encoder is the top one (0) or the bottom (1).
    template<uint8_t K>
    class Encoder {
        public:

            /// @brief  Constructs the object.
            Encoder() {
                // ~~~ Timer ~~~

                // This timer decodes the quadrature signal from the rotary encoder. It counts 
                // either up or down depending on the quadrature phase.

                if constexpr (K == 0) {
                    // Choose TIM3 since it has pins bound to the encoder.
                    tim = TIM3;

                    // Turn the clock on for TIM3.
                    SET_BIT(RCC->APB1LENR, 1);
                } else {
                    // Choose TIM4 since it has pins bound to the encoder.
                    tim = TIM4;

                    // Turn the clock on for TIM4.
                    SET_BIT(RCC->APB1LENR, 2);
                }

                // Set the timer up.
                CLEAR_BIT(tim->CR1, 4);                    // Count upwards.
                CLEAR_BIT(tim->CR1, 3);                    // Keep counting.
                CLEAR_BIT(tim->CR2, 7);                    // Bind CH1 to T1.
                WRITE_FIELD(tim->SMCR, 0, 0x7, 0x03);      // Set Encoder mode 3.
                WRITE_FIELD(tim->CCMR1, 0, 0b11, 0b01);    // Set CH1 to an input.
                WRITE_FIELD(tim->CCMR1, 8, 0b11, 0b01);    // Set CH2 to an input.

                // CH2
                CLEAR_BIT(tim->CCER, 7);                   // Count on the rising edge.
                CLEAR_BIT(tim->CCER, 5);                   // Set the polarity to active high.
                // CH1
                CLEAR_BIT(tim->CCER, 3);                   // Count on the rising edge.
                CLEAR_BIT(tim->CCER, 1);                   // Set the polarity to active high.

                tim->PSC = 4;     // Set the prescaler.
                tim->ARR = 255;   // Set the period.
                tim->CCR1 = 128;  // Set the duty-cycle.

                if constexpr (K == 0) {
                    // Set four pins up as inputs.
                    WRITE_FIELD(GPIOB->MODER, 4*2, 0b11, 0b10); // Set PB4 as an alternate function.
                    WRITE_FIELD(GPIOB->AFR[0], 4*4, 0xF, 0x2);  // Use Alternate Function 2.
                    WRITE_FIELD(GPIOB->MODER, 5*2, 0b11, 0b10); // Set PB5 as an alternate function.
                    WRITE_FIELD(GPIOB->AFR[0], 5*4, 0xF, 0x2);  // Use Alternate Function 2.
                } else {
                    // Set four pins up as inputs.
                    WRITE_FIELD(GPIOB->MODER, 6*2, 0b11, 0b10); // Set PB6 as an alternate function.
                    WRITE_FIELD(GPIOB->AFR[0], 6*4, 0xF, 0x2);  // Use Alternate Function 2.
                    WRITE_FIELD(GPIOB->MODER, 7*2, 0b11, 0b10); // Set PB7 as an alternate function.
                    WRITE_FIELD(GPIOB->AFR[0], 7*4, 0xF, 0x2);  // Use Alternate Function 2.
                }

                SET_BIT(tim->CR1, 0); // Begin counting.
            }

            /// @brief  Gets the current value of the encoder.
            /// @return The count register of the timer.
            const uint8_t get_count() const {
                count = tim->CNT;
                return count;
            }

            /// @brief  Gets the shift of the encoder.
            /// @return The shift in the timer's counter.
            const int8_t get_shift() {
                int8_t shift = int8_t(count) - int8_t(tim->CNT);
                count = tim->CNT;
                return shift;
            }

        private:
            /// @brief  A reference to the peripheral timer.
            /// @note   A better approach may be to use a reference instead.
            TIM_TypeDef* tim = TIM3;
            /// @brief  A copy of the timer's count.
            uint8_t count = 0;
    };

} // namespace device