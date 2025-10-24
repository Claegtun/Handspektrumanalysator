#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "peripheral/register.hpp"

#include "utility/language.h"

#include <algorithm>

#include "../lib/CMSIS/DSP/Include/arm_math.h"

#include "coefficients.hpp"
#include "device/AADU.hpp"
#include "device/DigitalFilter.hpp"
#include "device/Encoder.hpp"
#include "module/Spectrum.hpp"
#include "../lib/driver/st7789/src/driver_st7789.h"
#include "../lib/driver/st7789/interface/driver_st7789_interface.h"

int main(void) {
    HAL_SYSTICK_Config(SystemCoreClock / (1000UL / (uint32_t)uwTickFreq));

    // Clock
    SET_BIT(RCC->CR, 24); // Enable PLL1 for SPI2.

    SET_BIT(RCC->AHB4ENR, 0);   // Enable the clock for GPIOA.
    SET_BIT(RCC->AHB4ENR, 1);   // Enable the clock for GPIOB.
    SET_BIT(RCC->AHB4ENR, 2);   // Enable the clock for GPIOC.
    SET_BIT(RCC->AHB1ENR, 0);   // Enable the clock for DMA1.

    // PA0
    WRITE_FIELD(GPIOA->MODER, 0*2, 0b11, 0b01); // Set pin-1 to output.

    SET_BIT(GPIOA->BSRR, 0+16); // Reset pin-1.
    //SET_BIT(GPIOA->BSRR, 0); // Set pin-1.

    // PA1
    WRITE_FIELD(GPIOA->MODER, 1*2, 0b11, 0b01); // Set pin-1 to output.

    SET_BIT(GPIOA->BSRR, 1+16); // Reset pin-1.

    // TIM5
    SET_BIT(RCC->APB1LENR, 3);  // Enable the clock for TIM5.

    CLEAR_BIT(TIM5->CR1, 4); // Count upwards.
    CLEAR_BIT(TIM5->CR1, 3); // Keep counting.

    SET_BIT(TIM5->CR2, 7); // Use CH3.

    SET_BIT(TIM5->EGR, 3); // Generate capture/compare.

    CLEAR_FIELD(TIM5->CCMR2, 0, 0b11);      // Set CH3 to an output.
    WRITE_FIELD(TIM5->CCMR2, 4, 0xF, 0x6);  // Set to PWM Mode-1.

    CLEAR_BIT(TIM5->CCER, 9);   // Set the polarity to active high.
    SET_BIT(TIM5->CCER, 8);     // Enable the capture.

    // Peripheral clock is 64 MHz.
    TIM5->PSC = 999; // Set the prescaler.
    TIM5->ARR = 63999;   // Set the period.
    TIM5->CCR3 = 900;  // Set the duty-cycle.

    NVIC_EnableIRQ(TIM5_IRQn); // Enable the NVIC interrupt.

    SET_BIT(TIM5->DIER, 0); // Enable the update-interrupt.
    
    SET_BIT(TIM5->CR1, 0); // Begin counting.

    HAL_Delay(3000);

    st7789_handle_t lcd_module;

    DRIVER_ST7789_LINK_INIT(&lcd_module, st7789_handle_t);
    DRIVER_ST7789_LINK_SPI_INIT(&lcd_module, st7789_interface_spi_init);
    DRIVER_ST7789_LINK_SPI_DEINIT(&lcd_module, st7789_interface_spi_deinit);
    DRIVER_ST7789_LINK_SPI_WRITE_COMMAND(&lcd_module, st7789_interface_spi_write_cmd);
    DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_INIT(&lcd_module, st7789_interface_cmd_data_gpio_init);
    DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_DEINIT(&lcd_module, st7789_interface_cmd_data_gpio_deinit);
    DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_WRITE(&lcd_module, st7789_interface_cmd_data_gpio_write);
    DRIVER_ST7789_LINK_RESET_GPIO_INIT(&lcd_module, st7789_interface_reset_gpio_init);
    DRIVER_ST7789_LINK_RESET_GPIO_DEINIT(&lcd_module, st7789_interface_reset_gpio_deinit);
    DRIVER_ST7789_LINK_RESET_GPIO_WRITE(&lcd_module, st7789_interface_reset_gpio_write);
    DRIVER_ST7789_LINK_DELAY_MS(&lcd_module, st7789_interface_delay_ms);
    DRIVER_ST7789_LINK_DEBUG_PRINT(&lcd_module, st7789_interface_debug_print);

    st7789_init(&lcd_module);
    st7789_software_reset(&lcd_module);
    st7789_sleep_out(&lcd_module);
    st7789_set_interface_pixel_format(
        &lcd_module, 
        ST7789_RGB_INTERFACE_COLOR_FORMAT_65K, 
        ST7789_CONTROL_INTERFACE_COLOR_FORMAT_16_BIT
    );
    st7789_set_memory_data_access_control(&lcd_module, 0x00);
    st7789_set_column(&lcd_module, 240);
    st7789_set_row(&lcd_module, 320);
    st7789_display_inversion_on(&lcd_module);
    st7789_normal_display_mode_on(&lcd_module);
    st7789_display_on(&lcd_module);

    st7789_set_display_brightness(&lcd_module, 0x01);
    HAL_Delay(1000);

    st7789_clear(&lcd_module);

    char str[] = "Hallo, Welt!";
    st7789_write_string(&lcd_module, 120, 160, str, sizeof(str), 0xFFFFFFFF, ST7789_FONT_12);

    HAL_Delay(1000);
    st7789_clear(&lcd_module);

    device::AADU aadu;
    device::DigitalFilter fir;

    device::Encoder<0> encoder_top;
    device::Encoder<1> encoder_bottom;

    arm_rfft_fast_instance_f32 fft;
    arm_rfft_fast_init_2048_f32(&fft);

    module::Spectrum<310, 220, 0, 20> spectrum(lcd_module);

    volatile float32_t f_m = 1e3;
    volatile int8_t scaling = 1;

    // std::array<int16_t, 320> height;
    // std::fill(height.begin(), height.end(), 0x0000);

    aadu.begin_frame();

    fir.begin_frame();

    for (int i = 0; i < 8*1024; i++) {
        fir.get_raw_buffer()[i] = static_cast<int32_t>(aadu.get_full_buffer()[i]) - 0x80E8;
    }

    while (1) {

        if (aadu.check_frame() && fir.check_frame()) {

            aadu.ready_next_frame();

            fir.ready_next_frame();

            std::array<float32_t,device::frame_length> x;
            for (int i = 0; i < device::frame_length; i++) {
                x[i] = float32_t(fir.get_filtered_buffer()[i*8]);
            }

            std::array<float32_t,device::frame_length> X;
            arm_rfft_fast_f32(&fft, x.data(), X.data(), 0);

            std::array<float32_t,device::frame_length/2> abs_X;
            arm_cmplx_mag_f32(X.data(), abs_X.data(), device::frame_length/2);

            abs_X[0] *= 0.01;

            scaling += encoder_bottom.get_shift();
            float32_t delta = encoder_top.get_shift()*module::f_s/module::F;
            if (scaling > 1) {
                delta *= scaling;
            }
            f_m += delta;

            spectrum.draw(abs_X, f_m, scaling);

            aadu.begin_frame();

            fir.begin_frame();

            for (int i = 0; i < 8*1024; i++) {
                fir.get_raw_buffer()[i] = static_cast<int32_t>(aadu.get_full_buffer()[i]) - 0x80E8;
            }
        }
        
    }
}

extern "C" void TIM5_IRQHandler(void) {
    if ((TIM5->SR & 0b1) == 0b1) {
        CLEAR_BIT(TIM5->SR, 0); // Clear the pending interrupt bit.
        TOGGLE_BIT(GPIOA->ODR, 1); // Toggle the yellow LED.
    }
}

extern "C" void TIM8_CC_IRQHandler(void) {
    if ((TIM8->SR & (0b1 << 1)) == (0b1 << 1)) {
        CLEAR_BIT(TIM8->SR, 1); // Clear the pending interrupt bit.
        //TOGGLE_BIT(GPIOC->ODR, 6); // Toggle CNVST.
    }
}

extern "C" void SysTick_Handler(void) { 
    HAL_IncTick(); 
}