/**
 *Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_st7789_interface_template.c
 * @brief     driver st7789 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2023-04-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/04/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "../lib/driver/st7789/interface/driver_st7789_interface.h"

#include "stm32h7xx_hal.h"
#include "peripheral/register.hpp"

#include "utility/language.h"

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t st7789_interface_spi_init(void)
{
    // Set the pins up for the SPI.
    // Only a MOSI, an nCS, and an SCLK are needed.
    WRITE_FIELD(GPIOA->MODER, 5*2, 0b11, 0b10);     // Set PA5 as the SCLK.
    WRITE_FIELD(GPIOA->MODER, 7*2, 0b11, 0b10);     // Set PA7 as the MOSI.
    WRITE_FIELD(GPIOA->AFR[0], 5*4, 0xF, 0x5);      // Use Alternate Function 5.
    WRITE_FIELD(GPIOA->AFR[0], 7*4, 0xF, 0x5);      // Use Alternate Function 5.
    WRITE_FIELD(GPIOA->OSPEEDR, 5*2, 0b11, 0b11);   // Set PA5 to very fast.
    WRITE_FIELD(GPIOA->OSPEEDR, 7*2, 0b11, 0b11);   // Set PA7 to very fast.

    WRITE_FIELD(GPIOA->MODER, 4*2, 0b11, 0b01);     // Set PA4 as the nCS.
    WRITE_FIELD(GPIOA->OSPEEDR, 4*2, 0b11, 0b11);   // Set PA4 to very fast.
    SET_BIT(GPIOA->BSRR, 4);                        // Set pin-4.

    // Turn the clock on for SPI1.
    SET_BIT(RCC->APB2ENR, 12);

    WRITE_FIELD(SPI1->CFG1, 28, 0b111, 0b000);  // Prescale the master clock by 1/2.

    // Turn the DMA request on for RX. Set the rest up.
    // SET_BIT(SPI2->CFG1, 14);                 // Turn the RX DMA on.
    WRITE_FIELD(SPI1->CFG1, 0, 0x1F, 0x7);      // Make the frame a byte.
    WRITE_FIELD(SPI1->CFG2, 24, 0b11, 0b00);    // Take data on the rising edge of SCLK.

    // Choose a half-duplex setup since the ATM0200B3 has a single data-line.
    WRITE_FIELD(SPI1->CFG2, 17, 0b11, 0b11);
    SET_BIT(SPI1->CR1, 11);

    // Effectively, ignore the built-in chip-select since a GPIO will be used instead. Importantly, 
    // the SSI must be set before choosing master lest a mode-fail is raised.
    SET_BIT(SPI1->CFG2, 26);    // Use software-driven SS.
    SET_BIT(SPI1->CR1, 12);     // Set the SSI.
    SET_BIT(SPI1->CFG2, 22);    // Set as a master.

    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t st7789_interface_spi_deinit(void)
{
    // Nothing needed.
    return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t st7789_interface_spi_write_cmd(uint8_t *buf, uint16_t len)
{
    // Reset the nCS.
    SET_BIT(GPIOA->BSRR, 4+16);

    // Get the SPI ready for however many bytes.
    WRITE_FIELD(SPI1->CFG1, 0, 0x1F, 0x7);  // 8-bit
    SPI1->CR2 = len;                        // Send however many frames.
    SET_BIT(SPI1->CR1, 0);                  // Enable the peripheral.
    SET_BIT(SPI1->CR1, 9);                  // Begin the transmission.

    // Send all bytes.
    for (int i = 0; i < len; i++) {
        until (SPI1->SR & (0b1 << 1));
        *(volatile uint8_t*)(&SPI1->TXDR) = buf[i];
    }

    // Wait until the tranfer has endedd.
    until (SPI1->SR & (0b1 << 3)); // EOT
    
    // Get the SPI ready for the next thing.
    SET_BIT(SPI1->IFCR, 4); // Clear TXTF.
    CLEAR_BIT(SPI1->CR1, 0); // Disable the peripheral.

    // Set the nCS.
    SET_BIT(GPIOA->BSRR, 4);
    
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void st7789_interface_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void st7789_interface_debug_print(const char *const fmt, ...)
{
    // Nothing needed.
}

/**
 * @brief  interface command && data gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t st7789_interface_cmd_data_gpio_init(void)
{
    WRITE_FIELD(GPIOA->MODER, 3*2, 0b11, 0b01);     // Set PA3 as the DCX.
    WRITE_FIELD(GPIOA->OSPEEDR, 3*2, 0b11, 0b11);   // Set PA3 to very fast.
    SET_BIT(GPIOA->BSRR, 3+16);                     // Reset pin-3.

    return 0;
}

/**
 * @brief  interface command && data gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t st7789_interface_cmd_data_gpio_deinit(void)
{
    // Nothing needed.
    return 0;
}

/**
 * @brief     interface command && data gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 gpio write failed
 * @note      none
 */
uint8_t st7789_interface_cmd_data_gpio_write(uint8_t value)
{
    if (value == 0)
        SET_BIT(GPIOA->BSRR, 3+16); // Reset D/CX.
    else
        SET_BIT(GPIOA->BSRR, 3);    // Set D/CX.
    return 0;
}

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t st7789_interface_reset_gpio_init(void)
{
    // Nothing needed.
    return 0;
}

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t st7789_interface_reset_gpio_deinit(void)
{
    // Nothing needed.
    return 0;
}

/**
 * @brief     interface reset gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 gpio write failed
 * @note      none
 */
uint8_t st7789_interface_reset_gpio_write(uint8_t value)
{
    // Nothing needed.
    return 0;
}
