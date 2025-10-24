#include "stdint.h"

#include <array>

#include "../lib/driver/st7789/src/driver_st7789.h"
#include "../lib/driver/st7789/interface/driver_st7789_interface.h"

namespace module {

    constexpr float32_t f_s = 48e3;
    constexpr uint16_t F = device::frame_length;

    /// @brief  Spectrum.
    template<uint16_t W, uint16_t H, uint16_t X, uint16_t Y>
    class Spectrum {
        /**
        
         */

        public:

            /// @brief  Constructs the object.
            Spectrum(st7789_handle_t& input_module) :
                lcd_module(input_module) 
            {
                std::fill(heights.begin(), heights.end(), 0x0000);
                volatile uint8_t res = st7789_fill_rect(
                    &input_module, Y, X, Y+H-1, X+W-1, 
                    pack_as_565(0x08, 0x10, 0x08)
                );
                st7789_fill_rect(&lcd_module, Y+1, X+1, Y+H-2, X+W-2, 0x00000000);

                for (int i = 0; i < 4; i++) {
                    std::fill(grid_map[i].begin(), grid_map[i].end(), false);
                    for (int j = 0; 100*(i+1)*j < f_s/2; j++) {
                        grid_map[i][std::round(F*100*(i+1)*j/f_s)] = true;
                    }
                }
            }

            /// @brief  Draws the spectrum.
            /// @param  abs_X The magnitude of the spectrum.
            /// @param  f_m The middle frequency.
            /// @param  scaling The scaling factor; when positive, scales up; when negative, scales 
            //          down.
            void draw(
                const std::array<float32_t,F/2>& abs_X, 
                volatile const float32_t& f_m, 
                volatile const int8_t& scaling
            ) {
                /// @brief  The greatest single magnitude within the spectrum. 
                const float32_t abs_X_max = *std::max_element(abs_X.begin(), abs_X.end());
                /// @brief  The middle bin.
                int16_t k_m = f_m * F / f_s;
                /// @brief  The lowest bin.
                int16_t k_0;
                /// @brief  The highest bin.
                int16_t k_1;
                /// @brief  The bin bandwidth.
                int16_t k_w;
                /// @brief  The scaling factor.
                int16_t factor;

                switch (scaling) {
                    case -3:
                        grid_division = division_5k;
                        break;
                    default:
                    case -2:
                    case -1:
                    case 1:
                    case 2:
                        grid_division = division_1k;
                        break;
                    case 3:
                        grid_division = division_500;
                        break;
                    case 4:
                        grid_division = division_100;
                        break;
                }

                // If the bandwidth is more than the window's width, then aggregate as many 
                // bins as the scaling factor into one bin.  
                if (scaling < -1) {
                    // Get the magnitude of the scaling.
                    factor = 0b1 << (-1 * scaling - 1);

                    // Get the bin bandwidth calculated as the window's width scaled up.
                    k_w = factor * W;
                    if (k_w > F) {
                        factor = F/W;
                        k_w = factor * W;
                    }

                    // Get the first bin.
                    k_0 = k_m - k_w/2;
                    if (k_0 < 0) {
                        k_0 = 0;
                        k_m = k_0 + k_w/2;
                    }

                    // Compute each column's growth in the window as the average of as many 
                    // bins as the scaling factor.
                    for (int i = 0; i < W; i++) {
                        growths[i] = 0;
                        is_gridline[i] = false;
                        
                        for (int j = 0; j < factor; j++) {
                            growths[i] += int16_t(H*0.9*abs_X[k_0 + factor*i + j] / abs_X_max);
                            is_gridline[i] |= grid_map[grid_division][k_0 + factor*i + j];
                        }

                        growths[i] -= heights[i];
                    }
                }
                // If the bandwidth is less than the window's width, then widen each bin.
                else if (scaling >= -1) {
                    // Get the magnitude of the scaling.
                    factor = scaling;
                    // If the scaling is not positive, then make the factor one.
                    if (scaling < 1)
                        factor = 1;
                    else
                        factor = 0b1 << (scaling - 1);

                    // Calculate the needed bin bandwidth given the window's width and the factor.
                    k_w = W / factor;

                    // Find out the lowest bin and the highest.
                    k_0 = k_m - k_w/2;
                    k_1 = k_0 + k_w - 1;

                    // Clamp the lowest bin to zero.
                    if (k_0 < 0) {
                        k_0 = 0;
                        k_1 = k_0 + k_w - 1;
                    }

                    // Clamp the highest bin to half the frame length.
                    if (k_1 >= F/2) {
                        k_1 = F/2 - 1;
                        k_0 = k_1 - k_w + 1;
                    }

                    // For each bin within the chosen band, draw as many columns of the same height 
                    // as the scaling factor.
                    for (int k = k_0; k <= k_1; k++) {
                        for (int j = 0; j < factor; j++) {
                            growths[(k - k_0)*factor + j] = 
                                int16_t(H*0.9*abs_X[k] / abs_X_max) - heights[(k - k_0)*factor + j];
                            is_gridline[(k - k_0)*factor + j] = grid_map[grid_division][
                                (k - k_0)*factor + j
                            ];
                        }
                    }
                }

                if ((k_0 != old.k_0) || (k_w != old.k_w)) {
                    is_new_grid = true;
                    old.k_0 = k_0;
                    old.k_w = k_w;
                }

                // Either draw or erase how ever much is needed for each column in the window.
                for (int i = 0; i < W; i++) {
                    // if (growths[i] == 0) {
                    //     continue;
                    // }
                    if (growths[i] > 0) {
                        st7789_fill_rect(
                            &lcd_module, 
                            Y+heights[i], X+i, 
                            Y+heights[i]+growths[i], X+i, 
                            0xFFFFFFFF
                        );
                        if ((is_new_grid) && (i != 0) && (i != W-1)) {
                            if (was_gridline[i]) {
                                st7789_fill_rect(
                                    &lcd_module, 
                                    Y+heights[i]+growths[i], X+i, 
                                    Y+H-1, X+i, 
                                    pack_as_565(0x00, 0x00, 0x00)
                                );
                            }
                            if (is_gridline[i]) {
                                st7789_fill_rect(
                                    &lcd_module, 
                                    Y+heights[i]+growths[i], X+i, 
                                    Y+H-1, X+i, 
                                    pack_as_565(0x08, 0x10, 0x08)
                                );
                            }
                        }   
                    }
                    else {
                        uint32_t blank_colour = pack_as_565(0x00, 0x00, 0x00);
                        uint16_t blank_height = heights[i];

                        if ((i == 0) || (i == W - 1)) {
                            blank_colour = pack_as_565(0x08, 0x10, 0x08);
                        }
                        else if (
                            (is_new_grid) 
                            && ((was_gridline[i]) || (is_gridline[i])) 
                            && (i != 0) && (i != W-1)
                        ) {
                            blank_colour = pack_as_565(0x08, 0x10, 0x08);
                            blank_height = H-1;
                        }
                        volatile uint8_t res = st7789_fill_rect(
                            &lcd_module, 
                            Y+heights[i]+growths[i], X+i, 
                            Y+blank_height, X+i, 
                            blank_colour
                        );
                        is_new_grid = res + 1;
                    }

                    heights[i] = heights[i] + growths[i];

                    was_gridline[i] = is_gridline[i];
                }

                is_new_grid = false;
            }

        private:
            /// @brief  A reference to the LCD module.
            st7789_handle_t& lcd_module;
            /// @brief  The heights of the bins.
            std::array<int16_t, W> heights;
            /// @brief  The growths of the bins in the new spectrum.
            std::array<int16_t, W> growths;
            /// @brief
            std::array<bool, W> is_gridline;
            /// @brief
            std::array<bool, W> was_gridline;
            /// @brief
            std::array<std::array<bool,F/2>,4> grid_map;
            /// @brief
            enum GridDivison {
                division_100 = 0,
                division_500 = 1,
                division_1k = 2,
                division_5k = 3
            } grid_division = division_1k;
            /// @brief
            struct {
                int16_t k_0 = 0;
                int16_t k_w = W;
            } old;
            /// @brief  
            bool is_new_grid = true;

            /// @brief  Packs red, green, and blue as a single integer.
            /// @param  red A five-bit integer.
            /// @param  green A six-bit integer.
            /// @param  blue A five-bit integer.
            /// @return An integer with red, green, and blue packed together.
            inline uint32_t pack_as_565(uint8_t red, uint8_t green, uint8_t blue) {
                return (red << 11) | (green << 5) | (blue << 0);
            }
    };

} // namespace device