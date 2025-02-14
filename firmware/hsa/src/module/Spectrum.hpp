#include "stdint.h"

#include <array>

#include "../lib/driver/st7789/src/driver_st7789.h"
#include "../lib/driver/st7789/interface/driver_st7789_interface.h"

namespace module {

    constexpr float32_t f_s = 48e3;
    constexpr uint16_t F = device::frame_length;

    /// @brief  Spectrum.
    template<uint16_t W, uint16_t H>
    class Spectrum {
        /**
        
         */

        public:

            /// @brief  Constructs the object.
            Spectrum(st7789_handle_t& input_module) :
                lcd_module(input_module) 
            {
                std::fill(heights.begin(), heights.end(), 0x0000);
                st7789_clear(&lcd_module);
            }

            void draw(
                const std::array<float32_t,F/2>& abs_X, 
                const float32_t& f_m, 
                const float32_t& f_w
            ) {
                float32_t abs_X_max = *std::max_element(abs_X.begin(), abs_X.end());

                int16_t k_w = f_w * F / f_s;

                if (k_w > F/2) {
                    k_w = F/2;
                }

                int16_t k_0 = (f_m - f_w / 2.0f) * F / f_s;

                int16_t k_1 = (f_m + f_w / 2.0f) * F / f_s - 1;

                if (k_0 < 0) {
                    k_0 = 0;
                    k_1 = k_0 + k_w - 1;
                }

                if (k_1 >= F/2) {
                    k_1 = F/2 - 1;
                    k_0 = k_1 - k_w + 1;
                }

                int16_t factor;

                if (k_w > W) {
                    factor = k_w / W;

                    for (int i = 0; i < W; i++) {
                        growths[i] = 0;
                        
                        for (int j = 0; j < factor; j++) {
                            growths[i] += int16_t(230.0*abs_X[k_0 + factor*i + j] / abs_X_max);
                        }

                        growths[i] -= heights[i];
                    }
                }
                else {
                    factor = W / k_w;

                    for (int k = k_0; k <= k_1; k++) {
                        for (int j = 0; j < factor; j++) {
                            growths[(k - k_0)*factor + j] = 
                                int16_t(230.0*abs_X[k] / abs_X_max) - heights[(k - k_0)*factor + j];
                        }

                        for (int j = (k_1 - k_0)*factor + factor; j < W; j++) {
                            growths[j] = -1*heights[j];
                        }
                    }
                }

                for (int i = 0; i < W; i++) {
                    if (growths[i] == 0) {
                        continue;
                    }
                    else if (growths[i] > 0) {
                        st7789_fill_rect(
                            &lcd_module, 
                            heights[i], i, 
                            heights[i]+growths[i], i, 
                            0xFFFFFFFF
                        );
                    }
                    else {
                        st7789_fill_rect(
                            &lcd_module, 
                            heights[i]+growths[i], i, 
                            heights[i], i, 
                            0x00000000
                        );
                    }

                    heights[i] = heights[i] + growths[i];
                }
            }

        private:
            /// @brief  A reference to the LCD module.
            st7789_handle_t& lcd_module;
            /// @brief  The heights of the bins.
            std::array<int16_t, W> heights;
            /// @brief  The growths of the bins in the new spectrum.
            std::array<int16_t, W> growths;
    };

} // namespace device