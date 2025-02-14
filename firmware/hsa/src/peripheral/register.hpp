#define SET_BIT(reg, spot) (reg) |= (0b1 << (spot))

#define CLEAR_BIT(reg, spot) (reg) &= ~(0b1 << (spot))

#define TOGGLE_BIT(reg, spot) (reg) ^= (0b1 << (spot))

#define WRITE_FIELD(reg, shift, mask, value)   \
    do {                                \
        (reg) &= ~((mask) << shift);    \
        (reg) |= ((value) << shift);    \
    } while(0);                         \

#define CLEAR_FIELD(reg, shift, mask) (reg) &= ~((mask) << (shift));
