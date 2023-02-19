static const uint32_t MAX_TUNE_FREQ = 146500000; // The 2m band is from 144 to 146 MHz, allow some margin
static const uint32_t MIN_TUNE_FREQ = 143500000;

extern uint32_t frequencies[];

bool is_valid_freq(uint32_t f);
bool is_valid_freq(uint64_t f);

