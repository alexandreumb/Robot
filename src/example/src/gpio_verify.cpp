#include "mmio_gpio.hpp"
#include <chrono>
#include <thread>

int main()
{
    const uintptr_t kPhysBase = 0x02200000; 
    const unsigned  kBit      = 0; 

    if (kPhysBase == 0x0) {
        std::fprintf(stderr,
            "Set kPhysBase to your confirmed port base address before running.\n");
        return 1;
    }

    try {
        MmioGpio gpio(kPhysBase, kBit, /*writable=*/false);

        std::printf("Reading OUTPUT_VALUE bit %u at base 0x%lx every 0.5s.\n",
                    kBit, static_cast<unsigned long>(kPhysBase));
        std::printf("Toggle the same pin via gpioset/gpioget in another terminal now.\n");

        while (true) {
            unsigned val = gpio.read_output_value();
            unsigned ctrl = gpio.read_output_control();
            std::printf("output_value=%u  output_control=%u\n", val, ctrl);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }

    return 0;
}
