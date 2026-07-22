#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// Safe AON GPIO Block Base Address
#define GPIO_AON_BASE      0x0c2f0000 

// Offset for Port AC (PAC) within the AON controller
#define PORT_AC_OFFSET     0x3000     

// Pin 32 corresponds to Pin 6 of Port AC. We calculate the exact bit-shift.
// Registers for Port AC are separated by 0x20 bytes (32 bytes) per individual pin block.
#define PIN_6_BLOCK_OFFSET (PORT_AC_OFFSET + (6 * 0x20))

// 32-bit array indices for Pin 32's control registers
#define REG_ENABLE_INDEX   ((PIN_6_BLOCK_OFFSET + 0x00) / 4) // Controller enablement
#define REG_OE_INDEX       ((PIN_6_BLOCK_OFFSET + 0x0c) / 4) // Output Enable / Direction
#define REG_OUT_INDEX      ((PIN_6_BLOCK_OFFSET + 0x10) / 4) // Output Value

#define PAGE_SIZE          0x10000 // AON block uses a 64KB page size allocation

int main()
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(fd < 0)
    {
        perror("Error: Must run executable with 'sudo'");
        return 1;
    }

    // Map the safely clocked AON register space
    void *map = mmap(
        nullptr,
        PAGE_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        GPIO_AON_BASE
    ); 

    if(map == MAP_FAILED)
    {
        perror("mmap failed");
        close(fd);
        return 1;
    }

    volatile uint32_t *gpio = reinterpret_cast<volatile uint32_t *>(map);

    // 1. Give control of the pin to the GPIO Controller
    gpio[REG_ENABLE_INDEX] |= 1;

    // 2. Set the pin direction to Output Mode (Drive enabled)
    gpio[REG_OE_INDEX] |= 1;

    printf("--- Safe Waveform Generation Started on Pin 32 (PAC.06) ---\n");
    printf("Connect your oscilloscope probe to Pin 32 and ground to Pin 34.\n");

    while(1)
    {
        gpio[REG_OUT_INDEX] |= 1;       // Drive HIGH (3.3V)
        usleep(500000);

        gpio[REG_OUT_INDEX] &= ~1;      // Drive LOW (0V)
        usleep(500000);
    }

    // Clean up (Unreachable due to loop, kept for structural completeness)
    munmap(map, PAGE_SIZE);
    close(fd);
    return 0;
}
