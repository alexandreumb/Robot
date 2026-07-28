// mmio_gpio.hpp
//
// Direct MMIO GPIO toggle for Tegra234 (Jetson AGX Orin), bypassing gpiod's
// ioctl path for lower, more deterministic latency in an RT loop.
//
// ── WHAT YOU NEED TO FILL IN ────────────────────────────────────────────────
// Everything you need to change is marked TODO below. Everything else
// (mmap page alignment, bit masking, register offsets you already found in
// the TRM) is generic and shouldn't need touching.
//
// ── BEFORE USING THIS FOR REAL ───────────────────────────────────────────────
// 1. Run the read-only verifier (mmio_gpio_verify.cpp, built separately) FIRST
//    and confirm the bit you read tracks a gpiod-driven toggle on the same
//    pin. Do not skip this — a wrong base address means undefined behavior
//    on unrelated physical memory.
// 2. Once verified read-only, switch PROT_READ -> PROT_READ|PROT_WRITE and
//    test set()/clear() the same way (toggle via this code, confirm with
//    `gpioget` from another terminal that the pin actually follows).
// 3. Stop using gpiod_line_set_value() on this same line once you switch to
//    MMIO — do not drive the same physical pin from both paths at once.
//
// Build: g++ -O2 -std=c++17 your_file.cpp -o your_binary
// Run: needs root (opens /dev/mem).

#pragma once

#include <cstdint>
#include <cstdio>
#include <csignal>
#include <fcntl.h>
#include <stdexcept>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>

class MmioGpio
{
public:
    // phys_base:      TODO — physical base address of the GPIO port's
    //                 register block (the address that GPIO_R_OUTPUT_VALUE_00_0
    //                 and friends are offset from). Confirm this against the
    //                 kernel's tegra234-gpio device tree / driver source
    //                 rather than TRM text alone.
    //
    // output_value_off / output_control_off / enable_config_off:
    //                 Offsets you already found in the TRM — defaults below
    //                 match what you posted. Change only if your port uses
    //                 a different bank numbering.
    //
    // bit:            TODO — bit index of your specific pin within the
    //                 port's register (e.g. PR.00 -> bit 0, PR.01 -> bit 1).
    MmioGpio(
        uintptr_t phys_base         = 0x02200000,                 // TODO: fill in
        uintptr_t pactl_base        = 0x02430000,
        unsigned bit                = 0,                        // TODO: fill in
        bool writable               = true,                // start false (read-only) for verification
        uint32_t pactl_offset       = 0x80,
        uint32_t output_value_off   = 0x12810,
        uint32_t output_control_off = 0x1280c,
        uint32_t enable_config_off  = 0x12800)
        : bit_(bit),
          pactl_offset_(pactl_offset),
          output_value_off_(output_value_off),
          output_control_off_(output_control_off),
          enable_config_off_(enable_config_off)
    {
        fd_ = open("/dev/mem", (writable ? O_RDWR : O_RDONLY) | O_SYNC);
        if (fd_ < 0) {
            throw std::runtime_error("open /dev/mem failed - are you root?");
        }

        //SOC
        const long page_size = sysconf(_SC_PAGESIZE);
        const uintptr_t aligned_base = phys_base & ~(static_cast<uintptr_t>(page_size) - 1);
        page_offset_ = phys_base - aligned_base;

        // Map enough to cover the highest offset we reference (enable_config
        // is lowest at 0x12800, output_value highest at 0x12810) plus margin.
        uint32_t max_offset = std::max({output_value_off, output_control_off, enable_config_off});
        size_t map_size = ((page_offset_ + max_offset + sizeof(uint32_t) + page_size - 1) / page_size) * page_size;
        gpio_map_total_ = map_size + page_offset_;

        void* map = mmap(
            nullptr,
            gpio_map_total_,
            writable ? (PROT_READ | PROT_WRITE) : PROT_READ,
            MAP_SHARED,
            fd_,
            aligned_base);

        if (map == MAP_FAILED) {
            close(fd_);
            throw std::runtime_error("mmap failed - check phys_base / permissions");
        }

        base_ = static_cast<volatile uint8_t*>(map);

        // PACTL
        const uintptr_t aligned_pactl = pactl_base & ~(static_cast<uintptr_t>(page_size) - 1);
        pactl_page_offset_ = pactl_base - aligned_pactl;

        // Map enough to cover the highest offset we reference (enable_config
        // is lowest at 0x12800, output_value highest at 0x12810) plus margin.
        map_size = ((pactl_page_offset_ + pactl_offset_ + sizeof(uint32_t) + page_size - 1) / page_size) * page_size;
        pactl_map_total_ = map_size + pactl_page_offset_;

        map = mmap(
            nullptr,
            pactl_map_total_,
            writable ? (PROT_READ | PROT_WRITE) : PROT_READ,
            MAP_SHARED,
            fd_,
            aligned_pactl);

        if (map == MAP_FAILED) {
            close(fd_);
            throw std::runtime_error("mmap failed - check phys_base / permissions");
        }

        pactl_ = static_cast<volatile uint8_t*>(map);
        writable_ = writable;
    }

    ~MmioGpio()
    {
        if (base_) {
            munmap(const_cast<uint8_t*>(base_), gpio_map_total_);
        }        
        if (pactl_) {
            munmap(const_cast<uint8_t*>(pactl_), pactl_map_total_);
        }
        if (fd_ >= 0) close(fd_);
    }

    // Read-only: current output value bit for this pin (0 or 1).
    unsigned read_output_value() const
    {
        volatile uint32_t* reg = reg_ptr(output_value_off_);
        return (*reg >> bit_) & 0x1u;
    }

    // Read-only: current output-control state for this pin.
    unsigned read_output_control() const
    {
        volatile uint32_t* reg = reg_ptr(output_control_off_);
        return (*reg >> bit_) & 0x1u;
    }

    // Write path — only call once you've verified read-only mapping first
    // AND constructed this object with writable = true.
    void set(bool high)
    {
        if (!writable_) {
            throw std::runtime_error("MmioGpio opened read-only; cannot set()");
        }
        volatile uint32_t* reg = reg_ptr(output_value_off_);
        uint32_t v = *reg;
        if (high) v |= (1u << bit_);
        else      v &= ~(1u << bit_);
        *reg = v;
    }

    void configure_pactl()
    {
        if (!writable_) {
            throw std::runtime_error("MmioGpio opened read-only; cannot set()");
        }
        volatile uint32_t* reg = pactl_ptr(pactl_offset_);
        uint32_t v = *reg;
        v = 0x000;
        *reg = v;
    }

private:
    volatile uint32_t* reg_ptr(uint32_t offset) const
    {
        return reinterpret_cast<volatile uint32_t*>(
            const_cast<uint8_t*>(base_) + page_offset_ + offset);
    }

    volatile uint32_t* pactl_ptr(uint32_t offset) const
    {
        return reinterpret_cast<volatile uint32_t*>(
            const_cast<uint8_t*>(pactl_) + pactl_page_offset_ + offset);
    }

    int fd_ = -1;
    volatile uint8_t* base_ = nullptr;
    volatile uint8_t* pactl_ = nullptr;
    uintptr_t page_offset_ = 0;
    uintptr_t pactl_page_offset_ = 0;
    unsigned bit_;
    uint32_t pactl_offset_;
    uint32_t output_value_off_;
    uint32_t output_control_off_;
    uint32_t enable_config_off_;
    size_t pactl_map_total_;
    size_t gpio_map_total_;
    bool writable_ = false;
};