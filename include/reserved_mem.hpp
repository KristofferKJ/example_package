#pragma once
/**************************** Include Files *******************************/
#include <iostream>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>

#include <iostream>

/******************************* Defines **********************************/
#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define MAX_LENGTH 0x01000000 // Bytes

#define i_P_START 0
#define i_LENGTH 1
#define i_U_BUFFER_PTR_L 2
#define i_U_BUFFER_PTR_H 3

/******************************** Class ***********************************/
class Reserved_Mem
{
private:
    int memLKM;             // file
    uint32_t write_info[4]; // [p_offset, length, u_buffer_low, u_buffer_high]
    uint32_t read_info[4];  // [p_offset, length, u_buffer_low, u_buffer_high]

public:
    Reserved_Mem()
    {
        memLKM = open(DEVICE_FILENAME, O_RDWR | O_NDELAY);
    };

    // writes any type of user buffer to the reserved memory at the given offset
    // length is the number of bytes to be written
    template <typename T>
    uint32_t transfer(T *src, int p_offset, int length)
    {
        int ret;
        write_info[i_P_START] = p_offset;
        write_info[i_LENGTH] = length;
        // std::cout << "length: " << write_info[i_LENGTH] << std::endl;
        write_info[i_U_BUFFER_PTR_L] = (uint32_t)((uintptr_t)src & 0xFFFFFFFFLL);
        write_info[i_U_BUFFER_PTR_H] = (uint32_t)(((uintptr_t)src & 0xFFFFFFFF00000000LL) >> 32);

        ret = write(memLKM, write_info, sizeof(write_info));
        return ret;
    }

    // reads any type of user buffer from the reserved memory at the given offset
    // length is the number of bytes to be read
    template <typename T>
    uint32_t gather(T *dst, int p_offset, int length)
    {
        int ret;
        read_info[i_P_START] = p_offset;
        read_info[i_LENGTH] = length;

        read_info[i_U_BUFFER_PTR_L] = (uint32_t)((uintptr_t)dst & 0xFFFFFFFFLL);
        read_info[i_U_BUFFER_PTR_H] = (uint32_t)(((uintptr_t)dst & 0xFFFFFFFF00000000LL) >> 32);

        ret = read(memLKM, read_info, sizeof(read_info));

        return ret;
    }

    ~Reserved_Mem()
    {
        close(memLKM);
    };
};
