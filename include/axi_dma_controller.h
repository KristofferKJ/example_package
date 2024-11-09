#pragma once

#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>

#include <string>
#include <sstream>
#include <vector>

#define MM2S_CONTROL_REGISTER       0x00
#define MM2S_STATUS_REGISTER        0x04
#define MM2S_SRC_ADDRESS_REGISTER   0x18
#define MM2S_TRNSFR_LENGTH_REGISTER 0x28

#define S2MM_CONTROL_REGISTER       0x30
#define S2MM_STATUS_REGISTER        0x34
#define S2MM_DST_ADDRESS_REGISTER   0x48
#define S2MM_BUFF_LENGTH_REGISTER   0x58

#define IOC_IRQ_FLAG                1<<12
#define IDLE_FLAG                   1<<1

#define STATUS_HALTED               0x00000001
#define STATUS_IDLE                 0x00000002
#define STATUS_SG_INCLDED           0x00000008
#define STATUS_DMA_INTERNAL_ERR     0x00000010
#define STATUS_DMA_SLAVE_ERR        0x00000020
#define STATUS_DMA_DECODE_ERR       0x00000040
#define STATUS_SG_INTERNAL_ERR      0x00000100
#define STATUS_SG_SLAVE_ERR         0x00000200
#define STATUS_SG_DECODE_ERR        0x00000400
#define STATUS_IOC_IRQ              0x00001000
#define STATUS_DELAY_IRQ            0x00002000
#define STATUS_ERR_IRQ              0x00004000

#define HALT_DMA                    0x00000000
#define RUN_DMA                     0x00000001
#define RESET_DMA                   0x00000004
#define ENABLE_IOC_IRQ              0x00001000
#define ENABLE_DELAY_IRQ            0x00002000
#define ENABLE_ERR_IRQ              0x00004000
#define ENABLE_ALL_IRQ              0x00007000

enum dma_status {
    DMA_HALTED = 0x00000001,
    DMA_IDLE = 0x00000002,
    DMA_SG_INCLDED = 0x00000008,
    DMA_INTERNAL_ERR = 0x00000010,
    DMA_SLAVE_ERR = 0x00000020,
    DMA_DECODE_ERR = 0x00000040,
    DMA_SG_INTERNAL_ERR = 0x00000100,
    DMA_SG_SLAVE_ERR = 0x00000200,
    DMA_SG_DECODE_ERR = 0x00000400,
    DMA_IOC_IRQ = 0x00001000,
    DMA_DELAY_IRQ = 0x00002000,
    DMA_ERR_IRQ = 0x00004000
};

class DMAStatus : public std::vector<dma_status> {
public:
    DMAStatus(uint32_t status) {
        if (status & DMA_HALTED) {
            push_back(DMA_HALTED);
        }
        if (status & DMA_IDLE) {
            push_back(DMA_IDLE);
        }
        if (status & DMA_SG_INCLDED) {
            push_back(DMA_SG_INCLDED);
        }
        if (status & DMA_INTERNAL_ERR) {
            push_back(DMA_INTERNAL_ERR);
        }
        if (status & DMA_SLAVE_ERR) {
            push_back(DMA_SLAVE_ERR);
        }
        if (status & DMA_DECODE_ERR) {
            push_back(DMA_DECODE_ERR);
        }
        if (status & DMA_SG_INTERNAL_ERR) {
            push_back(DMA_SG_INTERNAL_ERR);
        }
        if (status & DMA_SG_SLAVE_ERR) {
            push_back(DMA_SG_SLAVE_ERR);
        }
        if (status & DMA_SG_DECODE_ERR) {
            push_back(DMA_SG_DECODE_ERR);
        }
        if (status & DMA_IOC_IRQ) {
            push_back(DMA_IOC_IRQ);
        }
        if (status & DMA_DELAY_IRQ) {
            push_back(DMA_DELAY_IRQ);
        }
        if (status & DMA_ERR_IRQ) {
            push_back(DMA_ERR_IRQ);
        }
    }

    std::string to_string() {
        std::stringstream ss;
        for (auto status : *this) {
            switch (status) {
                case DMA_HALTED:
                    ss << "DMA_HALTED ";
                    break;
                case DMA_IDLE:
                    ss << "DMA_IDLE ";
                    break;
                case DMA_SG_INCLDED:
                    ss << "DMA_SG_INCLDED ";
                    break;
                case DMA_INTERNAL_ERR:
                    ss << "DMA_INTERNAL_ERR ";
                    break;
                case DMA_SLAVE_ERR:
                    ss << "DMA_SLAVE_ERR ";
                    break;
                case DMA_DECODE_ERR:
                    ss << "DMA_DECODE_ERR ";
                    break;
                case DMA_SG_INTERNAL_ERR:
                    ss << "DMA_SG_INTERNAL_ERR ";
                    break;
                case DMA_SG_SLAVE_ERR:
                    ss << "DMA_SG_SLAVE_ERR ";
                    break;
                case DMA_SG_DECODE_ERR:
                    ss << "DMA_SG_DECODE_ERR ";
                    break;
                case DMA_IOC_IRQ:
                    ss << "DMA_IOC_IRQ ";
                    break;
                case DMA_DELAY_IRQ:
                    ss << "DMA_DELAY_IRQ ";
                    break;
                case DMA_ERR_IRQ:
                    ss << "DMA_ERR_IRQ ";
                    break;
            }
        }

        return ss.str();
    }
};

// class named AXIDMAController
class AXIDMAController {
public:
    // Constructor
    AXIDMAController(unsigned int uio_number, unsigned int uio_size) {
        char device_file_name[20];
        sprintf(device_file_name, "/dev/uio%d", uio_number);

        int device_file;

        if ((device_file = open(device_file_name, O_RDWR | O_SYNC)) < 0) {
            std::stringstream ss;
            ss << device_file_name << " could not be opened";
            throw ss.str();
        }

        uio_map = (uint32_t *)mmap(NULL, uio_size, PROT_READ | PROT_WRITE, MAP_SHARED, device_file, 0);

        if (uio_map == MAP_FAILED) {
            std::stringstream ss;
            ss << device_file_name << " could not be mapped";
            throw ss.str();
        }

    }

    // Destructor
    ~AXIDMAController() {

        munmap(uio_map, 0x1000);

    }

    DMAStatus MM2SGetStatus() {
        
        DMAStatus status(getMM2SStatus());

        return status;

    }

    DMAStatus S2MMGetStatus() {
        
        DMAStatus status(getS2MMStatus());

        return status;

    }

    bool MM2SIsSynced() {
        
        unsigned int status = getMM2SStatus();

        return !(!(status & IOC_IRQ_FLAG) && !(status & IDLE_FLAG));
            
    }

    bool S2MMIsSynced() {
        
        unsigned int status = getS2MMStatus();

        return !(!(status & IOC_IRQ_FLAG) && !(status & IDLE_FLAG));
            
    }

    void MM2SReset() {

        writeAXI(MM2S_CONTROL_REGISTER, RESET_DMA);

    }

    void S2MMReset() {

        writeAXI(S2MM_CONTROL_REGISTER, RESET_DMA);

    }

    void MM2SHalt() {

        writeAXI(MM2S_CONTROL_REGISTER, HALT_DMA);

    }

    void S2MMHalt() {

        writeAXI(S2MM_CONTROL_REGISTER, HALT_DMA);

    }

    void MM2SInterruptEnable() {

        writeAXI(MM2S_CONTROL_REGISTER, ENABLE_ALL_IRQ);

    }

    void S2MMInterruptEnable() {

        writeAXI(S2MM_CONTROL_REGISTER, ENABLE_ALL_IRQ);

    }

    void MM2SSetSourceAddress(unsigned int address) {

        writeAXI(MM2S_SRC_ADDRESS_REGISTER, address);

    }

    void S2MMSetDestinationAddress(unsigned int address) {

        writeAXI(S2MM_DST_ADDRESS_REGISTER, address);

    }

    void MM2SSetLength(unsigned int length) {

        writeAXI(MM2S_TRNSFR_LENGTH_REGISTER, length);

    }

    void S2MMSetLength(unsigned int length) {

        writeAXI(S2MM_BUFF_LENGTH_REGISTER, length);

    }

    void MM2SStart() {

        writeAXI(MM2S_CONTROL_REGISTER, RUN_DMA);

    }

    void S2MMStart() {

        writeAXI(S2MM_CONTROL_REGISTER, RUN_DMA);

    }

    uint32_t *uio_map;
    
private:

    unsigned int writeAXI(uint32_t offset, uint32_t value) {
        uio_map[offset>>2] = value;
        return 0;
    }

    unsigned int readAXI(uint32_t offset) {
        return uio_map[offset>>2];
    }

    unsigned int getMM2SStatus() {

        return readAXI(MM2S_STATUS_REGISTER);

    }

    unsigned int getS2MMStatus() {

        return readAXI(S2MM_STATUS_REGISTER);
    }


};