//
// Created by matzipan on 11/03/17.
//

#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdexcept>
#include "AcceleratorHandler.h"

#define OCM_SIZE 256*1024
#define OCM_LOC 0xFFFC0000

AcceleratorHandler::AcceleratorHandler() {
    printf("Mapping OCM: 256 KB @ 0x%x\n", OCM_LOC);

    // The parameter for open controls whether it uses caching or not.
    memd = open("/dev/mem" , O_RDWR | O_SYNC);
    if(memd < 0) {
        throw std::runtime_error("Error opening memory device for OCM");
    }

    ocm = mmap(NULL, OCM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memd, OCM_LOC);
    if(ocm == MAP_FAILED) {
        throw std::runtime_error("Error mapping memory device for OCM");
    }

    XToplevel_Initialize(&xToplevelInstance, "toplevel");
}

AcceleratorHandler::~AcceleratorHandler() {
    if(ocm != 0) {
        munmap(ocm, OCM_SIZE);
    }

    if(memd > 0) {
        close(memd);
    }

    XToplevel_Release(&xToplevelInstance);
}

void* AcceleratorHandler::getMemoryPointer() {
    return ocm;
}

#ifdef MULTIPARTICLE_ACCELERATOR
void AcceleratorHandler::setParticlesCount(uint32_t particles_count) {
    XToplevel_Set_particles_count_V(&xToplevelInstance, particles_count);
}
#else
void AcceleratorHandler::setN(uint32_t n) {
    XToplevel_Set_n_V(&xToplevelInstance, n);
}
#endif

void AcceleratorHandler::start() {
    XToplevel_Start(&xToplevelInstance);
}

uint AcceleratorHandler::isDone() {
    return XToplevel_IsDone(&xToplevelInstance);
}

