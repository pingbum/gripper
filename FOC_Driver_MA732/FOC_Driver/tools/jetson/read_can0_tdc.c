/* Read-only TDC check for the Orin CAN0 controller at 0x0c310000.
 * Reads timing/control registers, PSR byte 2, and CAN0 pad configuration.
 * Never writes registers or sends CAN.
 * Reading PSR byte 2 gives TDCV without clearing LEC/DLEC (byte 0 is avoided).
 * See https://forums.developer.nvidia.com/t/367473 for the sysfs/TDCR issue.
 */
#define _POSIX_C_SOURCE 200809L
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

static int read_file(const char *path, void *out, size_t size)
{
    FILE *file = fopen(path, "rb");
    if (!file) {
        perror(path);
        return -1;
    }
    size_t count = fread(out, 1, size, file);
    int failed = ferror(file);
    fclose(file);
    return failed ? -1 : (int)count;
}

static uint32_t be32(const unsigned char *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8) | p[3];
}

int main(void)
{
    if (geteuid() != 0) {
        fputs("Run with sudo: read access to /dev/mem is required.\n", stderr);
        return 1;
    }
    char compatible[128] = {0}, flags_text[32] = {0}, power[32] = {0};
    unsigned char reg[16];
    unsigned flags = 0;
    if (read_file("/sys/class/net/can0/device/of_node/compatible",
                  compatible, sizeof(compatible) - 1) <= 0
        || strcmp(compatible, "nvidia,tegra194-mttcan") != 0
        || read_file("/sys/class/net/can0/device/of_node/reg", reg, sizeof(reg)) != 16
        || be32(reg) != 0 || be32(reg + 4) != 0x0c310000
        || be32(reg + 8) != 0 || be32(reg + 12) < 0x4c) {
        fputs("Refusing access: can0 is not the expected Orin controller.\n", stderr);
        return 1;
    }
    if (read_file("/sys/class/net/can0/flags", flags_text, sizeof(flags_text) - 1) <= 0
        || sscanf(flags_text, "%x", &flags) != 1 || !(flags & 1)
        || read_file("/sys/class/net/can0/device/power/runtime_status",
                     power, sizeof(power) - 1) <= 0
        || strcmp(power, "active\n") != 0) {
        fputs("Refusing access: bring can0 UP first; runtime power must be active.\n", stderr);
        return 1;
    }
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0 || 0x0c310000UL % (unsigned long)page_size != 0) {
        fputs("Unsupported page size.\n", stderr);
        return 1;
    }
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("/dev/mem");
        return 1;
    }
    void *mapped = mmap(NULL, (size_t)page_size, PROT_READ, MAP_SHARED, fd, 0x0c310000);
    close(fd);
    if (mapped == MAP_FAILED) {
        perror("mmap");
        return 1;
    }
    volatile const uint32_t *registers = mapped;
    uint32_t crel = registers[0x00 / 4];
    uint32_t dbtp = registers[0x0c / 4];
    uint32_t cccr = registers[0x18 / 4];
    uint32_t tdcr = registers[0x48 / 4];
    uint8_t tdcv = ((volatile const uint8_t *)mapped)[0x46] & 0x7f;
    munmap(mapped, (size_t)page_size);
    printf("HW CREL = 0x%08" PRIx32 "\n", crel);
    printf("HW CCCR = 0x%08" PRIx32 " (INIT=%u, DAR=%u, FDOE=%u, BRSE=%u)\n",
           cccr, (unsigned)(cccr & 1), (unsigned)((cccr >> 6) & 1),
           (unsigned)((cccr >> 8) & 1), (unsigned)((cccr >> 9) & 1));
    printf("HW DBTP @0x0c31000c = 0x%08" PRIx32 " (TDC=%u)\n",
           dbtp, (unsigned)((dbtp >> 23) & 1));
    printf("HW TDCR @0x0c310048 = 0x%08" PRIx32 " (TDCO=%u, TDCF=%u)\n",
           tdcr, (unsigned)((tdcr >> 8) & 0x7f), (unsigned)(tdcr & 0x7f));
    printf("HW PSR.TDCV = %u mtq (SSP position: measured loop delay + TDCO; reset in INIT)\n",
           (unsigned)tdcv);
    /* Orin CAN0 pad addresses from the NVIDIA CAN developer guide. */
    off_t pad_page = (off_t)(0x0c303010UL - 0x0c303010UL % (unsigned long)page_size);
    fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("/dev/mem (pads)");
        return 1;
    }
    mapped = mmap(NULL, (size_t)page_size, PROT_READ, MAP_SHARED, fd, pad_page);
    close(fd);
    if (mapped == MAP_FAILED) {
        perror("mmap (pads)");
        return 1;
    }
    registers = mapped;
    uint32_t dout = registers[(0x0c303010UL - (unsigned long)pad_page) / 4];
    uint32_t din = registers[(0x0c303018UL - (unsigned long)pad_page) / 4];
    munmap(mapped, (size_t)page_size);
    printf("HW CAN0_DOUT pad @0x0c303010 = 0x%08" PRIx32 " (NVIDIA reference: 0x0000c400)\n", dout);
    printf("HW CAN0_DIN  pad @0x0c303018 = 0x%08" PRIx32 " (NVIDIA reference: 0x0000c458)\n", din);
    return 0;
}
