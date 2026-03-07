#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "video_common.h"

// ================== 硬件定义 ==================
// DBI (PCIe Controller)
#define DBI_BASE       0x3600000
#define PCIE_ATU_VIEWPORT  0x900
#define PCIE_ATU_CTRL1     0x904
#define PCIE_ATU_CTRL2     0x908
#define PCIE_ATU_LBAR      0x90C
#define PCIE_ATU_UBAR      0x910
#define PCIE_ATU_LIMIT     0x914
#define PCIE_ATU_LTAR      0x918
#define PCIE_ATU_UTAR      0x91C

// Memory Map
#define SRC_ADDR_HI    0x90
#define SRC_ADDR_LO    0x40000000
#define DPU_PHY_ADDR   0x9040000000ULL  // DPU 视角的 Host 映射地址
#define TGT_ADDR_HI    0xF
#define TGT_ADDR_LO    0x00000000
#define MAP_LEN0      (RING0_MAP_LEN)
#define MAP_LEN1      (RING1_MAP_LEN)
#define MAP_GUARD     (RING_GUARD_BYTES)
#define MAP_LEN_TOTAL (MAP_LEN0 + MAP_GUARD + MAP_LEN1)

#define UDP_PORT       8888
#define UDP_PACKET_SIZE 65535


// ================== Ring 参数（双流不同大小） ==================
#define R0_SIZE   ((uint32_t)RING0_SIZE_BYTES)
#define R1_SIZE   ((uint32_t)RING1_SIZE_BYTES)
#define R0_MASK   ((uint32_t)RING0_MASK)
#define R1_MASK   ((uint32_t)RING1_MASK)

static inline uint32_t ring_free(uint32_t head, uint32_t tail, uint32_t ring_size) {
    return ring_size - (head - tail);
}

// ================== 辅助函数 ==================
void write_reg(volatile uint32_t *base, int offset, uint32_t val) {
    base[offset / 4] = val;
    volatile uint32_t dummy = base[offset / 4]; // Read back
}

uint32_t read_reg(volatile uint32_t *base, int offset) {
    return base[offset / 4];
}

void safe_write_pcie(volatile uint32_t *dst_base32, uint32_t ring_size, uint32_t offset, const void *src, uint32_t len) {
    // WC 映射下的写入器：保持环形缓冲区字节布局不变（[header][payload][0~3 padding]）
    // - 协议层：head/tail 仍按 4B 对齐推进（aligned_size=align4）
    // - 实现层：默认采用 32u 展开（32B=4×64-bit）写入，以减少事务与循环开销
    const uint8_t *s = (const uint8_t *)src;

    // ring_size 必须是 2 的幂，可用掩码快速取模
    const uint32_t R = ring_size;
    const uint32_t mask_b = R - 1u;
    const uint32_t mask_q = (R >> 3) - 1u; // qword(8B) mask（要求 ring_size 为 8 的倍数）

    // data_area 在 video_common.h 的布局下通常具备至少 8B 对齐；并且本函数会自行处理非 8B 起始
    volatile uint64_t *dst64 = (volatile uint64_t *)dst_base32;

    // 1) 前缀：把 offset 推到 8B 对齐（保持字节序列一致）
    while ((offset & 7u) && len) {
        ((volatile uint8_t *)dst_base32)[offset & mask_b] = *s;
        offset++; s++; len--;
    }

    // 2) 主循环：默认 32u 展开（32B = 4×8B）
    while (len >= 32u) {
        uint64_t v0, v1, v2, v3;
        memcpy(&v0, s + 0,  8);
        memcpy(&v1, s + 8,  8);
        memcpy(&v2, s + 16, 8);
        memcpy(&v3, s + 24, 8);

        uint32_t base = (offset >> 3) & mask_q;
        dst64[(base + 0u) & mask_q] = v0;
        dst64[(base + 1u) & mask_q] = v1;
        dst64[(base + 2u) & mask_q] = v2;
        dst64[(base + 3u) & mask_q] = v3;

        offset += 32u;
        s      += 32u;
        len    -= 32u;
    }

    // 3) 剩余：按 8B 写
    while (len >= 8u) {
        uint64_t v;
        memcpy(&v, s, 8);
        uint32_t idx64 = (offset >> 3) & mask_q;
        dst64[idx64] = v;
        offset += 8u;
        s += 8u;
        len -= 8u;
    }

    // 4) 尾部剩余字节
    while (len) {
        ((volatile uint8_t *)dst_base32)[offset & mask_b] = *s;
        offset++; s++; len--;
    }
}


// ================== 核心功能：配置 PCIe ATU ==================
int configure_pcie_atu() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("Open /dev/mem (DBI) failed"); return -1; }

    void *dbi_map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, DBI_BASE);
    if (dbi_map == MAP_FAILED) { perror("mmap DBI failed"); close(fd); return -1; }
    
    volatile uint32_t *regs = (volatile uint32_t *)dbi_map;

    printf("[Init] Configuring PCIe ATU...\n");

    // 1. 启用 Bus Master
    uint32_t cmd = read_reg(regs, 0x04);
    if ((cmd & 0x07) != 0x07) {
        write_reg(regs, 0x04, cmd | 0x07);
        printf("  > Bus Master Enabled.\n");
    }

    // 2. 配置 Region 0
    write_reg(regs, PCIE_ATU_VIEWPORT, 0x00000000); // Select Region 0 (Outbound)
    write_reg(regs, PCIE_ATU_LBAR, SRC_ADDR_LO);
    write_reg(regs, PCIE_ATU_UBAR, SRC_ADDR_HI);
    write_reg(regs, PCIE_ATU_LTAR, TGT_ADDR_LO);
    write_reg(regs, PCIE_ATU_UTAR, TGT_ADDR_HI);
    write_reg(regs, PCIE_ATU_LIMIT, 0x7FFFFFFF); // 1GB Limit
    write_reg(regs, PCIE_ATU_CTRL1, 0x0);        // MEM Type
    write_reg(regs, PCIE_ATU_CTRL2, 0x80000000); // Enable

    usleep(1000);
    
    // 检查是否成功
    write_reg(regs, PCIE_ATU_VIEWPORT, 0x00000000);
    uint32_t check = read_reg(regs, PCIE_ATU_CTRL2);
    
    munmap(dbi_map, 4096);
    close(fd);

    if (check != 0x80000000) {
        fprintf(stderr, "  [Error] ATU Configuration Failed! (Reg=0x%x)\n", check);
        return -1;
    }
    printf("  > ATU Configured Successfully.\n");
    return 0;
}

// ================== 主程序 ==================
int main() {
    // 1. 先配置 PCIe 硬件
    if (configure_pcie_atu() < 0) return -1;

    // 2. 映射数据传输内存
    int fd = open("/dev/pcie_wc0", O_RDWR | O_SYNC);
    if (fd < 0) { perror("Open /dev/pcie_wc0 (WC Data) failed"); return -1; }
    
    // 映射两块连续 ring_buffer（Ring0=推理流，Ring1=落盘流）
    // 注意：/dev/pcie_wc0 由内核模块按 phys 参数固定映射，因此 offset 必须为 0
    void *map_base = mmap(NULL, MAP_LEN_TOTAL, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map_base == MAP_FAILED) { perror("mmap Data failed"); return -1; }
    volatile struct ring_buffer_hdr *ring0 = (volatile struct ring_buffer_hdr *)map_base;
    volatile uint32_t *ring0_data = (volatile uint32_t *)((uint8_t*)map_base + sizeof(struct ring_buffer_hdr));
    volatile struct ring_buffer_hdr *ring1 = (volatile struct ring_buffer_hdr *)((uint8_t*)map_base + MAP_LEN0 + MAP_GUARD);
    volatile uint32_t *ring1_data = (volatile uint32_t *)((uint8_t*)map_base + MAP_LEN0 + MAP_GUARD + sizeof(struct ring_buffer_hdr));

    // 3. 链路连通性测试 (自检)
    printf("[Init] Testing PCIe Link...\n");
    // 尝试读取 Magic，如果是上次残留的也没关系，我们只是确认能读
    volatile uint32_t test_read = ring0->head; 
    printf("  > Link Alive. Current Head: %u\n", test_read);

    // 4. 重置 DPU 端的指针 (Host 端也会重置，双重保险)
    ring0->head = 0;
    ring0->tail = 0;
    ring1->head = 0;
    ring1->tail = 0;
    // WC 映射下：初始化时写指针清零后做一次 store barrier
    asm volatile("dsb st" ::: "memory");

    // 5. 初始化 UDP
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { perror("Socket failed"); return -1; }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) { perror("Bind failed"); return -1; }
    
    // 6. 绑定 CPU 核心 (优化)
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(1, &mask); // Core 1
    sched_setaffinity(0, sizeof(mask), &mask);

    printf("=============================================\n");
    printf("   DPU Integrated Sender Ready (Port %d)\n", UDP_PORT);
    printf("=============================================\n");

    uint8_t *packet_buf = malloc(UDP_PACKET_SIZE);
    socklen_t len_cli = sizeof(cliaddr);
    int frame_count = 0;

    while (1) {
        int n = recvfrom(sockfd, packet_buf, UDP_PACKET_SIZE, 0, (struct sockaddr *)&cliaddr, &len_cli);
        if (n <= 0) continue;

        uint32_t data_len = n;
        uint32_t packet_total_size = sizeof(struct frame_header) + data_len;
        uint32_t aligned_size = (packet_total_size + 3) & ~3;

        // Ring0: 推理流（优先保证）
        uint32_t head0 = ring0->head;
        uint32_t tail0 = ring0->tail;
        uint32_t used0 = head0 - tail0;
        uint32_t free0 = R0_SIZE - used0;

        // Ring1: 落盘流（可容忍丢帧）
        uint32_t head1 = ring1->head;
        uint32_t tail1 = ring1->tail;
        uint32_t used1 = head1 - tail1;
        uint32_t free1 = R1_SIZE - used1;

        if (free0 > aligned_size + 128) {
            struct frame_header hdr;
            hdr.magic = FRAME_MAGIC;
            hdr.frame_len = data_len;
            hdr.reserved1 = 0;
            hdr.reserved2 = 0;

            // 写入 Ring0
            safe_write_pcie(ring0_data, R0_SIZE, head0 & R0_MASK, &hdr, sizeof(hdr));
            safe_write_pcie(ring0_data, R0_SIZE, (head0 + sizeof(hdr)) & R0_MASK, packet_buf, data_len);

            // 同步写入 Ring1（如果空间不足则跳过，不阻塞推理流）
            if (free1 > aligned_size + 128) {
                safe_write_pcie(ring1_data, R1_SIZE, head1 & R1_MASK, &hdr, sizeof(hdr));
                safe_write_pcie(ring1_data, R1_SIZE, (head1 + sizeof(hdr)) & R1_MASK, packet_buf, data_len);
            }

            // WC 映射下：发布 head 前使用 store barrier，保证数据写入先于指针可见
            asm volatile("dsb st" ::: "memory");
            ring0->head = head0 + aligned_size;
            if (free1 > aligned_size + 128) ring1->head = head1 + aligned_size;
            
            frame_count++;
            if (frame_count % 100 == 0) printf("."); // 心跳
        }
    }

    close(sockfd);
    close(fd);
    return 0;
}
