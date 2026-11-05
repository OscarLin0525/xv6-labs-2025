#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "e1000_dev.h"

#define TX_RING_SIZE 16
static struct tx_desc tx_ring[TX_RING_SIZE] __attribute__((aligned(16)));

#define RX_RING_SIZE 16
static struct rx_desc rx_ring[RX_RING_SIZE] __attribute__((aligned(16)));

// remember where the e1000's registers live.
static volatile uint32 *regs;

struct spinlock e1000_lock;

// called by pci_init().
// xregs is the memory address at which the
// e1000's registers are mapped.
// this code loosely follows the initialization directions
// in Chapter 14 of Intel's Software Developer's Manual.
void
e1000_init(uint32 *xregs)
{
  int i;

  initlock(&e1000_lock, "e1000");

  regs = xregs;

  // Reset the device
  regs[E1000_IMC] = 0xffffffff; // disable interrupts
  regs[E1000_CTL] |= E1000_CTL_RST;
  regs[E1000_IMC] = 0xffffffff; // redisable interrupts
  __sync_synchronize();

  // [E1000 14.5] Transmit initialization
  memset(tx_ring, 0, sizeof(tx_ring));
  for (i = 0; i < TX_RING_SIZE; i++) {
    tx_ring[i].status = E1000_TXD_STAT_DD;
    tx_ring[i].addr = 0;
  }
  regs[E1000_TDBAL] = (uint64) tx_ring;
  if(sizeof(tx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_TDLEN] = sizeof(tx_ring);
  regs[E1000_TDH] = regs[E1000_TDT] = 0;
  
  // [E1000 14.4] Receive initialization
  memset(rx_ring, 0, sizeof(rx_ring));
  for (i = 0; i < RX_RING_SIZE; i++) {
    rx_ring[i].addr = (uint64) kalloc();
    if (!rx_ring[i].addr)
      panic("e1000");
  }
  regs[E1000_RDBAL] = (uint64) rx_ring;
  if(sizeof(rx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_RDH] = 0;
  regs[E1000_RDT] = RX_RING_SIZE - 1;
  regs[E1000_RDLEN] = sizeof(rx_ring);

  // filter by qemu's MAC address, 52:54:00:12:34:56
  regs[E1000_RA] = 0x12005452;
  regs[E1000_RA+1] = 0x5634 | (1<<31);
  // multicast table
  for (int i = 0; i < 4096/32; i++)
    regs[E1000_MTA + i] = 0;

  // transmitter control bits.
  regs[E1000_TCTL] = E1000_TCTL_EN |  // enable
    E1000_TCTL_PSP |                  // pad short packets
    (0x10 << E1000_TCTL_CT_SHIFT) |   // collision stuff
    (0x40 << E1000_TCTL_COLD_SHIFT);
  regs[E1000_TIPG] = 10 | (8<<10) | (6<<20); // inter-pkt gap

  // receiver control bits.
  regs[E1000_RCTL] = E1000_RCTL_EN | // enable receiver
    E1000_RCTL_BAM |                 // enable broadcast
    E1000_RCTL_SZ_2048 |             // 2048-byte rx buffers
    E1000_RCTL_SECRC;                // strip CRC
  
  // ask e1000 for receive interrupts.
  regs[E1000_RDTR] = 0; // interrupt after every received packet (no timer)
  regs[E1000_RADV] = 0; // interrupt after every packet (no timer)
  regs[E1000_IMS] = (1 << 7); // RXDW -- Receiver Descriptor Write Back
}

int
e1000_transmit(char *buf, int len)
{
  //
  // Your code here.
  //
  // buf contains an ethernet frame; program it into
  // the TX descriptor ring so that the e1000 sends it. Stash
  // a pointer so that it can be freed after send completes.
  //
  // return 0 on success.
  // return -1 on failure (e.g., there is no descriptor available)
  // so that the caller knows to free buf.
  
  // RAM--DMA_read->NIC--->Network
  // 本作業的傳送 buffer 由 kalloc() 配置一頁。
  if (buf == 0 || len <= 0 || len > PGSIZE) {
      return -1;
  }
  // 保護 TX ring 的檢查與後續修改。
  acquire(&e1000_lock);
  // 找下一個要填入傳送工作的 descriptor。
  int i = regs[E1000_TDT];
  int next = (i+1)%TX_RING_SIZE;
  if(next == regs[E1000_TDH]) {
    release(&e1000_lock);
    return -1;
  }
  struct tx_desc *desc = &tx_ring[i];
  // 前一次工作還沒完成，不能覆寫。
  if((desc->status & E1000_TXD_STAT_DD) == 0) {
    release(&e1000_lock);
    return -1;
  }
  // 已確認前一次工作完成 DD＝1，NIC 不再需要這格的舊傳送 buffer。
  if(desc->addr != 0) {
    kfree((char *)desc->addr);
    desc->addr = 0;
  }
  // 在 RAM 裡準備工作內容。
  desc->addr = (uint64)buf;
  desc->length = len;
  // 本作業每個 TX descriptor 對應一個完整 frame。
  // EOP：這個 buffer 是封包的最後一段。
  // RS：要求 NIC 寫回完成狀態。
  desc->cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS; //driver
  desc->status = 0;
  // RISC-V：確保先前的記憶體／I/O 存取，排在後面的 TDT 通知之前。
  asm volatile ("fence iorw, iorw" ::: "memory");
  //寫 NIC register，告訴它有新工作。 descriptor i 已準備好，將 TX tail 推進到下一格。
  regs[E1000_TDT] = next;
  release(&e1000_lock);
  return 0;
}

static void
e1000_recv(void)
{
  //
  // Your code here.
  //
  // Check for packets that have arrived from the e1000
  // Create and deliver a buf for each packet (using net_rx()).
  
  // Network--->NIC--DMA_write->RAM
  //記錄「目前是否正在丟棄一個跨 buffer 的封包」
  static int dropping_packets = 0; //static補充1. 名稱只"持續"存在這個function裡面 2.=0是初始化並非每次都賦值0
  static char* assembly_buf = 0; //driver copy完整的封包存放在這額外空間
  static uint assembly_len = 0; //完整封包有效長度
  while(1) {
    acquire(&e1000_lock);
    // 找下一個要檢查的 RX descriptor。
    int i = (regs[E1000_RDT]+1)%RX_RING_SIZE;
    // 指向 ring 裡的原始 descriptor，沒有複製它。
    struct rx_desc *desc = &rx_ring[i];
    // NIC 尚未宣告完成，就結束這次DD檢查。
    if((desc->status & E1000_RXD_STAT_DD) == 0) {
      release(&e1000_lock);
      break;
    }
    asm volatile ("fence iorw, iorw" ::: "memory");
    //EOP 檢查
    int eop = desc->status & E1000_RXD_STAT_EOP;
    if(dropping_packets) {
      if (eop) {
        // 正在丟棄的封包，終於到結尾(EOP==1)了。
        dropping_packets = 0;
      }
      desc->status = 0;
      asm volatile("fence iorw, iorw" ::: "memory");
      regs[E1000_RDT] = i;
      release(&e1000_lock);
      continue;
    }

    int frame_error = eop && (desc->errors & E1000_RXD_FRAME_ERR_MASK);

    // 保存本次接收的 buffer 位址與長度。
    int length = desc->length;
    char *packet = (char *)desc->addr; 
    // 下一步：驗證接收結果、準備替代 buffer，再回收 descriptor。
    // 配置下一次接收要用的 buffer。
    if(frame_error || length==0 || length > 2048 || length > PGSIZE - assembly_len) {
      if(assembly_buf != 0) {
        kfree(assembly_buf);
        assembly_buf = 0;
      }
      assembly_len = 0;
      desc->status = 0;
      dropping_packets = !eop;
      asm volatile("fence iorw, iorw" ::: "memory");
      regs[E1000_RDT] = i;
      release(&e1000_lock);
      continue;
    }

    if(assembly_buf == 0) {
      assembly_buf = kalloc();
      if(assembly_buf == 0) {
        assembly_len = 0;
        dropping_packets = !eop;
        desc->status = 0;
        asm volatile ("fence iorw, iorw" ::: "memory");
        regs[E1000_RDT] = i;
        release(&e1000_lock);
        continue;
      }
    }
      // 前面的長度檢查已確認剩餘容量足夠。
    memmove(assembly_buf + assembly_len, packet, length);
    assembly_len += length;

    // 若整包完成，先保存交付資料，再清掉組包狀態。
    char *complete_packet = 0;
    uint complete_len = 0;

    if (eop) {
      complete_packet = assembly_buf;
      complete_len = assembly_len;

      assembly_buf = 0;
      assembly_len = 0;
    }

    // 複製完成後，原 RX buffer 就可以回收重用。
    // desc->addr 保持不變。
    desc->status = 0;
    asm volatile("fence iorw, iorw" ::: "memory");
    regs[E1000_RDT] = i;

    release(&e1000_lock);

    // 只有完整封包才交給上層，而且不能持有這把鎖。
    if (complete_packet != 0) {
      net_rx(complete_packet, complete_len);
    }
  }
}

void
e1000_intr(void)
{
  // tell the e1000 we've seen this interrupt;
  // without this the e1000 won't raise any
  // further interrupts.
  regs[E1000_ICR] = 0xffffffff;

  e1000_recv();
}
