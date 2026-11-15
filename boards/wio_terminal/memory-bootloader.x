/* Production layout: keep the first 16k for the Wio Terminal UF2 bootloader. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000 + 16K, LENGTH = 512K - 16K
  RAM   (rxw) : ORIGIN = 0x20000000, LENGTH = 192K
}
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
