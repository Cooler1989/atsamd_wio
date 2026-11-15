/* Development layout: firmware at 0x0, flashed over SWD (J-Link), no bootloader. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 512K
  RAM   (rxw) : ORIGIN = 0x20000000, LENGTH = 192K
}
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
