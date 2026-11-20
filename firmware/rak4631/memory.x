/* RAK4631 with RAK's stock bootloader: the Adafruit nRF52 UF2 bootloader
 * (RAK build 0.4.x) with Nordic's MBR and SoftDevice S140 6.1.1.
 *
 * Flash (sources in README.md, "Memory layout"):
 *   0x00000 - 0x00FFF  MBR
 *   0x01000 - 0x25FFF  SoftDevice S140 6.1.1 (present, never enabled here)
 *   0x26000 - 0xECFFF  this application (where the bootloader jumps)
 *   0xED000 - 0xF3FFF  left free (the Arduino core keeps a file system here)
 *   0xF4000 - 0xFFFFF  bootloader, its config, MBR params, bootloader settings
 *
 * RAM: the bootloader keeps its double-reset flag at 0x20007F7C and a
 * no-init block at 0x20007F80. The application starts at 0x20008000, above
 * them and above any MBR or SoftDevice use. It needs only a few kB.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00026000, LENGTH = 0xED000 - 0x26000
  RAM   : ORIGIN = 0x20008000, LENGTH = 0x20040000 - 0x20008000
}
