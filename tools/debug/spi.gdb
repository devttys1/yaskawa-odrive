# Is the motion controller actually polling us?
#
#   ./tools/debug/gdb.sh -f tools/debug/spi.gdb
#
# g_nss_count is the decisive number: 0 means no chip-select edge has ever
# arrived, so the controller is not talking to this pin at all.
monitor halt
printf "NSS (CS) edges seen : %u\n", g_nss_count
printf "words staged        : %u\n", g_tx_count
printf "SPI4 SR=0x%04x  OVR=%d FRLVL=%d BSY=%d\n", ((SPI_TypeDef*)0x40013400)->SR, (((SPI_TypeDef*)0x40013400)->SR>>6)&1, (((SPI_TypeDef*)0x40013400)->SR>>9)&3, (((SPI_TypeDef*)0x40013400)->SR>>7)&1
printf "PE4 (NSS) level now : %d   PE2 (SCK) level: %d\n", (((GPIO_TypeDef*)0x40021000)->IDR>>4)&1, (((GPIO_TypeDef*)0x40021000)->IDR>>2)&1
detach
