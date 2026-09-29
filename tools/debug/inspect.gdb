# Halts the target inside the decode path and dumps everything needed to see
# why a frame did or did not decode.  Run with:
#
#   ./tools/debug/gdb.sh -f tools/debug/inspect.gdb

monitor reset halt

# Stop once a full request/response cycle has completed and the frame struct
# is populated, rather than at some arbitrary point in the main loop.
break yask_rx_decode
continue

# Step out so the return value and the caller's frame are both visible.
finish

echo \n=== decoded frame ===\n
print/d f

echo \n=== raw capture, first 64 samples (RX bit is 1<<11) ===\n
# rx_samples is static in yaskawa_rx.c; qualify it so GDB finds it.
print/x 'yaskawa_rx.c'::rx_samples[0]@64

echo \n=== recovered data bits, first 64 ===\n
print/d 'yaskawa_rx.c'::rx_bits[0]@64

echo \n=== timer state ===\n
printf "TIM1 CR1=0x%04x ARR=%d DIER=0x%04x CNT=%d\n", \
       ((TIM_TypeDef*)0x40010000)->CR1, ((TIM_TypeDef*)0x40010000)->ARR, \
       ((TIM_TypeDef*)0x40010000)->DIER, ((TIM_TypeDef*)0x40010000)->CNT
printf "TIM8 CR1=0x%04x ARR=%d DIER=0x%04x CNT=%d\n", \
       ((TIM_TypeDef*)0x40010400)->CR1, ((TIM_TypeDef*)0x40010400)->ARR, \
       ((TIM_TypeDef*)0x40010400)->DIER, ((TIM_TypeDef*)0x40010400)->CNT

echo \n=== DMA2 stream state (S1 = RX, S5 = TX) ===\n
printf "S1 CR=0x%08x NDTR=%d PAR=0x%08x M0AR=0x%08x\n", \
       ((DMA_Stream_TypeDef*)0x40026428)->CR, \
       ((DMA_Stream_TypeDef*)0x40026428)->NDTR, \
       ((DMA_Stream_TypeDef*)0x40026428)->PAR, \
       ((DMA_Stream_TypeDef*)0x40026428)->M0AR
printf "S5 CR=0x%08x NDTR=%d PAR=0x%08x M0AR=0x%08x\n", \
       ((DMA_Stream_TypeDef*)0x40026488)->CR, \
       ((DMA_Stream_TypeDef*)0x40026488)->NDTR, \
       ((DMA_Stream_TypeDef*)0x40026488)->PAR, \
       ((DMA_Stream_TypeDef*)0x40026488)->M0AR

echo \n=== GPIOE mode/state ===\n
printf "MODER=0x%08x IDR=0x%04x ODR=0x%04x\n", \
       ((GPIO_TypeDef*)0x40021000)->MODER, \
       ((GPIO_TypeDef*)0x40021000)->IDR, \
       ((GPIO_TypeDef*)0x40021000)->ODR

detach
