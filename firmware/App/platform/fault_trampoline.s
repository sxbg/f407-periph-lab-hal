; fault_trampoline.s -- HardFault handler takeover (armasm, Thumb-2).
; Strong symbol replaces the CubeMX-generated C HardFault_Handler in
; Core/Src/stm32f4xx_it.c (that C copy must be deleted; see bringup-log).
; The exception frame is already stacked; hand the SP to the C handler
; (MSP or PSP per EXC_RETURN bit 2). ALIGN=3 (8B) avoids armasm A1581W.
    AREA    |.text|, CODE, READONLY, ALIGN=3
    THUMB
    EXPORT  HardFault_Handler
    IMPORT  fault_capture_handler_c

HardFault_Handler
    tst     lr, #4
    ite     eq
    mrseq   r0, msp
    mrsne   r0, psp
    ldr     r1, =fault_capture_handler_c
    bx      r1
    ALIGN
    LTORG

    END
