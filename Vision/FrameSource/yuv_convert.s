
// This is a reorder of bytes using pshufb to convert 4 pixels
// from interlace YUV to deinterlaced YUV
yuv_order1:
    .byte 0, 3, 1, 2, 3, 1, 4, 7, 5, 6, 7, 5
    .byte 0, 0, 0, 0

yuv_order2:
    .byte 8, 11, 9, 10, 11, 9, 12, 15, 13, 14, 15, 13
    .byte 0, 0, 0, 0

// yuv_convert(char *input, char *output, int pixels)
// $rdi: source
// $rsi: destination
// $rdx: length (should be multiple of 8)
// NOTE:
// Output array block should be 4 bytes larger than the actual needed
// size to process
.globl yuv_convert
yuv_convert:
    // Saving registers
    pushq %rbp
    movq %rsp, %rbp
    pushq %rax
    pushq %rbx
    pushq %rdx
    pushq %rsi
    pushq %rdi

    // Loading order in xmm6 & 7
    movq yuv_order1@GOTPCREL(%rip), %rax
    movups (%rax), %xmm6
    movq yuv_order2@GOTPCREL(%rip), %rax
    movups (%rax), %xmm7

    // Getting the number of blocks
    shrq $3, %rdx 

    // Computing address of the last pixel in %rbx
    movq %rdi, %rbx
    movq %rdx, %rax
    imulq $16, %rax
    addq %rax, %rbx

loop: // Each loop is processing 8 pixels
    movups (%rdi), %xmm0
    movups %xmm0, %xmm1
    pshufb %xmm6, %xmm0
    pshufb %xmm7, %xmm1
    movups %xmm0, (%rsi)
    movups %xmm1, 12(%rsi)
    addq $16, %rdi
    addq $24, %rsi

    // Checking wether we reached the last pixel
    cmpq %rbx, %rdi
    jl loop

    // Restoring registers
    popq %rdi
    popq %rsi
    popq %rdx
    popq %rbx
    popq %rax
    popq %rbp
    ret
