 .text
 .global _start
_start:
 mov $0x80000000,%eax
 cpuid
 mov %eax,maxleaf(%rip)
 mov $0x80000006,%eax
 xor %ecx,%ecx
 cpuid
 mov %ecx,cacheinfo(%rip)
 movzbl %cl,%ecx
 cmp $64,%ecx
 jne fail
 cmpl $0x80000006,maxleaf(%rip)
 jb fail
 mov $1088,%ebx
 lea -1(%rcx),%eax
 add %ebx,%eax
 cltd
 idiv %ecx
 cmp $17,%eax
 jne fail
 lea memory(%rip),%rdi
 mov %rax,%r8
loop:
 prefetcht0 (%rdi)
 add %rcx,%rdi
 dec %eax
 jne loop
 lea ok(%rip),%rsi
 mov $(ok_end-ok),%edx
 xor %r12d,%r12d
 jmp output
fail:
 lea bad(%rip),%rsi
 mov $(bad_end-bad),%edx
 mov $1,%r12d
output:
 mov $1,%eax
 mov $1,%edi
 syscall
 mov %r12,%rdi
 mov $60,%eax
 syscall
 .data
ok: .ascii "CPUID max extended leaf, 64-byte cache line, original ceil/prefetch sequence passed\n"
ok_end:
bad: .ascii "CPUID cache metadata is missing or original prefetch sequence differs\n"
bad_end:
 .bss
 .balign 16
maxleaf: .skip 4
cacheinfo: .skip 4
memory: .skip 1152
