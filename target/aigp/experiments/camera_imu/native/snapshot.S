.intel_syntax noprefix
.text
.equ published_count, 0
.equ clock_request, 4
.equ clock_ack, 8
.equ clock_sample, 24

# Wrap the snapshot writer with the same Windows x64 arguments.
sub rsp, 0x58
mov rax, [rsp+0x80]
mov [rsp+0x20], rax
mov rax, [rsp+0x88]
mov [rsp+0x28], rax
mov [rsp+0x30], rax
mov [rsp+0x38], rcx
call [rip+writer]
mov [rsp+0x40], rax

# The original call has published a completed entry. Append only sensor data.
mov r10, [rip+buffer]
mov eax, [r10+published_count]
cmp eax, 32768
jae done
mov rcx, [rsp+0x38]
mov edx, [rcx+0x38]
cmp edx, 7
ja done
imul eax, eax, 56
lea r8, [r10+rax+32]
mov [r8+8], rcx
mov [r8+20], edx
imul edx, edx, 0x90
lea r9, [rcx+rdx+0x40]
mov rax, [r9]
mov [r8], rax
mov eax, [rsp+0x30]
mov [r8+16], eax
mov rax, [r9+0x44]
mov [r8+24], rax
mov eax, [r9+0x4c]
mov [r8+32], eax
mov rax, [r9+0x20]
mov [r8+36], rax
mov eax, [r9+0x28]
mov [r8+44], eax
mov [rsp+0x48], r8
lea rcx, [r8+48]
call [rip+counter]
mov r8, [rsp+0x48]
mov r10, [rip+buffer]
mov eax, [r10+clock_request]
cmp eax, [r10+clock_ack]
je publish_record
mov rcx, [r8+48]
mov [r10+clock_sample], rcx
mov [r10+clock_ack], eax
publish_record:
inc dword ptr [r10+published_count]
done:
mov rax, [rsp+0x40]
add rsp, 0x58
ret
.p2align 3
writer: .quad 0x1122334455667788
buffer: .quad 0x8877665544332211
counter: .quad 0x1234567890abcdef
