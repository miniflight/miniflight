#include <windows.h>
#include <stdint.h>
#include <string.h>
#include "snapshot.h"

static BOOL relocate(uint64_t marker, uint64_t value) {
    for (size_t i = 0; i + 8 <= sizeof(stub); ++i) {
        if (memcmp(stub+i, &marker, 8) != 0) continue;
        memcpy(stub+i, &value, 8);
        return TRUE;
    }
    return FALSE;
}

BOOL WINAPI DllMain(HINSTANCE dll, DWORD reason, LPVOID reserved) {
    (void)dll;
    (void)reserved;
    if (reason != DLL_PROCESS_ATTACH) return TRUE;
    uintptr_t base = (uintptr_t)GetModuleHandleW(NULL);
    unsigned char *site = (void *)(base + 0x13a0959);
    const unsigned char original[] = {0xe8, 0x22, 0xd0, 0xca, 0xff};
    if (memcmp(site, original, sizeof(original)) != 0) return FALSE;
    wchar_t path[4096];
    DWORD length = GetEnvironmentVariableW(L"MINIFLIGHT_IMU_PATH", path, ARRAYSIZE(path));
    if (!length || length >= ARRAYSIZE(path)) return FALSE;
    HANDLE file = CreateFileW(path, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                              NULL, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, NULL);
    if (file == INVALID_HANDLE_VALUE) return FALSE;
    HANDLE mapping = CreateFileMappingW(file, NULL, PAGE_READWRITE, 0, 32+32768*56, NULL);
    CloseHandle(file);
    if (!mapping) return FALSE;
    unsigned char *buffer = MapViewOfFile(mapping, FILE_MAP_WRITE, 0, 0, 0);
    CloseHandle(mapping);
    if (!buffer) return FALSE;
    void *code = NULL;
    for (uintptr_t offset = 0x10000000; offset < 0x70000000 && !code; offset += 0x1000000)
        code = VirtualAlloc((void *)(base+offset), 4096, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    if (!code || !relocate(0x1122334455667788, base+0x104d980) ||
        !relocate(0x8877665544332211, (uintptr_t)buffer) ||
        !relocate(0x1234567890abcdef, (uintptr_t)QueryPerformanceCounter)) return FALSE;
    memcpy(code, stub, sizeof(stub));
    DWORD protection;
    if (!VirtualProtect(code, 4096, PAGE_EXECUTE_READ, &protection) ||
        !VirtualProtect(site, sizeof(original), PAGE_EXECUTE_READWRITE, &protection)) return FALSE;
    int32_t relative = (intptr_t)code - (intptr_t)(site+5);
    memcpy(site+1, &relative, sizeof(relative));
    DWORD ignored;
    VirtualProtect(site, sizeof(original), protection, &ignored);
    FlushInstructionCache(GetCurrentProcess(), NULL, 0);
    LARGE_INTEGER frequency;
    QueryPerformanceFrequency(&frequency);
    memcpy(buffer+16, &frequency.QuadPart, 8);
    return TRUE;
}
