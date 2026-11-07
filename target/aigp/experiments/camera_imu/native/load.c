#include <windows.h>
#include <tlhelp32.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>


static void check(int ok, const char *operation) {
    if (ok) return;
    fprintf(stderr, "%s failed (%lu)\n", operation, GetLastError());
    exit(1);
}

static DWORD find_sim(void) {
    HANDLE list = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    check(list != INVALID_HANDLE_VALUE, "process list");
    PROCESSENTRY32 entry = {.dwSize = sizeof(entry)};
    DWORD pid = 0;
    for (BOOL more = Process32First(list, &entry); more; more = Process32Next(list, &entry)) {
        if (_stricmp(entry.szExeFile, "DCGame-Win64-Shipping.exe") != 0) continue;
        check(!pid, "multiple simulator processes");
        pid = entry.th32ProcessID;
    }
    CloseHandle(list);
    return pid;
}

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    DWORD pid = 0;
    for (int i = 0; i < 200 && !pid; ++i) {
        pid = find_sim();
        if (!pid) Sleep(100);
    }
    check(pid != 0, "find owned simulator");
    FARPROC load = GetProcAddress(GetModuleHandleA("kernel32.dll"), "LoadLibraryA");
    HMODULE owner;
    check(load && GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                    (const char *)load, &owner), "resolve loader");
    char name[MAX_PATH];
    check(GetModuleFileNameA(owner, name, sizeof(name)) > 0, "resolve loader module");
    char *short_name = strrchr(name, 92);
    short_name = short_name ? short_name + 1 : name;
    HANDLE modules = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, pid);
    check(modules != INVALID_HANDLE_VALUE, "simulator modules");
    MODULEENTRY32 entry = {.dwSize = sizeof(entry)};
    uintptr_t remote_load = 0;
    for (BOOL more = Module32First(modules, &entry); more; more = Module32Next(modules, &entry)) {
        if (_stricmp(entry.szModule, short_name) == 0)
            remote_load = (uintptr_t)entry.modBaseAddr + (uintptr_t)load - (uintptr_t)owner;
    }
    check(remote_load != 0, "find remote loader");
    HANDLE sim = OpenProcess(PROCESS_CREATE_THREAD | PROCESS_QUERY_INFORMATION | PROCESS_VM_OPERATION |
                             PROCESS_VM_READ | PROCESS_VM_WRITE, FALSE, pid);
    check(sim != NULL, "open owned simulator");
    size_t length = strlen(argv[1]) + 1;
    void *path = VirtualAllocEx(sim, NULL, length, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    check(path && WriteProcessMemory(sim, path, argv[1], length, NULL), "copy probe path");
    HANDLE thread = CreateRemoteThread(sim, NULL, 0, (LPTHREAD_START_ROUTINE)remote_load, path, 0, NULL);
    check(thread && WaitForSingleObject(thread, 25000) == WAIT_OBJECT_0, "load probe");
    DWORD result;
    check(GetExitCodeThread(thread, &result) && result, "probe initialization");
    VirtualFreeEx(sim, path, 0, MEM_RELEASE);
    return 0;
}
