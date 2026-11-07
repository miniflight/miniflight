#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CAPACITY 65536
typedef struct { uint64_t d[16]; unsigned char raw[384]; } Record;
static uintptr_t base;
static uint64_t *header;
static Record *records;
static void (*capture_original)(void *, void *);
static uint64_t (*epoch_original)(void);
static unsigned char (*read_original)(void *, void *, void *, void *);
static void *(*resource_original)(void *);
static void (*surface_original)(void *, void *);
static void (*render_original)(void *, void *);
typedef void *(*CreateView)(uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t,uint64_t);
static CreateView create_original;
static volatile LONG64 serial;
static uint64_t current_capture;

static Record *begin(unsigned type, void *object, void *other) {
    LONG64 i = InterlockedIncrement64((LONG64 *)header) - 1;
    if (i >= CAPACITY) return NULL;
    Record *r = records + i;
    r->d[0] = type;
    r->d[1] = GetCurrentThreadId();
    QueryPerformanceCounter((LARGE_INTEGER *)&r->d[2]);
    r->d[3] = epoch_original();
    QueryPerformanceCounter((LARGE_INTEGER *)&r->d[4]);
    memcpy(r->d+5, (void *)(base+0x5320bf8), 8);
    memcpy(r->d+6, (void *)(base+0x537fb98), 8);
    r->d[7] = (uintptr_t)object;
    r->d[8] = (uintptr_t)other;
    return r;
}
static void commit(Record *r) {
    if (r) InterlockedExchange64((LONG64 *)&r->d[15], 1);
}
static void capture(void *object, void *scene) {
    header[7] = *(uintptr_t *)scene;
    header[8] = (*(uintptr_t **)scene)[0x118/8];
    uint64_t id = InterlockedIncrement64(&serial);
    current_capture = id;
    Record *r = begin(1, object, *(void **)((char *)object+0x2c0));
    if (r) {
        r->d[11] = id;
        memcpy(r->raw, (char *)object+0x100, sizeof(r->raw));
    }
    commit(r);
    capture_original(object, scene);
    r = begin(2, object, scene);
    if (r) r->d[11] = id;
    commit(r);
}
static unsigned char readback(void *resource, void *array, void *flags, void *rect) {
    Record *r = begin(3, resource, array);
    uint64_t stamp = *(uint64_t *)((char *)array-8);
    uint32_t frame = *(uint32_t *)((char *)array-16);
    if (r) { r->d[9]=stamp; r->d[10]=frame; }
    commit(r);
    unsigned char ok = read_original(resource, array, flags, rect);
    r = begin(4, resource, array);
    if (r) {
        r->d[9]=stamp; r->d[10]=frame; r->d[13]=ok;
        memcpy(r->raw, array, 16);
    }
    commit(r);
    return ok;
}
static void *resource(void *object) {
    void *result = resource_original(object);
    Record *r = begin(6, object, result);
    commit(r);
    return result;
}
static uint64_t camera_stamp(void) {
    LARGE_INTEGER before,after;
    QueryPerformanceCounter(&before);
    uint64_t value=epoch_original();
    QueryPerformanceCounter(&after);
    Record *r=begin(5,NULL,NULL);
    if (r) {
        r->d[2]=before.QuadPart; r->d[3]=value; r->d[4]=after.QuadPart;
        r->d[9]=value*100;
    }
    commit(r);
    return value;
}
static void surface(void *command, void *rhi) {
    Record *r = begin(7, command, rhi);
    if (r) memcpy(r->raw, command, 64);
    commit(r);
    surface_original(command, rhi);
}
static void render(void *command, void *rhi) {
    Record *r=begin(10,command,rhi);
    if (r) memcpy(r->raw,command,64);
    commit(r);
    uint64_t renderer=*(uint64_t *)command, target=((uint64_t *)command)[1];
    render_original(command,rhi);
    r=begin(11,(void *)renderer,(void *)target);
    commit(r);
}
static void *create_view(uint64_t a,uint64_t b,uint64_t c,uint64_t d,uint64_t e,uint64_t f,uint64_t g,
                         uint64_t h,uint64_t i,uint64_t j,uint64_t k,uint64_t l,uint64_t m) {
    Record *r=begin(8,(void *)b,(void *)c);
    if (r) {
        r->d[11]=current_capture;
        memcpy(r->raw,(void *)f,12);
        memcpy(r->raw+16,(void *)e,64);
        memcpy(r->raw+80,(void *)g,64);
    }
    commit(r);
    void *renderer=create_original(a,b,c,d,e,f,g,h,i,j,k,l,m);
    r=begin(9,renderer,(void *)c);
    if (r) r->d[11]=current_capture;
    commit(r);
    return renderer;
}
static BOOL patch_call(uintptr_t offset, uintptr_t expected, void *hook, unsigned char **next) {
    unsigned char *site = (void *)(base+offset);
    if (site[0] != 0xe8 || (uintptr_t)(site+5+*(int32_t *)(site+1)) != base+expected) return FALSE;
    unsigned char stub[12] = {0x48,0xb8,0,0,0,0,0,0,0,0,0xff,0xe0};
    memcpy(stub+2, &hook, 8);
    memcpy(*next, stub, 12);
    DWORD old;
    if (!VirtualProtect(site, 5, PAGE_EXECUTE_READWRITE, &old)) return FALSE;
    int32_t delta = *next-(site+5);
    memcpy(site+1, &delta, 4);
    DWORD ignored;
    VirtualProtect(site, 5, old, &ignored);
    *next += 16;
    return TRUE;
}
static void *patch_entry(uintptr_t offset,const unsigned char *expected,void *hook,unsigned char **next) {
    unsigned char *site=(void *)(base+offset);
    if (memcmp(site,expected,7)) return NULL;
    unsigned char *original=*next;
    memcpy(original,expected,7); original[7]=0xe9;
    int32_t back=(site+7)-(original+12);
    memcpy(original+8,&back,4); *next+=16;
    unsigned char stub[]={0x48,0xb8,0,0,0,0,0,0,0,0,0xff,0xe0};
    memcpy(stub+2,&hook,8); memcpy(*next,stub,12);
    DWORD old;
    if (!VirtualProtect(site,7,PAGE_EXECUTE_READWRITE,&old)) return NULL;
    int32_t delta=*next-(site+5);
    site[0]=0xe9; memcpy(site+1,&delta,4); site[5]=site[6]=0x90;
    DWORD ignored; VirtualProtect(site,7,old,&ignored); *next+=16;
    return original;
}
BOOL WINAPI DllMain(HINSTANCE dll, DWORD reason, LPVOID reserved) {
    (void)dll; (void)reserved;
    if (reason != DLL_PROCESS_ATTACH) return TRUE;
    base = (uintptr_t)GetModuleHandleW(NULL);
    wchar_t path[4096], input[4096];
    if (!GetEnvironmentVariableW(L"MINIFLIGHT_CAPTURE_DATA",path,4096) ||
        !GetEnvironmentVariableW(L"MINIFLIGHT_CAPTURE_OBJECT",input,4096)) return FALSE;
    FILE *f = _wfopen(input,L"r");
    unsigned long long address=0;
    if (!f || fscanf(f,"%llx",&address)!=1) return FALSE;
    fclose(f);
    uintptr_t *vtable=*(uintptr_t **)address;
    if (vtable[0x548/8]<base || vtable[0x548/8]>base+0x3e00000) return FALSE;
    HANDLE file=CreateFileW(path,GENERIC_READ|GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,NULL,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,NULL);
    if (file==INVALID_HANDLE_VALUE) return FALSE;
    HANDLE map=CreateFileMappingW(file,NULL,PAGE_READWRITE,0,4096+CAPACITY*sizeof(Record),NULL);
    CloseHandle(file);
    if (!map) return FALSE;
    header=MapViewOfFile(map,FILE_MAP_WRITE,0,0,0); CloseHandle(map);
    if (!header) return FALSE;
    records=(Record *)((char *)header+4096);
    QueryPerformanceFrequency((LARGE_INTEGER *)(header+1));
    header[2]=base; header[3]=address; header[4]=(uintptr_t)vtable;
    header[5]=vtable[0x548/8]; header[6]=sizeof(Record);
    memcpy(header+32,vtable,0x580);
    capture_original=(void *)header[5];
    epoch_original=(void *)(base+0x3dc28ac);
    read_original=(void *)(base+0x3414db0);
    resource_original=(void *)(base+0x33e95e0);
    create_original=(void *)(base+0x2018300);
    unsigned char *code=NULL;
    for (uintptr_t o=0x10000000;o<0x70000000 && !code;o+=0x1000000)
        code=VirtualAlloc((void *)(base+o),4096,MEM_RESERVE|MEM_COMMIT,PAGE_EXECUTE_READWRITE);
    if (!code) return FALSE;
    unsigned char *next=code;
    if (!patch_call(0x1050d46,0x3414db0,readback,&next) ||
        !patch_call(0x1050c88,0x33e95e0,resource,&next) ||
        !patch_call(0x1050cbb,0x3dc28ac,camera_stamp,&next) ||
        !patch_call(0x202d85f,0x2018300,create_view,&next)) return FALSE;
    const unsigned char surface_bytes[]={0x48,0x8b,0xc4,0x48,0x89,0x58,0x08};
    const unsigned char render_bytes[]={0x4c,0x8b,0xdc,0x49,0x89,0x5b,0x18};
    surface_original=patch_entry(0x33fbdf0,surface_bytes,surface,&next);
    render_original=patch_entry(0x2011770,render_bytes,render,&next);
    if (!surface_original || !render_original) return FALSE;
    DWORD old;
    VirtualProtect(code,4096,PAGE_EXECUTE_READ,&old);
    if (!VirtualProtect(vtable+0x548/8,8,PAGE_READWRITE,&old)) return FALSE;
    InterlockedExchange64((LONG64 *)(vtable+0x548/8),(LONG64)capture);
    DWORD ignored; VirtualProtect(vtable+0x548/8,8,old,&ignored);
    FlushInstructionCache(GetCurrentProcess(),NULL,0);
    return TRUE;
}
