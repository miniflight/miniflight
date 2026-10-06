typedef unsigned long long U64;
typedef long I64;
static long sc(long n,long a,long b,long c,long d,long e,long f){register long x10 __asm__("r10")=d;register long x8 __asm__("r8")=e;register long x9 __asm__("r9")=f;long r;__asm__ volatile("syscall":"=a"(r):"a"(n),"D"(a),"S"(b),"d"(c),"r"(x10),"r"(x8),"r"(x9):"rcx","r11","memory");return r;}
static void text(const char*s){U64 n=0;while(s[n])n++;sc(1,1,(long)s,n,0,0,0);}
static void hex(U64 v){char b[19]="0x0000000000000000\n";for(int i=0;i<16;i++){b[17-i]="0123456789abcdef"[v&15];v>>=4;}sc(1,1,(long)b,19,0,0,0);}
static U64 now(){long t[2]={0,0};sc(228,1,(long)t,0,0,0,0);return (U64)t[0]*1000000000ULL+t[1];}
static void path(char*out,U64 id){const char*p="/proc/";int n=0;while(*p)out[n++]=*p++;char digs[24];int k=0;do{digs[k++]='0'+id%10;id/=10;}while(id);while(k)out[n++]=digs[--k];p="/mem";while(*p)out[n++]=*p++;out[n]=0;}
static unsigned char pattern(U64 i){return (unsigned char)(i*37+11);}
static unsigned char result[8192];
static int check(long rc,long expected,const char*name){if(rc==expected)return 0;text(name);text(" got=");hex((U64)rc);return 1;}
static int bytes(U64 start,U64 n){for(U64 i=0;i<n;i++)if(result[i]!=pattern(start+i))return 0;return 1;}
long probe_main(){
 const U64 base=0x12345000ULL;unsigned char*mem=(void*)base;int ready[2],release[2];int failed=0;
 if(sc(9,base,12288,3,0x32,-1,0)!=(long)base)return 90;
 for(U64 i=0;i<8192;i++)mem[i]=0xa5;
 if(sc(11,base+8192,4096,0,0,0,0))return 91;
 if(sc(22,(long)ready,0,0,0,0,0)||sc(22,(long)release,0,0,0,0,0))return 92;
 long child=sc(56,17,0,0,0,0,0);if(child<0)return 93;
 if(!child){
  for(U64 i=0;i<8192;i++)mem[i]=pattern(i);*(volatile U64*)(mem+4096)=1;
  sc(10,base,4096,1,0,0,0);U64 info[3]={(U64)sc(39,0,0,0,0,0,0),(U64)sc(186,0,0,0,0,0,0),base};
  sc(1,ready[1],(long)info,sizeof(info),0,0,0);sc(72,release[0],4,0x800,0,0,0);
  char end;while(sc(0,release[0],(long)&end,1,0,0,0)<0){++*(volatile U64*)(mem+4096);sc(24,0,0,0,0,0,0);}
  sc(60,37,0,0,0,0,0);return 94;
 }
 U64 info[3]={0,0,0};if(sc(0,ready[0],(long)info,sizeof(info),0,0,0)!=sizeof(info))return 95;
 char name[48];path(name,(U64)child);long fd=sc(2,(long)name,0,0,0,0,0);text("PROC PID path ");text(name);text(" open=");hex(fd);
 if(fd<0){failed=1;goto finish;}
 failed|=check(sc(17,fd,(long)result,4096,base,0,0),4096,"whole4K");if(!bytes(0,4096)){text("whole4K bytes FAIL\n");failed=1;}
 for(U64 i=0;i<4096;i++)if(mem[i]!=0xa5){text("parent isolation FAIL\n");failed=1;break;}
 failed|=check(sc(8,fd,base+103,0,0,0,0),base+103,"seek");failed|=check(sc(0,fd,(long)result,73,0,0,0),73,"read73");if(!bytes(103,73))failed=1;
 failed|=check(sc(17,fd,(long)result,39,base+17,0,0),39,"pread39");if(!bytes(17,39))failed=1;failed|=check(sc(8,fd,0,1,0,0,0),base+176,"pread preserves position");
 for(int i=0;i<32;i++)result[i]=0xcc;
 failed|=check(sc(17,fd,(long)result,32,base+8176,0,0),16,"partial pread");if(!bytes(8176,16))failed=1;for(int i=16;i<32;i++)if(result[i]!=0xcc)failed=1;
 failed|=check(sc(8,fd,base+8176,0,0,0,0),base+8176,"seek partial");failed|=check(sc(0,fd,(long)result,32,0,0,0),16,"partial read");
 failed|=check(sc(17,fd,(long)result,1,base+8192,0,0),-5,"unmapped EIO");failed|=check(sc(8,fd,-1,0,0,0,0),-22,"negative seek");failed|=check(sc(17,fd,(long)result,1,-1,0,0),-22,"negative pread");
 failed|=check(sc(17,fd,(long)result,0,base+8192,0,0),0,"zero read");
 if(sc(2,(long)name,2,0,0,0,0)!=-13){text("write open denial FAIL\n");failed=1;}
 if(sc(1,fd,(long)result,1,0,0,0)>=0){text("write readonly denial FAIL\n");failed=1;}
 path(name,info[1]);long tidfd=sc(2,(long)name,0,0,0,0,0);if(tidfd<0)failed=1;else{failed|=check(sc(17,tidfd,(long)result,4096,base,0,0),4096,"TID whole4K");if(!bytes(0,4096))failed=1;sc(3,tidfd,0,0,0,0,0);}
 U64 c1=0,c2=0;failed|=check(sc(17,fd,(long)&c1,8,base+4096,0,0),8,"counter1");U64 deadline=now()+20000000;while(now()<deadline)sc(24,0,0,0,0,0,0);failed|=check(sc(17,fd,(long)&c2,8,base+4096,0,0),8,"counter2");if(c2<=c1){text("child continues FAIL\n");failed=1;}
 finish:;
 char end=1;sc(1,release[1],(long)&end,1,0,0,0);int status=0;failed|=check(sc(61,child,(long)&status,0,0,0,0),child,"wait child");if(status!=(37<<8))failed=1;
 if(fd>=0){failed|=check(sc(17,fd,(long)result,1,base,0,0),0,"exited target EOF");sc(3,fd,0,0,0,0,0);}
 text(failed?"PROC-MEM probe FAILED\n":"PROC-MEM: 4K, PID/TID, isolated memory, seek/pread, partial hole/EIO, readonly denial and live child passed\n");return failed;
}
__asm__(".global _start\n_start:\n mov %rsp,%rdi\n and $-16,%rsp\n call probe_main\n mov %rax,%rdi\n mov $60,%rax\n syscall\n");
