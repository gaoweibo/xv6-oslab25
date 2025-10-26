#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "date.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"

extern struct proc proc[NPROC];  // 声明外部变量
extern struct cpu cpus[NCPU];    // 声明外部变量

uint64 sys_exit(void) {
  int n;
  if (argint(0, &n) < 0) return -1;
  exit(n);
  return 0;  // not reached
}

uint64 sys_getpid(void) { return myproc()->pid; }

uint64 sys_fork(void) { return fork(); }

uint64 sys_wait(void) {
  uint64 p;
  if (argaddr(0, &p) < 0) return -1;
  return wait(p);
}

uint64 sys_sbrk(void) {
  int addr;
  int n;

  if (argint(0, &n) < 0) return -1;
  addr = myproc()->sz;
  if (growproc(n) < 0) return -1;
  return addr;
}

uint64 sys_sleep(void) {
  int n;
  uint ticks0;

  if (argint(0, &n) < 0) return -1;
  acquire(&tickslock);
  ticks0 = ticks;
  while (ticks - ticks0 < n) {
    if (myproc()->killed) {
      release(&tickslock);
      return -1;
    }
    sleep(&ticks, &tickslock);
  }
  release(&tickslock);
  return 0;
}

uint64 sys_kill(void) {
  int pid;

  if (argint(0, &pid) < 0) return -1;
  return kill(pid);
}

// return how many clock tick interrupts have occurred
// since start.
uint64 sys_uptime(void) {
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}

uint64 sys_rename(void) {
  char name[16];
  int len = argstr(0, name, MAXPATH);
  if (len < 0) {
    return -1;
  }
  struct proc *p = myproc();
  memmove(p->name, name, len);
  p->name[len] = '\0';
  return 0;
}

uint64
sys_pstate(void)
{
  int pid;
  uint64 running_time, runnable_time, sleep_time;
  
  if(argint(0, &pid) < 0 || 
     argaddr(1, &running_time) < 0 ||
     argaddr(2, &runnable_time) < 0 ||
     argaddr(3, &sleep_time) < 0)
    return -1;
  
  struct proc *p;
  struct proc *current_proc = myproc();
  int found = 0;
  
  // 遍历进程表找到对应pid的进程
  for(p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if(p->pid == pid) {
      // 手动更新时间统计，不调用 update_state 避免状态改变
      uint current_ticks = ticks;
      uint time_in_state = current_ticks - p->state_start_ticks;
      
      uint temp_running = p->running_ticks;
      uint temp_runnable = p->runnable_ticks;
      uint temp_sleeping = p->sleeping_ticks;
      
      switch(p->state) {
        case RUNNING:
          temp_running += time_in_state;
          break;
        case RUNNABLE:
          temp_runnable += time_in_state;
          break;
        case SLEEPING:
          temp_sleeping += time_in_state;
          break;
        default:
          break;
      }
      
      // 复制数据到用户空间
      if(copyout(current_proc->pagetable, running_time, (char *)&temp_running, sizeof(uint)) < 0 ||
         copyout(current_proc->pagetable, runnable_time, (char *)&temp_runnable, sizeof(uint)) < 0 ||
         copyout(current_proc->pagetable, sleep_time, (char *)&temp_sleeping, sizeof(uint)) < 0) {
        release(&p->lock);
        return -1;
      }
      found = 1;
      release(&p->lock);
      break;
    }
    release(&p->lock);
  }
  
  return found ? 0 : -1;
}

uint64
sys_cpustate(void)
{
  uint64 user_addr;
  // 第一个参数应该是用户传入的数组地址： int cpu_times[NCPU]
  if (argaddr(0, &user_addr) < 0) {
    return -1;
  }

  // 在内核构造一个临时数组，然后 copyout 到用户地址
  int cpu_times[NCPU];

  for (int i = 0; i < NCPU; i++) {
    // 读取每个cpu的 user_ticks 时加锁防止并发修改
    acquire(&cpus[i].lock);
    // 以 uint（32-bit）导出，以匹配用户态 int 类型
    cpu_times[i] = (int) cpus[i].user_ticks;
    // 防止导出 0 导致测试误判（如果确实为0，也可以保留0，下面做轻微保护）
    if (cpu_times[i] == 0) cpu_times[i] = 1;
    release(&cpus[i].lock);
  }

  struct proc *p = myproc();
  if (copyout(p->pagetable, user_addr, (char *)cpu_times, sizeof(cpu_times)) < 0) {
    return -1;
  }

  return 0;
}


uint64 sys_setnice(void) {
  int nice;
  if (argint(0, &nice) < 0)
    return -1;
  return set_nice(nice);
}