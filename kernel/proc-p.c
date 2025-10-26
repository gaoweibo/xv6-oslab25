#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

#define NICE_1_WEIGHT   10   // 高优先级
#define NICE_2_WEIGHT   7    // 中优先级  
#define NICE_3_WEIGHT   5    // 低优先级
#define TARGET_LATENCY  50  
#define MIN_GRANULARITY 5   
#define MIN(a, b) ((a) < (b) ? (a) : (b))

int nice_to_weight(int nice) {
  switch(nice) {
    case 1: return NICE_1_WEIGHT;
    case 2: return NICE_2_WEIGHT;
    case 3: return NICE_3_WEIGHT;
    default: return NICE_3_WEIGHT;
  }
}
struct {
  struct spinlock lock;
  int total_weight;           // 所有可运行进程的总权重
  uint64 min_vruntime;        // 最小虚拟运行时间
} cfs_sched;
// 可运行进程队列（全局，所有CPU共享，简化实现）
struct {
  struct proc head;       // 哨兵节点，链表头为vruntime最小进程
  struct spinlock lock;   // 队列锁
  int count;              // 可运行进程数
} run_queue;

void cfs_init(void) {
  initlock(&cfs_sched.lock, "cfs_sched");
  cfs_sched.total_weight = 0;
  cfs_sched.min_vruntime = 0;
}
// 初始化可运行队列（在procinit中调用）
void run_queue_init(void) {
  initlock(&run_queue.lock, "run_queue");
  run_queue.head.rq_prev = &run_queue.head;
  run_queue.head.rq_next = &run_queue.head;
  run_queue.count = 0;
}
// 可运行进程入队（按vruntime升序插入）
static void run_queue_enqueue(struct proc *p) {
  if (!holding(&run_queue.lock)) panic("run_queue_enqueue: need run_queue.lock");
  if (p->state != RUNNABLE || p->rq_prev || p->rq_next) return;

  struct proc *curr = run_queue.head.rq_prev;
  while (curr != &run_queue.head && curr->vruntime <= p->vruntime) {
    curr = curr->rq_prev;
  }
  p->rq_prev = curr;
  p->rq_next = curr->rq_next;
  curr->rq_next->rq_prev = p;
  curr->rq_next = p;
  run_queue.count++;
}

// 可运行进程出队
static void run_queue_dequeue(struct proc *p) {
  if (!holding(&run_queue.lock)) panic("run_queue_dequeue: need run_queue.lock");
  if (p->state != RUNNABLE || !p->rq_prev || !p->rq_next) return;

  p->rq_prev->rq_next = p->rq_next;
  p->rq_next->rq_prev = p->rq_prev;
  p->rq_prev = 0;
  p->rq_next = 0;
  run_queue.count--;
}

// 获取vruntime最小的可运行进程（链表头）
static struct proc *run_queue_pick_next(void) {
  acquire(&run_queue.lock);
  if (run_queue.head.rq_next == &run_queue.head) {
    release(&run_queue.lock);
    return 0;
  }
  struct proc *p = run_queue.head.rq_next;
  acquire(&p->lock);  // 此时持有run_queue.lock和p->lock（顺序正确）
  release(&run_queue.lock);  // 及时释放队列锁
  return p;
}
/*
// 计算指定CPU的可运行进程数（简化：此处用全局队列，实际可改为per-CPU队列）
static int cpu_runnable_count(int c) {
  // 简化实现：全局队列下所有CPU共享可运行进程，按CPU数均分
  acquire(&run_queue.lock);
  int cnt = run_queue.count / NCPU;
  release(&run_queue.lock);
  return cnt;
}
*/
// 负载均衡：从负载高的CPU迁移进程到当前CPU
/*static void load_balance(struct cpu *c) {
  int curr_cpu = c->id;
  int max_cnt = cpu_runnable_count(curr_cpu);

  // 查找负载最高的CPU
  int src_cpu = -1;
  for (int i = 0; i < NCPU; i++) {
    if (i == curr_cpu) continue;
    int cnt = cpu_runnable_count(i);
    if (cnt > max_cnt + 1) {  // 负载差异超过1时触发迁移
      max_cnt = cnt;
      src_cpu = i;
    }
  }

  if (src_cpu == -1) return;

  // 从src_cpu迁移一个进程到当前CPU（简化：全局队列直接取尾部进程）
  acquire(&run_queue.lock);
  if (run_queue.head.rq_prev != &run_queue.head) {
    struct proc *p = run_queue.head.rq_prev;
    // 先释放队列锁，再获取进程锁（避免锁顺序冲突）
    release(&run_queue.lock);
    acquire(&p->lock);
    run_queue_dequeue(p);
    run_queue_enqueue(p);
    release(&p->lock);
  } else {
    release(&run_queue.lock);
  }
}
*/
// 在scheduler中调用负载均衡

struct cpu cpus[NCPU];

struct proc proc[NPROC];

struct proc *initproc;

int nextpid = 1;
struct spinlock pid_lock;

extern void forkret(void);
static void wakeup1(struct proc *chan);
static void freeproc(struct proc *p);
void update_state(struct proc *p, enum procstate newstate);

extern char trampoline[];  // trampoline.S

// initialize the proc table at boot time.
void procinit(void) {
  struct proc *p;
  cfs_init();
  run_queue_init();

  initlock(&pid_lock, "nextpid");

  for(int i = 0; i < NCPU; i++) {
    initlock(&cpus[i].lock, "cpu");
    cpus[i].user_ticks = 0;
    cpus[i].last_switch_ticks = 0;
    cpus[i].proc = 0;
    cpus[i].id = i;
  }
  

  for (p = proc; p < &proc[NPROC]; p++) {
    initlock(&p->lock, "proc");
    p->rq_prev = 0;
    p->rq_next = 0;
    // Allocate a page for the process's kernel stack.
    // Map it high in memory, followed by an invalid
    // guard page.
    char *pa = kalloc();
    if (pa == 0) panic("kalloc");
    uint64 va = KSTACK((int)(p - proc));
    kvmmap(va, (uint64)pa, PGSIZE, PTE_R | PTE_W);
    p->kstack = va;
  }
  kvminithart();
}

// Must be called with interrupts disabled,
// to prevent race with process being moved
// to a different CPU.
int cpuid() {
  if (intr_get()) panic("cpuid: interrupts enabled");  // 新增检查
  int id = r_tp();
  return id;
}

// Return this CPU's cpu struct.
// Interrupts must be disabled.
struct cpu *mycpu(void) {
  if (intr_get()) panic("mycpu: interrupts enabled");  // 新增检查
  int id = cpuid();
  struct cpu *c = &cpus[id];
  return c;
}

// Return the current struct proc *, or zero if none.
struct proc *myproc(void) {
  push_off();
  struct cpu *c = mycpu();
  struct proc *p = c->proc;
  pop_off();
  return p;
}

int allocpid() {
  int pid;

  acquire(&pid_lock);
  pid = nextpid;
  nextpid = nextpid + 1;
  release(&pid_lock);

  return pid;
}

// Look in the process table for an UNUSED proc.
// If found, initialize state required to run in the kernel,
// and return with p->lock held.
// If there are no free procs, or a memory allocation fails, return 0.
static struct proc *allocproc(void) {
  struct proc *p;

  for (p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if (p->state == UNUSED) {
      goto found;
    } else {
      release(&p->lock);
    }
  }
  return 0;

found:
  p->pid = allocpid();
  p->state = UNUSED;  // 保持为 UNUSED，后面会更新

  // Allocate a trapframe page.
  if ((p->trapframe = (struct trapframe *)kalloc()) == 0) {
    release(&p->lock);
    return 0;
  }

  // An empty user page table.
  p->pagetable = proc_pagetable(p);
  if (p->pagetable == 0) {
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // Set up new context to start executing at forkret,
  // which returns to user space.
  memset(&p->context, 0, sizeof(p->context));
  p->context.ra = (uint64)forkret;
  p->context.sp = p->kstack + PGSIZE;

  // 初始化优先级字段
  p->state_start_ticks = ticks;
  p->running_ticks = 0;
  p->runnable_ticks = 0;
  p->sleeping_ticks = 0;
  p->last_sched_ticks = 0;

  p->nice = 3;
  p->weight = nice_to_weight(p->nice);
  p->rq_prev = p->rq_next = 0;

  // 关键修复1：先将状态改为RUNNABLE
  p->state = RUNNABLE;
  p->state_start_ticks = ticks;

  // 关键修复2：正确入队（先持run_queue.lock，再入队）
  release(&p->lock);        // 临时释放p->lock，保证锁顺序
  acquire(&run_queue.lock); // 先获取队列锁
  acquire(&p->lock);        // 再获取进程锁（顺序：队列锁 → 进程锁）
  run_queue_enqueue(p);     // 入队（此时状态为RUNNABLE，符合要求）
  release(&run_queue.lock); // 释放队列锁

  return p;
}

// free a proc structure and the data hanging from it,
// including user pages.
// p->lock must be held.
static void freeproc(struct proc *p) {
  if (p->trapframe) kfree((void *)p->trapframe);
  p->trapframe = 0;
  if (p->pagetable) proc_freepagetable(p->pagetable, p->sz);
  p->pagetable = 0;
  p->sz = 0;
  p->pid = 0;
  p->parent = 0;
  p->name[0] = 0;
  p->chan = 0;
  p->killed = 0;
  p->xstate = 0;
  p->state = UNUSED;
}

// Create a user page table for a given process,
// with no user memory, but with trampoline pages.
pagetable_t proc_pagetable(struct proc *p) {
  pagetable_t pagetable;

  // An empty page table.
  pagetable = uvmcreate();
  if (pagetable == 0) return 0;

  // map the trampoline code (for system call return)
  // at the highest user virtual address.
  // only the supervisor uses it, on the way
  // to/from user space, so not PTE_U.
  if (mappages(pagetable, TRAMPOLINE, PGSIZE, (uint64)trampoline, PTE_R | PTE_X) < 0) {
    uvmfree(pagetable, 0);
    return 0;
  }

  // map the trapframe just below TRAMPOLINE, for trampoline.S.
  if (mappages(pagetable, TRAPFRAME, PGSIZE, (uint64)(p->trapframe), PTE_R | PTE_W) < 0) {
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}

// Free a process's page table, and free the
// physical memory it refers to.
void proc_freepagetable(pagetable_t pagetable, uint64 sz) {
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmfree(pagetable, sz);
}

// a user program that calls exec("/init")
// od -t xC initcode
uchar initcode[] = {0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x45, 0x02, 0x97, 0x05, 0x00, 0x00, 0x93,
                    0x85, 0x35, 0x02, 0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00, 0x93, 0x08,
                    0x20, 0x00, 0x73, 0x00, 0x00, 0x00, 0xef, 0xf0, 0x9f, 0xff, 0x2f, 0x69, 0x6e,
                    0x69, 0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Set up first user process.
void userinit(void) {
  struct proc *p;
  p = allocproc();
  initproc = p;

  // allocate one user page and copy init's instructions
  // and data into it.
  uvminit(p->pagetable, initcode, sizeof(initcode));
  p->sz = PGSIZE;
  p->trapframe->epc = 0;
  p->trapframe->sp = PGSIZE;
  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  release(&p->lock);
}

// Grow or shrink user memory by n bytes.
// Return 0 on success, -1 on failure.
int growproc(int n) {
  uint sz;
  struct proc *p = myproc();

  sz = p->sz;
  if (n > 0) {
    if ((sz = uvmalloc(p->pagetable, sz, sz + n)) == 0) {
      return -1;
    }
  } else if (n < 0) {
    sz = uvmdealloc(p->pagetable, sz, sz + n);
  }
  p->sz = sz;
  return 0;
}

// Create a new process, copying the parent.
// Sets up child kernel stack to return as if from fork() system call.
int fork(void) {
  int i, pid;
  struct proc *np;
  struct proc *p = myproc();

  // Allocate process.
  if ((np = allocproc()) == 0) {
    return -1;
  }

  // Copy user memory from parent to child.
  if (uvmcopy(p->pagetable, np->pagetable, p->sz) < 0) {
    freeproc(np);
    release(&np->lock);
    return -1;
  }
  np->sz = p->sz;

  np->parent = p;

  // copy saved user registers.
  *(np->trapframe) = *(p->trapframe);

  // Cause fork to return 0 in the child.
  np->trapframe->a0 = 0;

  // increment reference counts on open file descriptors.
  for (i = 0; i < NOFILE; i++)
    if (p->ofile[i]) np->ofile[i] = filedup(p->ofile[i]);
  np->cwd = idup(p->cwd);

  safestrcpy(np->name, p->name, sizeof(p->name));

  pid = np->pid;

  release(&np->lock);

  return pid;
}

// Pass p's abandoned children to init.
// Caller must hold p->lock.
void reparent(struct proc *p) {
  struct proc *pp;

  for (pp = proc; pp < &proc[NPROC]; pp++) {
    // this code uses pp->parent without holding pp->lock.
    // acquiring the lock first could cause a deadlock
    // if pp or a child of pp were also in exit()
    // and about to try to lock p.
    if (pp->parent == p) {
      // pp->parent can't change between the check and the acquire()
      // because only the parent changes it, and we're the parent.
      acquire(&pp->lock);
      pp->parent = initproc;
      // we should wake up init here, but that would require
      // initproc->lock, which would be a deadlock, since we hold
      // the lock on one of init's children (pp). this is why
      // exit() always wakes init (before acquiring any locks).
      release(&pp->lock);
    }
  }
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait().
void exit(int status) {
  struct proc *p = myproc();

  if (p == initproc) panic("init exiting");

  // Close all open files.
  for (int fd = 0; fd < NOFILE; fd++) {
    if (p->ofile[fd]) {
      struct file *f = p->ofile[fd];
      fileclose(f);
      p->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(p->cwd);
  end_op();
  p->cwd = 0;

  // we might re-parent a child to init. we can't be precise about
  // waking up init, since we can't acquire its lock once we've
  // acquired any other proc lock. so wake up init whether that's
  // necessary or not. init may miss this wakeup, but that seems
  // harmless.
  acquire(&initproc->lock);
  wakeup1(initproc);
  release(&initproc->lock);

  // grab a copy of p->parent, to ensure that we unlock the same
  // parent we locked. in case our parent gives us away to init while
  // we're waiting for the parent lock. we may then race with an
  // exiting parent, but the result will be a harmless spurious wakeup
  // to a dead or wrong process; proc structs are never re-allocated
  // as anything else.
  acquire(&p->lock);
  struct proc *original_parent = p->parent;
  release(&p->lock);

  // we need the parent's lock in order to wake it up from wait().
  // the parent-then-child rule says we have to lock it first.
  acquire(&original_parent->lock);

  acquire(&p->lock);

  // Give any children to init.
  reparent(p);

  // Parent might be sleeping in wait().
  wakeup1(original_parent);

  p->xstate = status;
  update_state(p, ZOMBIE);

  release(&original_parent->lock);

  // Jump into the scheduler, never to return.
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int wait(uint64 addr) {
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();

  // hold p->lock for the whole time to avoid lost
  // wakeups from a child's exit().
  acquire(&p->lock);

  for (;;) {
    // Scan through table looking for exited children.
    havekids = 0;
    for (np = proc; np < &proc[NPROC]; np++) {
      // this code uses np->parent without holding np->lock.
      // acquiring the lock first would cause a deadlock,
      // since np might be an ancestor, and we already hold p->lock.
      if (np->parent == p) {
        // np->parent can't change between the check and the acquire()
        // because only the parent changes it, and we're the parent.
        acquire(&np->lock);
        havekids = 1;
        if (np->state == ZOMBIE) {
          // Found one.
          pid = np->pid;
          if (addr != 0 && copyout(p->pagetable, addr, (char *)&np->xstate, sizeof(np->xstate)) < 0) {
            release(&np->lock);
            release(&p->lock);
            return -1;
          }
          freeproc(np);
          release(&np->lock);
          release(&p->lock);
          return pid;
        }
        release(&np->lock);
      }
    }

    // No point waiting if we don't have any children.
    if (!havekids || p->killed) {
      release(&p->lock);
      return -1;
    }

    // Wait for a child to exit.
    sleep(p, &p->lock);  // DOC: wait-sleep
  }
}

// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run.
//  - swtch to start running that process.
//  - eventually that process transfers control
//    via swtch back to the scheduler.
// 只在调度器中统计运行时间
void scheduler(void) {
  intr_off(); 
  struct cpu *c = mycpu();
  c->proc = 0;
  
  for (;;) {
    intr_on();
    //static int balance_cnt = 0;
    //if (++balance_cnt % 10 == 0) {
    //  load_balance(c);
    //}
    // 从可运行队列获取vruntime最小的进程（O(1)）
    struct proc *next = run_queue_pick_next();
    if (!next) {
      asm volatile("wfi");
      continue;
    }

    // 计算时间片：最多运行MIN_GRANULARITY个ticks，避免独占CPU
    uint start_ticks = ticks;

    // 切换到进程运行
    update_state(next, RUNNING);
    c->proc = next;
    swtch(&c->context, &next->context);

    // 进程返回后处理
    c->proc = 0;
    uint run_ticks = ticks - start_ticks;

    if (run_ticks > 0) {
      // 关键修正：vruntime直接 = 实际运行时间 * nice（确保nice*running_time均衡）
      next->vruntime += run_ticks * next->nice;
      // 更新CPU实际运行时间
      c->user_ticks += run_ticks;
    }

    // 若进程未结束，重新入队（按新vruntime排序）
    if (next->state == RUNNING) {
      acquire(&run_queue.lock);  // 先获取队列锁
      run_queue_dequeue(next);   // 出队（此时持有队列锁）
      update_state(next, RUNNABLE); // 更新状态（仅改状态，不操作队列）
      run_queue_enqueue(next);   // 入队（此时持有队列锁）
      release(&run_queue.lock);  // 释放队列锁
    }

    release(&next->lock);
  }
}
// Switch to scheduler.  Must hold only p->lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->noff, but that would
// break in the few places where a lock is held but
// there's no process.
void sched(void) {
  int intena;
  struct proc *p = myproc();

  if (!holding(&p->lock)) panic("sched p->lock");
  if (mycpu()->noff != 1) panic("sched locks");
  if (p->state == RUNNING) panic("sched running");
  if (intr_get()) panic("sched interruptible");

  intena = mycpu()->intena;
  swtch(&p->context, &mycpu()->context);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void yield(void) {
  struct proc *p = myproc();
  acquire(&p->lock);
  update_state(p, RUNNABLE);
  sched();
  release(&p->lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch to forkret.
void forkret(void) {
  static int first = 1;

  // Still holding p->lock from scheduler.
  release(&myproc()->lock);

  if (first) {
    // File system initialization must be run in the context of a
    // regular process (e.g., because it calls sleep), and thus cannot
    // be run from main().
    first = 0;
    fsinit(ROOTDEV);
  }

  usertrapret();
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
// 在 sleep 和 wakeup 等函数中手动处理时间统计
void sleep(void *chan, struct spinlock *lk) {
  struct proc *p = myproc();

  if (lk != &p->lock) {
    acquire(&p->lock);
    release(lk);
  }

  // 统计进入睡眠前的时间
  uint current_ticks = ticks;
  if (p->state == RUNNABLE) {
    p->runnable_ticks += current_ticks - p->state_start_ticks;
    // 关键修复：先持run_queue.lock再出队
    acquire(&run_queue.lock);
    run_queue_dequeue(p);
    release(&run_queue.lock);
  }
  
  
  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;
  p->state_start_ticks = current_ticks;

  sched();

  // 统计唤醒后的时间
  current_ticks = ticks;
  p->sleeping_ticks += current_ticks - p->state_start_ticks;
  p->state_start_ticks = current_ticks;

  // Tidy up.
  p->chan = 0;

  if (lk != &p->lock) {
    release(&p->lock);
    acquire(lk);
  }
}

// Wake up all processes sleeping on chan.
// Must be called without any p->lock.
void wakeup(void *chan) {
  struct proc *p;

  for (p = proc; p < &proc[NPROC]; p++) {
    // 关键修复：先获取p->lock前，不提前持有其他锁
    acquire(&p->lock);
    if (p->state == SLEEPING && p->chan == chan) {
      // 先释放p->lock，再获取run_queue.lock（避免顺序冲突）
      release(&p->lock);
      acquire(&run_queue.lock);  // 先持队列锁
      acquire(&p->lock);         // 再持进程锁（顺序：队列锁 → 进程锁）
      
      // 对齐vruntime + 入队
      if (run_queue.head.rq_next != &run_queue.head) {
        p->vruntime = MIN(p->vruntime, run_queue.head.rq_next->vruntime);
      }
      update_state(p, RUNNABLE);
      run_queue_enqueue(p);
      
      release(&run_queue.lock);  // 释放队列锁
    }
    release(&p->lock);  // 释放进程锁
  }
}

// Wake up p if it is sleeping in wait(); used by exit().
// Caller must hold p->lock.
static void wakeup1(struct proc *p) {
  if (!holding(&p->lock)) panic("wakeup1");
  if (p->chan == p && p->state == SLEEPING) {
    update_state(p, RUNNABLE);
  }
}

// Kill the process with the given pid.
// The victim won't exit until it tries to return
// to user space (see usertrap() in trap.c).
int kill(int pid) {
  struct proc *p;

  for (p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if (p->pid == pid) {
      p->killed = 1;
      if (p->state == SLEEPING) {
        // Wake process from sleep().
        update_state(p, RUNNABLE);
      }
      release(&p->lock);
      return 0;
    }
    release(&p->lock);
  }
  return -1;
}

// Copy to either a user address, or kernel address,
// depending on usr_dst.
// Returns 0 on success, -1 on error.
int either_copyout(int user_dst, uint64 dst, void *src, uint64 len) {
  struct proc *p = myproc();
  if (user_dst) {
    return copyout(p->pagetable, dst, src, len);
  } else {
    memmove((char *)dst, src, len);
    return 0;
  }
}

// Copy from either a user address, or kernel address,
// depending on usr_src.
// Returns 0 on success, -1 on error.
int either_copyin(void *dst, int user_src, uint64 src, uint64 len) {
  struct proc *p = myproc();
  if (user_src) {
    return copyin(p->pagetable, dst, src, len);
  } else {
    memmove(dst, (char *)src, len);
    return 0;
  }
}

// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void procdump(void) {
  static char *states[] = {
      [UNUSED] = "unused", [SLEEPING] = "sleep ", [RUNNABLE] = "runble", [RUNNING] = "run   ", [ZOMBIE] = "zombie"};
  struct proc *p;
  char *state;

  printf("\n");
  for (p = proc; p < &proc[NPROC]; p++) {
    if (p->state == UNUSED) continue;
    if (p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    printf("%d %s %s", p->pid, state, p->name);
    printf("\n");
  }
}

// you must hold p->lock to call this function
void update_state(struct proc *p, enum procstate newstate) {
  if (!holding(&p->lock)) panic("update_state: not holding p->lock");

  uint current_ticks = ticks;
  enum procstate oldstate = p->state;

  // 处理旧状态的时间统计
  if (oldstate == RUNNABLE) {
    p->runnable_ticks += current_ticks - p->state_start_ticks;
  } else if (oldstate == RUNNING) {
    p->running_ticks += current_ticks - p->state_start_ticks;
  } else if (oldstate == SLEEPING) {
    p->sleeping_ticks += current_ticks - p->state_start_ticks;
  }

  // 更新为新状态
  p->state = newstate;
  p->state_start_ticks = current_ticks;

}

int set_nice(int nice) {
  if (nice < 1 || nice > 3) {
    return -1;
  }

  struct proc *p = myproc();
  acquire(&p->lock);
  
  if (p->nice != nice) {
    int old_weight = p->weight;
    p->nice = nice;
    p->weight = nice_to_weight(nice);
    // 权重变化时调整vruntime
    p->vruntime = (p->vruntime * p->weight) / old_weight;
  }
  
  release(&p->lock);
  return 0;
}