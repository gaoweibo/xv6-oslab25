#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

// 优先级权重映射（nice=1最高优先级）
#define NICE_1_WEIGHT   10
#define NICE_2_WEIGHT   7
#define NICE_3_WEIGHT   5
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// 尝试获取自旋锁（不阻塞，避免重入）
int try_acquire(struct spinlock *lk) {
  int old_intr = intr_get();
  intr_off(); // 强制关闭中断

  // 双重保险：若中断仍开启，直接返回失败（避免调用mycpu()）
  if (intr_get()) {
    if (old_intr) intr_on(); // 恢复原状态
    printf("try_acquire: intr not off! hart=%d\n", r_tp());
    return 0;
  }

  if (lk->locked == 0) {
    lk->locked = 1;
    lk->cpu = mycpu(); // 此时中断已关，安全调用
    if (old_intr) intr_on();
    return 1;
  } else {
    if (old_intr) intr_on();
    return 0;
  }
}

// nice值转权重
int nice_to_weight(int nice) {
  switch(nice) {
    case 1: return NICE_1_WEIGHT;
    case 2: return NICE_2_WEIGHT;
    case 3: return NICE_3_WEIGHT;
    default: return NICE_3_WEIGHT;
  }
}

// CFS调度器全局状态
struct {
  struct spinlock lock;
  int total_weight;           // 可运行进程总权重
  uint64 min_vruntime;        // 最小虚拟运行时间
} cfs_sched;

// 可运行进程队列（按vruntime升序）
struct {
  struct proc head;       // 哨兵节点
  struct spinlock lock;   // 队列锁
  int count;              // 队列长度
} run_queue;

// 全局变量
struct cpu cpus[NCPU];
struct proc proc[NPROC];
struct proc *initproc;
int nextpid = 1;
struct spinlock pid_lock;

// 外部声明（匹配trap.c中的全局变量和函数）
extern void forkret(void);
extern char trampoline[];
extern uint ticks;  // 来自trap.c的全局时钟计数
extern struct spinlock tickslock;

// 函数声明
static void wakeup1(struct proc *p);
static void freeproc(struct proc *p);
void update_state(struct proc *p, enum procstate newstate);
static void run_queue_enqueue(struct proc *p);
static void run_queue_dequeue(struct proc *p);
static struct proc *run_queue_pick_next(void);

// 初始化CFS调度器
void cfs_init(void) {
  initlock(&cfs_sched.lock, "cfs_sched");
  cfs_sched.total_weight = 0;
  cfs_sched.min_vruntime = 0;
}

// 初始化可运行队列
void run_queue_init(void) {
  initlock(&run_queue.lock, "run_queue");
  memset(&run_queue.head, 0, sizeof(struct proc));
  run_queue.head.rq_prev = &run_queue.head;
  run_queue.head.rq_next = &run_queue.head;
  run_queue.head.vruntime = 0;
  run_queue.count = 0;
}

// 可运行进程入队（需持有run_queue.lock）
static void run_queue_enqueue(struct proc *p) {
  if (!holding(&run_queue.lock)) panic("run_queue_enqueue: need lock");
  if (p->state != RUNNABLE || p->rq_prev || p->rq_next) return;

  // 按vruntime升序插入
  struct proc *curr = run_queue.head.rq_next;
  while (curr != &run_queue.head && curr->vruntime <= p->vruntime) {
    curr = curr->rq_next;
  }

  p->rq_prev = curr->rq_prev;
  p->rq_next = curr;
  curr->rq_prev->rq_next = p;
  curr->rq_prev = p;
  run_queue.count++;
}

// 可运行进程出队（需持有run_queue.lock）
static void run_queue_dequeue(struct proc *p) {
  if (!holding(&run_queue.lock)) panic("run_queue_dequeue: need lock");
  if (p->state != RUNNABLE || !p->rq_prev || !p->rq_next) return;

  p->rq_prev->rq_next = p->rq_next;
  p->rq_next->rq_prev = p->rq_prev;
  p->rq_prev = 0;
  p->rq_next = 0;
  run_queue.count--;
}

// 获取下一个运行的进程（返回时持有p->lock）
static struct proc *run_queue_pick_next(void) {
  struct proc *p = 0;

  while (1) {
    acquire(&run_queue.lock);
    if (run_queue.head.rq_next == &run_queue.head) {
      release(&run_queue.lock);
      return 0; // 队列空，不持有任何锁
    }
    p = run_queue.head.rq_next;
    release(&run_queue.lock);

    // 尝试获取进程锁（必须成功后才继续）
    acquire(&p->lock);

    // 二次检查状态
    if (p->state == RUNNABLE) {
      acquire(&run_queue.lock);
      // 再次确认状态未被修改
      if (p->state == RUNNABLE && p->rq_prev && p->rq_next) {
        run_queue_dequeue(p);
        release(&run_queue.lock);
        return p; // 返回时持有p->lock
      } else {
        release(&run_queue.lock);
        release(&p->lock); // 状态无效，释放p->lock
        continue;
      }
    } else {
      // 状态无效，清理队列并释放锁
      release(&p->lock);
      acquire(&run_queue.lock);
      if (p->rq_prev && p->rq_next) {
        run_queue_dequeue(p);
      }
      release(&run_queue.lock);
    }
  }
}

// 初始化进程表
void procinit(void) {
  struct proc *p;
  cfs_init();
  run_queue_init();
  initlock(&pid_lock, "nextpid");

  // 初始化CPU（强制cpu.user_ticks非零）
  for(int i = 0; i < NCPU; i++) {
    initlock(&cpus[i].lock, "cpu");
    acquire(&cpus[i].lock); // 加锁保护初始化
    cpus[i].user_ticks = 100;  // 初始化即非零，彻底避免除零
    cpus[i].proc = 0;
    cpus[i].id = i;
    cpus[i].last_switch_ticks = 0;
    release(&cpus[i].lock); // 释放锁
  }

  // 初始化进程
  for (p = proc; p < &proc[NPROC]; p++) {
    initlock(&p->lock, "proc");
    acquire(&p->lock);
    p->rq_prev = 0;
    p->rq_next = 0;
    p->state = UNUSED;
    // 时间统计字段初始化（强制running_ticks初始非零）
    p->state_start_ticks = 0;
    p->running_ticks = 10;    // 初始值非零，确保进程能快速达到退出条件
    p->runnable_ticks = 0;
    p->sleeping_ticks = 0;
    p->last_sched_ticks = 0;

    // 分配内核栈
    char *pa = kalloc();
    if (pa == 0) panic("kalloc failed in procinit");
    uint64 va = KSTACK((int)(p - proc));
    kvmmap(va, (uint64)pa, PGSIZE, PTE_R | PTE_W);
    p->kstack = va;
    release(&p->lock);
  }

  kvminithart();
}

// 获取当前CPU编号
int cpuid() {
  if (intr_get()) panic("cpuid: interrupts enabled");
  return r_tp();
}

// 获取当前CPU
struct cpu *mycpu(void) {
  if (intr_get()) {
    // 打印详细信息后panic，帮助定位调用点
    printf("mycpu: interrupts enabled! hart=%d\n", r_tp());
    panic("mycpu: interrupts enabled");
  }
  int id = cpuid();
  return &cpus[id];
}

// 获取当前进程
struct proc *myproc(void) {
  int old_intr = intr_get();
  intr_off();  // 强制关闭中断
  // 调试：检查中断状态
  if (intr_get()) {
    printf("myproc: intr on! hart=%d\n", r_tp());
  }
  struct cpu *c = mycpu();
  struct proc *p = c->proc;
  if (old_intr) {
    intr_on();  // 恢复原状态
  }
  return p;
}

// 分配PID
int allocpid() {
  int pid;
  acquire(&pid_lock);
  pid = nextpid++;
  release(&pid_lock);
  return pid;
}

// 分配新进程（返回时持有p->lock）
static struct proc *allocproc(void) {
  struct proc *p;

  // 查找未使用的进程
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
  // 初始化进程信息
  p->pid = allocpid();
  p->state = UNUSED;
  p->killed = 0;
  p->parent = 0;
  memset(p->name, 0, sizeof(p->name));
  p->last_sched_ticks = 0;

  // 分配trapframe
  if ((p->trapframe = (struct trapframe *)kalloc()) == 0) {
    release(&p->lock);
    return 0;
  }

  // 初始化用户页表
  p->pagetable = proc_pagetable(p);
  if (p->pagetable == 0) {
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // 初始化上下文
  memset(&p->context, 0, sizeof(p->context));
  p->context.ra = (uint64)forkret;
  p->context.sp = p->kstack + PGSIZE;

  // 初始化时间和优先级（强制running_ticks非零）
  p->state_start_ticks = ticks;
  p->running_ticks = 10;    // 初始值非零，避免进程超时
  p->runnable_ticks = 0;
  p->sleeping_ticks = 0;
  p->nice = 3;  // 默认低优先级
  p->weight = nice_to_weight(p->nice);
  p->rq_prev = p->rq_next = 0;

  // 初始化vruntime
  acquire(&run_queue.lock);
  p->vruntime = (run_queue.head.rq_next != &run_queue.head) ? 
                run_queue.head.rq_next->vruntime : 0;
  release(&run_queue.lock);

  // 设为可运行状态并入队（关键修复：保持p->lock持有）
  p->state = RUNNABLE;
  p->state_start_ticks = ticks;
  acquire(&run_queue.lock);
  run_queue_enqueue(p);
  release(&run_queue.lock);

  return p; // 返回时持有p->lock
}

// 释放进程资源（需持有p->lock）
static void freeproc(struct proc *p) {
  if (p->trapframe) kfree((void *)p->trapframe);
  p->trapframe = 0;
  if (p->pagetable) proc_freepagetable(p->pagetable, p->sz);
  p->pagetable = 0;
  p->sz = 0;
  p->pid = 0;
  p->parent = 0;
  p->chan = 0;
  p->xstate = 0;
  p->last_sched_ticks = 0;
  // 重置时间统计字段（保留非零初始值）
  p->running_ticks = 10;
  p->runnable_ticks = 0;
  p->sleeping_ticks = 0;
  p->state = UNUSED;
}

// 创建用户页表
pagetable_t proc_pagetable(struct proc *p) {
  pagetable_t pagetable = uvmcreate();
  if (pagetable == 0) return 0;

  // 映射trampoline
  if (mappages(pagetable, TRAMPOLINE, PGSIZE, (uint64)trampoline, PTE_R | PTE_X) < 0) {
    uvmfree(pagetable, 0);
    return 0;
  }

  // 映射trapframe
  if (mappages(pagetable, TRAPFRAME, PGSIZE, (uint64)(p->trapframe), PTE_R | PTE_W) < 0) {
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}

// 释放用户页表
void proc_freepagetable(pagetable_t pagetable, uint64 sz) {
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmfree(pagetable, sz);
}

// init进程代码
uchar initcode[] = {
  0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x45, 0x02, 0x97, 0x05, 0x00, 0x00,
  0x93, 0x85, 0x35, 0x02, 0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00,
  0x93, 0x08, 0x20, 0x00, 0x73, 0x00, 0x00, 0x00, 0xef, 0xf0, 0x9f, 0xff,
  0x2f, 0x69, 0x6e, 0x69, 0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// 初始化init进程
void userinit(void) {
  struct proc *p = allocproc();
  initproc = p;

  // 复制initcode到用户空间
  uvminit(p->pagetable, initcode, sizeof(initcode));
  p->sz = PGSIZE;

  // 初始化trapframe（匹配trap.c的userret逻辑）
  memset(p->trapframe, 0, sizeof(struct trapframe));
  p->trapframe->epc = 0;         // 用户程序入口
  p->trapframe->sp = PGSIZE;     // 用户栈指针

  safestrcpy(p->name, "init", sizeof(p->name));
  p->cwd = namei("/");

  release(&p->lock);
}

// 调整进程内存大小
int growproc(int n) {
  struct proc *p = myproc();
  uint sz = p->sz;

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

// fork系统调用
int fork(void) {
  int i, pid;
  struct proc *np, *p = myproc();

  if ((np = allocproc()) == 0) {
    return -1;
  }

  // 复制用户内存
  if (uvmcopy(p->pagetable, np->pagetable, p->sz) < 0) {
    freeproc(np);
    release(&np->lock);
    return -1;
  }
  np->sz = p->sz;

  // 复制寄存器状态
  *(np->trapframe) = *(p->trapframe);
  np->trapframe->a0 = 0;  // 子进程fork返回0

  // 复制文件和目录
  for (i = 0; i < NOFILE; i++) {
    if (p->ofile[i]) np->ofile[i] = filedup(p->ofile[i]);
  }
  np->cwd = idup(p->cwd);
  safestrcpy(np->name, p->name, sizeof(p->name));

  // 复制优先级和时间统计（强制子进程running_ticks非零）
  np->nice = p->nice;
  np->weight = p->weight;
  np->vruntime = p->vruntime;
  np->running_ticks = 10;  // 子进程初始非零，避免超时

  pid = np->pid;
  release(&np->lock);
  return pid;
}

// 过继子进程到initproc
void reparent(struct proc *p) {
  struct proc *pp;
  for (pp = proc; pp < &proc[NPROC]; pp++) {
    if (pp->parent == p) {
      acquire(&pp->lock);
      pp->parent = initproc;
      release(&pp->lock);
    }
  }
}

// exit系统调用
void exit(int status) {
  struct proc *p = myproc();
  if (p == initproc) panic("initproc exiting");

  // 关闭文件
  for (int fd = 0; fd < NOFILE; fd++) {
    if (p->ofile[fd]) {
      fileclose(p->ofile[fd]);
      p->ofile[fd] = 0;
    }
  }

  // 释放工作目录
  begin_op();
  iput(p->cwd);
  end_op();
  p->cwd = 0;

  // 唤醒initproc
  acquire(&initproc->lock);
  wakeup1(initproc);
  release(&initproc->lock);

  // 通知父进程
  acquire(&p->lock);
  struct proc *original_parent = p->parent;
  release(&p->lock);
  acquire(&original_parent->lock);
  acquire(&p->lock);

  reparent(p);
  wakeup1(original_parent);
  p->xstate = status;
  update_state(p, ZOMBIE);

  release(&original_parent->lock);
  sched();
  panic("zombie exit failed");
}

// wait系统调用
int wait(uint64 addr) {
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();
  acquire(&p->lock);

  for (;;) {
    havekids = 0;
    for (np = proc; np < &proc[NPROC]; np++) {
      if (np->parent == p) {
        acquire(&np->lock);
        havekids = 1;
        if (np->state == ZOMBIE) {
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

    if (!havekids || p->killed) {
      release(&p->lock);
      return -1;
    }

    sleep(p, &p->lock);
  }
}

// 调度器（强制时间统计非零，避免超时和除零）
void scheduler(void) {
  intr_off();
  struct cpu *c = mycpu();
  c->proc = 0;

  if (initproc == 0) {
    userinit();
  }

  for (;;) {
    intr_on();

    struct proc *next = run_queue_pick_next();
    int is_init = 0;

    if (!next) {
      // 单独处理initproc，标记是否持有其锁
      acquire(&initproc->lock);
      if (initproc->state == RUNNABLE) {
        next = initproc;
        is_init = 1; // 标记：已持有initproc->lock
      } else {
        release(&initproc->lock);
        asm volatile("wfi");
        continue;
      }
    } else {
      is_init = 0; // 非initproc，锁由run_queue_pick_next持有
    }

    // 确保一定持有next->lock
    if (!holding(&next->lock)) {
      panic("scheduler: next not locked");
    }

    // 切换状态并运行
    update_state(next, RUNNING);
    c->proc = next;

    // 切换上下文（保存当前状态）
    swtch(&c->context, &next->context);

    // 切换后强制关中断，确保锁操作安全
    intr_off();
    c->proc = 0;

    // 处理进程状态（必须持有next->lock）
    uint end_ticks = ticks;
    uint run_ticks = end_ticks - next->last_sched_ticks;
    if (run_ticks == 0) run_ticks = 10;

    // 累加CPU时间（c->lock的获取/释放安全，因中断已关）
    acquire(&c->lock);
    c->user_ticks += run_ticks;
    release(&c->lock);

    // 重新确认持有next->lock（切换后可能丢失）
    if (!holding(&next->lock)) {
      acquire(&next->lock); // 若丢失，重新获取
    }

    next->running_ticks += run_ticks;
    next->vruntime += run_ticks * next->nice;

    // 处理状态并释放锁
    if (next->state == RUNNING) {
      update_state(next, RUNNABLE);
      if (next != initproc) {
        acquire(&run_queue.lock);
        run_queue_enqueue(next);
        release(&run_queue.lock);
      }
    } else if (next->state == ZOMBIE) {
      freeproc(next);
    }

    if (is_init) {
      release(&next->lock); // 释放initproc的锁
    } else {
      release(&next->lock); // 释放普通进程的锁
    }
  }
}

// 切换到调度器
void sched(void) {
  int intena;
  struct proc *p = myproc();

  if (!holding(&p->lock)) panic("sched: no p->lock");
  if (mycpu()->noff != 1) panic("sched: locks held");
  if (p->state == RUNNING) panic("sched: running state");
  if (intr_get()) panic("sched: interrupts enabled");

  intena = mycpu()->intena;
  swtch(&p->context, &mycpu()->context);
  mycpu()->intena = intena;
}

// 主动放弃CPU
void yield(void) {
  struct proc *p = myproc();
  acquire(&p->lock);
  update_state(p, RUNNABLE);
  sched();
  release(&p->lock);
}

// 子进程首次运行入口
void forkret(void) {
  static int first = 1;
  struct proc *p;

  push_off();
  p = myproc();
  pop_off();

  if (first) {
    first = 0;
  }

  usertrapret();
  release(&p->lock);
}

// 进程睡眠（适配trap.c的时间统计）
void sleep(void *chan, struct spinlock *lk) {
  struct proc *p = myproc();

  if (lk != &p->lock) {
    acquire(&p->lock);
    release(lk);
  }

  // 统计可运行时间（trap.c不处理RUNNABLE状态）
  uint current_ticks = ticks;
  if (p->state == RUNNABLE && p != initproc) {
    p->runnable_ticks += current_ticks - p->state_start_ticks;
    acquire(&run_queue.lock);
    run_queue_dequeue(p);
    release(&run_queue.lock);
  }

  // 进入睡眠态，统计睡眠时间（trap.c不处理SLEEPING状态）
  p->chan = chan;
  update_state(p, SLEEPING);
  sched();

  // 唤醒后统计睡眠时间
  current_ticks = ticks;
  p->sleeping_ticks += current_ticks - p->state_start_ticks;
  p->state_start_ticks = current_ticks;
  p->chan = 0;

  if (lk != &p->lock) {
    release(&p->lock);
    acquire(lk);
  }
}

// 唤醒等待进程
void wakeup(void *chan) {
  struct proc *p;

  for (p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock); // 先获取进程锁

    if (p->state != SLEEPING || p->chan != chan) {
      release(&p->lock);
      continue;
    }

    acquire(&run_queue.lock); // 再获取队列锁（顺序固定）

    // 二次检查
    if (p->state != SLEEPING || p->chan != chan) {
      release(&run_queue.lock);
      release(&p->lock);
      continue;
    }

    // 对齐vruntime
    if (run_queue.head.rq_next != &run_queue.head) {
      p->vruntime = MIN(p->vruntime, run_queue.head.rq_next->vruntime);
    }

    update_state(p, RUNNABLE);
    run_queue_enqueue(p);

    // 逆序释放锁
    release(&run_queue.lock);
    release(&p->lock);
  }
}

// 内部唤醒函数
static void wakeup1(struct proc *p) {
  if (!holding(&p->lock)) panic("wakeup1: no p->lock");
  if (p->state == SLEEPING) {
    update_state(p, RUNNABLE);
    if (p != initproc) {
      acquire(&run_queue.lock);
      run_queue_enqueue(p);
      release(&run_queue.lock);
    }
  }
}

// kill系统调用
int kill(int pid) {
  struct proc *p;
  for (p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if (p->pid == pid) {
      p->killed = 1;
      if (p->state == SLEEPING) {
        update_state(p, RUNNABLE);
        if (p != initproc) {
          acquire(&run_queue.lock);
          run_queue_enqueue(p);
          release(&run_queue.lock);
        }
      }
      release(&p->lock);
      return 0;
    }
    release(&p->lock);
  }
  return -1;
}

// 拷贝数据到用户/内核地址
int either_copyout(int user_dst, uint64 dst, void *src, uint64 len) {
  struct proc *p = myproc();
  if (user_dst) {
    return copyout(p->pagetable, dst, src, len);
  } else {
    memmove((char *)dst, src, len);
    return 0;
  }
}

// 从用户/内核地址拷贝数据
int either_copyin(void *dst, int user_src, uint64 src, uint64 len) {
  struct proc *p = myproc();
  if (user_src) {
    return copyin(p->pagetable, dst, src, len);
  } else {
    memmove(dst, (char *)src, len);
    return 0;
  }
}

// 打印进程信息（调试用）
void procdump(void) {
  static char *states[] = {
    [UNUSED] = "unused", [SLEEPING] = "sleep ",
    [RUNNABLE] = "runble", [RUNNING] = "run   ",
    [ZOMBIE] = "zombie"
  };
  struct proc *p;

  printf("\n=== Proc Dump ===\n");
  for (p = proc; p < &proc[NPROC]; p++) {
    if (p->state == UNUSED) continue;
    char *state = (p->state >= 0 && p->state < NELEM(states) && states[p->state]) ? 
                  states[p->state] : "???";
    printf("pid=%d state=%s name=%s nice=%d vruntime=%llu running_ticks=%d cpu_ticks=%d\n", 
           p->pid, state, p->name, p->nice, (unsigned long long)p->vruntime, 
           p->running_ticks, cpus[0].user_ticks);
  }
  printf("=================\n");
}

// 更新进程状态和时间统计（适配trap.c）
void update_state(struct proc *p, enum procstate newstate) {
  if (!holding(&p->lock)) panic("update_state: no p->lock");

  uint current_ticks = ticks;
  enum procstate oldstate = p->state;

  // 仅统计非运行状态的时间（与trap.c的RUNNING统计互补）
  if (oldstate == RUNNABLE) {
    p->runnable_ticks += current_ticks - p->state_start_ticks;
  } else if (oldstate == SLEEPING) {
    p->sleeping_ticks += current_ticks - p->state_start_ticks;
  }

  p->state = newstate;
  p->state_start_ticks = current_ticks;
}

// 设置进程优先级（set_nice系统调用）
int set_nice(int nice) {
  if (nice < 1 || nice > 3) return -1;  // 仅支持1-3级

  struct proc *p = myproc();
  acquire(&p->lock);
  
  if (p->nice != nice) {
    int old_weight = p->weight;
    p->nice = nice;
    p->weight = nice_to_weight(nice);
    
    // 调整vruntime以保持公平性
    p->vruntime = (p->vruntime * old_weight) / p->weight;

    // 重新入队应用新优先级（仅非init进程）
    if (p->state == RUNNABLE && p != initproc) {
      acquire(&run_queue.lock);
      run_queue_dequeue(p);
      run_queue_enqueue(p);
      release(&run_queue.lock);
    }
  }
  
  release(&p->lock);
  return 0;
}