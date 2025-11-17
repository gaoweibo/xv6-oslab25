// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct kmem {
  struct spinlock lock;
  struct run *freelist;
};

struct kmem kmems[NCPU]; 


void kinit(){
  for (int i = 0; i < NCPU; i++) {
    initlock(&kmems[i].lock, "kmem");
  }
  freerange(end, (void*)PHYSTOP);
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  int current_cpu = 0;
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE) {
    kfree(p);
    current_cpu = (current_cpu + 1) % NCPU;
  }
}

void kfree(void *pa){
  struct run *r;
  
  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  memset(pa, 1, PGSIZE);
  
  r = (struct run*)pa;
  
  push_off();
  int id = cpuid();
  
  acquire(&kmems[id].lock);
  r->next = kmems[id].freelist;
  kmems[id].freelist = r;
  release(&kmems[id].lock);
  
  pop_off();
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *kalloc(void)
{
  struct run *r;
  push_off();
  int id = cpuid();
  
  acquire(&kmems[id].lock);
  r = kmems[id].freelist;
  if(r) {
    kmems[id].freelist = r->next;
  }
  release(&kmems[id].lock);
  
  if(!r) {
    for (int i = 0; i < NCPU; i++) {
      if (i == id) continue;
      
      acquire(&kmems[i].lock);
      r = kmems[i].freelist;
      if(r) {
        kmems[i].freelist = r->next;
        release(&kmems[i].lock);
        break;
      }
      release(&kmems[i].lock);
    }
  }
  pop_off();
  if(r)
    memset((char*)r, 5, PGSIZE); 
  return (void*)r;
}
