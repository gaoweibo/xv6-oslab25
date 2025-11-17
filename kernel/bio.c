// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define NBUCKETS 13
#define NBUCKET 13

// Buffer cache with hash buckets
struct {
  struct spinlock lock[NBUCKET];      // 每个桶一个锁
  struct buf buf[NBUF];               // 缓存块数组
  
  // 哈希桶链表头
  struct buf bucket[NBUCKET];
} bcache;

// 哈希函数
int
hash(uint blockno) {
  return blockno % NBUCKET;
}

void
binit(void)
{
  // 初始化所有桶的锁
  for (int i = 0; i < NBUCKET; i++) {
    char lockname[16];
    snprintf(lockname, sizeof(lockname), "bcache.bucket%d", i);
    initlock(&bcache.lock[i], lockname);
    // 初始化每个桶为空链表
    bcache.bucket[i].prev = &bcache.bucket[i];
    bcache.bucket[i].next = &bcache.bucket[i];
  }
  
  // 将所有buf分配到哈希桶中
  for (int i = 0; i < NBUF; i++) {
    struct buf *b = &bcache.buf[i];
    initsleeplock(&b->lock, "buffer");
    
    // 初始放入第一个桶，后续会根据blockno重新分配
    int bucket_id = 0;
    acquire(&bcache.lock[bucket_id]);
    b->next = bcache.bucket[bucket_id].next;
    b->prev = &bcache.bucket[bucket_id];
    bcache.bucket[bucket_id].next->prev = b;
    bcache.bucket[bucket_id].next = b;
    release(&bcache.lock[bucket_id]);
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;
  int bucket_id = hash(blockno);
  
  acquire(&bcache.lock[bucket_id]);
  
  // 在对应桶中查找是否已缓存
  for(b = bcache.bucket[bucket_id].next; b != &bcache.bucket[bucket_id]; b = b->next){
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bcache.lock[bucket_id]);
      acquiresleep(&b->lock);
      return b;
    }
  }
  
  // 未找到，需要在当前桶中寻找可替换的块
  // 首先在当前桶中寻找引用计数为0的块
  for(b = bcache.bucket[bucket_id].next; b != &bcache.bucket[bucket_id]; b = b->next){
    if(b->refcnt == 0) {
      // 找到可重用的块
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      release(&bcache.lock[bucket_id]);
      acquiresleep(&b->lock);
      return b;
    }
  }
  
  // 当前桶没有可用块，需要从其他桶窃取
  release(&bcache.lock[bucket_id]);
  
  // 在其他桶中寻找引用计数为0的块
  for (int i = 0; i < NBUCKET; i++) {
    if (i == bucket_id) continue;
    
    acquire(&bcache.lock[i]);
    
    // 在桶i中寻找引用计数为0的块
    for(b = bcache.bucket[i].next; b != &bcache.bucket[i]; b = b->next){
      if(b->refcnt == 0) {
        // 从原桶移除
        b->prev->next = b->next;
        b->next->prev = b->prev;
        
        // 添加到目标桶
        acquire(&bcache.lock[bucket_id]);
        b->next = bcache.bucket[bucket_id].next;
        b->prev = &bcache.bucket[bucket_id];
        bcache.bucket[bucket_id].next->prev = b;
        bcache.bucket[bucket_id].next = b;
        
        b->dev = dev;
        b->blockno = blockno;
        b->valid = 0;
        b->refcnt = 1;
        
        release(&bcache.lock[bucket_id]);
        release(&bcache.lock[i]);
        acquiresleep(&b->lock);
        return b;
      }
    }
    release(&bcache.lock[i]);
  }
  
  panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  int bucket_id = hash(b->blockno);
  
  acquire(&bcache.lock[bucket_id]);
  b->refcnt--;
  if (b->refcnt == 0) {
    // 不需要移动，保持在同一桶中
  }
  release(&bcache.lock[bucket_id]);
}

void
bpin(struct buf *b) {
  int bucket_id = hash(b->blockno);
  acquire(&bcache.lock[bucket_id]);
  b->refcnt++;
  release(&bcache.lock[bucket_id]);
}

void
bunpin(struct buf *b) {
  int bucket_id = hash(b->blockno);
  acquire(&bcache.lock[bucket_id]);
  b->refcnt--;
  release(&bcache.lock[bucket_id]);
}


