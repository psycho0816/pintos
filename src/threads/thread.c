#include "threads/thread.h"
#include <debug.h>
#include <stddef.h>
#include <random.h>
#include <stdio.h>
#include <string.h>
#include "threads/flags.h"
#include "threads/interrupt.h"
#include "threads/intr-stubs.h"
#include "threads/palloc.h"
#include "threads/switch.h"
#include "threads/synch.h"
#include "threads/vaddr.h"
#ifdef USERPROG
#include "userprog/process.h"
#endif

#define THREAD_MAGIC 0xcd6abf4b
static struct list ready_list;
static struct list all_list;
static struct thread *idle_thread;
static struct thread *initial_thread;
static struct lock tid_lock;

struct kernel_thread_frame
{
  void *eip;             /**< Return address. */
  thread_func *function; /**< Function to call. */
  void *aux;             /**< Auxiliary data for function. */
};

/** Statistics. */
static long long idle_ticks;   /**< # of timer ticks spent idle. */
static long long kernel_ticks; /**< # of timer ticks in kernel threads. */
static long long user_ticks;   /**< # of timer ticks in user programs. */

/** Scheduling. */
#define TIME_SLICE 4          /**< # of timer ticks to give each thread. */
static unsigned thread_ticks; /**< # of timer ticks since last yield. */

bool thread_mlfqs;

static void kernel_thread(thread_func *, void *aux);

static void idle(void *aux UNUSED);
static struct thread *running_thread(void);
static struct thread *next_thread_to_run(void);
static void init_thread(struct thread *, const char *name, int priority);
static bool is_thread(struct thread *) UNUSED;
static void *alloc_frame(struct thread *, size_t size);
static void schedule(void);
void thread_schedule_tail(struct thread *prev);
static tid_t allocate_tid(void);

void thread_init(void)
{
  ASSERT(intr_get_level() == INTR_OFF);

  lock_init(&tid_lock);
  list_init(&ready_list);
  list_init(&all_list);

  /* Set up a thread structure for the running thread. */
  initial_thread = running_thread();
  init_thread(initial_thread, "main", PRI_DEFAULT);
  initial_thread->status = THREAD_RUNNING;
  initial_thread->tid = allocate_tid();
}

void thread_start(void)
{
  struct semaphore idle_started;
  sema_init(&idle_started, 0);
  thread_create("idle", PRI_MIN, idle, &idle_started);

  /* Start preemptive thread scheduling. */
  intr_enable();

  /* Wait for the idle thread to initialize idle_thread. */
  sema_down(&idle_started);
}

void thread_sleep(int64_t ticks){
  if(ticks <= 0) return;
  struct thread *cur = thread_current();

  // 禁用中断并保存当前中断级别
  enum intr_level old_level = intr_disable();
  
  if (cur != idle_thread)   // 确保CPU不是在空等待
  {
    cur->status = THREAD_SLEEP;       // 将当前进程状态改为休眠
    cur->wake_time = timer_ticks() + ticks;   // 设置进程休眠结束的时间
    schedule();     // 调度进程，将进程插入ready队列而不是直接执行，也不是插入waiting队列
  }

  // 恢复之前的中断级别
  intr_set_level(old_level);
}

void check_and_wakeup_sleep_thread(void) {
  int64_t cur_ticks = timer_ticks();  // 获取当前滴答数
  struct thread *highest_priority_thread = NULL;  // 记录被唤醒线程中最高优先级线程

  // 遍历所有线程列表
  struct list_elem *e = list_begin(&all_list);
  while (e != list_end(&all_list)) {
    enum intr_level old_level = intr_disable();  // 在整个操作中禁用中断，减少频繁的中断开关

    struct thread *t = list_entry(e, struct thread, allelem);
    e = list_next(e);  // 提前获取下一个线程，防止后续对当前线程状态的修改影响遍历

    // 如果线程处于休眠状态且当前时间已达到或超过唤醒时间
    if (t->status == THREAD_SLEEP && cur_ticks >= t->wake_time) {
      // 禁用中断以进行安全操作
      enum intr_level old_level = intr_disable();

      t->status = THREAD_READY;  // 将线程状态设置为就绪
      list_insert_ordered(&ready_list, &t->elem, prio_cmp_func, NULL); // 插入到就绪队列

      // 更新被唤醒线程中最高优先级线程
      if (!highest_priority_thread || t->priority > highest_priority_thread->priority) {
        highest_priority_thread = t;
      }
      printf("Wake up thread %s at tick %lld with priority %d.\n", t->name, cur_ticks, t->priority);  // 输出日志

      // 恢复中断状态
      intr_set_level(old_level);
    }
  }

  if(list_empty(&ready_list))
    printf("Empty\n");
  else printf("Something\n");

  // 抢占逻辑
  if (highest_priority_thread) {
    struct thread *current = thread_current(); // 获取当前运行线程
    // 如果唤醒的线程优先级高于当前线程，进行抢占
    if (highest_priority_thread->priority > current->priority) {
      if(!intr_context)
        thread_yield();
    }
  }
}

/** Returns the name of the running thread. */
const char *
thread_name(void)
{
  return thread_current()->name;
}

struct thread *
thread_current(void)
{
  struct thread *t = running_thread();

  ASSERT(is_thread(t));
  ASSERT(t->status == THREAD_RUNNING);

  return t;
}

tid_t thread_create(const char *name, int priority,
                    thread_func *function, void *aux)
{
  struct thread *t;
  struct kernel_thread_frame *kf;
  struct switch_entry_frame *ef;
  struct switch_threads_frame *sf;
  tid_t tid;

  ASSERT(function != NULL);

  /* Allocate thread. */
  t = palloc_get_page(PAL_ZERO);
  if (t == NULL)
    return TID_ERROR;

  /* Initialize thread. */
  init_thread(t, name, priority);
  tid = t->tid = allocate_tid();

  /* Stack frame for kernel_thread(). */
  kf = alloc_frame(t, sizeof *kf);
  kf->eip = NULL;
  kf->function = function;
  kf->aux = aux;

  /* Stack frame for switch_entry(). */
  ef = alloc_frame(t, sizeof *ef);
  ef->eip = (void (*)(void))kernel_thread;

  /* Stack frame for switch_threads(). */
  sf = alloc_frame(t, sizeof *sf);
  sf->eip = switch_entry;
  sf->ebp = 0;

  /* Add to run queue. */
  thread_unblock(t);

  return tid;
}

void thread_tick(void)
{
  struct thread *t = thread_current();

  /* Update statistics. */
  if (t == idle_thread)
    idle_ticks++;
#ifdef USERPROG
  else if (t->pagedir != NULL)
    user_ticks++;
#endif
  else
    kernel_ticks++;

  /* Enforce preemption. */
  if (++thread_ticks >= TIME_SLICE)
    intr_yield_on_return();
}

/** Prints thread statistics. */
void thread_print_stats(void)
{
  printf("Thread: %lld idle ticks, %lld kernel ticks, %lld user ticks\n",
         idle_ticks, kernel_ticks, user_ticks);
}

void thread_block(void)
{
  ASSERT(!intr_context());
  ASSERT(intr_get_level() == INTR_OFF);

  thread_current()->status = THREAD_BLOCKED;
  schedule();
}

void thread_unblock(struct thread *t)
{
  enum intr_level old_level;

  ASSERT(is_thread(t));

  old_level = intr_disable();
  ASSERT(t->status == THREAD_BLOCKED);
  // list_push_back (&ready_list, &t->elem);
  t->status = THREAD_READY;
  list_insert_ordered(&ready_list, &t->elem, prio_cmp_func, NULL);
  intr_set_level(old_level);
}

/** Returns the running thread's tid. */
tid_t thread_tid(void)
{
  return thread_current()->tid;
}

/** Deschedules the current thread and destroys it.  Never
   returns to the caller. */
void thread_exit(void)
{
  ASSERT(!intr_context());

#ifdef USERPROG
  process_exit();
#endif

  intr_disable();
  list_remove(&thread_current()->allelem);
  thread_current()->status = THREAD_DYING;
  schedule();
  NOT_REACHED();
}

static void
idle(void *idle_started_ UNUSED)
{
  struct semaphore *idle_started = idle_started_;
  idle_thread = thread_current();
  sema_up(idle_started);

  for (;;)
  {
    /* Let someone else run. */
    intr_disable();
    thread_block();

    asm volatile("sti; hlt" : : : "memory");
  }
}

void thread_yield(void)
{
  struct thread *cur = thread_current();
  enum intr_level old_level;

  ASSERT(!intr_context());

  old_level = intr_disable();
  if (cur != idle_thread)
    list_insert_ordered(&ready_list, &cur->elem, prio_cmp_func, NULL);
  printf("Yield:thread %s at tick %lld.\n", cur->name, (uint64_t)timer_ticks());
  cur->status = THREAD_READY;
  schedule();
  
  intr_set_level(old_level);
}

static void
schedule(void)
{
  struct thread *cur = running_thread();
  struct thread *next = next_thread_to_run();
  struct thread *prev = NULL;

  ASSERT(intr_get_level() == INTR_OFF);
  ASSERT(cur->status != THREAD_RUNNING);
  ASSERT(is_thread(next));

  if (cur != next)
    prev = switch_threads(cur, next);
  thread_schedule_tail(prev);
}

/** Does basic initialization of T as a blocked thread named
   NAME. */
static void
init_thread(struct thread *t, const char *name, int priority)
{
  enum intr_level old_level;

  ASSERT(t != NULL);
  ASSERT(PRI_MIN <= priority && priority <= PRI_MAX);
  ASSERT(name != NULL);

  memset(t, 0, sizeof *t);
  t->status = THREAD_BLOCKED;
  strlcpy(t->name, name, sizeof t->name);
  t->stack = (uint8_t *)t + PGSIZE;
  t->priority = priority;
  t->magic = THREAD_MAGIC;

  old_level = intr_disable();
  // list_push_back (&all_list, &t->allelem);
  list_insert_ordered(&all_list, &t->allelem, prio_cmp_func, NULL);
  intr_set_level(old_level);
}

void thread_foreach(thread_action_func *func, void *aux)
{
  struct list_elem *e;

  ASSERT(intr_get_level() == INTR_OFF);

  for (e = list_begin(&all_list); e != list_end(&all_list);
       e = list_next(e))
  {
    struct thread *t = list_entry(e, struct thread, allelem);
    func(t, aux);
  }
}

/** Sets the current thread's priority to NEW_PRIORITY. */
void thread_set_priority(int new_priority)
{
  thread_current()->priority = new_priority;
}

/** Returns the current thread's priority. */
int thread_get_priority(void)
{
  return thread_current()->priority;
}

/** Sets the current thread's nice value to NICE. */
void thread_set_nice(int nice UNUSED)
{
  /* Not yet implemented. */
}

/** Returns the current thread's nice value. */
int thread_get_nice(void)
{
  /* Not yet implemented. */
  return 0;
}

/** Returns 100 times the system load average. */
int thread_get_load_avg(void)
{
  /* Not yet implemented. */
  return 0;
}

/** Returns 100 times the current thread's recent_cpu value. */
int thread_get_recent_cpu(void)
{
  /* Not yet implemented. */
  return 0;
}

/** Function used as the basis for a kernel thread. */
static void
kernel_thread(thread_func *function, void *aux)
{
  ASSERT(function != NULL);

  intr_enable(); /**< The scheduler runs with interrupts off. */
  function(aux); /**< Execute the thread function. */
  thread_exit(); /**< If function() returns, kill the thread. */
}

/** Returns the running thread. */
struct thread *
running_thread(void)
{
  uint32_t *esp;

  asm("mov %%esp, %0" : "=g"(esp));
  return pg_round_down(esp);
}

/** Returns true if T appears to point to a valid thread. */
static bool
is_thread(struct thread *t)
{
  return t != NULL && t->magic == THREAD_MAGIC;
}

static void *
alloc_frame(struct thread *t, size_t size)
{
  /* Stack data is always allocated in word-size units. */
  ASSERT(is_thread(t));
  ASSERT(size % sizeof(uint32_t) == 0);

  t->stack -= size;
  return t->stack;
}

static struct thread *
next_thread_to_run(void)
{
  if (list_empty(&ready_list))
    return idle_thread;
  else
    return list_entry(list_pop_front(&ready_list), struct thread, elem);
}

void thread_schedule_tail(struct thread *prev)
{
  struct thread *cur = running_thread();

  ASSERT(intr_get_level() == INTR_OFF);

  /* Mark us as running. */
  cur->status = THREAD_RUNNING;

  /* Start new time slice. */
  thread_ticks = 0;

#ifdef USERPROG
  /* Activate the new address space. */
  process_activate();
#endif

  if (prev != NULL && prev->status == THREAD_DYING && prev != initial_thread)
  {
    ASSERT(prev != cur);
    palloc_free_page(prev);
  }
}

/** Returns a tid to use for a new thread. */
static tid_t
allocate_tid(void)
{
  static tid_t next_tid = 1;
  tid_t tid;

  lock_acquire(&tid_lock);
  tid = next_tid++;
  lock_release(&tid_lock);

  return tid;
}

uint32_t thread_stack_ofs = offsetof(struct thread, stack);