#ifndef _SCHED_RT_H
#define _SCHED_RT_H

#include <linux/sched/prio.h>

#define MAX_NICE	19
#define MIN_NICE	-20

/*
 * Convert user-nice values [ -20 ... 0 ... 19 ]
 * to static priority [ MAX_RT_PRIO..MAX_PRIO-1 ],
 * and back.
 */
#define NICE_TO_PRIO(nice)	(MAX_RT_PRIO + (nice) + 20)
#define PRIO_TO_NICE(prio)	((prio) - MAX_RT_PRIO - 20)
#define TASK_NICE(p)		PRIO_TO_NICE((p)->static_prio)

static inline int rt_prio(int prio)
{
	if (unlikely(prio < MAX_RT_PRIO))
		return 1;
	return 0;
}

static inline int rt_task(struct task_struct *p)
{
	return rt_prio(p->prio);
}

#ifdef CONFIG_RT_MUTEXES
extern int rt_mutex_getprio(struct task_struct *p);
extern void rt_mutex_setprio(struct task_struct *p, int prio);
extern void rt_mutex_adjust_pi(struct task_struct *p);
static inline bool tsk_is_pi_blocked(struct task_struct *tsk)
{
	return tsk->pi_blocked_on != NULL;
}
#else
static inline int rt_mutex_getprio(struct task_struct *p)
{
	return p->normal_prio;
}
# define rt_mutex_adjust_pi(p)		do { } while (0)
static inline bool tsk_is_pi_blocked(struct task_struct *tsk)
{
	return false;
}
#endif

extern void normalize_rt_tasks(void);


/*
 * default timeslice is 100 msecs (used only for SCHED_RR tasks).
 * Timeslices get refilled after they expire.
 */
#define RR_TIMESLICE		(100 * HZ / 1000)

/*
 * Convert nice value [19,-20] to rlimit style value [1,40].
 */
#define nice_to_rlimit(nice) (MAX_NICE - nice + 1)

/*
 * Convert rlimit style value [1,40] to nice value [-20, 19].
 */
#define rlimit_to_nice(prio) (MAX_NICE - prio + 1)

#endif /* _SCHED_RT_H */
