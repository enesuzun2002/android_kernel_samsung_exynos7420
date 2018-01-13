/*
 * drivers/cpufreq/cpufreq_nexus.c
 *
 * Copyright (C) 2017 Lukas Berger
 *
 * Uses code from cpufreq_alucard and cpufreq_interactive
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include "cpufreq_governor.h"

// enable debugging-mode
#define CPUGOV_NEXUS_DEBUG				0
// set to 0 for workqueues, 1 for kthreads
#define CPUGOV_NEXUS_KTHREAD				1
// use also may choose panic
#define CPUGOV_NEXUS_ERROR				WARN

static struct cpufreq_nexus_tunables *global_tunables = NULL;
static DEFINE_MUTEX(cpufreq_governor_nexus_mutex);

#if CPUGOV_NEXUS_DEBUG
#define nexus_debug(format, ...) \
	pr_info(format, ##__VA_ARGS__)
#else
#define nexus_debug(format, ...) \
	do { } while(0)
#endif

#define TASK_NAME_LEN 15

struct cpufreq_nexus_cpuinfo {
	int init;

	int cpu;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;

	cputime64_t prev_idle;
	cputime64_t prev_wall;

	u64 last_tick_time;

	unsigned int down_delay_counter;
	unsigned int up_delay_counter;
	unsigned int hispeed_delay_counter;

#if CPUGOV_NEXUS_KTHREAD
	struct task_struct *work;
#else
	struct delayed_work work;
#endif

	struct mutex timer_mutex;
};

struct cpufreq_nexus_tunables {
	// load at which the cpugov decides to scale down
	#define DEFAULT_DOWN_LOAD 40
	unsigned int down_load;

	// delay in timer-ticks to scale down CPU
	#define DEFAULT_DOWN_DELAY 0
	unsigned int down_delay;

	// frequency-steps if cpugov scales down
	#define DEFAULT_DOWN_STEP 1
	unsigned int down_step;

	// defines the required ratio IN HUNDREDS between down-load and current load
	// to trigger an enhanced down-step elevation, done steps defined by
	// [down_load_to_step_elevation]
	//
	//   notice:
	//
	// the load-to-step (lts) enhancement would start to trigger at a load of
	//
	// 		TRIGGER_LOAD = {  ([down_load] * 100) / [down_load_to_step_ratio]  }
	//
	//   remark #1:	 [TRIGGER_LOAD] needs to be checked according to current state
	//               of up/downscaling before computing and applying the elevation
	//
	//   remark #2:	 for upscaling, the correct value is the difference of [TRIGGER_LOAD]
	//               to [up_load] and adding [up_load] again:
	//
	//               		UP_TRIGGER_LOAD = {  [up_load] + ([up_load] - (([up_load] * 100) / [up_load_to_step_ratio]))  }
	//
	#define DEFAULT_DOWN_LOAD_TO_STEP_RATIO 125
	unsigned int down_load_to_step_ratio;

	// defined the down-steps taken if the load-to-step ratio is being triggered.
	// [down_load_to_step_ratio] is being accounted to the final lts-added
	//
	//   notice:
	//
	// the final step-elevation is being computed by using the current ratio
	// between [down_load] and the lts trigger-load by taking the ratio of
	// [down_load] and [TRIGGER_LOAD]. Afterwards, multiply if with the requeste
	// elevation-steps
	// 
	// 		ELEVATION_STEPS = {  (([down_load] * 100) / ([TRIGGER_LOAD] * 100)) * [down_load_to_step_elevation]  }
	//
	#define DEFAULT_DOWN_LOAD_TO_STEP_ELEVATION 1
	unsigned int down_load_to_step_elevation;

	// load at which the cpugov decides to scale up
	#define DEFAULT_UP_LOAD 50
	unsigned int up_load;

	// delay in timer-ticks to scale up CPU
	#define DEFAULT_UP_DELAY 0
	unsigned int up_delay;

	// frequency-steps if cpugov scales up
	#define DEFAULT_UP_STEP 1
	unsigned int up_step;

	// READ MORE AT [down_load_to_step_ratio]
	#define DEFAULT_UP_LOAD_TO_STEP_RATIO 125
	unsigned int up_load_to_step_ratio;

	// READ MORE AT [down_load_to_step_elevation]
	#define DEFAULT_UP_LOAD_TO_STEP_ELEVATION 2
	unsigned int up_load_to_step_elevation;

	// interval of the scaling-timer
	#define DEFAULT_TIMER_RATE 15000
	unsigned int timer_rate;

	// indicates if I/O-time should be added to cputime
	#define DEFAULT_IO_IS_BUSY 1
	int io_is_busy;

	// minimal frequency chosen by the cpugov
	unsigned int freq_min;
	int freq_min_do_revalidate;

	// maximal frequency chosen by the cpugov
	unsigned int freq_max;
	int freq_max_do_revalidate;

	// frequency used when governor is in boost-mode
	unsigned int boost_freq;
	int boost_freq_do_revalidate;

	// simple boost to freq_max
	#define DEFAULT_BOOST 0
	int boost;

	// time in usecs when current boostpulse ends
	u64 boostpulse;

	// ktime when current boostpulse ends
	u64 boostpulse_end;

	// time in usecs when current boostpulse ends
	#define DEFAULT_BOOSTPULSE_DURATION 50000
	u64 boostpulse_duration;

	// determines if the power-efficient frequency-selection should be used
	#define DEFAULT_POWER_EFFICIENT 1
	int power_efficient;

	// used frequency-coefficient to support SoCs with a non-linear frequency-tables
	#define DEFAULT_FREQUENCY_STEP 108000
	unsigned int frequency_step;

	// time without ticks in microseconds after which the governor fully revalidates the cputime and resets the frequency
	#define DEFAULT_RESET_STUCK_TIMESPAN DEFAULT_TIMER_RATE * 3
	unsigned int reset_stuck_timespan;

	// hispeed-frequency which can only be exceeded after persting hispeed-load
	unsigned int hispeed_freq;
	int hispeed_freq_do_revalidate;

	// load which is used to determine if cpugov should exceed hispeed-frequency
	#define DEFAULT_HISPEED_LOAD 100
	unsigned int hispeed_load;

	// delay in ticks after which th hispeed-exceeding is revalidated
	#define DEFAULT_HISPEED_DELAY 1
	unsigned int hispeed_delay;

	// indicates if hispeed-revalidation should work as power-efficient as possible
	#define DEFAULT_HISPEED_POWER_EFFICIENT 0
	unsigned int hispeed_power_efficient;
};

static DEFINE_PER_CPU(struct cpufreq_nexus_cpuinfo, gov_cpuinfo);

static unsigned int choose_frequency(struct cpufreq_nexus_cpuinfo *cpuinfo, int *index, unsigned int base_freq) {
	struct cpufreq_policy *policy;
	struct cpufreq_nexus_tunables *tunables;

	// checks have already been done in cpufreq_nexus_timer
	policy = cpuinfo->policy;
	tunables = policy->governor_data;

	if (tunables->power_efficient) {
		cpufreq_frequency_table_target(policy, cpuinfo->freq_table, base_freq,
			CPUFREQ_RELATION_H, index);
	} else {
		cpufreq_frequency_table_target(policy, cpuinfo->freq_table, base_freq,
			CPUFREQ_RELATION_C, index);
	}

	return cpuinfo->freq_table[*index].frequency;
}

static int cpufreq_nexus_timer(struct cpufreq_nexus_cpuinfo *cpuinfo, struct cpufreq_policy *policy, struct cpufreq_nexus_tunables *tunables, int is_stopping)
{
	unsigned int ktime_now = ktime_to_us(ktime_get());
	int ret = 0, /* 0: error/abort, 1: success */
	    cpu = 0,
	    load = 0,
	    freq_signed = 0,
		steps = 0;
	unsigned int index = 0,
	             freq = 0,
	             freq_next = 0,
	             lts_trigger_load = 0;
	cputime64_t curr_idle, curr_wall, idle, wall;

	int load_debug = 0;
	unsigned int freq_debug = 0;

	cpu = cpuinfo->cpu;

	if (mutex_lock_interruptible(&cpuinfo->timer_mutex))
		return 0;

	if (!cpu_online(cpu)) {
		ret = 0;
		goto exit;
	}

	// calculate new load
	curr_idle = get_cpu_idle_time(cpu, &curr_wall, tunables->io_is_busy);
	idle = (curr_idle - cpuinfo->prev_idle);
	wall = (curr_wall - cpuinfo->prev_wall);

	cpuinfo->prev_idle = curr_idle;
	cpuinfo->prev_wall = curr_wall;

	if (cpuinfo->init) {
		// apply the current cputimes and skip this sample
		cpuinfo->init = 0;

		ret = 1;
		goto exit;
	}

	// revalidate custom frequencies
	// -----
	// we don't have to care about concurrency as this isn't a
	// life-devastating routine if it is executed twice on different cores
	if (tunables->freq_min_do_revalidate) {
		freq_debug = tunables->freq_min;
		tunables->freq_min = choose_frequency(cpuinfo, &index, tunables->freq_min);
		tunables->freq_min_do_revalidate = 0;
		nexus_debug("%s: cpu%d: revalidated freq_min: %u -> %u\n", __func__, cpu, freq_debug, tunables->freq_min);
	}

	if (tunables->freq_max_do_revalidate) {
		freq_debug = tunables->freq_max;
		tunables->freq_max = choose_frequency(cpuinfo, &index, tunables->freq_max);
		tunables->freq_max_do_revalidate = 0;
		nexus_debug("%s: cpu%d: revalidated freq_max: %u -> %u\n", __func__, cpu, freq_debug, tunables->freq_max);
	}

	if (tunables->boost_freq_do_revalidate) {
		freq_debug = tunables->boost_freq;
		tunables->boost_freq = choose_frequency(cpuinfo, &index, tunables->boost_freq);
		tunables->boost_freq_do_revalidate = 0;
		nexus_debug("%s: cpu%d: revalidated boost_freq: %u -> %u\n", __func__, cpu, freq_debug, tunables->boost_freq);
	}

	if (tunables->hispeed_freq_do_revalidate && tunables->hispeed_freq != 0) {
		freq_debug = tunables->hispeed_freq;
		tunables->hispeed_freq = choose_frequency(cpuinfo, &index, tunables->hispeed_freq);
		tunables->hispeed_freq_do_revalidate = 0;
		nexus_debug("%s: cpu%d: revalidated hispeed_freq: %u -> %u\n", __func__, cpu, freq_debug, tunables->hispeed_freq);
	}

	// calculate frequencies
	nexus_debug("%s: cpu%d: init = %u\n", __func__, cpu, policy->cur);
	freq = policy->cur;

	if (wall >= idle) {
		load = (100 * (wall - idle)) / wall;
		load_debug = load;
		nexus_debug("%s: cpu%d: load = %u\n", __func__, cpu, load);

		if (tunables->boost || ktime_now < tunables->boostpulse_end) {
			nexus_debug("%s: cpu%d: boost = %u\n", __func__, cpu, tunables->boost_freq);
			freq = tunables->boost_freq;
		} else {
			if (load >= tunables->up_load) {
				if (tunables->up_delay == 0 || cpuinfo->up_delay_counter >= tunables->up_delay) {
					steps = (int)tunables->up_step;
					nexus_debug("%s: cpu%d: up steps         = %d\n", __func__, cpu, steps);
					nexus_debug("%s: cpu%d: up load          = %d\n", __func__, cpu, tunables->up_load);

					// load-to-step
					if (tunables->up_load_to_step_ratio > 0) {
						lts_trigger_load = tunables->up_load + (tunables->up_load - ((tunables->up_load * 100) / tunables->up_load_to_step_ratio));
						nexus_debug("%s: cpu%d: lts up trg load  = %d\n", __func__, cpu, lts_trigger_load);
						if (load >= lts_trigger_load) {
							int lts_ratio = DIV_ROUND_UP(lts_trigger_load * 100, load * 100);

							steps += lts_ratio * tunables->up_load_to_step_elevation;
							nexus_debug("%s: cpu%d: lts up trg       = %d / %d\n", __func__, cpu, lts_ratio, steps);
						}
					}

					freq_signed = (int)freq + (steps * (int)tunables->frequency_step);
					nexus_debug("%s: cpu%d: up-scaling       = %d\n", __func__, cpu, freq_signed);

					if (freq_signed > (int)policy->max)
						freq_signed = (int)policy->max;

					freq = (unsigned int)freq_signed;
					cpuinfo->up_delay_counter = 0;

					nexus_debug("%s: cpu%d: up-scaling corr. = %u\n", __func__, cpu, freq);
				} else if (tunables->up_delay > 0) {
					cpuinfo->up_delay_counter++;
				}
				cpuinfo->down_delay_counter = 0;
			} else if (load <= tunables->down_load) {
				if (tunables->down_delay == 0 || cpuinfo->down_delay_counter >= tunables->down_delay) {
					steps = (int)tunables->down_step;
					nexus_debug("%s: cpu%d: down steps       = %d\n", __func__, cpu, steps);
					nexus_debug("%s: cpu%d: down load        = %d\n", __func__, cpu, tunables->down_load);

					// load-to-step
					if (tunables->down_load_to_step_ratio > 0) {
						lts_trigger_load = ((tunables->down_load * 100) / tunables->down_load_to_step_ratio);
						nexus_debug("%s: cpu%d: lts dwn trg load = %d\n", __func__, cpu, lts_trigger_load);
						if (load <= lts_trigger_load) {
							int lts_ratio = DIV_ROUND_UP(lts_trigger_load * 100, load * 100);

							steps += lts_ratio * tunables->down_load_to_step_elevation;
							nexus_debug("%s: cpu%d: lts dwn trg        = %d / %d\n", __func__, cpu, lts_ratio, steps);
						}
					}

					freq_signed = (int)freq - (steps * (int)tunables->frequency_step);
					nexus_debug("%s: cpu%d: down-scaling       = %d\n", __func__, cpu, freq_signed);

					if (freq_signed < (int)policy->min)
						freq_signed = (int)policy->min;

					freq = (unsigned int)freq_signed;
					cpuinfo->down_delay_counter = 0;

					nexus_debug("%s: cpu%d: down-scaling corr. = %u\n", __func__, cpu, freq);
				} else if (tunables->down_delay > 0) {
					cpuinfo->down_delay_counter++;
				}
				cpuinfo->up_delay_counter = 0;
			} else {
				cpuinfo->up_delay_counter = 0;
				cpuinfo->down_delay_counter = 0;
			}

			// dynamic hispeed frequency limiting
			if (tunables->hispeed_freq > 0 && tunables->hispeed_delay > 0) {
				if (cpuinfo->hispeed_delay_counter < tunables->hispeed_delay) {
					freq_signed = (int)tunables->hispeed_freq;
					if (freq_signed < (int)freq)
						freq = (unsigned int)freq_signed;

					if (load < tunables->hispeed_load && tunables->hispeed_power_efficient) {
						cpuinfo->hispeed_delay_counter = 0;
					} else {
						cpuinfo->hispeed_delay_counter++;
					}
				} else {
					if (load < tunables->hispeed_load) {
						cpuinfo->hispeed_delay_counter = 0;
					}
				}
			}

			// apply tunables
			if (freq < tunables->freq_min) {
				freq_debug = freq;
				freq = max(policy->min, tunables->freq_min);
				nexus_debug("%s: cpu%d: frequency %u is lower than allowed! scaling to %u\n", __func__, cpu, freq_debug, freq);
			}

			if (freq > tunables->freq_max){
				freq_debug = freq;
				freq = min(policy->max, tunables->freq_max);
				nexus_debug("%s: cpu%d: frequency %u is higher than allowed! scaling to %u\n", __func__, cpu, freq_debug, freq);
			}
		}

		// check if the policy is very unresponsive and reset it if that's the case
		if (cpuinfo->last_tick_time + tunables->reset_stuck_timespan <= ktime_now) {
			freq = max(policy->min, tunables->freq_min);
			nexus_debug("%s: cpu%d: resetting policy because of reaching reset_stuck_timespan\n", __func__, cpu);
		}

		// choose lowest allowed frequency when cpugov is about to stop
		if (is_stopping) {
			// decide between absolute lowest frequency or user-set minimal frequency

			/* freq = policy->min; */
			freq = max(policy->min, tunables->freq_min);
		}

		// choose frequency
		freq_next = choose_frequency(cpuinfo, &index, freq);
		nexus_debug("%s: cpu%d: === freq = %u ===\n", __func__, cpu, freq_next);

		if (freq_next != policy->cur) {
			__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
		}
	}

	ret = 1;
exit:
	cpuinfo->last_tick_time = ktime_now;
	mutex_unlock(&cpuinfo->timer_mutex);
	return ret;
}

#if CPUGOV_NEXUS_KTHREAD
static int cpufreq_nexus_task(void *data)
{
	int cpu = -1, ret = 0;
	struct cpufreq_nexus_cpuinfo *cpuinfo;
	struct cpufreq_policy *policy;
	struct cpufreq_nexus_tunables *tunables;

	cpu = smp_processor_id();

	cpuinfo = &per_cpu(gov_cpuinfo, cpu);
	if (!cpuinfo) {
		CPUGOV_NEXUS_ERROR("%s: cpuinfo is null", __func__);
		return 0;
	}

	policy = cpuinfo->policy;
	if (!policy) {
		CPUGOV_NEXUS_ERROR("%s: policy is null", __func__);
		return 0;
	}

	tunables = policy->governor_data;

	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		/* main task */
		ret = cpufreq_nexus_timer(cpuinfo, policy, tunables, 0 /* is_stopping */);

		/* wait */
		usleep(tunables->timer_rate);
	}

	/* prepare cpu for sleep */
	cpufreq_nexus_timer(cpuinfo, policy, tunables, 1 /* is_stopping */);

	return 0;
}
#else
static void cpufreq_nexus_task(struct work_struct *work)
{
	int delay = 0, cpu = 0, ret = 0;
	struct cpufreq_nexus_cpuinfo *cpuinfo;
	struct cpufreq_policy *policy;
	struct cpufreq_nexus_tunables *tunables;

	cpuinfo = container_of(work, struct cpufreq_nexus_cpuinfo, work.work);
	if (!cpuinfo) {
		CPUGOV_NEXUS_ERROR("%s: cpuinfo is null", __func__);
		return;
	}

	policy = cpuinfo->policy;
	if (!policy) {
		CPUGOV_NEXUS_ERROR("%s: policy is null", __func__);
		return;
	}

	tunables = policy->governor_data;
	cpu = cpuinfo->cpu;

	/* main task */
	ret = cpufreq_nexus_timer(cpuinfo, policy, tunables, 0 /* is_stopping */);

	if (ret) {
		delay = usecs_to_jiffies(tunables->timer_rate < 1000 ? 1000 : tunables->timer_rate);
		if (num_online_cpus() > 1) {
			delay -= jiffies % delay;
		}
	}

	queue_delayed_work_on(cpu, system_wq, &cpuinfo->work, delay);
}
#endif

#define gov_show_store(_name) \
	gov_show(_name);          \
	gov_store(_name)

#define gov_sys_pol_show_store(_name)                                         \
	gov_sys_show(_name);                                                      \
	gov_sys_store(_name);                                                     \
	gov_pol_show(_name);                                                      \
	gov_pol_store(_name);                                                     \
	static struct global_attr _name##_gov_sys =                               \
		__ATTR(_name, 0666, show_##_name##_gov_sys, store_##_name##_gov_sys); \
	static struct freq_attr _name##_gov_pol =                                 \
		__ATTR(_name, 0666, show_##_name##_gov_pol, store_##_name##_gov_pol)

#define gov_sys_pol_store(_name)                            \
	gov_sys_store(_name);                                   \
	gov_pol_store(_name);                                   \
	static struct global_attr _name##_gov_sys =             \
		__ATTR(_name, 0666, NULL, store_##_name##_gov_sys); \
	static struct freq_attr _name##_gov_pol =               \
		__ATTR(_name, 0666, NULL, store_##_name##_gov_pol)

/*
 * Show-Macros
 */
#define gov_show(_name)                                                 \
static ssize_t show_##_name                                             \
(struct cpufreq_nexus_tunables *tunables, char *buf)                    \
{                                                                       \
	return sprintf(buf, "%llu\n", (unsigned long long)tunables->_name); \
}

#define gov_sys_show(_name)                               \
static ssize_t show_##_name##_gov_sys                     \
(struct kobject *kobj, struct attribute *attr, char *buf) \
{                                                         \
	return show_##_name(global_tunables, buf);          \
}

#define gov_pol_show(_name)                                                             \
static ssize_t show_##_name##_gov_pol                                                   \
(struct cpufreq_policy *policy, char *buf)                                              \
{                                                                                       \
	return show_##_name((struct cpufreq_nexus_tunables *)policy->governor_data, buf); \
}

/*
 * Store-Macros
 */
#define gov_store(_name)                                                 \
static ssize_t store_##_name                                             \
(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count) \
{                                                                        \
	unsigned long val = 0;                                               \
	int ret = kstrtoul(buf, 0, &val);                                    \
	if (ret < 0)                                                         \
		return ret;                                                      \
	tunables->_name = val;                                               \
	return count;                                                        \
}

#define gov_sys_store(_name)                                                  \
static ssize_t store_##_name##_gov_sys                                        \
(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count) \
{                                                                             \
	return store_##_name(global_tunables, buf, count);                      \
}

#define gov_pol_store(_name)                                                                    \
static ssize_t store_##_name##_gov_pol                                                          \
(struct cpufreq_policy *policy, const char *buf, size_t count)                                  \
{                                                                                               \
	return store_##_name((struct cpufreq_nexus_tunables *)policy->governor_data, buf, count); \
}

static ssize_t show_freq_min(struct cpufreq_nexus_tunables *tunables, char *buf)
{
	return sprintf(buf, "%u\n", tunables->freq_min);
}

static ssize_t store_freq_min(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->freq_min = val;
	tunables->freq_min_do_revalidate = 1;

	return count;
}

static ssize_t show_freq_max(struct cpufreq_nexus_tunables *tunables, char *buf)
{
	return sprintf(buf, "%u\n", tunables->freq_max);
}

static ssize_t store_freq_max(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->freq_max = val;
	tunables->freq_max_do_revalidate = 1;

	return count;
}

static ssize_t show_boost_freq(struct cpufreq_nexus_tunables *tunables, char *buf)
{
	return sprintf(buf, "%u\n", tunables->boost_freq);
}

static ssize_t store_boost_freq(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->boost_freq = val;
	tunables->boost_freq_do_revalidate = 1;

	return count;
}

static ssize_t show_hispeed_freq(struct cpufreq_nexus_tunables *tunables, char *buf)
{
	return sprintf(buf, "%u\n", tunables->hispeed_freq);
}

static ssize_t store_hispeed_freq(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->hispeed_freq = val;
	tunables->hispeed_freq_do_revalidate = 1;

	return count;
}

static ssize_t store_boostpulse(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->boostpulse_end = ktime_to_us(ktime_get()) + tunables->boostpulse_duration;
	return count;
}

gov_show_store(down_load);
gov_show_store(down_delay);
gov_show_store(down_step);
gov_show_store(down_load_to_step_ratio);
gov_show_store(down_load_to_step_elevation);
gov_show_store(up_load);
gov_show_store(up_delay);
gov_show_store(up_step);
gov_show_store(up_load_to_step_ratio);
gov_show_store(up_load_to_step_elevation);
gov_show_store(timer_rate);
gov_show_store(io_is_busy);
gov_show_store(boost);
gov_show_store(boostpulse_duration);
gov_show_store(power_efficient);
gov_show_store(frequency_step);
gov_show_store(reset_stuck_timespan);
gov_show_store(hispeed_load);
gov_show_store(hispeed_delay);
gov_show_store(hispeed_power_efficient);

gov_sys_pol_show_store(down_load);
gov_sys_pol_show_store(down_delay);
gov_sys_pol_show_store(down_step);
gov_sys_pol_show_store(down_load_to_step_ratio);
gov_sys_pol_show_store(down_load_to_step_elevation);
gov_sys_pol_show_store(up_load);
gov_sys_pol_show_store(up_delay);
gov_sys_pol_show_store(up_step);
gov_sys_pol_show_store(up_load_to_step_ratio);
gov_sys_pol_show_store(up_load_to_step_elevation);
gov_sys_pol_show_store(timer_rate);
gov_sys_pol_show_store(io_is_busy);
gov_sys_pol_show_store(freq_min);
gov_sys_pol_show_store(freq_max);
gov_sys_pol_show_store(boost_freq);
gov_sys_pol_show_store(boost);
gov_sys_pol_store(boostpulse);
gov_sys_pol_show_store(boostpulse_duration);
gov_sys_pol_show_store(power_efficient);
gov_sys_pol_show_store(frequency_step);
gov_sys_pol_show_store(reset_stuck_timespan);
gov_sys_pol_show_store(hispeed_freq);
gov_sys_pol_show_store(hispeed_load);
gov_sys_pol_show_store(hispeed_delay);
gov_sys_pol_show_store(hispeed_power_efficient);

static struct attribute *attributes_gov_sys[] = {
	&down_load_gov_sys.attr,
	&down_delay_gov_sys.attr,
	&down_step_gov_sys.attr,
	&down_load_to_step_ratio_gov_sys.attr,
	&down_load_to_step_elevation_gov_sys.attr,
	&up_load_gov_sys.attr,
	&up_delay_gov_sys.attr,
	&up_step_gov_sys.attr,
	&up_load_to_step_ratio_gov_sys.attr,
	&up_load_to_step_elevation_gov_sys.attr,
	&timer_rate_gov_sys.attr,
	&io_is_busy_gov_sys.attr,
	&freq_min_gov_sys.attr,
	&freq_max_gov_sys.attr,
	&boost_freq_gov_sys.attr,
	&boost_gov_sys.attr,
	&boostpulse_gov_sys.attr,
	&boostpulse_duration_gov_sys.attr,
	&power_efficient_gov_sys.attr,
	&frequency_step_gov_sys.attr,
	&reset_stuck_timespan_gov_sys.attr,
	&hispeed_freq_gov_sys.attr,
	&hispeed_load_gov_sys.attr,
	&hispeed_delay_gov_sys.attr,
	&hispeed_power_efficient_gov_sys.attr,
	NULL // NULL has to be terminating entry
};

static struct attribute_group attribute_group_gov_sys = {
	.attrs = attributes_gov_sys,
	.name = "nexus",
};

static struct attribute *attributes_gov_pol[] = {
	&down_load_gov_pol.attr,
	&down_delay_gov_pol.attr,
	&down_step_gov_pol.attr,
	&down_load_to_step_ratio_gov_pol.attr,
	&down_load_to_step_elevation_gov_pol.attr,
	&up_load_gov_pol.attr,
	&up_delay_gov_pol.attr,
	&up_step_gov_pol.attr,
	&up_load_to_step_ratio_gov_pol.attr,
	&up_load_to_step_elevation_gov_pol.attr,
	&timer_rate_gov_pol.attr,
	&io_is_busy_gov_pol.attr,
	&freq_min_gov_pol.attr,
	&freq_max_gov_pol.attr,
	&boost_freq_gov_pol.attr,
	&boost_gov_pol.attr,
	&boostpulse_gov_pol.attr,
	&boostpulse_duration_gov_pol.attr,
	&power_efficient_gov_pol.attr,
	&frequency_step_gov_pol.attr,
	&reset_stuck_timespan_gov_pol.attr,
	&hispeed_freq_gov_pol.attr,
	&hispeed_load_gov_pol.attr,
	&hispeed_delay_gov_pol.attr,
	&hispeed_power_efficient_gov_pol.attr,
	NULL // NULL has to be terminating entry
};

static struct attribute_group attribute_group_gov_pol = {
	.attrs = attributes_gov_pol,
	.name = "nexus",
};

static struct attribute_group *get_attribute_group(void) {
	if (have_governor_per_policy())
		return &attribute_group_gov_pol;
	else
		return &attribute_group_gov_sys;
}

static int cpufreq_governor_nexus(struct cpufreq_policy *policy, unsigned int event) {
	int rc = 0;
	int cpu, delay, work_cpu;
	struct cpufreq_nexus_cpuinfo *cpuinfo;
	struct cpufreq_nexus_tunables *tunables;
#if CPUGOV_NEXUS_KTHREAD
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	char cpufreq_nexus_task_name[TASK_NAME_LEN];
#endif

	cpu = policy->cpu;

	switch (event) {
		case CPUFREQ_GOV_POLICY_INIT:
			nexus_debug("%s: received CPUFREQ_GOV_POLICY_INIT for cpu%d\n", __func__, cpu);
			mutex_lock(&cpufreq_governor_nexus_mutex);

			if (!have_governor_per_policy()) {
				tunables = global_tunables;
			}

			tunables = kzalloc(sizeof(struct cpufreq_nexus_tunables), GFP_KERNEL);
			if (!tunables) {
				pr_err("%s: POLICY_INIT: kzalloc failed\n", __func__);
				mutex_unlock(&cpufreq_governor_nexus_mutex);
				return -ENOMEM;
			}

			tunables->down_load = DEFAULT_DOWN_LOAD;
			tunables->down_delay = DEFAULT_DOWN_DELAY;
			tunables->down_step = DEFAULT_DOWN_STEP;
			tunables->down_load_to_step_ratio = DEFAULT_DOWN_LOAD_TO_STEP_RATIO;
			tunables->down_load_to_step_elevation = DEFAULT_DOWN_LOAD_TO_STEP_ELEVATION;
			tunables->up_load = DEFAULT_UP_LOAD;
			tunables->up_delay = DEFAULT_UP_DELAY;
			tunables->up_step = DEFAULT_UP_STEP;
			tunables->up_load_to_step_ratio = DEFAULT_UP_LOAD_TO_STEP_RATIO;
			tunables->up_load_to_step_elevation = DEFAULT_UP_LOAD_TO_STEP_ELEVATION;
			tunables->timer_rate = DEFAULT_TIMER_RATE;
			tunables->io_is_busy = DEFAULT_IO_IS_BUSY;
			tunables->freq_min = policy->min;
			tunables->freq_max = policy->max;
			tunables->boost_freq = policy->max;
			tunables->boost = DEFAULT_BOOST;
			tunables->boostpulse = 0;
			tunables->boostpulse_duration = DEFAULT_BOOSTPULSE_DURATION;
			tunables->power_efficient = DEFAULT_POWER_EFFICIENT;
			tunables->frequency_step = DEFAULT_FREQUENCY_STEP;
			tunables->reset_stuck_timespan = DEFAULT_RESET_STUCK_TIMESPAN;
			tunables->hispeed_freq = policy->max;
			tunables->hispeed_load = DEFAULT_HISPEED_LOAD;
			tunables->hispeed_delay = DEFAULT_HISPEED_DELAY;
			tunables->hispeed_power_efficient = DEFAULT_HISPEED_POWER_EFFICIENT;

			rc = sysfs_create_group(get_governor_parent_kobj(policy), get_attribute_group());
			if (rc) {
				pr_err("%s: POLICY_INIT: sysfs_create_group failed\n", __func__);
				kfree(tunables);
				mutex_unlock(&cpufreq_governor_nexus_mutex);
				return rc;
			}

			policy->governor_data = tunables;

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;

		case CPUFREQ_GOV_POLICY_EXIT:
			nexus_debug("%s: received CPUFREQ_GOV_POLICY_EXIT for cpu%d\n", __func__, cpu);
			cpuinfo = &per_cpu(gov_cpuinfo, cpu);
			tunables = policy->governor_data;

			mutex_lock(&cpufreq_governor_nexus_mutex);
			sysfs_remove_group(get_governor_parent_kobj(policy), get_attribute_group());

			if (tunables)
				kfree(tunables);

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;

		case CPUFREQ_GOV_START:
			nexus_debug("%s: received CPUFREQ_GOV_START for cpu%d\n", __func__, cpu);
			if (!cpu_online(cpu) || !policy->cur)
				return -EINVAL;

			mutex_lock(&cpufreq_governor_nexus_mutex);

			for_each_cpu(work_cpu, policy->cpus) {

				cpuinfo = &per_cpu(gov_cpuinfo, work_cpu);
				tunables = policy->governor_data;

				cpuinfo->cpu = work_cpu;
				cpuinfo->freq_table = cpufreq_frequency_get_table(cpu);
				cpuinfo->policy = policy;
				cpuinfo->init = 1;

				mutex_init(&cpuinfo->timer_mutex);

				delay = usecs_to_jiffies(tunables->timer_rate);
				if (num_online_cpus() > 1) {
					delay -= jiffies % delay;
				}

#if CPUGOV_NEXUS_KTHREAD
				snprintf(cpufreq_nexus_task_name, TASK_NAME_LEN, "cpufreq-nexus%d\n", work_cpu);

				cpuinfo->work = kthread_create(cpufreq_nexus_task, NULL, cpufreq_nexus_task_name);
				kthread_bind(cpuinfo->work, work_cpu);

				sched_setscheduler_nocheck(cpuinfo->work, SCHED_FIFO, &param);
				get_task_struct(cpuinfo->work);

				wake_up_process(cpuinfo->work);
#else
				INIT_DEFERRABLE_WORK(&cpuinfo->work, cpufreq_nexus_timer);
				queue_delayed_work_on(work_cpu, system_wq, &cpuinfo->work, delay);
#endif

			};

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;

		case CPUFREQ_GOV_STOP:
			nexus_debug("%s: received CPUFREQ_GOV_STOP for cpu%d\n", __func__, cpu);

			for_each_cpu(work_cpu, policy->cpus) {

				cpuinfo = &per_cpu(gov_cpuinfo, work_cpu);
				tunables = policy->governor_data;

#if CPUGOV_NEXUS_KTHREAD
				kthread_stop(cpuinfo->work);
				put_task_struct(cpuinfo->work);
				cpuinfo->work = NULL;
#else
				cancel_delayed_work_sync(&cpuinfo->work);
#endif
			}

			break;

		case CPUFREQ_GOV_LIMITS:
			nexus_debug("%s: received CPUFREQ_GOV_LIMITS for cpu%d\n", __func__, cpu);
			mutex_lock(&cpufreq_governor_nexus_mutex);

			cpuinfo = &per_cpu(gov_cpuinfo, cpu);
			if (policy->max < cpuinfo->policy->cur) {
				__cpufreq_driver_target(cpuinfo->policy,
					policy->max, CPUFREQ_RELATION_H);
			} else if (policy->min > cpuinfo->policy->cur) {
				__cpufreq_driver_target(cpuinfo->policy,
					policy->min, CPUFREQ_RELATION_L);
			}

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_NEXUS
static
#endif
struct cpufreq_governor cpufreq_gov_nexus = {
	.name = "nexus",
	.governor = cpufreq_governor_nexus,
	.owner = THIS_MODULE,
};

static int __init cpufreq_nexus_init(void) {
	return cpufreq_register_governor(&cpufreq_gov_nexus);
}

static void __exit cpufreq_nexus_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_nexus);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_NEXUS
fs_initcall(cpufreq_nexus_init);
#else
module_init(cpufreq_nexus_init);
#endif
module_exit(cpufreq_nexus_exit);

MODULE_AUTHOR("Lukas Berger <mail@lukasberger.at>");
MODULE_DESCRIPTION("'cpufreq_nexus' - cpufreq-governor for interactive "
	"and battery-based devices");
MODULE_LICENSE("GPL");
