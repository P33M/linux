/*
 * irq_fuzzer.c
 *
 *  Created on: 4 Apr 2014
 *      Author: Jonathan2
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/irqflags.h>
#include <linux/printk.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/random.h>

static int fuzz_irqs(void *unused)
{
	unsigned int disable_delta = 0;
	unsigned long flags;
	while (1) {
		/* Randomly disable IRQs for between 10uS - 1000uS. */
		get_random_bytes(&disable_delta, sizeof(disable_delta));
		/* Get a sensible range. shift so that the max is 1024 */
		disable_delta = disable_delta >> 22;
		if (disable_delta < 10)
			disable_delta = 10;
		//pr_info("irq_fuzzer: delta = %d\n", disable_delta);
		local_irq_save(flags);
		udelay(disable_delta);
		local_irq_restore(flags);

		usleep_range(200, 1000);

		if (kthread_should_stop())
			break;
	}
	return 0;
}

static struct task_struct *tsk = NULL;


static int __init fuzz_irqs_init(void)
{

	pr_info("fuzz_irqs: probe\n");
	tsk = kthread_run(&fuzz_irqs, NULL, "fuzz_irqs");
	if (!tsk) {
		pr_err("fuzz_irqs: kthread creation failed\n");
		return -1;
	}
	return 0;
}

static void __exit fuzz_irqs_remove(void)
{
	int ret = kthread_stop(tsk);
	if(ret) {
		pr_err("fuzz_irqs: removal retval %d\n", ret);
	} else {
		pr_info("fuzz_irqs: stopped\n");
	}
	return;
}

module_init(fuzz_irqs_init);
module_exit(fuzz_irqs_remove);

MODULE_LICENSE("GPL");
