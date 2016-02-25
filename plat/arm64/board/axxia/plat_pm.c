/*
 * Copyright (c) 2013, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <arch_helpers.h>
#include <debug.h>
#include <errno.h>
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
#include <mmio.h>
#include <debug.h>

#include <axxia_def.h>
#include <axxia_private.h>

extern uint64_t axxia_sec_entry_point;

#define SYSCON_PSCRATCH     (SYSCON_BASE + 0x00dc)
#define SYSCON_RESET_KEY	(SYSCON_BASE + 0x2000)
#define SYSCON_RESET_CTRL	(SYSCON_BASE + 0x2008)
#define   RSTCTL_RST_CHIP       (1<<1)
#define   RSTCTL_RST_SYS        (1<<0)
#define SYSCON_RESET_HOLD	(SYSCON_BASE + 0x2010)
#define SYSCON_RESET_WO_SM  (SYSCON_BASE + 0x204c)

/*
 * The following platform setup functions are weakly defined. They
 * provide typical implementations that will be overridden by a SoC.
 */
#pragma weak axxia_soc_pwr_domain_suspend
#pragma weak axxia_soc_pwr_domain_on
#pragma weak axxia_soc_pwr_domain_off
#pragma weak axxia_soc_pwr_domain_on_finish
#pragma weak axxia_soc_prepare_system_reset

int axxia_soc_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	return PSCI_E_NOT_SUPPORTED;
}

int axxia_soc_pwr_domain_on(u_register_t mpidr)
{
	return PSCI_E_SUCCESS;
}

int axxia_soc_pwr_domain_off(const psci_power_state_t *target_state)
{
	return PSCI_E_SUCCESS;
}

int axxia_soc_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	return PSCI_E_SUCCESS;
}

int axxia_soc_prepare_system_reset(void)
{
	return PSCI_E_SUCCESS;
}


#if (ENABLE_PLAT_COMPAT == 1)
/*
 * Handler called when an affinity instance is about to be turned on. The level
 * and mpidr determine the affinity instance.
 */
static int
axxia_affinst_on(unsigned long mpidr, unsigned long sec_entrypoint,
		 unsigned int afflvl, unsigned int state)
{
	unsigned int id, hold;

	ERROR("Charlie %s-%d\n", __FILE__, __LINE__);
	switch (afflvl) {
	case MPIDR_AFFLVL1:
		id = (mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;
		break;
	case MPIDR_AFFLVL0:
		//id = platform_get_core_pos(mpidr);
		id = plat_core_pos_by_mpidr(mpidr);
		ERROR("Charlie %s-%d id=0x%x\n", __FILE__, __LINE__, id);
		mmio_write_32(0x8031000000,
			      (0x14000000 |
			       (sec_entrypoint - 0x8031000000) / 4));
		dsb();
		hold = mmio_read_32(SYSCON_RESET_HOLD);
		hold &= ~(1 << id);
		mmio_write_32(SYSCON_RESET_KEY, 0xab);
		mmio_write_32(SYSCON_RESET_HOLD, hold | (1 << id));
		mmio_write_32(SYSCON_RESET_HOLD, hold);
		mmio_write_32(SYSCON_RESET_KEY, 0x00);
		break;
	default:
		WARN("Unsupported affinity level: %u\n", afflvl);
		break;
	}

	return PSCI_E_SUCCESS;
}

/*
 * Handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 */

static void
axxia_affinst_on_finish(unsigned int afflvl, unsigned int state)
{
	unsigned int oslar_el1 = 0;
	unsigned int mpidr;

	ERROR("Charlie %s-%d\n", __FILE__, __LINE__);
	mpidr = read_mpidr();

	switch (afflvl) {
	case MPIDR_AFFLVL1:
		if (0 != set_cluster_coherency((mpidr >> MPIDR_AFF1_SHIFT) &
					       MPIDR_AFFLVL_MASK, 1))
			return;
		break;
	case MPIDR_AFFLVL0:
		/* Clear entrypoint branch */
		mmio_write_32(0x8031000000, 0);
		/* Enable the gic cpu interface */
		gic_cpuif_setup();
		/* Write the OS Debug Lock. */
		__asm__ __volatile__ ("msr OSLAR_EL1, %0" : : "r" (oslar_el1));
		break;
	default:
		WARN("Unsupported affinity level: %u\n", afflvl);
		break;
	}

	return;
}

/*
 * Handler called when an affinity instance is about to be turned off. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take
 * appropriate actions.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across
 * calls to this function as each affinity level is dealt with. So do not write
 * & read global variables across calls. It will be wise to do flush a write to
 * the global to prevent unpredictable results.
 */
static void
axxia_affinst_off(unsigned int afflvl, unsigned int state)
{
	unsigned int mpidr;

	ERROR("Charlie %s-%d Affinity: %d State:%d\n", __FILE__, __LINE__, afflvl, state);
	mpidr = read_mpidr();

	/* Prevent interrupts from spuriously waking up this cpu */
	gic_cpuif_deactivate(GICC_BASE);

	/* Cluster is to be turned off, so disable coherency */
	switch (afflvl) {
	case MPIDR_AFFLVL1:
		if (0 != set_cluster_coherency((mpidr >> MPIDR_AFF1_SHIFT) &
					       MPIDR_AFFLVL_MASK, 0))
			return;
		break;
	case MPIDR_AFFLVL0:
		break;
	default:
		WARN("Unsupported affinity level: %u\n", afflvl);
		break;
	}

	return;
}

/*
 * Handler called when an affinity instance is about to be suspended. The level
 * and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions. The 'sec_entrypoint' determines the address in BL3-1 from where
 * execution should resume.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across
 * calls to this function as each affinity level is dealt with. So do not write
 * & read global variables across calls. It will be wise to do flush a write to
 * the global to prevent unpredictable results.
 */
static void
axxia_affinst_suspend(unsigned long sec_entrypoint,
		      unsigned int afflvl,
		      unsigned int state)
{
	ERROR("Charlie %s-%d\n", __FILE__, __LINE__);
	axxia_affinst_off(afflvl, state);

	return;
}
#endif

static void __dead2 axxia_system_off(void)
{
	ERROR("Charlie %s-%d\n", __FILE__, __LINE__);
	/* Best we can do here */
	psci_power_down_wfi();
}

extern void __dead2 initiate_retention_reset(void);

static void __dead2 axxia_system_reset(void)
{
	unsigned int ctrl;
    unsigned int pscratch;

	ERROR("Charlie %s-%d\n", __FILE__, __LINE__);
	mmio_write_32(SYSCON_RESET_KEY, 0xab);

    /* check for DDR retention reset */
    pscratch = mmio_read_32(SYSCON_PSCRATCH);
    if (pscratch & 0x00000001) {
        initiate_retention_reset();
    }

	ctrl = mmio_read_32(SYSCON_RESET_CTRL);
	mmio_write_32(SYSCON_RESET_CTRL, ctrl | RSTCTL_RST_SYS);
	/* ...in case it fails */
	psci_power_down_wfi();
}

#if (ENABLE_PLAT_COMPAT == 1)

static const plat_pm_ops_t axxia_ops = {
	.affinst_on		= axxia_affinst_on,
	.affinst_on_finish	= axxia_affinst_on_finish,
	.affinst_off		= axxia_affinst_off,
	.affinst_suspend	= axxia_affinst_suspend,
	.affinst_suspend_finish	= axxia_affinst_on_finish,
	.system_off		= axxia_system_off,
	.system_reset		= axxia_system_reset
};


int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	/* Enable cross cluster events */
	mmio_write_32(SYSCON_BASE + 0x14, 0xffff);
	*plat_ops = &axxia_ops;
	return 0;
}
#endif

void __dead2 axxia_system_reset_wo_sm(void)
{
#ifdef CHARLIE_DEF
	unsigned int reg;

    mmio_write_32(SYSCON_RESET_WO_SM, 1);
    do {
        reg = mmio_read_32(SYSCON_RESET_WO_SM);
    } while (reg == 0);

	reg = mmio_read_32(SYSCON_RESET_CTRL);
	mmio_write_32(SYSCON_RESET_CTRL, reg | RSTCTL_RST_CHIP);
	/* ...in case it fails */
	psci_power_down_wfi();
#endif
	WARN("Called axxia_system_reset_wo_sm\n");
	while (1);


}

/*******************************************************************************
 * Function which implements the common FVP specific operations to power down a
 * cpu in response to a CPU_OFF or CPU_SUSPEND request.
 ******************************************************************************/
static void axxia_cpu_pwrdwn_common(void)
{
	/* Prevent interrupts from spuriously waking up this cpu */
	//plat_arm_gic_cpuif_disable();

	/* Program the power controller to power off this cpu. */
}

#ifdef CHARLIE_DEF
/*******************************************************************************
 * Function which implements the common FVP specific operations to power down a
 * cluster in response to a CPU_OFF or CPU_SUSPEND request.
 ******************************************************************************/
static void axxia_cluster_pwrdwn_common(void)
{
	uint64_t mpidr = read_mpidr_el1();

	/* Disable coherency if this cluster is to be turned off */
	axxia_cci_disable();

	/* Program the power controller to turn the cluster off */
	axxia_pwrc_write_pcoffr(mpidr);
}
#endif

#ifdef CHARLIE_DEF
static void axxia_power_domain_on_finish_common(const psci_power_state_t *target_state)
{
	unsigned long mpidr;

	assert(target_state->pwr_domain_state[ARM_PWR_LVL0] ==
					ARM_LOCAL_STATE_OFF);

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();

	/* Perform the common cluster specific operations */
	if (target_state->pwr_domain_state[ARM_PWR_LVL1] ==
					ARM_LOCAL_STATE_OFF) {
		/*
		 * This CPU might have woken up whilst the cluster was
		 * attempting to power down. In this case the FVP power
		 * controller will have a pending cluster power off request
		 * which needs to be cleared by writing to the PPONR register.
		 * This prevents the power controller from interpreting a
		 * subsequent entry of this cpu into a simple wfi as a power
		 * down request.
		 */
		axxia_pwrc_write_pponr(mpidr);

		/* Enable coherency if this cluster was off */
		axxia_cci_enable();
	}

	/*
	 * Clear PWKUPR.WEN bit to ensure interrupts do not interfere
	 * with a cpu power down unless the bit is set again
	 */
	axxia_pwrc_clr_wen(mpidr);
}
#endif


/*******************************************************************************
 * AXXIA handler called when a CPU is about to enter standby.
 ******************************************************************************/
void axxia_cpu_standby(plat_local_state_t cpu_state)
{
	WARN("Called axxia_cpu_standby\n");

	assert(cpu_state == ARM_LOCAL_STATE_RET);

	/*
	 * Enter standby state
	 * dsb is good practice before using wfi to enter low power states
	 */
	dsb();
	wfi();

}

/*******************************************************************************
 * AXXIA handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 ******************************************************************************/
int axxia_pwr_domain_on(u_register_t mpidr)
{
	//int cpu = mpidr & MPIDR_CPU_MASK;

#ifdef CHARLIE_DEF
	int rc = PSCI_E_SUCCESS;
	unsigned int psysr;

	/*
	 * Ensure that we do not cancel an inflight power off request for the
	 * target cpu. That would leave it in a zombie wfi. Wait for it to power
	 * off and then program the power controller to turn that CPU on.
	 */
	do {
		psysr = axxia_pwrc_read_psysr(mpidr);
	} while (psysr & PSYSR_AFF_L0);

	axxia_pwrc_write_pponr(mpidr);
	return rc;
#endif
	WARN("Called axxia_pwr_domain_on\n");
	return 0;
}

/*******************************************************************************
 * FVP handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void axxia_pwr_domain_off(const psci_power_state_t *target_state)
{

	assert(target_state->pwr_domain_state[ARM_PWR_LVL0] == ARM_LOCAL_STATE_OFF);

	unsigned long mpidr = read_mpidr_el1();
	unsigned long cpu = mpidr & MPIDR_CPU_MASK;
	ERROR("CHARLIE: %s-%d cpu=%ld mpidr=0x%lx\n", __FILE__, __LINE__, cpu, mpidr);


	/*
	 * If execution reaches this stage then this power domain will be
	 * suspended. Perform at least the cpu specific actions followed
	 * by the cluster specific operations if applicable.
	 */
	axxia_cpu_pwrdwn_common();

	//if (target_state->pwr_domain_state[ARM_PWR_LVL1] ==
	//				ARM_LOCAL_STATE_OFF);
		//axxia_cluster_pwrdwn_common();

}

/*******************************************************************************
 * FVP handler called when a power domain is about to be suspended. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void axxia_pwr_domain_suspend(const psci_power_state_t *target_state)
{
#ifdef CHARLIE_DEF
	unsigned long mpidr;

	/*
	 * FVP has retention only at cpu level. Just return
	 * as nothing is to be done for retention.
	 */
	if (target_state->pwr_domain_state[ARM_PWR_LVL0] ==
					ARM_LOCAL_STATE_RET)
		return;

	assert(target_state->pwr_domain_state[ARM_PWR_LVL0] ==
					ARM_LOCAL_STATE_OFF);

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();

	/* Program the power controller to enable wakeup interrupts. */
	axxia_pwrc_set_wen(mpidr);

	/* Perform the common cpu specific operations */
	axxia_cpu_pwrdwn_common();

	/* Perform the common cluster specific operations */
	if (target_state->pwr_domain_state[ARM_PWR_LVL1] ==
					ARM_LOCAL_STATE_OFF)
		axxia_cluster_pwrdwn_common();
#endif
	WARN("Called axxia_pwr_domain_suspend\n");
}

/*******************************************************************************
 * FVP handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
void axxia_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
#ifdef CHARLIE_DEF
	axxia_power_domain_on_finish_common(target_state);

	/* Enable the gic cpu interface */
	plat_arm_gic_pcpu_init();

	/* Program the gic per-cpu distributor or re-distributor interface */
	plat_arm_gic_cpuif_enable();
#endif
	WARN("Called axxia_pwr_domain_on_finish\n");
}

/*******************************************************************************
 * FVP handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
void axxia_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
#ifdef CHARLIE_DEF
	/*
	 * Nothing to be done on waking up from retention from CPU level.
	 */
	if (target_state->pwr_domain_state[ARM_PWR_LVL0] ==
					ARM_LOCAL_STATE_RET)
		return;

	axxia_power_domain_on_finish_common(target_state);

	/* Enable the gic cpu interface */
	plat_arm_gic_cpuif_enable();
#endif
	WARN("Called axxia_pwr_domain_suspend_finish\n");
}

int axxia_validate_power_state(unsigned int power_state,
			    psci_power_state_t *req_state)
{
	WARN("Called axxia_validate_power_state\n");
	return 0;
}

int axxia_validate_ns_entrypoint(uintptr_t entrypoint)
{
	WARN("Called axxia_validate_ns_entrypoint\n");
	return 0;
}

/*******************************************************************************
 * Export the platform handlers via plat_arm_psci_pm_ops. The ARM Standard
 * platform layer will take care of registering the handlers with PSCI.
 ******************************************************************************/
const plat_psci_ops_t axxia_psci_pm_ops = {
	.cpu_standby = axxia_cpu_standby,
	.pwr_domain_on = axxia_pwr_domain_on,
	.pwr_domain_off = axxia_pwr_domain_off,
	.pwr_domain_suspend = axxia_pwr_domain_suspend,
	.pwr_domain_on_finish = axxia_pwr_domain_on_finish,
	.pwr_domain_suspend_finish = axxia_pwr_domain_suspend_finish,
	.system_off = axxia_system_off,
	.system_reset = axxia_system_reset,
	.validate_power_state = axxia_validate_power_state,
	.validate_ns_entrypoint = axxia_validate_ns_entrypoint
};

#if (ENABLE_PLAT_COMPAT == 0)
/*******************************************************************************
 * Export the platform specific power ops and initialize Power Controller
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	psci_power_state_t target_state = { { PSCI_LOCAL_STATE_RUN } };

	/*
	 * Flush entrypoint variable to PoC since it will be
	 * accessed after a reset with the caches turned off.
	 */
	axxia_sec_entry_point = sec_entrypoint;
	flush_dcache_range((uint64_t)&axxia_sec_entry_point, sizeof(uint64_t));

	/*
	 * Reset hardware settings.
	 */
	axxia_soc_pwr_domain_on_finish(&target_state);

	/*
	 * Initialize PSCI ops struct
	 */
	*psci_ops = &axxia_psci_pm_ops;

	return 0;
}
#endif
