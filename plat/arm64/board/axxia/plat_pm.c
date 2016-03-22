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
#include "include/axxia_pwrc.h"

extern uint64_t axxia_sec_entry_point;
extern int axxia_pwrc_validate_power_state(unsigned int power_state, psci_power_state_t *req_state);

#define SYSCON_PSCRATCH     (SYSCON_BASE + 0x00dc)
#define SYSCON_RESET_KEY	(SYSCON_BASE + 0x2000)
#define SYSCON_RESET_CTRL	(SYSCON_BASE + 0x2008)
#define   RSTCTL_RST_CHIP       (1<<1)
#define   RSTCTL_RST_SYS        (1<<0)
#define SYSCON_RESET_HOLD	(SYSCON_BASE + 0x2010)
#define SYSCON_RESET_WO_SM  (SYSCON_BASE + 0x204c)


static void __dead2 axxia_system_off(void)
{
	/* Best we can do here */
	psci_power_down_wfi();
}

extern void __dead2 initiate_retention_reset(void);


static void __dead2 axxia_system_reset(void)
{
	unsigned int ctrl;
	unsigned int pscratch;

	INFO("axxia_system_reset called\n");
	mmio_write_32(SYSCON_RESET_KEY, 0xab);

	/* check for DDR retention reset */
	pscratch = mmio_read_32(SYSCON_PSCRATCH);
	if (pscratch & 0x00000001)
		initiate_retention_reset();

	ctrl = mmio_read_32(SYSCON_RESET_CTRL);
	mmio_write_32(SYSCON_RESET_CTRL, ctrl | RSTCTL_RST_SYS);
	/* ...in case it fails */
	psci_power_down_wfi();
}


void __dead2 axxia_system_reset_wo_sm(void)
{
	unsigned int reg;

	mmio_write_32(SYSCON_RESET_WO_SM, 1);
	do {
		reg = mmio_read_32(SYSCON_RESET_WO_SM);
	} while (reg == 0);

	reg = mmio_read_32(SYSCON_RESET_CTRL);
	mmio_write_32(SYSCON_RESET_CTRL, reg | RSTCTL_RST_CHIP);
	/* ...in case it fails */
	psci_power_down_wfi();

	WARN("Called axxia_system_reset_wo_sm\n");
	while (1)
		;
}


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
	int rc = PSCI_E_SUCCESS;

	int cpu = mpidr & MPIDR_CPU_MASK;

	rc = axxia_pwrc_cpu_powerup(cpu);

	return rc;
}

/*******************************************************************************
 * FVP handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void axxia_pwr_domain_off(const psci_power_state_t *target_state)
{

	assert(target_state->pwr_domain_state[ARM_PWR_LVL0] == PLAT_MAX_OFF_STATE);

	/* Get the mpidr for this cpu */
	unsigned long mpidr = read_mpidr();
	unsigned long cpu = mpidr & MPIDR_CPU_MASK;

	/* Prevent interrupts from spuriously waking up this cpu */
	gic_cpuif_deactivate(GICC_BASE);

	/* Perform the common cluster specific operations */
	if (target_state->pwr_domain_state[ARM_PWR_LVL1] != PLAT_MAX_OFF_STATE) {

		/* Cluster is to be turned off, so disable coherency */
		if (0 != set_cluster_coherency((mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK, 0))
			return;

		/*
		 * If execution reaches this stage then this power domain will be
		 * suspended. Perform at least the cpu specific actions followed
		 * by the cluster specific operations if applicable.
		 */
		axxia_pwrc_cpu_shutdown(cpu);

	}

	return;

}

/*******************************************************************************
 * FVP handler called when a power domain is about to be suspended. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void axxia_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	/*
	 * FVP has retention only at cpu level. Just return
	 * as nothing is to be done for retention.
	 */
	if (target_state->pwr_domain_state[ARM_PWR_LVL0] == ARM_LOCAL_STATE_RET)
		return;

	assert(target_state->pwr_domain_state[ARM_PWR_LVL0] == ARM_LOCAL_STATE_OFF);

	axxia_cpu_standby(target_state->pwr_domain_state[ARM_PWR_LVL0]);


	WARN("Called axxia_pwr_domain_suspend\n");
}

/*******************************************************************************
 * FVP handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
void axxia_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	unsigned int oslar_el1 = 0;
	unsigned int mpidr;

	if (target_state->pwr_domain_state[ARM_PWR_LVL0] == PLAT_MAX_OFF_STATE) {

		/* Clear entrypoint branch */
		mmio_write_32(0x8031000000, 0);
		/* Enable the gic cpu interface */
		gic_cpuif_setup();
		/* Write the OS Debug Lock. */
		__asm__ __volatile__ ("msr OSLAR_EL1, %0" : : "r" (oslar_el1));

	}

	/* Perform the common cluster specific operations */
	if (target_state->pwr_domain_state[ARM_PWR_LVL1] == PLAT_MAX_OFF_STATE) {


		/* Get the mpidr for this cpu */
		mpidr = read_mpidr();

		/* Enable coherency if this cluster was off */
		if (0 != set_cluster_coherency((mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK, 1))
			return;
	}

	return;

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
	int pwr_lvl = psci_get_pstate_pwrlvl(power_state);

	INFO("Called axxia_validate_power_state: pwr_lvl: %d\n", pwr_lvl);

	assert(req_state);

	if (pwr_lvl > PLAT_MAX_PWR_LVL)
		return PSCI_E_INVALID_PARAMS;

	return axxia_pwrc_validate_power_state(power_state, req_state);
}

int axxia_validate_ns_entrypoint(uintptr_t entrypoint)
{
	/*
	 * Check if the non secure entrypoint lies within the non
	 * secure DRAM.
	 */
	if ((entrypoint >= DRAM_BASE) && (entrypoint <= (DRAM_BASE + DRAM_SIZE)))
		return PSCI_E_SUCCESS;

	ERROR("CPU entry point not validated: entrypoint=0x%lx\n", entrypoint);

	return PSCI_E_INVALID_ADDRESS;
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

/*******************************************************************************
 * Export the platform specific power ops and initialize Power Controller
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{

	INFO("plat_setup_psci_ops called\n");
	/*
	 * Flush entrypoint variable to PoC since it will be
	 * accessed after a reset with the caches turned off.
	 */
	axxia_sec_entry_point = sec_entrypoint;
	flush_dcache_range((uint64_t)&axxia_sec_entry_point, sizeof(uint64_t));

	/*
	 * Reset hardware settings.
	 */
	/* axxia_soc_pwr_domain_on_finish(&target_state); */

	/*
	 * Initialize PSCI ops struct
	 */
	*psci_ops = &axxia_psci_pm_ops;

	return 0;
}
