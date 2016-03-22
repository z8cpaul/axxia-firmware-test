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
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
#include <debug.h>
#include <mmio.h>
#include <delay_timer.h>
#include <axxia_def.h>
#include <axxia_private.h>
#include "../../include/axxia_pwrc.h"

#undef DEBUG_CPU_PM

//#define PM_WAIT_TIME (10000)
#define PM_WAIT_TIME	(1) // This for simulation only
#define IPI_IRQ_MASK (0xFFFF)

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))


bool axxia_pwrc_in_progress[PLATFORM_CORE_COUNT];
bool cluster_power_up[PLATFORM_CLUSTER_COUNT];

static const unsigned int cluster_to_node[PLATFORM_CLUSTER_COUNT] = { DKN_CLUSTER0_NODE,
DKN_CLUSTER1_NODE,
DKN_CLUSTER2_NODE,
DKN_CLUSTER3_NODE };

static const unsigned int cluster_to_poreset[PLATFORM_CLUSTER_COUNT] = {
PORESET_CLUSTER0,
PORESET_CLUSTER1,
PORESET_CLUSTER2,
PORESET_CLUSTER3 };

static const unsigned int cluster_to_mask[PLATFORM_CLUSTER_COUNT] = {
		IPI0_MASK,
		IPI1_MASK,
		IPI2_MASK,
		IPI3_MASK
};

static const unsigned int ipi_register[MAX_IPI] = {
		SYSCON_MASK_IPI0,
		SYSCON_MASK_IPI1,
		SYSCON_MASK_IPI2,
		SYSCON_MASK_IPI3,
		SYSCON_MASK_IPI4,
		SYSCON_MASK_IPI5,
		SYSCON_MASK_IPI6,
		SYSCON_MASK_IPI7,
		SYSCON_MASK_IPI8,
		SYSCON_MASK_IPI9,
		SYSCON_MASK_IPI10,
		SYSCON_MASK_IPI11,
		SYSCON_MASK_IPI12,
		SYSCON_MASK_IPI13,
		SYSCON_MASK_IPI14,
		SYSCON_MASK_IPI15,
		SYSCON_MASK_IPI16,
		SYSCON_MASK_IPI17,
		SYSCON_MASK_IPI18
};

enum axxia_pwrc_error_code {
	PM_ERR_DICKENS_IOREMAP = 200,
	PM_ERR_DICKENS_SNOOP_DOMAIN,
	PM_ERR_FAILED_PWR_DWN_RAM,
	PM_ERR_FAILED_STAGE_1,
	PM_ERR_ACK1_FAIL,
	PM_ERR_RAM_ACK_FAIL,
	PM_ERR_FAIL_L2ACK,
	PM_ERR_FAIL_L2HSRAM
};

unsigned int axxia_pwrc_cpu_powered_down;


/*======================= LOCAL FUNCTIONS ==============================*/
static void axxia_pwrc_set_bits_syscon_register(unsigned int reg, unsigned int data);
static void axxia_pwrc_or_bits_syscon_register(unsigned int reg, unsigned int data);
static void axxia_pwrc_clear_bits_syscon_register(unsigned int reg, unsigned int data);
static unsigned int axxia_pwrc_test_for_bit_with_timeout(unsigned int reg, unsigned int bit);
static unsigned int axxia_pwrc_wait_for_bit_clear_with_timeout(unsigned int reg,
		unsigned int bit);
static void axxia_pwrc_dickens_logical_shutdown(unsigned int cluster);
static int axxia_pwrc_dickens_logical_powerup(unsigned int cluster);
static int axxia_pwrc_cpu_physical_isolation_and_power_down(int cpu);
static void axxia_pwrc_L2_isolation_and_power_down(int cluster);
static int axxia_pwrc_cpu_physical_connection_and_power_up(int cpu);
static int axxia_pwrc_L2_physical_connection_and_power_up(unsigned int cluster);
static int axxia_pwrc_L2_logical_powerup(unsigned int cluster, unsigned int cpu);

static bool axxia_pwrc_first_cpu_of_cluster(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU_L2_POWER_DOWN
	unsigned int count = 0;
	switch (cpu) {
	case (0):
	case (1):
	case (2):
	case (3):
		/* This will never happen because cpu 0 will never be turned off */
		break;
	case (4):
	case (5):
	case (6):
	case (7):
		if (axxia_pwrc_cpu_powered_down & (1 << 4))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 5))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 6))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 7))
			count++;
		if (count == 4)
			return TRUE;
		break;
	case (8):
	case (9):
	case (10):
	case (11):
		if (axxia_pwrc_cpu_powered_down & (1 << 8))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 9))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 10))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 11))
			count++;
		if (count == 4)
			return TRUE;
		break;
	case (12):
	case (13):
	case (14):
	case (15):
		if (axxia_pwrc_cpu_powered_down & (1 << 12))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 13))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 14))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 15))
			count++;
		if (count == 4)
			return TRUE;
		break;
	default:
		ERROR("ERROR: the cpu does not exist: %d - %s:%d\n", cpu, __FILE__,
				__LINE__);
		break;
	}
#endif
	return FALSE;
}

bool axxia_pwrc_cpu_last_of_cluster(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU_L2_POWER_DOWN

	unsigned int count = 0;
	switch (cpu) {
	case (0):
	case (1):
	case (2):
	case (3):
		/* This will never happen because cpu 0 will never be turned off */
		break;
	case (4):
	case (5):
	case (6):
	case (7):
		if (axxia_pwrc_cpu_powered_down & (1 << 4))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 5))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 6))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 7))
			count++;
		if (count == 3)
			return TRUE;
		break;
	case (8):
	case (9):
	case (10):
	case (11):
		if (axxia_pwrc_cpu_powered_down & (1 << 8))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 9))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 10))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 11))
			count++;
		if (count == 3)
			return TRUE;
		break;
	case (12):
	case (13):
	case (14):
	case (15):
		if (axxia_pwrc_cpu_powered_down & (1 << 12))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 13))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 14))
			count++;
		if (axxia_pwrc_cpu_powered_down & (1 << 15))
			count++;
		if (count == 3)
			return TRUE;
		break;
	default:
		ERROR("ERROR: the cpu does not exist: %d - %s:%d\n", cpu,  __FILE__,
				__LINE__);
		break;
	}
#endif
	return FALSE;
}

static void axxia_pwrc_set_bits_syscon_register(unsigned int reg, unsigned int data)
{
	mmio_write_32((SYSCON_BASE + reg), data);
}

static void axxia_pwrc_or_bits_syscon_register(unsigned int reg, unsigned int data)
{
	unsigned int tmp;

	tmp = mmio_read_32(SYSCON_BASE + reg);
	tmp |= data;
	mmio_write_32((SYSCON_BASE + reg), tmp);
}


static void axxia_pwrc_clear_bits_syscon_register(unsigned int reg, unsigned int data)
{
	unsigned int tmp;

	tmp = mmio_read_32(SYSCON_BASE + reg);
	tmp &= ~(data);
	mmio_write_32((SYSCON_BASE + reg), tmp);
}

static unsigned int axxia_pwrc_test_for_bit_with_timeout(unsigned int reg, unsigned int bit)
{

	unsigned int tmp = 0;
	unsigned int cnt = 0;

	while (cnt < PM_WAIT_TIME) {
		tmp = mmio_read_32(SYSCON_BASE + reg);
		if (CHECK_BIT(tmp, bit))
			break;
		cnt++;
	}
	if (cnt == PM_WAIT_TIME) {
		ERROR("reg=0x%x tmp:=0x%x\n", reg, tmp);
		ERROR("TIMEOUT override during simulation 'test for bit'\n");
		return PSCI_E_SUCCESS;
	}
	return PSCI_E_SUCCESS;
}

static unsigned int axxia_pwrc_wait_for_bit_clear_with_timeout(unsigned int reg, unsigned int bit)
{
	unsigned int cnt = 0;
	unsigned int tmp = 0;

	while (cnt < PM_WAIT_TIME) {
		tmp = mmio_read_32(SYSCON_BASE + reg);
		if (!(CHECK_BIT(tmp, bit)))
			break;
		cnt++;
	}
	if (cnt == PM_WAIT_TIME) {
		ERROR("reg=0x%x tmp:=0x%x\n", reg, tmp);
		return PSCI_E_INTERN_FAIL;
	}

	return PSCI_E_SUCCESS;
}
static void axxia_pwrc_dickens_logical_shutdown(unsigned int cluster)
{
	int i;
	int status;
	unsigned int bit;
	unsigned int bit_pos;
	int retries;

	bit = (0x01 << cluster_to_node[cluster]);
	bit_pos = cluster_to_node[cluster];

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		mmio_write_32((DICKENS_BASE_X9 + (0x10000 * (DKN_HNF_NODE_ID + i)) + DKN_HNF_SNOOP_DOMAIN_CTL_CLR), bit);

		retries = PM_WAIT_TIME;

		do {
			status = mmio_read_32(DICKENS_BASE_X9 + (0x10000 * (DKN_HNF_NODE_ID + i)) + DKN_HNF_SNOOP_DOMAIN_CTL);
			udelay(1);
		} while ((0 < --retries) && CHECK_BIT(status, bit_pos));

		if (0 == retries) {
			ERROR("DICKENS: Failed to clear the SNOOP main control. LOOP:%d reg: 0x%x\n", i, status);
			return;

		}

	}
	/* Clear the domain cluster */
	mmio_write_32((DICKENS_BASE_X9 + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL_CLR), bit);

	/* Check for complete */
	retries = PM_WAIT_TIME;

	do {
		status = mmio_read_32(DICKENS_BASE_X9 + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL);
		udelay(1);
	} while ((0 < --retries) && CHECK_BIT(status, bit_pos));

	if (0 == retries) {
		ERROR("DICKENS: failed to set DOMAIN OFFSET Reg=0x%x\n", status);
		return;
	}
}

static int axxia_pwrc_dickens_logical_powerup(unsigned int cluster)
{
	int i;
	unsigned int status;
	unsigned int bit;
	unsigned int bit_pos;
	int retries;
	int rval = PSCI_E_SUCCESS;

	bit = (0x01 << cluster_to_node[cluster]);
	bit_pos = cluster_to_node[cluster];

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		mmio_write_32((DICKENS_BASE_X9 + (0x10000 * (DKN_HNF_NODE_ID + i)) + DKN_HNF_SNOOP_DOMAIN_CTL_SET), bit);

		retries = PM_WAIT_TIME;

		do {
			status = mmio_read_32(DICKENS_BASE_X9 + (0x10000 * (DKN_HNF_NODE_ID + i)) + DKN_HNF_SNOOP_DOMAIN_CTL);
			udelay(1);
		} while ((0 < --retries) && !CHECK_BIT(status, bit_pos));

		if (0 == retries) {
			ERROR("DICKENS: Failed on the SNOOP DONAIN\n");
			return -PM_ERR_DICKENS_SNOOP_DOMAIN;
		}

	}

	/* Clear the domain cluster */
	mmio_write_32((DICKENS_BASE_X9 + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL_SET), bit);

	/* Check for complete */
	retries = PM_WAIT_TIME;

	do {
		status = mmio_read_32(DICKENS_BASE_X9 + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL);
		udelay(1);
	} while ((0 < --retries) && !CHECK_BIT(status, bit_pos));

	if (0 == retries) {
		ERROR("DICKENS: Failed on the SNOOP DONAIN CTL SET\n");
		return -PM_ERR_DICKENS_SNOOP_DOMAIN;
	}

	return rval;
}

static void axxia_pwrc_disable_ipi_interrupts(unsigned int cpu)
{
#if 0
	axxia_pwrc_clear_bits_syscon_register(ipi_register[cpu], IPI_IRQ_MASK);
#endif
}

static void axxia_pwrc_enable_ipi_interrupts(unsigned int cpu)
{
#if 0
	unsigned int i;
	unsigned int powered_on_cpu = (~(axxia_pwrc_cpu_powered_down) & IPI_IRQ_MASK);

	axxia_pwrc_set_bits_syscon_register(ipi_register[cpu], powered_on_cpu);

	for (i = 0; i < PLATFORM_CORE_COUNT; i++) {
		if ((1 << i) & powered_on_cpu)
			axxia_pwrc_or_bits_syscon_register(ipi_register[i], (1 << cpu));
	}
#endif

	return;
}

bool axxia_pwrc_cpu_active(unsigned int cpu)
{

	bool failure = FALSE;
	unsigned int reg;

	reg = mmio_read_32(SYSCON_BASE + SYSCON_PWR_QACTIVE);
	if (reg & (1 << cpu))
		failure = TRUE;

	return failure;

}

int axxia_pwrc_cpu_shutdown(unsigned int reqcpu)
{

	unsigned int cluster = reqcpu / PLATFORM_MAX_CPUS_PER_CLUSTER;
	unsigned int cluster_mask = (0x01 << cluster);
	bool last_cpu;
	int rval = PSCI_E_SUCCESS;

	/* Check to see if the cpu is powered up */
	if (axxia_pwrc_cpu_powered_down & (1 << reqcpu)) {
		ERROR("CPU %u is already powered off - %s:%d\n", reqcpu, __FILE__, __LINE__);
		return PSCI_E_INTERN_FAIL;
	}

	/*
	 * Is this the last cpu of a cluster then turn off the L2 cache
	 * along with the CPU.
	 */
	last_cpu = axxia_pwrc_cpu_last_of_cluster(reqcpu);
	if (last_cpu) {

		/* Disable all the interrupts to the cluster gic */
		axxia_pwrc_or_bits_syscon_register(SYSCON_GIC_DISABLE, cluster_mask);

		/* Remove the cluster from the Dickens coherency domain */
		axxia_pwrc_dickens_logical_shutdown(cluster);

		/* Power down the cpu */
		axxia_pwrc_cpu_physical_isolation_and_power_down(reqcpu);

		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_CSYSREQ_CNT, cluster_mask);
		rval = axxia_pwrc_wait_for_bit_clear_with_timeout(SYSCON_PWR_CACTIVE_CNT, cluster);
		if (!rval) {
			ERROR(
					"Failed to keep other cluster count going on cluster %u: %s-%d\n",
					cluster, __FILE__, __LINE__);

			goto axxia_pwrc_shutdown_exit;
		}

		/* Turn off the ACE */
		axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_ACEPWRDNRQ, cluster_mask);

		/* Wait for ACE to complete power off */
		rval = axxia_pwrc_wait_for_bit_clear_with_timeout(SYSCON_PWR_NACEPWRDNACK, cluster);
		if (!rval) {
			ERROR("Failed to power off ACE on cluster %u: %s-%d\n",
					cluster, __FILE__, __LINE__);
			goto axxia_pwrc_shutdown_exit;
		}

		/* Isolate the cluster */
		axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_ISOLATEL2MISC, cluster_mask);

		/* Wait for WFI L2 to go to standby */
		rval = axxia_pwrc_test_for_bit_with_timeout(SYSCON_PWR_STANDBYWFIL2, cluster);
		if (!rval) {
			ERROR("Failed to enter L2 WFI on cluster %u: %s-%d\n",
					cluster, __FILE__, __LINE__);
			goto axxia_pwrc_shutdown_exit;
		}

		/* Power off the L2 */
		axxia_pwrc_L2_isolation_and_power_down(cluster);
		if (rval == PSCI_E_SUCCESS) {
			INFO("CPU %u is powered down with cluster: %u\n", reqcpu, cluster);
			axxia_pwrc_cpu_powered_down |= (1 << reqcpu);
		} else
			ERROR("CPU %u failed to power down\n", reqcpu);


	} else {

		rval = axxia_pwrc_cpu_physical_isolation_and_power_down(reqcpu);
		if (rval == PSCI_E_SUCCESS)
			axxia_pwrc_cpu_powered_down |= (1 << reqcpu);
		else
			ERROR("CPU %u failed to power down\n", reqcpu);
	}

axxia_pwrc_shutdown_exit:
	return rval;
}

int axxia_pwrc_cpu_powerup(unsigned int reqcpu)
{

	bool first_cpu;
	int rval = PSCI_E_SUCCESS;
	unsigned int cpu_mask = (0x01 << reqcpu);

	unsigned int cluster = reqcpu / PLATFORM_MAX_CPUS_PER_CLUSTER;
	unsigned int cluster_mask = (0x01 << cluster);

	/*
	 * Is this the first cpu of a cluster to come back on?
	 * Then power up the L2 cache.
	 */
	first_cpu = axxia_pwrc_first_cpu_of_cluster(reqcpu);
	if (first_cpu) {

		rval = axxia_pwrc_L2_logical_powerup(cluster, reqcpu);
		if (rval) {
			ERROR("CPU: Failed the logical L2 power up\n");
			goto axxia_pwrc_power_up;
		} else
			INFO("CPU %u is powered up with cluster: %u\n", reqcpu, cluster);

		cluster_power_up[cluster] = TRUE;
		axxia_pwrc_clear_bits_syscon_register(SYSCON_GIC_DISABLE, cluster_mask);


	} else {

		/* Set the CPU into reset */
		mmio_write_32(0x8031000000,
			      (0x14000000 |
			       (axxia_sec_entry_point - 0x8031000000) / 4));
		dsb();

		axxia_pwrc_set_bits_syscon_register(SYSCON_KEY, VALID_KEY_VALUE);
		axxia_pwrc_or_bits_syscon_register(SYSCON_HOLD_CPU, (1 << reqcpu));

		axxia_pwrc_or_bits_syscon_register(SYSCON_PWRUP_CPU_RST, cpu_mask);
	}


	/*
	 * Power up the CPU
	 */
	rval = axxia_pwrc_cpu_physical_connection_and_power_up(reqcpu);
	if (rval) {
		ERROR("Failed to power up physical connection of cpu: %u\n", reqcpu);
		goto axxia_pwrc_power_up;
	}

	/*
	 * The key value must be written before the CPU RST can be written.
	 */

	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWRUP_CPU_RST,	cpu_mask);

	axxia_pwrc_clear_bits_syscon_register(SYSCON_HOLD_CPU, (1 << reqcpu));
	axxia_pwrc_set_bits_syscon_register(SYSCON_KEY, 0x00);

	/*
	 * Clear the powered down mask
	 */
	axxia_pwrc_cpu_powered_down &= ~(1 << reqcpu);

	/* Enable the CPU IPI */
	axxia_pwrc_enable_ipi_interrupts(reqcpu);

axxia_pwrc_power_up:
	return rval;
}

unsigned int axxia_pwrc_get_powered_down_cpu(void)
{
	return axxia_pwrc_cpu_powered_down;
}


inline void axxia_pwrc_cpu_logical_powerup(void)
{
#if 0
	unsigned int v;
	__asm__ volatile(
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (v)
	  : "Ir" (1 << 2), "Ir" (1 << 12)
	  : "cc");

	/*
	 *  Iniitalize the ACTLR2 register (all cores).
	 */

	__asm__ volatile(
	"	mrc		p15, 1, %0, c15, c0, 4\n"
	"	bic	%0, %0, %1\n"
	"	mcr		p15, 1, %0, c15, c0, 4\n"
	: "=&r" (v)
	: "Ir" (0x1)
	: "cc");

	isb();
	dsb();
#endif
}

inline void axxia_pwrc_cluster_logical_powerup(void)
{
#if 0
	//unsigned int v;

	/*
	 * Initialize the L2CTLR register (primary core in each cluster).
	 */
	__asm__ volatile(
	"	mrc	p15, 1, %0, c9, c0, 2\n"
	"	orr	%0, %0, %1\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 1, %0, c9, c0, 2"
	  : "=&r" (v)
	  : "Ir" (0x01), "Ir" (0x1 << 21)
	  : "cc");
	isb();
	dsb();

	/*
	 * Initialize the L2ACTLR register (primary core in each cluster).
	 */
	__asm__ volatile(
	"	mrc	p15, 1, r0, c15, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	orr	%0, %0, %2\n"
	"	orr	%0, %0, %3\n"
	"	orr	%0, %0, %4\n"
	"	orr	%0, %0, %5\n"
	"	mcr	p15, 1, %0, c15, c0, 0"
	  : "=&r" (v)
	  : "Ir" (0x1 << 3), "Ir" (0x1 << 7), "Ir" (0x1 << 12), "Ir" (0x1 << 13), "Ir" (0x1 << 14)
	  : "cc");
	isb();
	dsb();
#endif

}

static int axxia_pwrc_cpu_physical_isolation_and_power_down(int cpu)
{

	int rval = PSCI_E_SUCCESS;

	bool failure;
	unsigned int mask = (0x01 << cpu);

	/* Disable the CPU IPI */
	axxia_pwrc_disable_ipi_interrupts(cpu);

	/* Initiate power down of the CPU's HS Rams */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPCPURAM, mask);

	/* Wait until the RAM power down is complete */
	failure = axxia_pwrc_test_for_bit_with_timeout(SYSCON_PWR_NPWRUPCPURAM_ACK, cpu);
	if (failure) {
		rval = PSCI_E_INTERN_FAIL;
		ERROR("CPU: Failed to power down CPU RAM\n");
		goto power_down_cleanup;
	}

	/* Activate the CPU's isolation clamps */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_ISOLATECPU, mask);

	/* Initiate power down of the CPU logic */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPCPUSTG2, mask);

	//udelay(16);

	/* Continue power down of the CPU logic */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPCPUSTG1, mask);

	failure = axxia_pwrc_test_for_bit_with_timeout(SYSCON_PWR_NPWRUPCPUSTG1_ACK, cpu);
	if (failure) {
		rval = PSCI_E_INTERN_FAIL;
		ERROR("CPU: Failed to power down stage 1 cpu\n");
		goto power_down_cleanup;
	}

power_down_cleanup:

	return rval;
}

static int axxia_pwrc_cpu_physical_connection_and_power_up(int cpu)
{
	int rval = PSCI_E_SUCCESS;

	bool failure;
	unsigned int mask = (0x01 << cpu);

	/* Initiate power up of the CPU */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_PWRUPCPUSTG1, mask);

	/* Wait until CPU logic power is compete */
	failure = axxia_pwrc_wait_for_bit_clear_with_timeout(SYSCON_PWR_NPWRUPCPUSTG1_ACK, cpu);
	if (failure) {
		rval = PSCI_E_INTERN_FAIL;
		ERROR("CPU: Failed to get ACK from power up stage 1\n");
		goto power_up_cleanup;
	}

	/* Continue stage 2 power up of the CPU*/
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_PWRUPCPUSTG2, mask);

	udelay(16);

	/* Initiate power up of HS Rams */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_PWRUPCPURAM, mask);

	/* Wait until the RAM power up is complete */
	failure = axxia_pwrc_wait_for_bit_clear_with_timeout(SYSCON_PWR_NPWRUPCPURAM_ACK, cpu);
	if (failure) {
		rval = PSCI_E_INTERN_FAIL;
		ERROR("CPU: Failed to get ACK of power power up\n");
		goto power_up_cleanup;
	}

	/* Release the CPU's isolation clamps */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_ISOLATECPU, mask);

	udelay(16);

power_up_cleanup:

	return rval;

}
/*========================================== L2 FUNCTIONS ========================================*/

static void axxia_pwrc_L2_isolation_and_power_down(int cluster)
{

	unsigned int mask = (0x1 << cluster);

	/* Enable the chip select for the cluster */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_CHIPSELECTEN, mask);

	/* Disable the hsram */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL2HSRAM, mask);

	switch (cluster) {
	case (0):

#ifdef PM_POWER_OFF_ONLY_DATARAM
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);

#endif
		break;
	case (1):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (2):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (3):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	default:
		ERROR("Illegal cluster: %d > 3\n", cluster);
		break;
	}

	/* Power down stage 2 */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL2LGCSTG2, mask);

	/* Power down stage 1 */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_PWRUPL2LGCSTG1, mask);

}

static int axxia_pwrc_L2_physical_connection_and_power_up(unsigned int cluster)
{

	unsigned int mask = (0x1 << cluster);
	int rval = PSCI_E_SUCCESS;

	/* Power up stage 1 */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_PWRUPL2LGCSTG1, mask);

	/* Wait for the stage 1 power up to complete */
	rval = axxia_pwrc_wait_for_bit_clear_with_timeout(SYSCON_PWR_NPWRUPL2LGCSTG1_ACK, cluster);
	if (!rval) {
		ERROR("CPU: Failed to ack the L2 Stage 1 Power up\n");
		goto power_up_l2_cleanup;
	}

	/* Power on stage 2 */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_PWRUPL2LGCSTG2, mask);

	/* Set the chip select */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_CHIPSELECTEN, mask);

	/* Power up the snoop ram */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_PWRUPL2HSRAM, mask);

	/* Wait for the stage 1 power up to complete */
	rval = axxia_pwrc_wait_for_bit_clear_with_timeout(SYSCON_PWR_NPWRUPL2HSRAM_ACK, cluster);
	if (!rval) {
		ERROR("CPU: failed to get the HSRAM power up ACK\n");
		goto power_up_l2_cleanup;
	}

	switch (cluster) {
	case (0):

#ifdef PM_POWER_OFF_ONLY_DATARAM
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);

#endif
		break;
	case (1):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
	udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (2):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (3):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(syscon,
				SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		axxia_pwrc_set_bits_syscon_register(SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	default:
		ERROR("Illegal cluster: %u > 3\n", cluster);
		break;
	}

	/* Clear the chip select */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_CHIPSELECTEN, mask);

	/* Release the isolation clamps */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_ISOLATEL2MISC, mask);

	/* Turn the ACE bridge power on*/
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_ACEPWRDNRQ, mask);

power_up_l2_cleanup:
	return rval;
}

static int axxia_pwrc_L2_logical_powerup(unsigned int cluster, unsigned int cpu)
{

	unsigned int mask = (0x1 << cluster);
	int rval = PSCI_E_SUCCESS;
	unsigned int cluster_mask;

	if (cluster == 0)
		cluster_mask = 0xe;
	else
		cluster_mask = 0xf << (cluster * 4);

	/* put the cluster into a cpu hold */
	axxia_pwrc_or_bits_syscon_register(SYSCON_RESET_AXIS,
			cluster_to_poreset[cluster]);

	/*
	 * The key value has to be written before the CPU RST can be written.
	 */
	axxia_pwrc_set_bits_syscon_register(SYSCON_KEY, VALID_KEY_VALUE);
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWRUP_CPU_RST, cluster_mask);

	/* Hold the chip debug cluster */
	axxia_pwrc_set_bits_syscon_register(SYSCON_KEY, VALID_KEY_VALUE);
	axxia_pwrc_or_bits_syscon_register(SYSCON_HOLD_DBG, mask);

	/* Hold the L2 cluster */
	axxia_pwrc_set_bits_syscon_register(SYSCON_KEY, VALID_KEY_VALUE);
	axxia_pwrc_or_bits_syscon_register(SYSCON_HOLD_L2, mask);


	/* Cluster physical power up */
	rval = axxia_pwrc_L2_physical_connection_and_power_up(cluster);
	if (rval)
		goto exit_axxia_pwrc_L2_logical_powerup;

	udelay(16);

	/* take the cluster out of a cpu hold */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_RESET_AXIS,
			cluster_to_poreset[cluster]);

	udelay(64);

	/* Enable the system counter */
	axxia_pwrc_or_bits_syscon_register(SYSCON_PWR_CSYSREQ_CNT, mask);

	/* Release the L2 cluster */
	axxia_pwrc_set_bits_syscon_register(SYSCON_KEY, VALID_KEY_VALUE);
	axxia_pwrc_clear_bits_syscon_register(SYSCON_HOLD_L2, mask);

	/* Release the chip debug cluster */
	axxia_pwrc_set_bits_syscon_register(SYSCON_KEY, VALID_KEY_VALUE);
	axxia_pwrc_clear_bits_syscon_register(SYSCON_HOLD_DBG, mask);

	/* Power up the dickens */
	rval = axxia_pwrc_dickens_logical_powerup(cluster);
	if (rval)
		goto exit_axxia_pwrc_L2_logical_powerup;

	/* start L2 */
	axxia_pwrc_clear_bits_syscon_register(SYSCON_PWR_ACINACTM, mask);

exit_axxia_pwrc_L2_logical_powerup:

	return rval;

}

#ifdef DEBUG_CPU_PM

void axxia_pwrc_debug_read_pwr_registers(void)
{
	unsigned int reg;

	reg = readl(syscon + 0x1400);
	ERROR("SYSCON_PWR_CLKEN: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_PWR_ACINACTM);
	ERROR("SYSCON_PWR_ACINACTM: 0x%x\n", reg);
	reg = readl(syscon + 0x140c);
	ERROR("SYSCON_PWR_CHIPSELECTEN: 0x%x\n", reg);
	reg = readl(syscon + 0x1410);
	ERROR("SYSCON_PWR_CSYSREQ_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x1414);
	ERROR("SYSCON_PWR_CSYSREQ_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1418);
	ERROR("SYSCON_PWR_CSYSREQ_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x141c);
	ERROR("SYSCON_PWR_CSYSREQ_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1420);
	ERROR("SYSCON_PWR_PWRUPL2LGCSTG1: 0x%x\n", reg);
	reg = readl(syscon + 0x1424);
	ERROR("SYSCON_PWR_PWRUPL2LGCSTG2: 0x%x\n", reg);
	reg = readl(syscon + 0x1428);
	ERROR("SYSCON_PWR_PWRUPL2HSRAM: 0x%x\n", reg);
	reg = readl(syscon + 0x142c);
	ERROR("SYSCON_PWR_ACEPWRDNRQ: 0x%x\n", reg);
	reg = readl(syscon + 0x1430);
	ERROR("SYSCON_PWR_ISOLATEL2MIS: 0x%x\n", reg);
	reg = readl(syscon + 0x1438);
	ERROR("SYSCON_PWR_NPWRUPL2LGCSTG1_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x143c);
	ERROR("SYSCON_PWR_NPWRUPL2HSRAM_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1440);
	ERROR("SYSCON_PWR_STANDBYWFIL2: 0x%x\n", reg);
	reg = readl(syscon + 0x1444);
	ERROR("SYSCON_PWR_CSYSACK_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x1448);
	ERROR("SYSCON_PWR_CACTIVE_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x144c);
	ERROR("SYSCON_PWR_CSYSACK_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1450);
	ERROR("SYSCON_PWR_CACTIVE_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1454);
	ERROR("SYSCON_PWR_CSYSACK_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x1458);
	ERROR("SYSCON_PWR_CACTIVE_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x145c);
	ERROR("SYSCON_PWR_CSYSACK_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1460);
	ERROR("SYSCON_PWR_CACTIVE_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1464);
	ERROR("SYSCON_PWR_NACEPWRDNACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1468);
	ERROR("SYSCON_PWR_CACTIVEM_EAGM: 0x%x\n", reg);
	reg = readl(syscon + 0x146c);
	ERROR("SYSCON_PWR_CACTIVEM_EAGS: 0x%x\n", reg);
	reg = readl(syscon + 0x1470);
	ERROR("SYSCON_PWR_CACTIVES_EAGM: 0x%x\n", reg);
	reg = readl(syscon + 0x1474);
	ERROR("SYSCON_PWR_CACTIVES_EAGS: 0x%x\n", reg);
	reg = readl(syscon + 0x1480);
	ERROR("SYSCON_PWR_PWRUPCPUSTG1: 0x%x\n", reg);
	reg = readl(syscon + 0x1484);
	ERROR("SYSCON_PWR_PWRUPCPUSTG2: 0x%x\n", reg);
	reg = readl(syscon + 0x1488);
	ERROR("SYSCON_PWR_PWRUPCPURAM: 0x%x\n", reg);
	reg = readl(syscon + 0x148c);
	ERROR("SYSCON_PWR_ISOLATECPU: 0x%x\n", reg);
	reg = readl(syscon + 0x1490);
	ERROR("SYSCON_PWR_NPWRUPCPUSTG1_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1494);
	ERROR("SYSCON_PWR_NPWRUPCPURAM_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1498);
	ERROR("SYSCON_PWR_QACTIVE: 0x%x\n", reg);
	reg = readl(syscon + 0x149C);
	ERROR("SYSCON_PWR_STANDBYWFI: 0x%x\n", reg);
	reg = readl(syscon + 0x14A0);
	ERROR("SYSCON_PWR_STANDBYWFE: 0x%x\n", reg);
	reg = readl(syscon + 0x14A4);
	ERROR("SYSCON_PWR_DBGNOPWRDWN: 0x%x\n", reg);
	reg = readl(syscon + 0x14A8);
	ERROR("SYSCON_PWR_DBGPWRUPREQ: 0x%x\n", reg);
	reg = readl(syscon + 0x1040);
	ERROR("SYSCON_RESET_AXIS: 0x%x\n", reg);
	reg = readl(syscon + 0x1044);
	ERROR("SYSCON_RESET_AXIS-WORD1: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_RESET_CPU);
	ERROR("SYSCON_RESET_CPU: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_HOLD_DBG);
	ERROR("SYSCON_HOLD_DBG: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_HOLD_L2);
	ERROR("SYSCON_HOLD_L2: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_HOLD_CPU);
	ERROR("SYSCON_HOLD_CPU: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_PWRUP_CPU_RST);
	ERROR("SYSCON_PWRUP_CPU_RST: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_RESET_STATUS);
	ERROR("SYSCON_RESET_STATUS: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_RESET_CORE_STATUS);
	ERROR("SYSCON_RESET_CORE_STATUS: 0x%x\n", reg);


#if 0
	reg = readl(syscon + SYSCON_MCG_CSW_CPU);
	ERROR("SYSCON_MCG_CSW_CPU: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MCG_CSW_SYS);
	ERROR("SYSCON_MCG_CSW_SYS: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MCG_DIV_CPU);
	ERROR("SYSCON_MCG_DIV_CPU: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MCG_DIV_SYS);
	ERROR("SYSCON_MCG_DIV_SYS: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_CLKDEBUG);
	ERROR("SYSCON_CLKDEBUG: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_EVENT_ENB);
	ERROR("SYSCON_EVENT_ENB: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_CPU_FAST_INT);
	ERROR("SYSCON_CPU_FAST_INT: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_GIC_DISABLE);
	ERROR("SYSCON_GIC_DISABLE: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_CP15SDISABLE);
	ERROR("SYSCON_CP15SDISABLE: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_LDO_CTL);
	ERROR("SYSCON_LDO_CTL: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_SHWK_QOS);
	ERROR("SYSCON_SHWK_QOS: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_FUSE_RTO);
	ERROR("SYSCON_FUSE_RTO: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_PFUSE);
	ERROR("SYSCON_PFUSE: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_FUSE_STAT);
	ERROR("SYSCON_FUSE_STAT: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_SCRATCH);
	ERROR("SYSCON_SCRATCH: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI0);
	ERROR("SYSCON_MASK_IPI0: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI1);
	ERROR("SYSCON_MASK_IPI1: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI2);
	ERROR("SYSCON_MASK_IPI2: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI3);
	ERROR("SYSCON_MASK_IPI3: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI4);
	ERROR("SYSCON_MASK_IPI4: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI5);
	ERROR("SYSCON_MASK_IPI5: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI6);
	ERROR("SYSCON_MASK_IPI6: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI7);
	ERROR("SYSCON_MASK_IPI7: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI8);
	ERROR("SYSCON_MASK_IPI8: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI9);
	ERROR("SYSCON_MASK_IPI9: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI10);
	ERROR("SYSCON_MASK_IPI10: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI11);
	ERROR("SYSCON_MASK_IPI11: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI12);
	ERROR("SYSCON_MASK_IPI12: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI13);
	ERROR("SYSCON_MASK_IPI13: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI14);
	ERROR("SYSCON_MASK_IPI14: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_MASK_IPI15);
	ERROR("SYSCON_MASK_IPI15: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_SPARE0);
	ERROR("SYSCON_SPARE0: 0x%x\n", reg);
	reg = readl(syscon + SYSCON_STOP_CLK_CPU);
	ERROR("SYSCON_STOP_CLK_CPU: 0x%x\n", reg);
#endif


}


void axxia_pwrc_dump_L2_registers(void)
{
	unsigned int reg;


	reg = readl(syscon + 0x1580);
	ERROR("SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x1584);
	ERROR("SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x1588);
	ERROR("SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x158c);
	ERROR("SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x1590);
	ERROR("SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x1594);
	ERROR("SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x1598);
	ERROR("SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x159c);
	ERROR("SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x15a0);
	ERROR("SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x15a4);
	ERROR("SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x15a8);
	ERROR("SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x15ac);
	ERROR("SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0: 0x%x\n", reg);




void axxia_pwrc_dump_dickens(void)
{

	unsigned int status;
	unsigned int i;

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		status = readl(
				dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
						+ DKN_HNF_SNOOP_DOMAIN_CTL);
		udelay(1);
		ERROR("DKN_HNF_SNOOP_DOMAIN_CTL[%d]: 0x%x\n", i, status);
	}

	status = readl(
			dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET)
					+ DKN_MN_DVM_DOMAIN_CTL);

	ERROR("DKN_MN_DVM_DOMAIN_CTL: 0x%x\n", status);
}

#endif


