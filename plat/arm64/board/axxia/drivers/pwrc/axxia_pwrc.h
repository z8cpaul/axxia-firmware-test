/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
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

#ifndef AXXIA_PWRC_H_
#define AXXIA_PWRC_H_


#define     SYSCON_MCG_CSW_CPU                              (0x00001000)
#define     SYSCON_MCG_CSW_SYS                              (0x00001004)
#define     SYSCON_MCG_DIV_CPU                              (0x00001008)
#define     SYSCON_MCG_DIV_SYS                              (0x0000100c)
#define     SYSCON_CLKDEBUG                                 (0x00001010)
#define     SYSCON_EVENT_ENB                                (0x00001014)
#define     SYSCON_CPU_FAST_INT                             (0x00001018)
#define     SYSCON_GIC_DISABLE                              (0x0000101c)
#define     SYSCON_CP15SDISABLE                             (0x00001020)
#define     SYSCON_LRSTDISABLE                              (0x00001024)
#define     SYSCON_LDO_CTL                                  (0x00001028)
#define     SYSCON_SHWK_QOS                                 (0x0000102c)
#define     SYSCON_FUSE_RTO                                 (0x00001030)
#define     SYSCON_PFUSE                                    (0x00001034)
#define     SYSCON_FUSE_STAT                                (0x00001038)
#define     SYSCON_SCRATCH                                  (0x0000103c)
#define     SYSCON_MASK_IPI0                                (0x00001040)
#define     SYSCON_MASK_IPI1                                (0x00001044)
#define     SYSCON_MASK_IPI2                                (0x00001048)
#define     SYSCON_MASK_IPI3                                (0x0000104c)
#define     SYSCON_MASK_IPI4                                (0x00001050)
#define     SYSCON_MASK_IPI5                                (0x00001054)
#define     SYSCON_MASK_IPI6                                (0x00001058)
#define     SYSCON_MASK_IPI7                                (0x0000105c)
#define     SYSCON_MASK_IPI8                                (0x00001060)
#define     SYSCON_MASK_IPI9                                (0x00001064)
#define     SYSCON_MASK_IPI10                               (0x00001068)
#define     SYSCON_MASK_IPI11                               (0x0000106c)
#define     SYSCON_MASK_IPI12                               (0x00001070)
#define     SYSCON_MASK_IPI13                               (0x00001074)
#define     SYSCON_MASK_IPI14                               (0x00001078)
#define     SYSCON_MASK_IPI15                               (0x0000107c)
#define     SYSCON_MASK_IPI16                               (0x00001080)
#define     SYSCON_MASK_IPI17                               (0x00001084)
#define     SYSCON_MASK_IPI18                               (0x00001088)
#define     SYSCON_SPARE0                                   (0x0000108c)
#define     SYSCON_STOP_CLK_CPU                             (0x00001090)


#define     SYSCON_RESET_STATUS                             (0x00001100)
#define     SYSCON_RESET_CORE_STATUS                        (0x00001108)

#define     SYSCON_KEY                                      (0x00002000)
#define     SYSCON_RESET_CTL                                (0x00002008)
#define     SYSCON_RESET_CPU                                (0x0000200c)
#define     SYSCON_HOLD_CPU                                 (0x00002010)
#define     SYSCON_HOLD_PTM                                 (0x00002014)
#define     SYSCON_HOLD_L2                                  (0x00002018)
#define     SYSCON_HOLD_DBG                                 (0x0000201c)

#define     SYSCON_PWRUP_CPU_RST                            (0x00002030)

#define     SYSCON_RESET_AXIS                               (0x00002040)
#define     SYSCON_RESET_AXIS_ACCESS_SIZE                   (0x00000008)

#define     SYSCON_PWR_CLKEN                                (0x00002400)
#define     SYSCON_ENABLE_CLKEN_SET                         (0x00002404)
#define     SYSCON_PWR_ACINACTM                             (0x00002408)
#define     SYSCON_PWR_CHIPSELECTEN                         (0x0000240c)
#define     SYSCON_PWR_CSYSREQ_TS                           (0x00002410)
#define     SYSCON_PWR_CSYSREQ_CNT                          (0x00002414)
#define     SYSCON_PWR_CSYSREQ_ATB                          (0x00002418)
#define     SYSCON_PWR_CSYSREQ_APB                          (0x0000241c)
#define     SYSCON_PWR_PWRUPL2LGCSTG1                       (0x00002420)
#define     SYSCON_PWR_PWRUPL2LGCSTG2                       (0x00002424)
#define     SYSCON_PWR_PWRUPL2HSRAM                         (0x00002428)
#define     SYSCON_PWR_ACEPWRDNRQ                           (0x0000242c)
#define     SYSCON_PWR_ISOLATEL2MISC                        (0x00002430)
#define     SYSCON_PWR_NPWRUPL2LGCSTG1_ACK                  (0x00002438)
#define     SYSCON_PWR_NPWRUPL2HSRAM_ACK                    (0x0000243c)
#define     SYSCON_PWR_STANDBYWFIL2                         (0x00002440)
#define     SYSCON_PWR_CSYSACK_TS                           (0x00002444)
#define     SYSCON_PWR_CACTIVE_TS                           (0x00002448)
#define     SYSCON_PWR_CSYSACK_CNT                          (0x0000244c)
#define     SYSCON_PWR_CACTIVE_CNT                          (0x00002450)
#define     SYSCON_PWR_CSYSACK_ATB                          (0x00002454)
#define     SYSCON_PWR_CACTIVE_ATB                          (0x00002458)
#define     SYSCON_PWR_CSYSACK_APB                          (0x0000245c)
#define     SYSCON_PWR_CACTIVE_APB                          (0x00002460)
#define     SYSCON_PWR_NACEPWRDNACK                         (0x00002464)
#define     SYSCON_PWR_CACTIVEM_EAGM                        (0x00002468)
#define     SYSCON_PWR_CACTIVEM_EAGS                        (0x0000246c)
#define     SYSCON_PWR_CACTIVES_EAGM                        (0x00002470)
#define     SYSCON_PWR_CACTIVES_EAGS                        (0x00002474)
#define     SYSCON_PWR_PWRUPCPUSTG1                         (0x00002480)
#define     SYSCON_PWR_PWRUPCPUSTG2                         (0x00002484)
#define     SYSCON_PWR_PWRUPCPURAM                          (0x00002488)
#define     SYSCON_PWR_ISOLATECPU                           (0x0000248c)
#define     SYSCON_PWR_NPWRUPCPUSTG1_ACK                    (0x00002490)
#define     SYSCON_PWR_NPWRUPCPURAM_ACK                     (0x00002494)
#define     SYSCON_PWR_QACTIVE                              (0x00002498)
#define     SYSCON_PWR_STANDBYWFI                           (0x0000249c)
#define     SYSCON_PWR_STANDBYWFE                           (0x000024a0)
#define     SYSCON_PWR_DBGNOPWRDWN                          (0x000024a4)
#define     SYSCON_PWR_DBGPWRUPREQ                          (0x000024a8)
#define     SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2              (0x00002580)
#define     SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1              (0x00002584)
#define     SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0              (0x00002588)
#define     SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2              (0x0000258c)
#define     SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1              (0x00002590)
#define     SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0              (0x00002594)
#define     SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2              (0x00002598)
#define     SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1              (0x0000259c)
#define     SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0              (0x000025a0)
#define     SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2              (0x000025a4)
#define     SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1              (0x000025a8)
#define     SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0              (0x000025ac)

#define		RAM_BANK0_MASK			(0x0FFF0000)
#define		RAM_BANK1_LS_MASK		(0xF0000000)
#define		RAM_BANK1_MS_MASK		(0x000000FF)
#define		RAM_BANK2_MASK			(0x000FFF00)
#define		RAM_BANK3_MASK			(0xFFF00000)
#define		RAM_ALL_MASK			(0xFFFFFFFF)

/* DICKENS REGISTERS (Miscelaneous Node) */
#define		DKN_MN_NODE_ID				(0x0)
#define		DKN_DVM_DOMAIN_OFFSET		(0x0)
#define		DKN_MN_DVM_DOMAIN_CTL		(0x200)
#define		DKN_MN_DVM_DOMAIN_CTL_SET	(0x210)
#define		DKN_MN_DVM_DOMAIN_CTL_CLR	(0x220)

/* DICKENS HN-F (Fully-coherent Home Node) */
#define		DKN_HNF_NODE_ID					(0x20)
#define		DKN_HNF_TOTAL_NODES				(0x8)
#define		DKN_HNF_SNOOP_DOMAIN_CTL		(0x200)
#define		DKN_HNF_SNOOP_DOMAIN_CTL_SET	(0x210)
#define		DKN_HNF_SNOOP_DOMAIN_CTL_CLR	(0x220)

/* DICKENS clustid to Node */
#define		DKN_CLUSTER0_NODE		(1)
#define		DKN_CLUSTER1_NODE		(9)
#define		DKN_CLUSTER2_NODE		(11)
#define		DKN_CLUSTER3_NODE		(19)

/* PO RESET cluster id to bit */
#define		PORESET_CLUSTER0		(0x10000)
#define		PORESET_CLUSTER1		(0x20000)
#define		PORESET_CLUSTER2		(0x40000)
#define		PORESET_CLUSTER3		(0x80000)

/* IPI Masks */
#define		IPI0_MASK				(0x1111)
#define		IPI1_MASK				(0x2222)
#define		IPI2_MASK				(0x4444)
#define 	IPI3_MASK				(0x8888)

/* SYSCON KEY Value */
#define VALID_KEY_VALUE			(0xAB)

#define MAX_NUM_CLUSTERS    (4)
#define CORES_PER_CLUSTER   (4)
#define MAX_IPI				(19)
#define MAX_CPUS			(MAX_NUM_CLUSTERS * CORES_PER_CLUSTER)

typedef struct {
	u32 cpu;
	u32 cluster;
} pm_data;


void pm_cpu_shutdown(u32 cpu);
int pm_cpu_powerup(u32 cpu);
void pm_debug_read_pwr_registers(void);
void pm_dump_L2_registers(void);
int pm_cpu_logical_die(pm_data *pm_request);
int pm_cpul2_logical_die(pm_data *pm_request);
unsigned long pm_get_powered_down_cpu(void);
bool pm_cpu_last_of_cluster(u32 cpu);
void pm_dump_dickens(void);
void pm_init_cpu(u32 cpu);
void pm_cpu_logical_powerup(void);
void pm_cluster_logical_powerup(void);
bool pm_cpu_active(u32 cpu);
void pm_init_syscon(void);
extern bool pm_in_progress[];
extern bool cluster_power_up[];
extern u32 pm_cpu_powered_down;

#endif /* AXXIA_PWRC_H_ */
