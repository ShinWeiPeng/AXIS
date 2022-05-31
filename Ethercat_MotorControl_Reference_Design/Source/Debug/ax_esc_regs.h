/*
 ******************************************************************************
 * Software License Agreement
 *     Copyright (c) 2021 ASIX Electronics Corporation   All rights reserved.
 * (1) This software is owned by ASIX Electronics Corporation and is protected 
 *     under all applicable laws, including copyright laws.
 * (2) This software is provided by ASIX Electronics Corporation, including 
 *     modifications and/or derivative works of this software, are only authorized
 *     for use on ASIX products. 
 * (3) Redistribution and use of this software without specific written permission
 *     is void and will automatically terminate your rights under this license. 
 * (4) Redistribution of source code or in binary form must retain/reproduce the 
 *     copyright notice above and the following disclaimer in the documentation or other
 *     materials provided with the distribution.
 *
 * DISCLAIMER
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ASIX MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
 * LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
 * TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER ASIX
 * ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
 * FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
 * ANY REASON RELATED TO THIS SOFTWARE, EVEN IF ASIX OR ITS AFFILIATES HAVE
 * BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * ASIX reserves the right, without notice, to make changes to this software
 * and to discontinue the availability of this software.
 ******************************************************************************
 */

#ifndef __ESC_REGS_H__
#define __ESC_REGS_H__

/* INCLUDE FILE DECLARATIONS */
#include "NuMicro.h"


/* NAMING CONSTANT DECLARATIONS */


/* ====================== Register Bit Definition ====================== */

/* ------------- Features ------------- */
#define ESC_CORE_TYPE               0x0000  /* Type register */
#define ESC_CORE_MAJOR_REV_X        0x0001	/* Revision register */
#define ESC_CORE_BUILD              0x0002	/* Build register */
typedef union
{
  uint16_t d16;
  struct{
    uint8_t maintenance_ver_z:       4;  /* Bit0:3    Maintenance version Z */
    uint8_t minor_ver_y:             4;  /* Bit7:4    Minor version Y */	
    uint8_t patch_level:             8;  /* Bit15:8   Patch level/development build */			
  } b;
} oESC_CORE_BUILD;
#define FMMUS_SUPPORTED             0x0004
#define SYNC_MANAGERS_SUPPORTED     0x0005
#define RAM_SIZE                    0x0006
#define PORT_DESCRIPTOR             0x0007
typedef union
{
  uint8_t d8;
  struct{
    uint8_t port0_config:            2;  /* 00: Not implemented, 01: Not configured, 10: EBUS, 11: MII/RMII/RGMII */
    uint8_t port1_config:            2;
    uint8_t port2_config:            2;		
    uint8_t port3_config:            2;		
  } b;
} oPORT_DESCRIPTOR;
#define ESC_FEATURES_SUPPORTED      0x0008
typedef union
{
  uint16_t d16;
  struct{
    uint16_t fmmu_operation:             1;  /* Bit0   0: Bit oriented, 1: Byte oriented */
    uint16_t unused_register_access:     1;  /* Bit1   0: Allowed, 1: Not supported */
    uint16_t distributed_clocks:         1;  /* Bit2   0: Not available, 1: Available */		
    uint16_t dc_width:                   1;  /* Bit3   0: 32bit, 1: 64bit */
    uint16_t reserved_5_4:               2;  /* Bit5:4 */
    uint16_t enhanced_link_detec_mii:    1;  /* Bit6   0: Not available, 1: Available */
    uint16_t separate_handle_of_fcs_err: 1;  /* Bit7   0: Not supported, 1: Supported */
    uint16_t enhanced_dc_sync_act:       1;  /* Bit8   0: Not available, 1: Available */		
    uint16_t lrw_command_support:        1;  /* Bit9   0: Supported, 1: Not supported */
    uint16_t read_write_cmd_support:     1;  /* Bit10  0: Supported, 1: Not supported */		
    uint16_t fixed_fmmu_sm_config:       1;  /* Bit11  0: Variable configuration, 1: Fixed configuration */				
    uint16_t reserved_15_12:             4;
  } b;
} oESC_FEATURES_SUPPORTED;
#define CONFIG_STATION_ADDR         0x0010
#define CONFIG_STATION_ALIAS        0x0012
#define REG_WRITE_ENABLE            0x0020
typedef union
{
  uint8_t d8;
  struct{
    uint8_t reg_write_enable:            1;  /* Bit0   0: Register write is enabled, 1: Register write is disabled */
    uint8_t reserved_7_1:                7;
  } b;
} oREG_WRITE_ENABLE;
#define REG_WRITE_PROTECTION        0x0021
typedef union
{
  uint8_t d8;
  struct{
    uint8_t reg_write_protect:           1;  /* Bit0   Register 0x0000:0x0F7F(except for 0x0020 and 0x0030) write protection is 0: disabled, 1: enabled */
    uint8_t reserved_7_1:                7;		
  } b;
} oREG_WRITE_PROTECTION;
#define ESC_WRITE_ENABLE            0x0030
typedef union
{
  uint8_t d8;
  struct{
    uint8_t esc_write_enable:            1;  /* Bit0   All areas write protection is 0: enabled, 1: Register write is disabled */
    uint8_t reserved_7_1:                7;		
  } b;
} oESC_WRITE_ENABLE;
#define ESC_WRITE_PROTECTION        0x0031
typedef union
{
  uint8_t d8;
  struct{
    uint8_t esc_write_protect:           1;  /* Bit0   All areas (except for 0x0030) write protection is 0: disabled, 1: enabled */
    uint8_t reserved_7_1:                7;		
  } b;
} oESC_WRITE_PROTECTION;

/* ------------- Application Layer ------------- */
#define AL_CONTROL                  0x0120
typedef union
{
  uint16_t d16;
  struct{
    uint16_t initiate_state:                  4;  /* Bit3:0    1:Request Init State
                                                          3:Request Bootstrap State
                                                          2:Request Pre-Operational State		
                                                          4:Request Safe-Operational State		
                                                          8:Request Operational State		
                                                  */
    uint16_t err_ind_ack:                     1;  /* Bit4 0:No ack of error ind in AL status, 1:Ack of error ind in AL status */
    uint16_t device_identification:           1;  /* Bit5 0:No request, 1:Request  */
    uint16_t reserved_15_6:                  10;  /* Bit15:6 */
  } b;
} oAL_CONTROL;
#define AL_STATUS                   0x0130
typedef union
{
  uint16_t d16;
  struct{
    uint16_t actual_state:                    4;  /* Bit3:0    1:Init State
                                                          3:Bootstrap State
                                                          2:Pre-Operational State		
                                                          4:Safe-Operational State		
                                                          8:Operational State		
                                                  */
    uint16_t err_ind:                         1;  /* Bit4 0:Device is in state as requested, 1:Device has not entered requested state */
    uint16_t device_identification:           1;  /* Bit5 0:Device indentification not valid, 1:Device indentification loaded */
    uint16_t reserved_15_6:                  10;  /* Bit15:6 */
  } b;
} oAL_STATUS;
#define AL_STATUS_CODE              0x0134
#define PDI_CONTROL                 0x0140
#define ESC_CONFIG                  0x0141 /*  ESC Configuration Register (0x0141) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t device_emulation:                 1;  /* Bit0 0:AL status has to be set by PDI, 1: */
    uint8_t enhanced_link_detec_all_port:     1;  /* Bit1 0:disable, 1:enable */
    uint8_t distributed_clk_sync_out_unit:    1;  /* Bit2 0:disable(power saving), 1:enable */
    uint8_t distributed_clk_latch_in_unit:    1;  /* Bit3 0:disable(power saving), 1:enable */
    uint8_t enhanced_link_port0:              1;  /* Bit4 0:disable, 1:enable  */
    uint8_t enhanced_link_port1:              1;  /* Bit5 0:disable, 1:enable  */
    uint8_t enhanced_link_port2:              1;  /* Bit6 0:disable, 1:enable  */
    uint8_t enhanced_link_port3:              1;  /* Bit7 0:disable, 1:enable */
  } b;
} oESC_CONFIG;

#define PDI_CONFIG                       0x0150 /*  PDI Configuration Register (0x0150:0x0153) */
typedef union
{
  uint32_t d32;

		/* PDI General Terms */
  struct{
		uint32_t pdi_cfg_7_0:             8;  /* Bit7:0 */
		uint32_t sync0_driver_pol:        2;  /* Bit9:8    00b: Push-Pull/Active Low
		                                              01b: Open-Drain/Active Low 
		                                              10b: Push-Pull/Active High
		                                              11b: Open-Source/Active High
		                                      */
		uint32_t sync0_latch0_cfg:        1;  /* Bit10     0b: Latch0 Input
		                                              1b: Sync0 Output
		                                      */
		uint32_t sync0_map_al_event:      1;  /* Bit11     0b: Disable
		                                              1b: Enable
		                                      */
		
		uint32_t sync1_driver_pol:        2;  /* Bit13:12  00b: Push-Pull/Active Low
		                                              01b: Open-Drain/Active Low 
		                                              10b: Push-Pull/Active High
		                                              11b: Open-Source/Active High
		                                      */
		uint32_t sync1_latch1_cfg:        1;  /* Bit14     0b: Latch1 Input
		                                              1b: Sync1 Output
		                                      */
		uint32_t sync1_map_al_event:      1;  /* Bit15     0b: Disable
		                                              1b: Enable
		                                      */		
		uint32_t pdi_cfg_31_16:          16;  /* Bit31:16 */
  } bGeneralTerm;

	/* PDI Digital I/O Configuration */
  struct{
    uint32_t output_valid_pol:        1;  /* Bit0      OUTVALID polarity, 0: Active high, 1: Active low */
    uint32_t output_valid_mode:       1;  /* Bit1      OUTVALID mode, 0: Output event signaling, 1: Process data watchdog trigger */
    uint32_t unidir_bidir_mode:       1;  /* Bit2     */
    uint32_t watchdog_behavior:       1;  /* Bit3     */	
    uint32_t input_sample_at:         2;  /* Bit5:4   */	
    uint32_t output_update_at:        2;  /* Bit7:6   */
    uint32_t general_term_15_8:       8;  /* Bit15:8  */	
    uint32_t io_dir:                 16;  /* Bit31:16 */
  } bDIO;
	
	/* PDI SPI Slave Configuration */
  struct{
    uint32_t spi_mode:                2;  /* Bit1:0    00b: Mode0
                                                  01b: Mode1
                                                  10b: Mode2
                                                  11b: Mode3 
		                                      */
    uint32_t spi_irq_driver_pol:      2;  /* Bit3:2    00b: Push-Pull/Active Low 
                                                  01b: Open-Drain/Active Low
                                                  10b: Push-Pull/Active High
                                                  11b: Open-Source/Active High
		                                      */
    uint32_t spi_sel_pol:             1;  /* Bit4      0b:  Active Low
                                                  1b:  Active High
                                          */
    uint32_t data_out_sampl_mode:     1;  /* Bit5      0b:  Normal sample
                                                  1b:  Late sample
                                          */	
    uint32_t reserved_7_6:            2;  /* Bit7:6   */
    uint32_t general_term_15_8:       8;  /* Bit15:8  */
    uint32_t io_dir:                 16;  /* Bit31:16 */
  } bSPIS;
	
} oPDI_CONFIG;

#define ECAT_EVENT_MASK           0x0200 /* ECAT Event Mask (0x0200:0x0201) */
#define PDI_AL_EVENT_MASK         0x0204 /* PDI AL Event Mask (0x0204:0x0207) */
#define ECAT_EVENT_REQUEST        0x0210 /* ECAT Event Request (0x0210:0x0211) */
#define AL_EVENT_REQUEST          0x0220 /* AL Event Request (0x0220:0x0223) */
typedef union
{
  uint16_t d16;
  struct{
    uint16_t dc_latch_event:                 1;  /* Bit0 0: No change on DC latch input, 1: At least one change on DC latch input */
    uint16_t reserved_1:                     1;  /* Bit1 */
    uint16_t dc_status_event:                1;  /* Bit2 0: No change in DL status, 1: DL status change */
    uint16_t al_status_event:                1;  /* Bit3 0: No change in AL status, 1: AL status change */
    uint16_t sync_0_7_manager_status:        8;  /* Bit11:4 0: No Sync channel 0 event, 1: Sync channel 0 event pending */
		uint16_t reserved_15_12:                 4;
  } b;
} oECAT_EVENT;

typedef union
{
  uint32_t d32;
  struct{
    uint32_t al_control_event:               1;  /* Bit0 0: No AL control register change, 1: AL control register has been written */
    uint32_t dc_latch_event:                 1;  /* Bit1 */
    uint32_t state_of_dc_sync_0:             1;  /* Bit2 */
    uint32_t state_of_dc_sync_1:             1;  /* Bit3 */
    uint32_t sm_activation:                  1;  /* Bit4 0: No change in any SyncManager, 1: At least one SyncManager changed */
    uint32_t eep_emulation:                  1;  /* Bit5 0: No command pending, 1: EEPROM command pending */		
    uint32_t watchdog_process_data:          1;  /* Bit6 0: Has not expired, 1: Has expired */				
    uint32_t sm_0_15_interrupts:            16;  /* Bit23:7 0: No SyncManager[x] interrupt, 1: SyncManager[x] interrupt pending */				
    uint32_t reserved_32_24:                15;  /* Bit32:24 */						
  } b;
} oAL_EVENT;
#define PDI_ERR_COUNTER           0x030D /* PDI Error Counter (0x030D) */
#define PDI_ERR_CODE              0x030E /* PDI Error Code (0x030E) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t spi_clk_num_of_access:           3;  /* Bit2:0 */
		uint8_t busy_violation_during_read:      1;  /* Bit3 */
		uint8_t read_termination_missing:        1;  /* Bit4 */		
		uint8_t access_continue_after_read:      1;  /* Bit5 */
		uint8_t cmd_err:                         2;  /* Bit7:6 */		
  } bSPI;
	
  struct{
		uint8_t busy_viola_during_read:          1;  /* Bit0 */
		uint8_t busy_viola_during_write:         1;  /* Bit1 */		
		uint8_t addr_err_for_read:               1;  /* Bit2 */
		uint8_t addr_err_for_write:              1;  /* Bit3 */		
		uint8_t reserved_7_4:                    4;  /* Bit7:4 */		
  } bASYNC_SYNC;	
} oPDI_ERR_CODE;
#define LOST_LINK_COUNTER         0x0310 /* Lost link counter (0x0310:0x313) */

/* ------------- Watchdog ------------- */
#define WD_DIVIDER                0x0400 /* Watchdog Divider (0x0400:0x0401) */
#define WD_TIME_PDI               0x0410 /* Watchdog Time PDI (0x0410:0x0411) */
#define WD_TIME_PROCESS_DATA      0x0420 /* Watchdog Time Process Data (0x0420:0x0421) */
#define WD_STATUS_PROCESS_DATA    0x0440 /* Watchdog Status Process Data (0x0440:0x0441) */
typedef union
{
  uint16_t d16;
  struct{
        uint16_t wd_process_data_status:         1;  /* Bit0 0: Watchdog process data expired, 1: Watchdog process data is active or disable */				
        uint16_t reserved_15_1:                 15;  /* Bit15:1 */						
  } b;
} oWD_STATUS_PROCESS_DATA;
#define WD_COUNTER_PROCESS_DATA   0x0442 /* Watchdog Counter Process Data (0x0442) */
#define WD_COUNTER_PDI            0x0443 /* Watchdog Counter PDI (0x0442) */

/* ------------- SII EEPROM Interface ------------- */
#define EEPCFGR                   0x0500
#define EEPPASR                   0x0501
#define EEPCSR                    0x0502
#define EEPAR                     0x0504
#define EEPDR                     0x0508

	/* EEPROM Configuration Register (0x0500) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t eep_offered_to_pdi:      1;  /* EEPCFGR.0      0: No, 1: Yes */
    uint8_t force_ecat_access:       1;  /* EEPCFGR.1      0: Do not change bit 0x0501[0], 1: Reset bit 0x0501[0] to 0 */	
		uint8_t reserved_7_2:            6;
  } b;
} oEEPCFGR;

	/* EEPROM PDI Access State Register (0x0501) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t pdi_access_to_eep:       1;  /* EEPPASR.0      0: PDI release EEPROM access, 1: PDI takes EEPROM access */
		uint8_t reserved_7_1:            7;
  } b;
} oEEPPASR;

/* EEPROM Control/Status Register (0x0502:0x503) */
#define EEPCSR_CMD_IDLE           0x00
#define EEPCSR_CMD_READ           0x01
#define EEPCSR_CMD_WRITE          0x02
#define EEPCSR_CMD_RELOAD         0x04
typedef union
{
  uint16_t d16;
  struct{
    uint16_t write_enable:            1;  /* EEPCSR.0      0: Write disabled, 1: Write enabled */
    uint16_t reserved_4_1:            4;  /* EEPCSR.4:1 */		
    uint16_t eep_emulation:           1;  /* EEPCSR.5      0: Normal operation, 1: PDI emulates EEPROM */			
    uint16_t read_8bytes:             1;  /* EEPCSR.6      0: 4 bytes, 1: 8 bytes */					
    uint16_t two_addr_bytes:          1;  /* EEPCSR.7      0: 1 address byte(1Kbit~16Kbit), 1: 2 address bytes(32Kbit~4Mbit) */							
    uint16_t cmd:                     3;  /* EEPCSR.10:8   000: No command, 001: Read, 010:Write, 100:Reload */									
    uint16_t checksum_err:            1;  /* EEPCSR.11     0: Checksum ok, 1: checksum error */									
    uint16_t eep_loading_status:      1;  /* EEPCSR.12     0: Device information ok, 1: Device information not available */
    uint16_t err_ack_cmd:             1;  /* EEPCSR.13     0: No error, 1: Missing ACK or command error */		
    uint16_t err_write_enable:        1;  /* EEPCSR.14     0: No error, 1: Write command without write enable */				
    uint16_t eep_intf_busy:           1;  /* EEPCSR.15     0: Idle, 1: Busy */						
  } b;
} oEEPCSR;

typedef struct
{
  oEEPCFGR   eepcfgr;	
  oEEPPASR   eeppasr;		
  oEEPCSR    eepcsr;
	uint32_t        eepar;
	uint8_t         eepdr[8];	
} EEP_REGS;

/* ------------- MII Management Interface ------------- */
#define MMCSR                    0x0510
#define PAR                      0x0512
#define PRAR                     0x0513
#define PDR                      0x0514
#define MMEASR                   0x0516
#define MMPASR                   0x0517
#define PPSR                     0x0518

	/* MII Management Control/Status Register (0x0510:0x0511) */
typedef union
{
  uint16_t d16;
  struct{
    uint16_t write_enable:            1;  /* MMCSR.0      0: Write disabled, 1: Write enabled */
    uint16_t pdi_ctrl_possible:       1;  /* MMCSR.1      0: Only ECAT control, 1: PDI control possible */		
    uint16_t link_detec_enable:       1;  /* MMCSR.2      MI link detection and configuration */		
    uint16_t port0_phy_addr:          5;  /* MMCSR.7:3    PHY address for port 0 */
    uint16_t cmd:                     2;  /* MMCSR.9:8    Command */		
    uint16_t reseverd_12_10:          3;  /* MMCSR.12:10 */
    uint16_t read_err:                1;  /* MMCSR.13     Read error */
    uint16_t cmd_err:                 1;  /* MMCSR.14     Command error */
    uint16_t busy:                    1;  /* MMCSR.15     Busy */		
  } b;
} oMMCSR;

#define MMCSR_CMD_IDLE           0x00
#define MMCSR_CMD_READ           0x01
#define MMCSR_CMD_WRITE          0x02

  /* PHY Address Register (0x0512) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t  phy_addr:                5;  /* PAR.4:0   PHY Address */
    uint8_t  reserved_6_5:            2;  /* PAR.6:5 */
    uint8_t  show_phy_addr:           1;  /* PAR.7     Show configured PHY address of port0-3 */	
  } b;
} oPAR;

  /* PHY Register Address Register (0x0513) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t  phy_reg_addr:            5;  /* PRAR.4:0   Address of PHY Register that shall be read/written */
    uint8_t  reserved_7_5:            3;  /* PRAR.7:5 */		
  } b;
} oPRAR;

	/* PHY Data Register (0x0514:0x0515) */
typedef union
{
  uint16_t d16;
  struct{
    uint16_t rw_data:                 16;  /* PDR.15:0   PHY Read/Write Data */
  } b;
} oPDR;

  /* MII Management ECAT Acces State (0x0516) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t exclusive_access_mii:      1;  /* MMEASR.0   Access to MII management */
    uint8_t reserved_7_1:              7;  /* MMEASR.7:1 */		
  } b;
} oMMEASR;

  /* MII Management PDI Access State (0x0517) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t pdi_access_mii:            1;  /* MMPASR.0    Access to MII management */
    uint8_t ecat_force_pdi_access:     1;  /* MMPASR.1    ECAT Force PDI Access State */
    uint8_t reserved_7_2:              6;  /* MMPASR.7:2 */
  } b;
} oMMPASR;

/* PHY Port(x) Status Register (0x0518:0x051B) */
typedef union
{
  uint8_t d8;
  struct{
    uint8_t phy_link_detected:         1;  /* PPSR.0   Phsical link status */
    uint8_t link_detected:             1;  /* PPSR.1   Link status */		
    uint8_t link_error:                1;  /* PPSR.2   Link status error */				
    uint8_t read_error:                1;  /* PPSR.3   Read error */						
    uint8_t link_partner_error:        1;  /* PPSR.4   Link partner error */								
    uint8_t phy_config_updated:        1;  /* PPSR.5   PHY configuration updated */										
    uint8_t reserved_7_6:              2;  /* PPSR.7:6 */												
  } b;
} oPPSR;

typedef struct
{
  oMMCSR    mmcsr;
  oPAR      par;
  oPRAR     prar;
  oPDR      pdr;	
  oMMEASR   mmeasr;	
  oMMPASR   mmpasr;		
  oPPSR     ppsr[4];			
} MII_REGS;

/* ------------- FMMU ------------- */
#define FMMU_0_LOGIC_START_ADDR   0x0600 /* FMMU[x] Logical Start Address (0x0600:0x0603) */
#define FMMU_0_LENGTH             0x0604 /* FMMU[x] Length (0x0604:0x0605) */
#define FMMU_0_LOGIC_START_BIT    0x0606 /* FMMU[x] Logical Start bit (0x0606) */
#define FMMU_0_LOGIC_STOP_BIT     0x0607 /* FMMU[x] Logical Stop bit (0x0607) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t logical_bit:                    3;  /* Bit2:0 */				
		uint8_t reserved_7_3:                   5;  /* Bit7:3 */						
  } b;
} oFMMU_LOGIC_START_STOP_BIT;
#define FMMU_0_PHY_START_ADDR     0x0608 /* FMMU[x] Physical Start Address (0x0608:0x0609) */
#define FMMU_0_PHY_START_BIT      0x060A /* FMMU[x] Physical Start bit (0x060A) */
#define FMMU_0_TYPE               0x060B /* FMMU[x] Type (0x060B) */
#define FMMU_0_ACTIVATE           0x060C /* FMMU[x] Activate (0x060C) */
#define FMMU_OFFSET               0x0010

/* ------------- SyncManagers ------------- */
#define SM_0_PHY_START_ADDR       0x0800 /* SM[x] Physical Start Address (0x0800:0x0801) */
#define SM_0_LENGTH               0x0802 /* SM[x] Length (0x0802:0x0803) */
#define SM_0_CONTROL              0x0804 /* SM[x] Control Register (0x0804) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t operation_mode:                 2;  /* Bit1:0  00: 3 buffer mode, 01: reserved, 10: Mailbox, 11: reserved */				
		uint8_t direction:                      2;  /* Bit3:2  00: ECAT read, PDI write 01: ECAT write, PDI read 10: reserved 11: reserved */
		uint8_t intr_in_ecat_event_request:     1;  /* Bit4 */		
		uint8_t intr_in_al_event_request:       1;  /* Bit5 */				
		uint8_t watchdog_trigger_enable:        1;  /* Bit6 */						
		uint8_t reserved_7:                     1;  /* Bit7 */								
  } b;
} oSM_CONTROL;
#define SM_0_STATUS               0x0805 /* SM[x] Status Register (0x0805) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t intr_write:                     1;  /* Bit0    0: After first byte of buffer was read, 1: After buffer was completely write */				
		uint8_t intr_read:                      1;  /* Bit1    0: After first byte of buffer was written, 1: After buffer was completely read */
		uint8_t reserved_2:                     1;  /* Bit2 */		
		uint8_t mailbox_status:                 1;  /* Bit3    0: Mailbox empty, 1: Mailbox full */				
		uint8_t three_buf_status:               2;  /* Bit5:4  00: 1st buffer, 01: 2nd buffer, 10: 3rd buffer, 11: no buffer written */						
		uint8_t read_buf_in_use:                1;  /* Bit6 */								
		uint8_t write_buf_in_use:               1;  /* Bit7 */										
  } b;
} oSM_STATUS;
#define SM_0_ACTIVATE             0x0806 /* SM[x] Activate (0x0806) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t sm_enable:                      1;  /* Bit0    0: SM disable, 1: SM enable */				
		uint8_t repeat_request:                 1;  /* Bit1 */
		uint8_t reserved_5_2:                   4;  /* Bit5:2 */		
		uint8_t latch_event_ecat:               1;  /* Bit6    0: No, 1: Generate latch event when EtherCAT master issue a buffer exchange */				
		uint8_t latch_event_pdi:                1;  /* Bit7    0: No, 1: Generate latch events when PDI issues a buffer change or when PDI access buffer start address */						
  } b;
} oSM_ACTIVATE;
#define SM_0_PDI_CONTROL          0x0807 /* SM[x] PDI Control (0x0807) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t deactivate_sm:                  1;  /* Bit0 */				
		uint8_t repeat_ack:                     1;  /* Bit1 */
		uint8_t reserved_7_2:                   4;  /* Bit7:2 */		
  } b;
} oSM_PDI_CONTROL;
#define SM_OFFSET                 0x0008

/* ------------- DC Module ------------- */
#define DC_PORT0_RECV_TIME        0x0900 /* Receive Time Port 0 (0x0900:0x0903) */
#define DC_PORT1_RECV_TIME        0x0904 /* Receive Time Port 1 (0x0904:0x0907) */
#define DC_PORT2_RECV_TIME        0x0908 /* Receive Time Port 2 (0x0908:0x090B) */
#define DC_PORT3_RECV_TIME        0x090F /* Receive Time Port 3 (0x090C:0x090F) */
typedef union
{
  uint32_t d32;
  struct{
		uint32_t local_time7_0_at_wr_reg_0x900:  8;  /* Bit7:0 */				
		uint32_t local_time24_8_at_wr_reg_0x900: 24; /* Bit31:8 */
  } b;
} oDC_PORT0_RECV_TIME;

#define DC_PROC_UNIT_RECV_TIME    0x0918 /* Receive Time ECAT Processing Unit (0x0918:0x091F) */
#define DC_SYSTEM_TIME            0x0910 /* System Time (0x0910:0x0917) */
typedef union
{
  uint32_t d32[2];
  struct{
		uint32_t value_compare_with_systim:    32; /* Bit31:0 */				
		uint32_t local_copy_of_systim:         32; /* Bit63:32 */
  } b;
} oDC_SYSTEM_TIME;

#define DC_SYSTEM_TIME_OFFSET     0x0920 /* System Time Offset (0x0920:0x0927) */
#define DC_SYSTEM_TIME_DELAY      0x0928 /* System Time Delay (0x0928:0x092B) */
#define DC_SYSTEM_TIME_DIFF       0x092C /* System Time Difference (0x092C:0x092F) */
typedef union
{
  int32_t d32;
  struct{
		uint32_t diff_rev_systim_loc_copy_systim: 31; /* Bit30:0 */				
		uint32_t loc_copy_systim_LorE_rev_systim: 1; /* Bit32 */
  } b;
} oDC_SYSTEM_TIME_DIFF;

#define DC_SPEED_COUNTER_START           0x0930 /* Speed Counter Start (0x0930:0x0931) */
typedef union
{
  uint16_t d16;
  struct{
		uint16_t bw_for_adj_of_loc_copy_of_systim: 15; /* Bit14:0 */				
		uint16_t reserved_15:                       1; /* Bit15 */
  } b;
} oDC_SPEED_COUNTER_START;

#define DC_SPEED_COUNTER_DIFF            0x0932 /* Speed Counter Diff (0x0932:0x0933) */
#define DC_SYSTIME_DIFF_FILTER_DEPTH     0x0934 /* System Time Diff Filter Depth (0x0934) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t filt_depth_for_avg_rev_systim_deva: 4; /* Bit3:0 */				
		uint8_t reserved_7_4:                       4; /* Bit7:4 */
  } b;
} oDC_SYSTIME_DIFF_FILTER_DEPTH;

#define DC_SPEED_COUNTER_FILTER_DEPTH     0x0935 /* Speed Counter Filter Depth (0x0935) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t filt_depth_for_avg_clk_period_deva: 4; /* Bit3:0 */				
		uint8_t reserved_7_4:                       4; /* Bit7:4 */
  } b;
} oDC_SPEED_COUNTER_FILTER_DEPTH;

#define DC_REV_TIME_LATCH_MODE            0x0936 /* Receive Time Latch Mode (0x0936) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t rev_time_latch_mode:                1; /* Bit0
		                                             0: Forwarding mode, used if frame are entering ESC at port 0 first.
		                                             1: Reverse mode, used if frame are entering ESC at port 1-3 first.
		                                               */				
		uint8_t reserved_7_1:                       7; /* Bit7:1 */
  } b;
} oDC_REV_TIME_LATCH_MODE;

#define DC_CYCLIC_UNIT_CTRL               0x0980 /* Cyclic Unit Control (0x0980) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t sync_out_unit_ctrl:                 1; /* Bit0 0: ECAT controlled, 1: PDI controlled */
		uint8_t reserved_3_1:                       3; /* Bit3:1 */
		uint8_t latch_in_unit_0:                    1; /* Bit4 0: ECAT controlled, 1: PDI controlled */
		uint8_t latch_in_unit_1:                    1; /* Bit5 0: ECAT controlled, 1: PDI controlled */		
		uint8_t reserved_7_6:                       2; /* Bit7:6 */		
  } b;
} oDC_CYCLIC_UNIT_CTRL;

#define DC_SYNC_ACTIVATION                0x0981 /* Sync. Acviation (0x0981) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t sync_out_unit_act:                  1; /* Bit0 0: Deactivated, 1: Activated */
		uint8_t sync0_gen:                          1; /* Bit1 0: Deactivated, 1: SYNC0 pulse is generated*/
		uint8_t sync1_gen:                          1; /* Bit2 0: Deactivated, 1: SYNC1 pulse is generated */		
		uint8_t auto_act_by_wr_start_time_cyc_op:   1; /* Bit3 0: Disabled, 1: Auto-activation enabled */
		uint8_t ext_of_start_time_cyc_op:           1; /* Bit4 0: No extension, 1: Extend 32 bit written Start Time to 64 bit */
		uint8_t start_time_plausibility_chk:        1; /* Bit5 0: Disabled, 1: Immediate SyncSignal generation if Start Time is outside near future */
		uint8_t near_future_configuration:          1; /* Bit6 0: 1/2 dc width future, 1: ~2.1 sec. future */
		uint8_t sync_debug_pulse:                   1; /* Bit7 0: Deactived
		                                                  1: Immediately generate one ping only on SYNC0-1 according to 0x0981 for debugging
                                          		     */		
  } b;
} oDC_SYNC_ACTIVATION;

#define DC_PULSE_LEN_OF_SYNC_SIGNAL       0x0982 /* Pulse length of SyncSignals (0x0982:0x983) */
#define DC_ACTIVATION_STATUS              0x0984 /* Activation Status (0x0984) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t sync0_act_state:                    1; /* Bit0 0/1: First SYNC0 pulse is not pending/pending */
		uint8_t sync1_act_state:                    1; /* Bit1 0/1: First SYNC1 pulse is not pending/pending */
		uint8_t start_time_plausibility_chk_result: 1; /* Bit2 0/1: Start Time was within/out of near future */		
		uint8_t reserved_7_3:                       5; /* Bit7:3 */				
  } b;
} oDC_ACTIVATION_STATUS;

#define DC_SYNC0_STATUS                   0x098E /* SYNC0 Status (0x098E) */
#define DC_SYNC1_STATUS                   0x098F /* SYNC1 Status (0x098F) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t syncx_state_for_ack_mode:           1; /* Bit0 */
		uint8_t reserved_7_1:                       7; /* Bit7:1 */				
  } b;
} oDC_SYNCx_STATUS;

#define DC_START_TIME_CYCLIC_OPERATION    0x0990 /* Start Time Cyclic Operation (0x0990:0x0997)
                                                    Write: Start system time of cyclic operation in ns.
                                                    Read: System time of next SYNC0 pulse in ns.
                                                  */

#define DC_NEXT_SYNC1_PULSE               0x0998 /* System time of next SYNC1 (0x0998:0x099B) */
#define DC_SYNC0_CYCLE_TIME               0x09A0 /* Time between two consecutive SYNC0 pulse in ns (0x09A0:0x09A3) */
#define DC_SYNC1_CYCLE_TIME               0x09A4 /* Time between SYNC0 pulse and SYNC1 pulse in ns (0x09A4:0x09A7) */
#define DC_LATCH0_CONTROL                 0x09A8 /* LATCH 0 Control (0x09A8) */
#define DC_LATCH1_CONTROL                 0x09A9 /* LATCH 1 Control (0x09A9) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t latchx_positive_edge:               1; /* Bit0, 0: Continue latch active, 1: Single event */
		uint8_t latchx_negative_edge:               1; /* Bit1, 0: Continue latch active, 1: Single event */				
		uint8_t reserved_7_2:                       6; /* Bit7:2 */						
  } b;
} oDC_LATCHx_CONTROL;

#define DC_LATCH0_STATUS                  0x09AE /* LATCH 0 Status (0x09AE) */
#define DC_LATCH1_STATUS                  0x09AF /* LATCH 1 Status (0x09AF) */
typedef union
{
  uint8_t d8;
  struct{
		uint8_t event_latchx_positive_edge:         1; /* Bit0, 0: Positive edge not detected or continue mode
		                                                 , 1: Positive edge detected in single event mode only
                                          		     */
		uint8_t event_latchx_negative_edge:         1; /* Bit1, 0: Negative edge not detected or continue mode
		                                                 , 1: Negative edge detected in single event mode only
                                          		     */
		uint8_t latchx_pin_state:                   1;
		uint8_t reserved_7_3:                       5; /* Bit7:3 */						
  } b;
} oDC_LATCHx_STATUS;

#define DC_LATCH0_TIME_POSITIVE_EDGE      0x09B0 /* LATCH 0 Time Positive Edge (0x09B0:0x09B7) */
#define DC_LATCH0_TIME_NEGATIVE_EDGE      0x09B8 /* LATCH 0 Time Negative Edge (0x09B8:0x09BF) */
#define DC_LATCH1_TIME_POSITIVE_EDGE      0x09C0 /* LATCH 1 Time Positive Edge (0x09C0:0x09C7) */
#define DC_LATCH1_TIME_NEGATIVE_EDGE      0x09C8 /* LATCH 1 Time Negative Edge (0x09C8:0x09CF) */
#define DC_ECAT_BUF_CHANGE_EVENT_TIME     0x09F0 /* ECAT Buffer Change Event Time (0x09F0:0x09F3) */
#define DC_PDI_BUF_START_EVENT_TIME       0x09F8 /* PDI Buffer Start Event Time (0x09F8:0x09FB) */
#define DC_PDI_BUF_CHANGE_EVENT_TIME      0x09FC /* PDI Buffer Change Event Time (0x09FC:0x09FF) */

/* --------- Product&Vendor ID ------------- */
#define IC_PRODUCT_ID               0x0E00
#define IC_VENDOR_ID                0x0E08

/* ------------- Digital I/O Registers ------------- */
#define DIOOR                    0x0F00
#define DIOIR                    0x1000

/* --------- General Purpose I/O Registers ------------- */
#define GIOOR                    0x0F10
#define GIOIR                    0x0F18

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES */

/* EXPORTED SUBPROGRAM SPECIFICATIONS */

#endif /* __ESC_REGS_H__ */

/* End of esc_regs.h */
