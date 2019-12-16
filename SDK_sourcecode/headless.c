/**
 * \file headless.c
 *
 * \brief Contains example code for user integration with their application
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 *
 */

/****< Insert User Includes Here >***/
#include <stdio.h>
#include "talise.h"
#include "talise_tx.h"
#include "adi_hal.h"
#include "parameters.h"
#include "io_control.h"
#include "rf_config.h"
#include "xscugic.h"
#include "intr_define.h"
#include "INTR_IRQHandler.h"
#include "intr_inital.h"
#include "spi_ctrl.h"
#include "xtime_l.h"
#include "ff.h"
#include "xsdps.h"
/**********************************************************/
/**********************************************************/
/********** Talise Data Structure Initializations ********/
uint32_t axi_lite_addr = XPAR_SYSTEM_TOP_I_AXI_LITEV3_0_BASEADDR;
struct spi_desc *spi_desc_lmk04610;
struct spi_desc *clk_auxdac;
uint32_t fpga_ver=0xa4;
uint32_t sw_ver=0xa3;
uint64_t RFPllLoFrequency_Hz=2500e6;
/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
uint32_t jesd_phy0_addr = XPAR_SYSTEM_TOP_I_JESD204_PHY_0_BASEADDR;
struct adi_hal hal_rf1,hal_rf2;
taliseDevice_t talDev_rf1 = { .devHalInfo = &hal_rf1, .devStateInfo = { 0 } };
taliseDevice_t talDev_rf2 = { .devHalInfo = &hal_rf2, .devStateInfo = { 0 } };
jesd_core tx_jesd;
jesd_core rx_jesd;
jesd_core_array jesd_array_rf1;
jesd_core_array jesd_array_rf2;
fsm_status fsm=IDEL;
struct gpio_desc *gpio_trx_sw;
struct gpio_desc *gpio_fddtdd_sw;
struct gpio_desc *rf2_gpio_ref_select2;
struct gpio_desc *gpio_lmk_sync;
struct gpio_desc *gpio_rf1_gpio0;
struct gpio_desc *gpio_rf2_gpio0;
struct gpio_desc *rf1_gpio_power_on;
struct gpio_desc *rf2_gpio_power_on;
struct gpio_desc *gpio_tx1_enable;
struct gpio_desc *gpio_tx2_enable;
struct gpio_desc *gpio_rx1_enable;
struct gpio_desc *gpio_rx2_enable;
struct gpio_desc *gpio_test;
struct gpio_desc *gpio_lmk04610_rst_n;
struct gpio_desc *gpio_lmk04610_sync;
const spi_init_param spi_lmk04610_param = { XILINX_SPI, SPI1_DEVICE_ID, 1000000, SPI_MODE_0, 4};
const spi_init_param spi_clkauxdac_param = { XILINX_SPI, SPI0_DEVICE_ID, 10000000, SPI_MODE_0, 4};
extern int yunsdr_sfp_init();
extern int intr_irq_proc();
/**********************************************************/
/**********************************************************/
int main(void)
{
	tx_jesd.base_address = XPAR_SYSTEM_TOP_I_JESD204_TX_0_BASEADDR;
	rx_jesd.base_address = XPAR_SYSTEM_TOP_I_JESD204_RX_0_BASEADDR;
	tx_jesd.rx_tx_n = 0;
	tx_jesd.scramble_enable = 1;
	tx_jesd.octets_per_frame = 2;
	tx_jesd.frames_per_multiframe = 32;
	tx_jesd.subclass_mode = 1;
	rx_jesd.rx_tx_n = 1;
	rx_jesd.scramble_enable = 1;
	rx_jesd.octets_per_frame = 4;
	rx_jesd.frames_per_multiframe = 32;
	rx_jesd.subclass_mode = 1;
	jesd_array_rf1.tx_jesd = tx_jesd;
	jesd_array_rf1.rx_jesd = rx_jesd;
	jesd_array_rf1.orx_jesd = rx_jesd;
	jesd_array_rf2.tx_jesd = tx_jesd;
	jesd_array_rf2.rx_jesd = rx_jesd;
	jesd_array_rf2.orx_jesd = rx_jesd;
	jesd_array_rf1.orx_jesd.base_address=XPAR_SYSTEM_TOP_I_JESD204_ORX_0_BASEADDR;
	jesd_array_rf2.orx_jesd.base_address=XPAR_SYSTEM_TOP_I_JESD204_ORX_1_BASEADDR;
	jesd_array_rf2.tx_jesd.base_address=XPAR_SYSTEM_TOP_I_JESD204_TX_1_BASEADDR;
	jesd_array_rf2.rx_jesd.base_address=XPAR_SYSTEM_TOP_I_JESD204_RX_1_BASEADDR;
	uint32_t talAction = TALACT_NO_ACTION;
	int32_t status;
	print("**************************************************\n\r");
	print("**********Y590 CH4 SDR SFP+ begin MOD V3**********\n\r");
	print("**************************************************\n\r");
	printf("Please wait...\n");

	/**********************************************************/
	/**********************************************************/
	/************ Talise Initialization Sequence *************/
	/**********************************************************/
	/**********************************************************/

	/** < Insert User System Clock(s) Initialization Code Here >
	 * System Clock should provide a device clock and SYSREF signal
	 * to the Talise device.
	 **/


	status = gpio_get(&rf1_gpio_power_on, RF1_POWER_ON);
	status = gpio_get(&rf2_gpio_power_on, RF2_POWER_ON);
	status = gpio_get(&gpio_tx1_enable, TX1_ENABLE);
	status = gpio_get(&gpio_tx2_enable, TX2_ENABLE);
	status = gpio_get(&gpio_rx1_enable, RX1_ENABLE);
	status = gpio_get(&gpio_rx2_enable, RX2_ENABLE);
	status = gpio_get(&gpio_test, TEST);
	status = gpio_get(&gpio_lmk04610_rst_n, LMK04610_RESET_N);
	status = gpio_get(&gpio_lmk04610_sync, LMK04610_SYNC);
	status = gpio_get(&gpio_rf1_gpio0, RF1_GPIO0);
	status = gpio_get(&gpio_rf2_gpio0, RF2_GPIO0);

	gpio_direction_output(rf1_gpio_power_on, 1);
	gpio_direction_output(rf2_gpio_power_on, 1);
	gpio_direction_output(gpio_test, 0);
	gpio_direction_output(gpio_tx1_enable, 1);
	gpio_direction_output(gpio_tx2_enable, 1);
	gpio_direction_output(gpio_rx1_enable, 1);
	gpio_direction_output(gpio_rx2_enable, 1);
	gpio_direction_output(gpio_lmk04610_sync, 0);

	/**************************************************************************/
	/*****      configure system clock					                  *****/
	/**************************************************************************/
	spi_init_clk_lmk04610(&spi_desc_lmk04610, spi_lmk04610_param);
	spi_init_clk(&clk_auxdac, spi_clkauxdac_param);

//	lmk04610_init(spi_desc_lmk04610);
	lmk04610_init_sync_func(spi_desc_lmk04610);

	printf("PLL1&PLL2 locked!\n");
	talAction = ad9009_initial(&talDev_rf1, &talDev_rf2, jesd_array_rf1,
			jesd_array_rf2, jesd_phy0_addr);
	TALISE_setTxAttenuation(&talDev_rf1, TAL_TX1, talInit.tx.tx1Atten_mdB);
	TALISE_setTxAttenuation(&talDev_rf1, TAL_TX2, talInit.tx.tx2Atten_mdB);
	TALISE_setTxAttenuation(&talDev_rf2, TAL_TX1, talInit.tx.tx1Atten_mdB);
	TALISE_setTxAttenuation(&talDev_rf2, TAL_TX2, talInit.tx.tx2Atten_mdB);
	TALISE_getRfPllFrequency(&talDev_rf1, TAL_RF_PLL, &RFPllLoFrequency_Hz);
	printf("RF1,rf_lo_freq=%lluHz\n", RFPllLoFrequency_Hz);
	TALISE_getRfPllFrequency(&talDev_rf2, TAL_RF_PLL, &RFPllLoFrequency_Hz);
	printf("RF2,rf_lo_freq=%lluHz\n", RFPllLoFrequency_Hz);
	if (talAction != TALACT_NO_ACTION) {
		/*** < User: decide what to do based on Talise recovery action returned > ***/
		printf("error: ad9009_initial() failed\n");
	} else
		printf("Y590 CH4 MOD V3 config successful!\n");

	//READ FILE CONFIG IP AND MAC
	yunsdr_sfp_init();
	//set yunsdr axi lite
	val_out = AXI_REG_READ(axi_lite_addr, 0);
	printf("The SDR model is %ld(4x4) rev V3!\n", val_out & 0xFFFF);
	val_out = AXI_REG_READ(axi_lite_addr, 63*4);
	printf("The pldma version is %ld\n", (val_out & 0xFFFF) + 1);
	printf("The FPGA firmware version=0x%2lx\n", fpga_ver);
	printf("The Software firmware version=0x%2lx\n", sw_ver);
	printf("The lasted build at 20191121_PM1\n");
	printf("The lasted build at %s %s\n", __TIME__, __DATE__);

	val_out = AXI_REG_READ(axi_lite_addr, 64 * 4);
	printf("yunsdr ip=%d.%d.%d.%d;", (uint8_t) (val_out >> 24),
			(uint8_t) (val_out >> 16), (uint8_t) (val_out >> 8),
			(uint8_t) (val_out));
	val_out = AXI_REG_READ(axi_lite_addr, 65 * 4);
	printf("PC ip=%d.%d.%d.%d\n", (uint8_t) (val_out >> 24),
			(uint8_t) (val_out >> 16), (uint8_t) (val_out >> 8),
			(uint8_t) (val_out));
	val_out = AXI_REG_READ(axi_lite_addr, 67* 4);
	printf("yunsdr mac=%02x.%02x.", (uint8_t) (val_out >> 8),
			(uint8_t) (val_out));
	val_out = AXI_REG_READ(axi_lite_addr, 66 * 4);
	printf("%02x.%02x.%02x.%02x\n", (uint8_t) (val_out >> 24),
			(uint8_t) (val_out >> 16), (uint8_t) (val_out >> 8),
			(uint8_t) (val_out));
	int i;
	for (i = 1; i <= 5; i++) {
		val_out = AXI_REG_READ(axi_lite_addr, (73 + i) * 4);
		printf("yunsdr ch%d port=%ld\n", i, (uint32_t) val_out);
	}
	//set yunsdr axi lite
	AXI_REG_WRITE(axi_lite_addr, 2 * 4, 0);					//pps_disable
	AXI_REG_WRITE(axi_lite_addr, 3 * 4,
			talDev_rf1.devStateInfo.rxOutputRate_kHz * 1000);//sample_1pps
	AXI_REG_WRITE(axi_lite_addr, 4 * 4, 1*128*1024*1024); 	//base_addr0
	AXI_REG_WRITE(axi_lite_addr, 5 * 4, 2*128*1024*1024); 	//base_addr1
	AXI_REG_WRITE(axi_lite_addr, 6 * 4, 3*128*1024*1024); 	//base_addr2
	AXI_REG_WRITE(axi_lite_addr, 7 * 4, 4*128*1024*1024); 	//base_addr3
	AXI_REG_WRITE(axi_lite_addr, 8 * 4, 2*128*1024*1024); 	//bon_addr0
	AXI_REG_WRITE(axi_lite_addr, 9 * 4, 3*128*1024*1024); 	//bon_addr1
	AXI_REG_WRITE(axi_lite_addr, 10 * 4, 4*128*1024*1024);	//bon_addr2
	AXI_REG_WRITE(axi_lite_addr, 11 * 4, 5*128*1024*1024);	//bon_addr3
	AXI_REG_WRITE(axi_lite_addr, 12 * 4, (1<<20)-8);		//pldma tx burst len 16~2^20-8
	AXI_REG_WRITE(axi_lite_addr, 13 * 4, (1<<20)-8);		//pldma rx burst len 16~2^20-8
	AXI_REG_WRITE(axi_lite_addr, 14 * 4, 0);				//tx_digital_en
	AXI_REG_WRITE(axi_lite_addr, 15 * 4, 0);				//rx_digital_en
	AXI_REG_WRITE(axi_lite_addr, 28 * 4, 0);				//int_dec_value
	AXI_REG_WRITE(axi_lite_addr, 29 * 4,
			talDev_rf1.devStateInfo.rxOutputRate_kHz * 1000);//clock_1pps
	AXI_REG_WRITE(axi_lite_addr, 48 * 4, 245*1024*1024); 	//rxfifo_pfull
	AXI_REG_WRITE(axi_lite_addr, 49 * 4, 60*1024*1024); 	//rxfifo_pless
	AXI_REG_WRITE(axi_lite_addr, 93 * 4, 1040); 			//gap_len
	printf("The MAX sample rate is %ld Hz\n",
			talDev_rf1.devStateInfo.rxOutputRate_kHz * 1000);
	printf("The sample rate is %ld Hz\n",AXI_REG_READ(axi_lite_addr, 3 * 4));
	printf("The rx buffer size is %ld Byte\n",AXI_REG_READ(axi_lite_addr, 48 * 4));
	//initial interrupt controller
	int32_t val;
	val = intr_inital();
	if (val == 0) {
		printf("ScuGic Interrupt Setup PASSED\r\n");
	}
	val = AXI_REG_READ(axi_lite_addr, 30 * 4);
	if (val == 1) {
		printf("PL DDR3 initial successful!\n");
		printf("Y590 initial successful!\n");
		AXI_REG_WRITE(axi_lite_addr, 30 * 4, 1); 	//config done
	} else {
		printf("PL DDR3 initial failed!\n");
		printf("Y590 initial failed!\n");
		AXI_REG_WRITE(axi_lite_addr, 30 * 4, 0); 	//config fail
	}
//	lmk04610_lock_check(spi_desc_lmk04610);
	gpio_direction_output(gpio_lmk04610_sync, 0);
	auxdac_update_register(clk_auxdac, 1839);
	XTime tEnd, tCur;
	int time_flag = 0;
	int j;
	while (1) {
		switch (fsm) {
		case IDEL:
			//			if (!time_flag) {
			//				XTime_GetTime(&tCur);
			//				time_flag = 1;
			//			} else {
			//				XTime_GetTime(&tEnd);
			//				if ((tEnd - tCur) >= COUNTS_PER_SECOND) {
			//					printf("\nrx overflow cyc:");
			//					for (j = 1; j <= 4; j++)
			//						printf("%ld, ",
			//								AXI_REG_READ(axi_lite_addr, (35 + j) * 4));
			//					printf("; rx count(MB) :");
			//					for (j = 1; j <= 4; j++)
			//						printf("%f, ",
			//								(double) AXI_REG_READ(axi_lite_addr,
			//										(43 + j) * 4) / 1024 / 1024);
			//					time_flag = 0;
			//				}
			//			}
			break;
		case CMD_IRQ:
			intr_irq_proc(intr_cmd_param, &fsm);
			break;
		default:
			break;
		}
	}
}
