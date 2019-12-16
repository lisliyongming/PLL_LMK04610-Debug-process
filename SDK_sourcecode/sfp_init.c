/*
 * sfp_init.c
 *
 *  Created on: 2019Äê1ÔÂ7ÈÕ
 *      Author: Eric
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../config/parser.h"
#include "io_control.h"

extern uint32_t axi_lite_addr;

int yunsdr_sfp_init()
{
	int i = 3;
	cJSON *root;
	char device[6];
	char *endptr;
	uint64_t tmp = 0;

	sprintf(device, "RF%d", 1);

	root = open_cfgfile(NULL);
	if(!root){
		fprintf(stderr, "Cannot open config.json file\n");
		return -1;
	}

	char *saveptr;

	char *dev_ip = get_string_from_cfgfile(root, device, "dev_ip");
	if(dev_ip == NULL)
		dev_ip = "169.254.45.119";
	char *devstr = strtok_r(dev_ip, ".", &saveptr);
	while(devstr != NULL) {
		tmp |= (atoi(devstr) << (i--)*8);
		devstr = strtok_r(NULL, ".", &saveptr);
	}
	AXI_REG_WRITE(axi_lite_addr, 64 * 4, tmp);  //FPGA_IP
	free(devstr); i = 3; tmp = 0;

	char *host_ip = get_string_from_cfgfile(root, device, "host_ip");
	if(host_ip == NULL)
		host_ip = "169.254.45.118";
	devstr = strtok_r(host_ip, ".", &saveptr);
	while(devstr != NULL) {
		tmp |= (atoi(devstr) << (i--)*8);
		devstr = strtok_r(NULL, ".", &saveptr);
	}
	AXI_REG_WRITE(axi_lite_addr, 65 * 4, tmp);  //HOST_IP
	free(devstr); i = 5; tmp = 0;

	char *dev_mac = get_string_from_cfgfile(root, device, "dev_mac");
	if(dev_mac == NULL)
		dev_mac = "00:0A:0B:0C:0D:0E";
	devstr = strtok_r(dev_mac, ":", &saveptr);
	while(devstr != NULL) {
		tmp |= (strtoll(devstr, &endptr, 16) << (i--)*8);
		devstr = strtok_r(NULL, ":", &saveptr);
	}
	free(devstr); i = 0;
	AXI_REG_WRITE(axi_lite_addr, 66 * 4, tmp); //FPGA_MAC[31:0]
	AXI_REG_WRITE(axi_lite_addr, 67 * 4, tmp>>32); //FPGA_MAC[47:32]

	close_cfgfile(root);

	return 0;
}
