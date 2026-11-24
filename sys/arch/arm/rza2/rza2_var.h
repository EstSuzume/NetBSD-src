// SPDX-License-Identifier: BSD-2-Clause

#ifndef _ARM_RZA2_RZA2_VAR_H_
#define _ARM_RZA2_RZA2_VAR_H_

#define	RZA2_P1_FREQ_DEFAULT	66000000

u_int	rza2_get_p1_freq(void);
uint32_t rza2_get_entropy32(void);
int	rza2_pinctrl_set_ether_mii(u_int, bool);

#endif /* _ARM_RZA2_RZA2_VAR_H_ */
