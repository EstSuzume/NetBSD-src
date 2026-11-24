// SPDX-License-Identifier: BSD-2-Clause

#ifndef _ARM_RZA1_RZA1_VAR_H_
#define _ARM_RZA1_RZA1_VAR_H_

/* Clocks of GR-PEACH and similar boards (EXTAL 13.33MHz, clock mode 0) */
#define	RZA1_P0_FREQ_DEFAULT	33333333
#define	RZA1_P1_FREQ_DEFAULT	66666666

u_int	rza1_get_p0_freq(void);
u_int	rza1_get_p1_freq(void);
uint32_t rza1_get_entropy32(void);

#endif /* _ARM_RZA1_RZA1_VAR_H_ */
