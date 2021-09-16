// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 1999 - 2006 Intel Corporation. */

/* e1000_hw.c
 * Shared functions for accessing and configuring the MAC
 */

#include "e1000.h"

static void e1000_init_rx_addrs(struct e1000_hw *hw);
static void e1000_write_reg_io(struct e1000_hw *hw, u32 offset, u32 value);
static s32 e1000_write_eeprom_microwire(struct e1000_hw *hw, u16 offset,
					u16 words, u16 *data);
static void e1000_raise_ee_clk(struct e1000_hw *hw, u32 *eecd);
static void e1000_lower_ee_clk(struct e1000_hw *hw, u32 *eecd);
static void e1000_shift_out_ee_bits(struct e1000_hw *hw, u16 data, u16 count);
static s32 e1000_write_phy_reg_ex(struct e1000_hw *hw, u32 reg_addr,
				  u16 phy_data);
static s32 e1000_read_phy_reg_ex(struct e1000_hw *hw, u32 reg_addr,
				 u16 *phy_data);
static u16 e1000_shift_in_ee_bits(struct e1000_hw *hw, u16 count);
static s32 e1000_acquire_eeprom(struct e1000_hw *hw);
static void e1000_release_eeprom(struct e1000_hw *hw);
static void e1000_standby_eeprom(struct e1000_hw *hw);
static s32 e1000_do_read_eeprom(struct e1000_hw *hw, u16 offset, u16 words,
				u16 *data);
static s32 e1000_do_write_eeprom(struct e1000_hw *hw, u16 offset, u16 words,
				 u16 *data);


static DEFINE_MUTEX(e1000_eeprom_lock);
static DEFINE_SPINLOCK(e1000_phy_lock);




/**
 * e1000_reset_hw - reset the hardware completely
 * @hw: Struct containing variables accessed by shared code
 *
 * Reset the transmit and receive units; mask and clear all interrupts.
 */
s32 e1000_reset_hw(struct e1000_hw *hw)
{
	u32 ctrl;

	/* Clear interrupt mask to stop board from generating interrupts */
	e_dbg("Masking off all interrupts\n");
	ew32(IMC, 0xffffffff);

	/* Disable the Transmit and Receive units.  Then delay to allow
	 * any pending transactions to complete before we hit the MAC with
	 * the global reset.
	 */
	ew32(RCTL, 0);
	ew32(TCTL, E1000_TCTL_PSP);
	E1000_WRITE_FLUSH();


	/* Delay to allow any outstanding PCI transactions to complete before
	 * resetting the device
	 */
	msleep(10);

	ctrl = er32(CTRL);


	/* Issue a global reset to the MAC.  This will reset the chip's
	 * transmit, receive, DMA, and link units.  It will not effect
	 * the current PCI configuration.  The global reset bit is self-
	 * clearing, and should clear within a microsecond.
	 */
	e_dbg("Issuing a global reset to MAC\n");

	switch (hw->mac_type) {
	case e1000_82545:
		/* These controllers can't ack the 64-bit write when issuing the
		 * reset, so use IO-mapping as a workaround to issue the reset
		 */
		E1000_WRITE_REG_IO(hw, CTRL, (ctrl | E1000_CTRL_RST));
		break;
	case e1000_82545_rev_3:
		/* Reset is performed on a shadow of the control register */
		ew32(CTRL_DUP, (ctrl | E1000_CTRL_RST));
		break;
	default:
		ew32(CTRL, (ctrl | E1000_CTRL_RST));
		break;
	}




	/* Clear interrupt mask to stop board from generating interrupts */
	e_dbg("Masking off all interrupts\n");
	ew32(IMC, 0xffffffff);

	/* Clear any pending interrupt events. */
	er32(ICR);



	return E1000_SUCCESS;
}

/**
 * e1000_init_hw - Performs basic configuration of the adapter.
 * @hw: Struct containing variables accessed by shared code
 *
 * Assumes that the controller has previously been reset and is in a
 * post-reset uninitialized state. Initializes the receive address registers,
 * multicast table, and VLAN filter table. Calls routines to setup link
 * configuration and flow control settings. Clears all on-chip counters. Leaves
 * the transmit and receive units disabled and uninitialized.
 */
s32 e1000_init_hw(struct e1000_hw *hw)
{
	s32 ret_val = 0;

	/* Setup the receive address. This involves initializing all of the
	 * Receive Address Registers (RARs 0 - 15).
	 */
	e1000_init_rx_addrs(hw);


	return ret_val;
}







/**
 * e1000_read_phy_reg - read a phy register
 * @hw: Struct containing variables accessed by shared code
 * @reg_addr: address of the PHY register to read
 * @phy_data: pointer to the value on the PHY register
 *
 * Reads the value from a PHY register, if the value is on a specific non zero
 * page, sets the page first.
 */
s32 e1000_read_phy_reg(struct e1000_hw *hw, u32 reg_addr, u16 *phy_data)
{
	u32 ret_val;
	unsigned long flags;

	spin_lock_irqsave(&e1000_phy_lock, flags);

	if ((hw->phy_type == e1000_phy_igp) &&
	    (reg_addr > MAX_PHY_MULTI_PAGE_REG)) {
		ret_val = e1000_write_phy_reg_ex(hw, IGP01E1000_PHY_PAGE_SELECT,
						 (u16) reg_addr);
		if (ret_val)
			goto out;
	}

	ret_val = e1000_read_phy_reg_ex(hw, MAX_PHY_REG_ADDRESS & reg_addr,
					phy_data);
out:
	spin_unlock_irqrestore(&e1000_phy_lock, flags);

	return ret_val;
}

static s32 e1000_read_phy_reg_ex(struct e1000_hw *hw, u32 reg_addr,
				 u16 *phy_data)
{
	u32 i;
	u32 mdic = 0;
	const u32 phy_addr = 1;

	if (reg_addr > MAX_PHY_REG_ADDRESS) {
		e_dbg("PHY Address %d is out of range\n", reg_addr);
		return -E1000_ERR_PARAM;
	}


		/* Set up Op-code, Phy Address, and register address in the MDI
		 * Control register.  The MAC will take care of interfacing with
		 * the PHY to retrieve the desired data.
		 */
	mdic = ((reg_addr << E1000_MDIC_REG_SHIFT) |
		(phy_addr << E1000_MDIC_PHY_SHIFT) |
		(E1000_MDIC_OP_READ));

	ew32(MDIC, mdic);

	/* Poll the ready bit to see if the MDI read
		* completed
		*/
	for (i = 0; i < 64; i++) {
		udelay(50);
		mdic = er32(MDIC);
		if (mdic & E1000_MDIC_READY)
			break;
	}
	if (!(mdic & E1000_MDIC_READY)) {
		e_dbg("MDI Read did not complete\n");
		return -E1000_ERR_PHY;
	}
	if (mdic & E1000_MDIC_ERROR) {
		e_dbg("MDI Error\n");
		return -E1000_ERR_PHY;
	}
	*phy_data = (u16)mdic;
	
	return E1000_SUCCESS;
}

/**
 * e1000_write_phy_reg - write a phy register
 *
 * @hw: Struct containing variables accessed by shared code
 * @reg_addr: address of the PHY register to write
 * @phy_data: data to write to the PHY
 *
 * Writes a value to a PHY register
 */
s32 e1000_write_phy_reg(struct e1000_hw *hw, u32 reg_addr, u16 phy_data)
{
	u32 ret_val;
	unsigned long flags;

	spin_lock_irqsave(&e1000_phy_lock, flags);

	if ((hw->phy_type == e1000_phy_igp) &&
	    (reg_addr > MAX_PHY_MULTI_PAGE_REG)) {
		ret_val = e1000_write_phy_reg_ex(hw, IGP01E1000_PHY_PAGE_SELECT,
						 (u16)reg_addr);
		if (ret_val) {
			spin_unlock_irqrestore(&e1000_phy_lock, flags);
			return ret_val;
		}
	}

	ret_val = e1000_write_phy_reg_ex(hw, MAX_PHY_REG_ADDRESS & reg_addr,
					 phy_data);
	spin_unlock_irqrestore(&e1000_phy_lock, flags);

	return ret_val;
}

static s32 e1000_write_phy_reg_ex(struct e1000_hw *hw, u32 reg_addr,
				  u16 phy_data)
{
	u32 i;
	u32 mdic = 0;
	const u32 phy_addr = 1;

	if (reg_addr > MAX_PHY_REG_ADDRESS) {
		e_dbg("PHY Address %d is out of range\n", reg_addr);
		return -E1000_ERR_PARAM;
	}

	/* Set up Op-code, Phy Address, register address, and data
		* intended for the PHY register in the MDI Control register.
		* The MAC will take care of interfacing with the PHY to send
		* the desired data.
		*/

	mdic = (((u32)phy_data) |
		(reg_addr << E1000_MDIC_REG_SHIFT) |
		(phy_addr << E1000_MDIC_PHY_SHIFT) |
		(E1000_MDIC_OP_WRITE));

	ew32(MDIC, mdic);

	/* Poll the ready bit to see if the MDI read
		* completed
		*/
	for (i = 0; i < 641; i++) {
		udelay(5);
		mdic = er32(MDIC);
		if (mdic & E1000_MDIC_READY)
			break;
	}
	if (!(mdic & E1000_MDIC_READY)) {
		e_dbg("MDI Write did not complete\n");
		return -E1000_ERR_PHY;
	}
	

	return E1000_SUCCESS;
}

/**
 * e1000_phy_hw_reset - reset the phy, hardware style
 * @hw: Struct containing variables accessed by shared code
 *
 * Returns the PHY to the power-on reset state
 */
s32 e1000_phy_hw_reset(struct e1000_hw *hw)
{
	u32 ctrl;
	e_dbg("Resetting Phy...\n");


	/* Read the device control register and assert the
		* E1000_CTRL_PHY_RST bit. Then, take it out of reset.
		* For e1000 hardware, we delay for 10ms between the assert
		* and de-assert.
		*/
	ctrl = er32(CTRL);
	ew32(CTRL, ctrl | E1000_CTRL_PHY_RST);
	E1000_WRITE_FLUSH();

	msleep(10);

	ew32(CTRL, ctrl);
	E1000_WRITE_FLUSH();

	udelay(150);


	/* Wait for FW to finish PHY configuration. */
	return 0;
}

/**
 * e1000_phy_reset - reset the phy to commit settings
 * @hw: Struct containing variables accessed by shared code
 *
 * Resets the PHY
 * Sets bit 15 of the MII Control register
 */
s32 e1000_phy_reset(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 phy_data;

	ret_val = e1000_read_phy_reg(hw, PHY_CTRL, &phy_data);
	if (ret_val)
		return ret_val;

	phy_data |= MII_CR_RESET;
	ret_val = e1000_write_phy_reg(hw, PHY_CTRL, phy_data);
	if (ret_val)
		return ret_val;

	udelay(1);

	return E1000_SUCCESS;
}




/**
 * e1000_init_eeprom_params - initialize sw eeprom vars
 * @hw: Struct containing variables accessed by shared code
 *
 * Sets up eeprom variables in the hw struct.  Must be called after mac_type
 * is configured.
 */
s32 e1000_init_eeprom_params(struct e1000_hw *hw)
{
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
	u32 eecd = er32(EECD);
	s32 ret_val = E1000_SUCCESS;


	eeprom->opcode_bits = 3;
	eeprom->delay_usec = 50;
	if (eecd & E1000_EECD_SIZE) {
		eeprom->word_size = 256;
		eeprom->address_bits = 8;
	} else {
		eeprom->word_size = 64;
		eeprom->address_bits = 6;
	}

	return ret_val;
}

/**
 * e1000_raise_ee_clk - Raises the EEPROM's clock input.
 * @hw: Struct containing variables accessed by shared code
 * @eecd: EECD's current value
 */
static void e1000_raise_ee_clk(struct e1000_hw *hw, u32 *eecd)
{
	/* Raise the clock input to the EEPROM (by setting the SK bit), and then
	 * wait <delay> microseconds.
	 */
	*eecd = *eecd | E1000_EECD_SK;
	ew32(EECD, *eecd);
	E1000_WRITE_FLUSH();
	udelay(hw->eeprom.delay_usec);
}

/**
 * e1000_lower_ee_clk - Lowers the EEPROM's clock input.
 * @hw: Struct containing variables accessed by shared code
 * @eecd: EECD's current value
 */
static void e1000_lower_ee_clk(struct e1000_hw *hw, u32 *eecd)
{
	/* Lower the clock input to the EEPROM (by clearing the SK bit), and
	 * then wait 50 microseconds.
	 */
	*eecd = *eecd & ~E1000_EECD_SK;
	ew32(EECD, *eecd);
	E1000_WRITE_FLUSH();
	udelay(hw->eeprom.delay_usec);
}

/**
 * e1000_shift_out_ee_bits - Shift data bits out to the EEPROM.
 * @hw: Struct containing variables accessed by shared code
 * @data: data to send to the EEPROM
 * @count: number of bits to shift out
 */
static void e1000_shift_out_ee_bits(struct e1000_hw *hw, u16 data, u16 count)
{
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
	u32 eecd;
	u32 mask;

	/* We need to shift "count" bits out to the EEPROM. So, value in the
	 * "data" parameter will be shifted out to the EEPROM one bit at a time.
	 * In order to do this, "data" must be broken down into bits.
	 */
	mask = 0x01 << (count - 1);
	eecd = er32(EECD);

	eecd &= ~E1000_EECD_DO;


	do {
		/* A "1" is shifted out to the EEPROM by setting bit "DI" to a
		 * "1", and then raising and then lowering the clock (the SK bit
		 * controls the clock input to the EEPROM).  A "0" is shifted
		 * out to the EEPROM by setting "DI" to "0" and then raising and
		 * then lowering the clock.
		 */
		eecd &= ~E1000_EECD_DI;

		if (data & mask)
			eecd |= E1000_EECD_DI;

		ew32(EECD, eecd);
		E1000_WRITE_FLUSH();

		udelay(eeprom->delay_usec);

		e1000_raise_ee_clk(hw, &eecd);
		e1000_lower_ee_clk(hw, &eecd);

		mask = mask >> 1;

	} while (mask);

	/* We leave the "DI" bit set to "0" when we leave this routine. */
	eecd &= ~E1000_EECD_DI;
	ew32(EECD, eecd);
}

/**
 * e1000_shift_in_ee_bits - Shift data bits in from the EEPROM
 * @hw: Struct containing variables accessed by shared code
 * @count: number of bits to shift in
 */
static u16 e1000_shift_in_ee_bits(struct e1000_hw *hw, u16 count)
{
	u32 eecd;
	u32 i;
	u16 data;

	/* In order to read a register from the EEPROM, we need to shift 'count'
	 * bits in from the EEPROM. Bits are "shifted in" by raising the clock
	 * input to the EEPROM (setting the SK bit), and then reading the value
	 * of the "DO" bit.  During this "shifting in" process the "DI" bit
	 * should always be clear.
	 */

	eecd = er32(EECD);

	eecd &= ~(E1000_EECD_DO | E1000_EECD_DI);
	data = 0;

	for (i = 0; i < count; i++) {
		data = data << 1;
		e1000_raise_ee_clk(hw, &eecd);

		eecd = er32(EECD);

		eecd &= ~(E1000_EECD_DI);
		if (eecd & E1000_EECD_DO)
			data |= 1;

		e1000_lower_ee_clk(hw, &eecd);
	}

	return data;
}

/**
 * e1000_acquire_eeprom - Prepares EEPROM for access
 * @hw: Struct containing variables accessed by shared code
 *
 * Lowers EEPROM clock. Clears input pin. Sets the chip select pin. This
 * function should be called before issuing a command to the EEPROM.
 */
static s32 e1000_acquire_eeprom(struct e1000_hw *hw)
{
	u32 eecd, i = 0;

	eecd = er32(EECD);

	/* Request EEPROM Access */
	eecd |= E1000_EECD_REQ;
	ew32(EECD, eecd);
	eecd = er32(EECD);
	while ((!(eecd & E1000_EECD_GNT)) &&
			(i < E1000_EEPROM_GRANT_ATTEMPTS)) {
		i++;
		udelay(5);
		eecd = er32(EECD);
	}
	if (!(eecd & E1000_EECD_GNT)) {
		eecd &= ~E1000_EECD_REQ;
		ew32(EECD, eecd);
		e_dbg("Could not acquire EEPROM grant\n");
		return -E1000_ERR_EEPROM;
	}

	/* Setup EEPROM for Read/Write */
	/* Clear SK and DI */
	eecd &= ~(E1000_EECD_DI | E1000_EECD_SK);
	ew32(EECD, eecd);

	/* Set CS */
	eecd |= E1000_EECD_CS;
	ew32(EECD, eecd);

	return E1000_SUCCESS;
}

/**
 * e1000_standby_eeprom - Returns EEPROM to a "standby" state
 * @hw: Struct containing variables accessed by shared code
 */
static void e1000_standby_eeprom(struct e1000_hw *hw)
{
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
	u32 eecd;

	eecd = er32(EECD);


	eecd &= ~(E1000_EECD_CS | E1000_EECD_SK);
	ew32(EECD, eecd);
	E1000_WRITE_FLUSH();
	udelay(eeprom->delay_usec);

	/* Clock high */
	eecd |= E1000_EECD_SK;
	ew32(EECD, eecd);
	E1000_WRITE_FLUSH();
	udelay(eeprom->delay_usec);

	/* Select EEPROM */
	eecd |= E1000_EECD_CS;
	ew32(EECD, eecd);
	E1000_WRITE_FLUSH();
	udelay(eeprom->delay_usec);

	/* Clock low */
	eecd &= ~E1000_EECD_SK;
	ew32(EECD, eecd);
	E1000_WRITE_FLUSH();
	udelay(eeprom->delay_usec);

}

/**
 * e1000_release_eeprom - drop chip select
 * @hw: Struct containing variables accessed by shared code
 *
 * Terminates a command by inverting the EEPROM's chip select pin
 */
static void e1000_release_eeprom(struct e1000_hw *hw)
{
	u32 eecd;

	eecd = er32(EECD);
	
	/* cleanup eeprom */

	/* CS on Microwire is active-high */
	eecd &= ~(E1000_EECD_CS | E1000_EECD_DI);

	ew32(EECD, eecd);

	/* Rising edge of clock */
	eecd |= E1000_EECD_SK;
	ew32(EECD, eecd);
	E1000_WRITE_FLUSH();
	udelay(hw->eeprom.delay_usec);

	/* Falling edge of clock */
	eecd &= ~E1000_EECD_SK;
	ew32(EECD, eecd);
	E1000_WRITE_FLUSH();
	udelay(hw->eeprom.delay_usec);

	/* Stop requesting EEPROM access */
	eecd &= ~E1000_EECD_REQ;
	ew32(EECD, eecd);
}



/**
 * e1000_read_eeprom - Reads a 16 bit word from the EEPROM.
 * @hw: Struct containing variables accessed by shared code
 * @offset: offset of  word in the EEPROM to read
 * @data: word read from the EEPROM
 * @words: number of words to read
 */
s32 e1000_read_eeprom(struct e1000_hw *hw, u16 offset, u16 words, u16 *data)
{
	s32 ret;

	mutex_lock(&e1000_eeprom_lock);
	ret = e1000_do_read_eeprom(hw, offset, words, data);
	mutex_unlock(&e1000_eeprom_lock);
	return ret;
}

static s32 e1000_do_read_eeprom(struct e1000_hw *hw, u16 offset, u16 words,
				u16 *data)
{
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
	u32 i = 0;


	/* A check for invalid values:  offset too large, too many words, and
	 * not enough words.
	 */
	if ((offset >= eeprom->word_size) ||
	    (words > eeprom->word_size - offset) ||
	    (words == 0)) {
		e_dbg("\"words\" parameter out of bounds. Words = %d,"
		      "size = %d\n", offset, eeprom->word_size);
		return -E1000_ERR_EEPROM;
	}

	/* EEPROM's that don't use EERD to read require us to bit-bang the SPI
	 * directly. In this case, we need to acquire the EEPROM so that
	 * FW or other port software does not interrupt.
	 */
	/* Prepare the EEPROM for bit-bang reading */
	if (e1000_acquire_eeprom(hw) != E1000_SUCCESS)
		return -E1000_ERR_EEPROM;

	/* Set up the Microwire EEPROM for bit-bang reading.  We have
	 * acquired the EEPROM at this point, so any returns should release it
	 */
	for (i = 0; i < words; i++) {
		/* Send the READ command (opcode + addr)  */
		e1000_shift_out_ee_bits(hw,
					EEPROM_READ_OPCODE_MICROWIRE,
					eeprom->opcode_bits);
		e1000_shift_out_ee_bits(hw, (u16)(offset + i),
					eeprom->address_bits);

		/* Read the data.  For microwire, each word requires the
			* overhead of eeprom setup and tear-down.
			*/
		data[i] = e1000_shift_in_ee_bits(hw, 16);
		e1000_standby_eeprom(hw);
		cond_resched();
	}


	/* End this read operation */
	e1000_release_eeprom(hw);

	return E1000_SUCCESS;
}

/**
 * e1000_validate_eeprom_checksum - Verifies that the EEPROM has a valid checksum
 * @hw: Struct containing variables accessed by shared code
 *
 * Reads the first 64 16 bit words of the EEPROM and sums the values read.
 * If the sum of the 64 16 bit words is 0xBABA, the EEPROM's checksum is
 * valid.
 */
s32 e1000_validate_eeprom_checksum(struct e1000_hw *hw)
{
	u16 checksum = 0;
	u16 i, eeprom_data;

	for (i = 0; i < (EEPROM_CHECKSUM_REG + 1); i++) {
		if (e1000_read_eeprom(hw, i, 1, &eeprom_data) < 0) {
			e_dbg("EEPROM Read Error\n");
			return -E1000_ERR_EEPROM;
		}
		checksum += eeprom_data;
	}

#ifdef CONFIG_PARISC
	/* This is a signature and not a checksum on HP c8000 */
	if ((hw->subsystem_vendor_id == 0x103C) && (eeprom_data == 0x16d6))
		return E1000_SUCCESS;

#endif
	if (checksum == (u16)EEPROM_SUM)
		return E1000_SUCCESS;
	else {
		e_dbg("EEPROM Checksum Invalid\n");
		return -E1000_ERR_EEPROM;
	}
}

/**
 * e1000_update_eeprom_checksum - Calculates/writes the EEPROM checksum
 * @hw: Struct containing variables accessed by shared code
 *
 * Sums the first 63 16 bit words of the EEPROM. Subtracts the sum from 0xBABA.
 * Writes the difference to word offset 63 of the EEPROM.
 */
s32 e1000_update_eeprom_checksum(struct e1000_hw *hw)
{
	u16 checksum = 0;
	u16 i, eeprom_data;

	for (i = 0; i < EEPROM_CHECKSUM_REG; i++) {
		if (e1000_read_eeprom(hw, i, 1, &eeprom_data) < 0) {
			e_dbg("EEPROM Read Error\n");
			return -E1000_ERR_EEPROM;
		}
		checksum += eeprom_data;
	}
	checksum = (u16)EEPROM_SUM - checksum;
	if (e1000_write_eeprom(hw, EEPROM_CHECKSUM_REG, 1, &checksum) < 0) {
		e_dbg("EEPROM Write Error\n");
		return -E1000_ERR_EEPROM;
	}
	return E1000_SUCCESS;
}

/**
 * e1000_write_eeprom - write words to the different EEPROM types.
 * @hw: Struct containing variables accessed by shared code
 * @offset: offset within the EEPROM to be written to
 * @words: number of words to write
 * @data: 16 bit word to be written to the EEPROM
 *
 * If e1000_update_eeprom_checksum is not called after this function, the
 * EEPROM will most likely contain an invalid checksum.
 */
s32 e1000_write_eeprom(struct e1000_hw *hw, u16 offset, u16 words, u16 *data)
{
	s32 ret;

	mutex_lock(&e1000_eeprom_lock);
	ret = e1000_do_write_eeprom(hw, offset, words, data);
	mutex_unlock(&e1000_eeprom_lock);
	return ret;
}

static s32 e1000_do_write_eeprom(struct e1000_hw *hw, u16 offset, u16 words,
				 u16 *data)
{
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
	s32 status = 0;


	/* A check for invalid values:  offset too large, too many words, and
	 * not enough words.
	 */
	if ((offset >= eeprom->word_size) ||
	    (words > eeprom->word_size - offset) ||
	    (words == 0)) {
		e_dbg("\"words\" parameter out of bounds\n");
		return -E1000_ERR_EEPROM;
	}

	/* Prepare the EEPROM for writing  */
	if (e1000_acquire_eeprom(hw) != E1000_SUCCESS)
		return -E1000_ERR_EEPROM;

	status = e1000_write_eeprom_microwire(hw, offset, words, data);


	/* Done with writing */
	e1000_release_eeprom(hw);

	return status;
}


/**
 * e1000_write_eeprom_microwire - Writes a 16 bit word to a given offset in a Microwire EEPROM.
 * @hw: Struct containing variables accessed by shared code
 * @offset: offset within the EEPROM to be written to
 * @words: number of words to write
 * @data: pointer to array of 8 bit words to be written to the EEPROM
 */
static s32 e1000_write_eeprom_microwire(struct e1000_hw *hw, u16 offset,
					u16 words, u16 *data)
{
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
	u32 eecd;
	u16 words_written = 0;
	u16 i = 0;

	/* Send the write enable command to the EEPROM (3-bit opcode plus
	 * 6/8-bit dummy address beginning with 11).  It's less work to include
	 * the 11 of the dummy address as part of the opcode than it is to shift
	 * it over the correct number of bits for the address.  This puts the
	 * EEPROM into write/erase mode.
	 */
	e1000_shift_out_ee_bits(hw, EEPROM_EWEN_OPCODE_MICROWIRE,
				(u16)(eeprom->opcode_bits + 2));

	e1000_shift_out_ee_bits(hw, 0, (u16)(eeprom->address_bits - 2));

	/* Prepare the EEPROM */
	e1000_standby_eeprom(hw);

	while (words_written < words) {
		/* Send the Write command (3-bit opcode + addr) */
		e1000_shift_out_ee_bits(hw, EEPROM_WRITE_OPCODE_MICROWIRE,
					eeprom->opcode_bits);

		e1000_shift_out_ee_bits(hw, (u16)(offset + words_written),
					eeprom->address_bits);

		/* Send the data */
		e1000_shift_out_ee_bits(hw, data[words_written], 16);

		/* Toggle the CS line.  This in effect tells the EEPROM to
		 * execute the previous command.
		 */
		e1000_standby_eeprom(hw);

		/* Read DO repeatedly until it is high (equal to '1').  The
		 * EEPROM will signal that the command has been completed by
		 * raising the DO signal. If DO does not go high in 10
		 * milliseconds, then error out.
		 */
		for (i = 0; i < 200; i++) {
			eecd = er32(EECD);
			if (eecd & E1000_EECD_DO)
				break;
			udelay(50);
		}
		if (i == 200) {
			e_dbg("EEPROM Write did not complete\n");
			return -E1000_ERR_EEPROM;
		}

		/* Recover from write */
		e1000_standby_eeprom(hw);
		cond_resched();

		words_written++;
	}

	/* Send the write disable command to the EEPROM (3-bit opcode plus
	 * 6/8-bit dummy address beginning with 10).  It's less work to include
	 * the 10 of the dummy address as part of the opcode than it is to shift
	 * it over the correct number of bits for the address.  This takes the
	 * EEPROM out of write/erase mode.
	 */
	e1000_shift_out_ee_bits(hw, EEPROM_EWDS_OPCODE_MICROWIRE,
				(u16)(eeprom->opcode_bits + 2));

	e1000_shift_out_ee_bits(hw, 0, (u16)(eeprom->address_bits - 2));

	return E1000_SUCCESS;
}

/**
 * e1000_read_mac_addr - read the adapters MAC from eeprom
 * @hw: Struct containing variables accessed by shared code
 *
 * Reads the adapter's MAC address from the EEPROM and inverts the LSB for the
 * second function of dual function devices
 */
s32 e1000_read_mac_addr(struct e1000_hw *hw)
{
	u16 offset;
	u16 eeprom_data, i;

	for (i = 0; i < NODE_ADDRESS_SIZE; i += 2) {
		offset = i >> 1;
		if (e1000_read_eeprom(hw, offset, 1, &eeprom_data) < 0) {
			e_dbg("EEPROM Read Error\n");
			return -E1000_ERR_EEPROM;
		}
		hw->perm_mac_addr[i] = (u8)(eeprom_data & 0x00FF);
		hw->perm_mac_addr[i + 1] = (u8)(eeprom_data >> 8);
	}

	for (i = 0; i < NODE_ADDRESS_SIZE; i++)
		hw->mac_addr[i] = hw->perm_mac_addr[i];
	return E1000_SUCCESS;
}

/**
 * e1000_init_rx_addrs - Initializes receive address filters.
 * @hw: Struct containing variables accessed by shared code
 *
 * Places the MAC address in receive address register 0 and clears the rest
 * of the receive address registers. Clears the multicast table. Assumes
 * the receiver is in reset when the routine is called.
 */
static void e1000_init_rx_addrs(struct e1000_hw *hw)
{
	u32 i;
	u32 rar_num;

	/* Setup the receive address. */
	e_dbg("Programming MAC Address into RAR[0]\n");

	e1000_rar_set(hw, hw->mac_addr, 0);

	rar_num = E1000_RAR_ENTRIES;

	/* Zero out the following 14 receive addresses. RAR[15] is for
	 * manageability
	 */
	e_dbg("Clearing RAR[1-14]\n");
	for (i = 1; i < rar_num; i++) {
		E1000_WRITE_REG_ARRAY(hw, RA, (i << 1), 0);
		E1000_WRITE_FLUSH();
		E1000_WRITE_REG_ARRAY(hw, RA, ((i << 1) + 1), 0);
		E1000_WRITE_FLUSH();
	}
}

/**
 * e1000_hash_mc_addr - Hashes an address to determine its location in the multicast table
 * @hw: Struct containing variables accessed by shared code
 * @mc_addr: the multicast address to hash
 */
u32 e1000_hash_mc_addr(struct e1000_hw *hw, u8 *mc_addr)
{
	u32 hash_value = 0;

	/* The portion of the address that is used for the hash table is
	 * determined by the mc_filter_type setting.
	 */
	switch (hw->mc_filter_type) {
		/* [0] [1] [2] [3] [4] [5]
		 * 01  AA  00  12  34  56
		 * LSB                 MSB
		 */
	case 0:
		/* [47:36] i.e. 0x563 for above example address */
		hash_value = ((mc_addr[4] >> 4) | (((u16)mc_addr[5]) << 4));
		break;
	case 1:
		/* [46:35] i.e. 0xAC6 for above example address */
		hash_value = ((mc_addr[4] >> 3) | (((u16)mc_addr[5]) << 5));
		break;
	case 2:
		/* [45:34] i.e. 0x5D8 for above example address */
		hash_value = ((mc_addr[4] >> 2) | (((u16)mc_addr[5]) << 6));
		break;
	case 3:
		/* [43:32] i.e. 0x634 for above example address */
		hash_value = ((mc_addr[4]) | (((u16)mc_addr[5]) << 8));
		break;
	}

	hash_value &= 0xFFF;
	return hash_value;
}

/**
 * e1000_rar_set - Puts an ethernet address into a receive address register.
 * @hw: Struct containing variables accessed by shared code
 * @addr: Address to put into receive address register
 * @index: Receive address register to write
 */
void e1000_rar_set(struct e1000_hw *hw, u8 *addr, u32 index)
{
	u32 rar_low, rar_high;

	/* HW expects these in little endian so we reverse the byte order
	 * from network order (big endian) to little endian
	 */
	rar_low = ((u32)addr[0] | ((u32)addr[1] << 8) |
		   ((u32)addr[2] << 16) | ((u32)addr[3] << 24));
	rar_high = ((u32)addr[4] | ((u32)addr[5] << 8));

	/* Disable Rx and flush all Rx frames before enabling RSS to avoid Rx
	 * unit hang.
	 *
	 * Description:
	 * If there are any Rx frames queued up or otherwise present in the HW
	 * before RSS is enabled, and then we enable RSS, the HW Rx unit will
	 * hang.  To work around this issue, we have to disable receives and
	 * flush out all Rx frames before we enable RSS. To do so, we modify we
	 * redirect all Rx traffic to manageability and then reset the HW.
	 * This flushes away Rx frames, and (since the redirections to
	 * manageability persists across resets) keeps new ones from coming in
	 * while we work.  Then, we clear the Address Valid AV bit for all MAC
	 * addresses and undo the re-direction to manageability.
	 * Now, frames are coming in again, but the MAC won't accept them, so
	 * far so good.  We now proceed to initialize RSS (if necessary) and
	 * configure the Rx unit.  Last, we re-enable the AV bits and continue
	 * on our merry way.
	 */
	switch (hw->mac_type) {
	default:
		/* Indicate to hardware the Address is Valid. */
		rar_high |= E1000_RAH_AV;
		break;
	}

	E1000_WRITE_REG_ARRAY(hw, RA, (index << 1), rar_low);
	E1000_WRITE_FLUSH();
	E1000_WRITE_REG_ARRAY(hw, RA, ((index << 1) + 1), rar_high);
	E1000_WRITE_FLUSH();
}

/**
 * e1000_write_vfta - Writes a value to the specified offset in the VLAN filter table.
 * @hw: Struct containing variables accessed by shared code
 * @offset: Offset in VLAN filer table to write
 * @value: Value to write into VLAN filter table
 */
void e1000_write_vfta(struct e1000_hw *hw, u32 offset, u32 value)
{
	E1000_WRITE_REG_ARRAY(hw, VFTA, offset, value);
	E1000_WRITE_FLUSH();
}





/**
 * e1000_write_reg_io
 * @hw: Struct containing variables accessed by shared code
 * @offset: offset to write to
 * @value: value to write
 *
 * Writes a value to one of the devices registers using port I/O (as opposed to
 * memory mapped I/O). Only 82544 and newer devices support port I/O.
 */
static void e1000_write_reg_io(struct e1000_hw *hw, u32 offset, u32 value)
{
	unsigned long io_addr = hw->io_base;
	unsigned long io_data = hw->io_base + 4;

	e1000_io_write(hw, io_addr, offset);
	e1000_io_write(hw, io_data, value);
}


