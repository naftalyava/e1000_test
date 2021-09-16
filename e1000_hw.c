// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 1999 - 2006 Intel Corporation. */

/* e1000_hw.c
 * Shared functions for accessing and configuring the MAC
 */

#include "e1000.h"

static s32 e1000_check_downshift(struct e1000_hw *hw);
static void e1000_clear_hw_cntrs(struct e1000_hw *hw);
static void e1000_clear_vfta(struct e1000_hw *hw);
static s32 e1000_config_dsp_after_link_change(struct e1000_hw *hw,
					      bool link_up);
static s32 e1000_config_fc_after_link_up(struct e1000_hw *hw);
static s32 e1000_detect_gig_phy(struct e1000_hw *hw);
static s32 e1000_get_auto_rd_done(struct e1000_hw *hw);
static s32 e1000_get_phy_cfg_done(struct e1000_hw *hw);
static void e1000_init_rx_addrs(struct e1000_hw *hw);
static s32 e1000_set_d3_lplu_state(struct e1000_hw *hw, bool active);
static s32 e1000_wait_autoneg(struct e1000_hw *hw);
static void e1000_write_reg_io(struct e1000_hw *hw, u32 offset, u32 value);
static s32 e1000_set_phy_type(struct e1000_hw *hw);
static void e1000_phy_init_script(struct e1000_hw *hw);
static s32 e1000_setup_copper_link(struct e1000_hw *hw);
static s32 e1000_phy_force_speed_duplex(struct e1000_hw *hw);
static s32 e1000_phy_reset_dsp(struct e1000_hw *hw);
static s32 e1000_write_eeprom_spi(struct e1000_hw *hw, u16 offset,
				  u16 words, u16 *data);
static s32 e1000_write_eeprom_microwire(struct e1000_hw *hw, u16 offset,
					u16 words, u16 *data);
static s32 e1000_spi_eeprom_ready(struct e1000_hw *hw);
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
static s32 e1000_set_phy_mode(struct e1000_hw *hw);
static s32 e1000_do_read_eeprom(struct e1000_hw *hw, u16 offset, u16 words,
				u16 *data);
static s32 e1000_do_write_eeprom(struct e1000_hw *hw, u16 offset, u16 words,
				 u16 *data);

/* IGP cable length table */
static const
u16 e1000_igp_cable_length_table[IGP01E1000_AGC_LENGTH_TABLE_SIZE] = {
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
	5, 10, 10, 10, 10, 10, 10, 10, 20, 20, 20, 20, 20, 25, 25, 25,
	25, 25, 25, 25, 30, 30, 30, 30, 40, 40, 40, 40, 40, 40, 40, 40,
	40, 50, 50, 50, 50, 50, 50, 50, 60, 60, 60, 60, 60, 60, 60, 60,
	60, 70, 70, 70, 70, 70, 70, 80, 80, 80, 80, 80, 80, 90, 90, 90,
	90, 90, 90, 90, 90, 90, 100, 100, 100, 100, 100, 100, 100, 100, 100,
	    100,
	100, 100, 100, 100, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110,
	    110, 110,
	110, 110, 110, 110, 110, 110, 120, 120, 120, 120, 120, 120, 120, 120,
	    120, 120
};

static DEFINE_MUTEX(e1000_eeprom_lock);
static DEFINE_SPINLOCK(e1000_phy_lock);

/**
 * e1000_set_phy_type - Set the phy type member in the hw struct.
 * @hw: Struct containing variables accessed by shared code
 */
static s32 e1000_set_phy_type(struct e1000_hw *hw)
{
	if (hw->mac_type == e1000_undefined)
		return -E1000_ERR_PHY_TYPE;

	switch (hw->phy_id) {
	case M88E1000_E_PHY_ID:
	case M88E1000_I_PHY_ID:
	case M88E1011_I_PHY_ID:
	case M88E1111_I_PHY_ID:
	case M88E1118_E_PHY_ID:
		hw->phy_type = e1000_phy_m88;
		break;
	case IGP01E1000_I_PHY_ID:
		break;
	case RTL8211B_PHY_ID:
		hw->phy_type = e1000_phy_8211;
		break;
	case RTL8201N_PHY_ID:
		hw->phy_type = e1000_phy_8201;
		break;
	default:
		/* Should never have loaded on this device */
		hw->phy_type = e1000_phy_undefined;
		return -E1000_ERR_PHY_TYPE;
	}

	return E1000_SUCCESS;
}

/**
 * e1000_phy_init_script - IGP phy init script - initializes the GbE PHY
 * @hw: Struct containing variables accessed by shared code
 */
static void e1000_phy_init_script(struct e1000_hw *hw)
{
	u16 phy_saved_data;

	if (hw->phy_init_script) {
		msleep(20);

		/* Save off the current value of register 0x2F5B to be restored
		 * at the end of this routine.
		 */
		e1000_read_phy_reg(hw, 0x2F5B, &phy_saved_data);

		/* Disabled the PHY transmitter */
		e1000_write_phy_reg(hw, 0x2F5B, 0x0003);
		msleep(20);

		e1000_write_phy_reg(hw, 0x0000, 0x0140);
		msleep(5);

		e1000_write_phy_reg(hw, 0x0000, 0x3300);
		msleep(20);

		/* Now enable the transmitter */
		e1000_write_phy_reg(hw, 0x2F5B, phy_saved_data);

	}
}

/**
 * e1000_set_mac_type - Set the mac type member in the hw struct.
 * @hw: Struct containing variables accessed by shared code
 */
s32 e1000_set_mac_type(struct e1000_hw *hw)
{
	switch (hw->device_id) {
	case E1000_DEV_ID_82545EM_COPPER:
		hw->mac_type = e1000_82545;
		break;
	default:
		/* Should never have loaded on this device */
		return -E1000_ERR_MAC_TYPE;
	}

	return E1000_SUCCESS;
}

/**
 * e1000_set_media_type - Set media type and TBI compatibility.
 * @hw: Struct containing variables accessed by shared code
 */
void e1000_set_media_type(struct e1000_hw *hw)
{
	u32 status;

	status = er32(STATUS);
	if (status & E1000_STATUS_TBIMODE) {
		hw->media_type = e1000_media_type_fiber;
		/* tbi_compatibility not valid on fiber */
		hw->tbi_compatibility_en = false;
	} else {
		hw->media_type = e1000_media_type_copper;
	}
}

/**
 * e1000_reset_hw - reset the hardware completely
 * @hw: Struct containing variables accessed by shared code
 *
 * Reset the transmit and receive units; mask and clear all interrupts.
 */
s32 e1000_reset_hw(struct e1000_hw *hw)
{
	u32 ctrl;
	s32 ret_val;


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

	/* The tbi_compatibility_on Flag must be cleared when Rctl is cleared. */
	hw->tbi_compatibility_on = false;

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

	/* After MAC reset, force reload of EEPROM to restore power-on settings
	 * to device.  Later controllers reload the EEPROM automatically, so
	 * just wait for reload to complete.
	 */
	/* Auto read done will delay 5ms or poll based on mac type */
	ret_val = e1000_get_auto_rd_done(hw);
	if (ret_val)
		return ret_val;


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
	u32 i;
	s32 ret_val = 0;
	u32 mta_size;


	/* Set the media type and TBI compatibility */
	e1000_set_media_type(hw);

	/* Disabling VLAN filtering. */
	e_dbg("Initializing the IEEE VLAN\n");
	if (hw->mac_type < e1000_82545_rev_3)
		ew32(VET, 0);
	e1000_clear_vfta(hw);

	/* Setup the receive address. This involves initializing all of the
	 * Receive Address Registers (RARs 0 - 15).
	 */
	e1000_init_rx_addrs(hw);


	/* Zero out the Multicast HASH table */
	e_dbg("Zeroing the MTA\n");
	mta_size = E1000_MC_TBL_SIZE;
	for (i = 0; i < mta_size; i++) {
		E1000_WRITE_REG_ARRAY(hw, MTA, i, 0);
		/* use write flush to prevent Memory Write Block (MWB) from
		 * occurring when accessing our register space
		 */
		E1000_WRITE_FLUSH();
	}


	/* Call a subroutine to configure the link and setup flow control. */
	//ret_val = e1000_setup_link(hw);


	/* Clear all of the statistics registers (clear on read).  It is
	 * important that we do this after we have tried to establish link
	 * because the symbol error count will increment wildly if there
	 * is no link.
	 */
	e1000_clear_hw_cntrs(hw);

	return ret_val;
}


/**
 * e1000_setup_link - Configures flow control and link settings.
 * @hw: Struct containing variables accessed by shared code
 *
 * Determines which flow control settings to use. Calls the appropriate media-
 * specific link configuration function. Configures the flow control settings.
 * Assuming the adapter has a valid link partner, a valid link should be
 * established. Assumes the hardware has previously been reset and the
 * transmitter and receiver are not enabled.
 */
s32 e1000_setup_link(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 eeprom_data;

	/* Read and store word 0x0F of the EEPROM. This word contains bits
	 * that determine the hardware's default PAUSE (flow control) mode,
	 * a bit that determines whether the HW defaults to enabling or
	 * disabling auto-negotiation, and the direction of the
	 * SW defined pins. If there is no SW over-ride of the flow
	 * control setting, then the variable hw->fc will
	 * be initialized based on a value in the EEPROM.
	 */
	if (hw->fc == E1000_FC_DEFAULT) {
		ret_val = e1000_read_eeprom(hw, EEPROM_INIT_CONTROL2_REG,
					    1, &eeprom_data);
		if (ret_val) {
			e_dbg("EEPROM Read Error\n");
			return -E1000_ERR_EEPROM;
		}
		if ((eeprom_data & EEPROM_WORD0F_PAUSE_MASK) == 0)
			hw->fc = E1000_FC_NONE;
		else if ((eeprom_data & EEPROM_WORD0F_PAUSE_MASK) ==
			 EEPROM_WORD0F_ASM_DIR)
			hw->fc = E1000_FC_TX_PAUSE;
		else
			hw->fc = E1000_FC_FULL;
	}


	hw->original_fc = hw->fc;

	e_dbg("After fix-ups FlowControl is now = %x\n", hw->fc);



	/* Call the necessary subroutine to configure the link. */
	// ret_val = (hw->media_type == e1000_media_type_copper) ?
	//     e1000_setup_copper_link(hw) : e1000_setup_fiber_serdes_link(hw);

	ret_val = e1000_setup_copper_link(hw);

	/* Initialize the flow control address, type, and PAUSE timer
	 * registers to their default values.  This is done even if flow
	 * control is disabled, because it does not hurt anything to
	 * initialize these registers.
	 */
	e_dbg("Initializing the Flow Control address, type and timer regs\n");

	ew32(FCT, FLOW_CONTROL_TYPE);
	ew32(FCAH, FLOW_CONTROL_ADDRESS_HIGH);
	ew32(FCAL, FLOW_CONTROL_ADDRESS_LOW);

	ew32(FCTTV, hw->fc_pause_time);

	/* Set the flow control receive threshold registers.  Normally,
	 * these registers will be set to a default threshold that may be
	 * adjusted later by the driver's runtime code.  However, if the
	 * ability to transmit pause frames in not enabled, then these
	 * registers will be set to 0.
	 */
	if (!(hw->fc & E1000_FC_TX_PAUSE)) {
		ew32(FCRTL, 0);
		ew32(FCRTH, 0);
	} else {
		/* We need to set up the Receive Threshold high and low water
		 * marks as well as (optionally) enabling the transmission of
		 * XON frames.
		 */
		if (hw->fc_send_xon) {
			ew32(FCRTL, (hw->fc_low_water | E1000_FCRTL_XONE));
			ew32(FCRTH, hw->fc_high_water);
		} else {
			ew32(FCRTL, hw->fc_low_water);
			ew32(FCRTH, hw->fc_high_water);
		}
	}
	return ret_val;
}



/**
 * e1000_copper_link_rtl_setup - Copper link setup for e1000_phy_rtl series.
 * @hw: Struct containing variables accessed by shared code
 *
 * Commits changes to PHY configuration by calling e1000_phy_reset().
 */
static s32 e1000_copper_link_rtl_setup(struct e1000_hw *hw)
{
	s32 ret_val;

	/* SW reset the PHY so all changes take effect */
	ret_val = e1000_phy_reset(hw);
	if (ret_val) {
		e_dbg("Error Resetting the PHY\n");
		return ret_val;
	}

	return E1000_SUCCESS;
}

static s32 gbe_dhg_phy_setup(struct e1000_hw *hw)
{
	s32 ret_val;
	u32 ctrl_aux;

	switch (hw->phy_type) {
	case e1000_phy_8211:
		ret_val = e1000_copper_link_rtl_setup(hw);
		if (ret_val) {
			e_dbg("e1000_copper_link_rtl_setup failed!\n");
			return ret_val;
		}
		break;
	case e1000_phy_8201:
		/* Set RMII mode */
		ctrl_aux = er32(CTL_AUX);
		ctrl_aux |= E1000_CTL_AUX_RMII;
		ew32(CTL_AUX, ctrl_aux);
		E1000_WRITE_FLUSH();

		/* Disable the J/K bits required for receive */
		ctrl_aux = er32(CTL_AUX);
		ctrl_aux |= 0x4;
		ctrl_aux &= ~0x2;
		ew32(CTL_AUX, ctrl_aux);
		E1000_WRITE_FLUSH();
		ret_val = e1000_copper_link_rtl_setup(hw);

		if (ret_val) {
			e_dbg("e1000_copper_link_rtl_setup failed!\n");
			return ret_val;
		}
		break;
	default:
		e_dbg("Error Resetting the PHY\n");
		return E1000_ERR_PHY_TYPE;
	}

	return E1000_SUCCESS;
}

/**
 * e1000_copper_link_preconfig - early configuration for copper
 * @hw: Struct containing variables accessed by shared code
 *
 * Make sure we have a valid PHY and change PHY mode before link setup.
 */
static s32 e1000_copper_link_preconfig(struct e1000_hw *hw)
{
	u32 ctrl;
	s32 ret_val;
	u16 phy_data;

	ctrl = er32(CTRL);
	/* With 82543, we need to force speed and duplex on the MAC equal to
	 * what the PHY speed and duplex configuration is. In addition, we need
	 * to perform a hardware reset on the PHY to take it out of reset.
	 */
	ctrl |= E1000_CTRL_SLU;
	ctrl &= ~(E1000_CTRL_FRCSPD | E1000_CTRL_FRCDPX);
	ew32(CTRL, ctrl);

	/* Make sure we have a valid PHY */
	ret_val = e1000_detect_gig_phy(hw);
	if (ret_val) {
		e_dbg("Error, did not detect valid phy.\n");
		return ret_val;
	}
	e_dbg("Phy ID = %x\n", hw->phy_id);

	/* Set PHY to class A mode (if necessary) */
	ret_val = e1000_set_phy_mode(hw);
	if (ret_val)
		return ret_val;

	if (hw->mac_type == e1000_82545_rev_3) {
		ret_val =
		    e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_data);
		phy_data |= 0x00000008;
		ret_val =
		    e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, phy_data);
	}

	return E1000_SUCCESS;
}

/**
 * e1000_copper_link_igp_setup - Copper link setup for e1000_phy_igp series.
 * @hw: Struct containing variables accessed by shared code
 */
static s32 e1000_copper_link_igp_setup(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 phy_data;

	if (hw->phy_reset_disable)
		return E1000_SUCCESS;

	ret_val = e1000_phy_reset(hw);
	if (ret_val) {
		e_dbg("Error Resetting the PHY\n");
		return ret_val;
	}

	/* Wait 15ms for MAC to configure PHY from eeprom settings */
	msleep(15);

	/* The NVM settings will configure LPLU in D3 for IGP2 and IGP3 PHYs */
	if (hw->phy_type == e1000_phy_igp) {
		/* disable lplu d3 during driver init */
		ret_val = e1000_set_d3_lplu_state(hw, false);
		if (ret_val) {
			e_dbg("Error Disabling LPLU D3\n");
			return ret_val;
		}
	}

	/* Configure mdi-mdix settings */
	ret_val = e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_CTRL, &phy_data);
	if (ret_val)
		return ret_val;


	hw->dsp_config_state = e1000_dsp_config_enabled;
	phy_data &= ~IGP01E1000_PSCR_AUTO_MDIX;

	switch (hw->mdix) {
		case 1:
			phy_data &= ~IGP01E1000_PSCR_FORCE_MDI_MDIX;
			break;
		case 2:
			phy_data |= IGP01E1000_PSCR_FORCE_MDI_MDIX;
			break;
		case 0:
		default:
			phy_data |= IGP01E1000_PSCR_AUTO_MDIX;
			break;
	}
	
	ret_val = e1000_write_phy_reg(hw, IGP01E1000_PHY_PORT_CTRL, phy_data);
	if (ret_val)
		return ret_val;

	/* set auto-master slave resolution settings */
	if (hw->autoneg) {
		e1000_ms_type phy_ms_setting = hw->master_slave;

		if (hw->ffe_config_state == e1000_ffe_config_active)
			hw->ffe_config_state = e1000_ffe_config_enabled;

		if (hw->dsp_config_state == e1000_dsp_config_activated)
			hw->dsp_config_state = e1000_dsp_config_enabled;

		/* when autonegotiation advertisement is only 1000Mbps then we
		 * should disable SmartSpeed and enable Auto MasterSlave
		 * resolution as hardware default.
		 */
		if (hw->autoneg_advertised == ADVERTISE_1000_FULL) {
			/* Disable SmartSpeed */
			ret_val =
			    e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
					       &phy_data);
			if (ret_val)
				return ret_val;
			phy_data &= ~IGP01E1000_PSCFR_SMART_SPEED;
			ret_val =
			    e1000_write_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
						phy_data);
			if (ret_val)
				return ret_val;
			/* Set auto Master/Slave resolution process */
			ret_val =
			    e1000_read_phy_reg(hw, PHY_1000T_CTRL, &phy_data);
			if (ret_val)
				return ret_val;
			phy_data &= ~CR_1000T_MS_ENABLE;
			ret_val =
			    e1000_write_phy_reg(hw, PHY_1000T_CTRL, phy_data);
			if (ret_val)
				return ret_val;
		}

		ret_val = e1000_read_phy_reg(hw, PHY_1000T_CTRL, &phy_data);
		if (ret_val)
			return ret_val;

		/* load defaults for future use */
		hw->original_master_slave = (phy_data & CR_1000T_MS_ENABLE) ?
		    ((phy_data & CR_1000T_MS_VALUE) ?
		     e1000_ms_force_master :
		     e1000_ms_force_slave) : e1000_ms_auto;

		switch (phy_ms_setting) {
		case e1000_ms_force_master:
			phy_data |= (CR_1000T_MS_ENABLE | CR_1000T_MS_VALUE);
			break;
		case e1000_ms_force_slave:
			phy_data |= CR_1000T_MS_ENABLE;
			phy_data &= ~(CR_1000T_MS_VALUE);
			break;
		case e1000_ms_auto:
			phy_data &= ~CR_1000T_MS_ENABLE;
		default:
			break;
		}
		ret_val = e1000_write_phy_reg(hw, PHY_1000T_CTRL, phy_data);
		if (ret_val)
			return ret_val;
	}

	return E1000_SUCCESS;
}

/**
 * e1000_copper_link_mgp_setup - Copper link setup for e1000_phy_m88 series.
 * @hw: Struct containing variables accessed by shared code
 */
static s32 e1000_copper_link_mgp_setup(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 phy_data;

	if (hw->phy_reset_disable)
		return E1000_SUCCESS;

	/* Enable CRS on TX. This must be set for half-duplex operation. */
	ret_val = e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_data);
	if (ret_val)
		return ret_val;

	phy_data |= M88E1000_PSCR_ASSERT_CRS_ON_TX;

	/* Options:
	 *   MDI/MDI-X = 0 (default)
	 *   0 - Auto for all speeds
	 *   1 - MDI mode
	 *   2 - MDI-X mode
	 *   3 - Auto for 1000Base-T only (MDI-X for 10/100Base-T modes)
	 */
	phy_data &= ~M88E1000_PSCR_AUTO_X_MODE;

	switch (hw->mdix) {
	case 1:
		phy_data |= M88E1000_PSCR_MDI_MANUAL_MODE;
		break;
	case 2:
		phy_data |= M88E1000_PSCR_MDIX_MANUAL_MODE;
		break;
	case 3:
		phy_data |= M88E1000_PSCR_AUTO_X_1000T;
		break;
	case 0:
	default:
		phy_data |= M88E1000_PSCR_AUTO_X_MODE;
		break;
	}

	/* Options:
	 *   disable_polarity_correction = 0 (default)
	 *       Automatic Correction for Reversed Cable Polarity
	 *   0 - Disabled
	 *   1 - Enabled
	 */
	phy_data &= ~M88E1000_PSCR_POLARITY_REVERSAL;
	if (hw->disable_polarity_correction == 1)
		phy_data |= M88E1000_PSCR_POLARITY_REVERSAL;
	ret_val = e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, phy_data);
	if (ret_val)
		return ret_val;

	if (hw->phy_revision < M88E1011_I_REV_4) {
		/* Force TX_CLK in the Extended PHY Specific Control Register
		 * to 25MHz clock.
		 */
		ret_val =
		    e1000_read_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL,
				       &phy_data);
		if (ret_val)
			return ret_val;

		phy_data |= M88E1000_EPSCR_TX_CLK_25;

		if ((hw->phy_revision == E1000_REVISION_2) &&
		    (hw->phy_id == M88E1111_I_PHY_ID)) {
			/* Vidalia Phy, set the downshift counter to 5x */
			phy_data &= ~(M88EC018_EPSCR_DOWNSHIFT_COUNTER_MASK);
			phy_data |= M88EC018_EPSCR_DOWNSHIFT_COUNTER_5X;
			ret_val = e1000_write_phy_reg(hw,
						      M88E1000_EXT_PHY_SPEC_CTRL,
						      phy_data);
			if (ret_val)
				return ret_val;
		} else {
			/* Configure Master and Slave downshift values */
			phy_data &= ~(M88E1000_EPSCR_MASTER_DOWNSHIFT_MASK |
				      M88E1000_EPSCR_SLAVE_DOWNSHIFT_MASK);
			phy_data |= (M88E1000_EPSCR_MASTER_DOWNSHIFT_1X |
				     M88E1000_EPSCR_SLAVE_DOWNSHIFT_1X);
			ret_val = e1000_write_phy_reg(hw,
						      M88E1000_EXT_PHY_SPEC_CTRL,
						      phy_data);
			if (ret_val)
				return ret_val;
		}
	}

	/* SW Reset the PHY so all changes take effect */
	ret_val = e1000_phy_reset(hw);
	if (ret_val) {
		e_dbg("Error Resetting the PHY\n");
		return ret_val;
	}

	return E1000_SUCCESS;
}

/**
 * e1000_copper_link_autoneg - setup auto-neg
 * @hw: Struct containing variables accessed by shared code
 *
 * Setup auto-negotiation and flow control advertisements,
 * and then perform auto-negotiation.
 */
static s32 e1000_copper_link_autoneg(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 phy_data;

	/* Perform some bounds checking on the hw->autoneg_advertised
	 * parameter.  If this variable is zero, then set it to the default.
	 */
	hw->autoneg_advertised &= AUTONEG_ADVERTISE_SPEED_DEFAULT;

	/* If autoneg_advertised is zero, we assume it was not defaulted
	 * by the calling code so we set to advertise full capability.
	 */
	if (hw->autoneg_advertised == 0)
		hw->autoneg_advertised = AUTONEG_ADVERTISE_SPEED_DEFAULT;

	/* IFE/RTL8201N PHY only supports 10/100 */
	if (hw->phy_type == e1000_phy_8201)
		hw->autoneg_advertised &= AUTONEG_ADVERTISE_10_100_ALL;

	e_dbg("Reconfiguring auto-neg advertisement params\n");
	ret_val = e1000_phy_setup_autoneg(hw);
	if (ret_val) {
		e_dbg("Error Setting up Auto-Negotiation\n");
		return ret_val;
	}
	e_dbg("Restarting Auto-Neg\n");

	/* Restart auto-negotiation by setting the Auto Neg Enable bit and
	 * the Auto Neg Restart bit in the PHY control register.
	 */
	ret_val = e1000_read_phy_reg(hw, PHY_CTRL, &phy_data);
	if (ret_val)
		return ret_val;

	phy_data |= (MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG);
	ret_val = e1000_write_phy_reg(hw, PHY_CTRL, phy_data);
	if (ret_val)
		return ret_val;

	/* Does the user want to wait for Auto-Neg to complete here, or
	 * check at a later time (for example, callback routine).
	 */
	if (hw->wait_autoneg_complete) {
		ret_val = e1000_wait_autoneg(hw);
		if (ret_val) {
			e_dbg
			    ("Error while waiting for autoneg to complete\n");
			return ret_val;
		}
	}

	hw->get_link_status = true;

	return E1000_SUCCESS;
}

/**
 * e1000_copper_link_postconfig - post link setup
 * @hw: Struct containing variables accessed by shared code
 *
 * Config the MAC and the PHY after link is up.
 *   1) Set up the MAC to the current PHY speed/duplex
 *      if we are on 82543.  If we
 *      are on newer silicon, we only need to configure
 *      collision distance in the Transmit Control Register.
 *   2) Set up flow control on the MAC to that established with
 *      the link partner.
 *   3) Config DSP to improve Gigabit link quality for some PHY revisions.
 */
static s32 e1000_copper_link_postconfig(struct e1000_hw *hw)
{
	s32 ret_val;

	e1000_config_collision_dist(hw);
	
	
	ret_val = e1000_config_fc_after_link_up(hw);
	if (ret_val) {
		e_dbg("Error Configuring Flow Control\n");
		return ret_val;
	}

	/* Config DSP to improve Giga link quality */
	if (hw->phy_type == e1000_phy_igp) {
		ret_val = e1000_config_dsp_after_link_change(hw, true);
		if (ret_val) {
			e_dbg("Error Configuring DSP after link up\n");
			return ret_val;
		}
	}

	return E1000_SUCCESS;
}

/**
 * e1000_setup_copper_link - phy/speed/duplex setting
 * @hw: Struct containing variables accessed by shared code
 *
 * Detects which PHY is present and sets up the speed and duplex
 */
static s32 e1000_setup_copper_link(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 i;
	u16 phy_data;

	/* Check if it is a valid PHY and set PHY mode if necessary. */
	ret_val = e1000_copper_link_preconfig(hw);
	if (ret_val)
		return ret_val;

	if (hw->phy_type == e1000_phy_igp) {
		ret_val = e1000_copper_link_igp_setup(hw);
		if (ret_val)
			return ret_val;
	} else if (hw->phy_type == e1000_phy_m88) {
		ret_val = e1000_copper_link_mgp_setup(hw);
		if (ret_val)
			return ret_val;
	} else {
		ret_val = gbe_dhg_phy_setup(hw);
		if (ret_val) {
			e_dbg("gbe_dhg_phy_setup failed!\n");
			return ret_val;
		}
	}

	if (hw->autoneg) {
		/* Setup autoneg and flow control advertisement
		 * and perform autonegotiation
		 */
		ret_val = e1000_copper_link_autoneg(hw);
		if (ret_val)
			return ret_val;
	} else {
		/* PHY will be set to 10H, 10F, 100H,or 100F
		 * depending on value from forced_speed_duplex.
		 */
		e_dbg("Forcing speed and duplex\n");
		ret_val = e1000_phy_force_speed_duplex(hw);
		if (ret_val) {
			e_dbg("Error Forcing Speed and Duplex\n");
			return ret_val;
		}
	}

	/* Check link status. Wait up to 100 microseconds for link to become
	 * valid.
	 */
	for (i = 0; i < 10; i++) {
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &phy_data);
		if (ret_val)
			return ret_val;
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &phy_data);
		if (ret_val)
			return ret_val;

		if (phy_data & MII_SR_LINK_STATUS) {
			/* Config the MAC and PHY after link is up */
			ret_val = e1000_copper_link_postconfig(hw);
			if (ret_val)
				return ret_val;

			e_dbg("Valid link established!!!\n");
			return E1000_SUCCESS;
		}
		udelay(10);
	}

	e_dbg("Unable to establish link!!!\n");
	return E1000_SUCCESS;
}

/**
 * e1000_phy_setup_autoneg - phy settings
 * @hw: Struct containing variables accessed by shared code
 *
 * Configures PHY autoneg and flow control advertisement settings
 */
s32 e1000_phy_setup_autoneg(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 mii_autoneg_adv_reg;
	u16 mii_1000t_ctrl_reg;

	/* Read the MII Auto-Neg Advertisement Register (Address 4). */
	ret_val = e1000_read_phy_reg(hw, PHY_AUTONEG_ADV, &mii_autoneg_adv_reg);
	if (ret_val)
		return ret_val;

	/* Read the MII 1000Base-T Control Register (Address 9). */
	ret_val = e1000_read_phy_reg(hw, PHY_1000T_CTRL, &mii_1000t_ctrl_reg);
	if (ret_val)
		return ret_val;
	else if (hw->phy_type == e1000_phy_8201)
		mii_1000t_ctrl_reg &= ~REG9_SPEED_MASK;

	/* Need to parse both autoneg_advertised and fc and set up
	 * the appropriate PHY registers.  First we will parse for
	 * autoneg_advertised software override.  Since we can advertise
	 * a plethora of combinations, we need to check each bit
	 * individually.
	 */

	/* First we clear all the 10/100 mb speed bits in the Auto-Neg
	 * Advertisement Register (Address 4) and the 1000 mb speed bits in
	 * the  1000Base-T Control Register (Address 9).
	 */
	mii_autoneg_adv_reg &= ~REG4_SPEED_MASK;
	mii_1000t_ctrl_reg &= ~REG9_SPEED_MASK;

	e_dbg("autoneg_advertised %x\n", hw->autoneg_advertised);

	/* Do we want to advertise 10 Mb Half Duplex? */
	if (hw->autoneg_advertised & ADVERTISE_10_HALF) {
		e_dbg("Advertise 10mb Half duplex\n");
		mii_autoneg_adv_reg |= NWAY_AR_10T_HD_CAPS;
	}

	/* Do we want to advertise 10 Mb Full Duplex? */
	if (hw->autoneg_advertised & ADVERTISE_10_FULL) {
		e_dbg("Advertise 10mb Full duplex\n");
		mii_autoneg_adv_reg |= NWAY_AR_10T_FD_CAPS;
	}

	/* Do we want to advertise 100 Mb Half Duplex? */
	if (hw->autoneg_advertised & ADVERTISE_100_HALF) {
		e_dbg("Advertise 100mb Half duplex\n");
		mii_autoneg_adv_reg |= NWAY_AR_100TX_HD_CAPS;
	}

	/* Do we want to advertise 100 Mb Full Duplex? */
	if (hw->autoneg_advertised & ADVERTISE_100_FULL) {
		e_dbg("Advertise 100mb Full duplex\n");
		mii_autoneg_adv_reg |= NWAY_AR_100TX_FD_CAPS;
	}

	/* We do not allow the Phy to advertise 1000 Mb Half Duplex */
	if (hw->autoneg_advertised & ADVERTISE_1000_HALF) {
		e_dbg
		    ("Advertise 1000mb Half duplex requested, request denied!\n");
	}

	/* Do we want to advertise 1000 Mb Full Duplex? */
	if (hw->autoneg_advertised & ADVERTISE_1000_FULL) {
		e_dbg("Advertise 1000mb Full duplex\n");
		mii_1000t_ctrl_reg |= CR_1000T_FD_CAPS;
	}

	/* Check for a software override of the flow control settings, and
	 * setup the PHY advertisement registers accordingly.  If
	 * auto-negotiation is enabled, then software will have to set the
	 * "PAUSE" bits to the correct value in the Auto-Negotiation
	 * Advertisement Register (PHY_AUTONEG_ADV) and re-start
	 * auto-negotiation.
	 *
	 * The possible values of the "fc" parameter are:
	 *      0:  Flow control is completely disabled
	 *      1:  Rx flow control is enabled (we can receive pause frames
	 *          but not send pause frames).
	 *      2:  Tx flow control is enabled (we can send pause frames
	 *          but we do not support receiving pause frames).
	 *      3:  Both Rx and TX flow control (symmetric) are enabled.
	 *  other:  No software override.  The flow control configuration
	 *          in the EEPROM is used.
	 */
	switch (hw->fc) {
	case E1000_FC_NONE:	/* 0 */
		/* Flow control (RX & TX) is completely disabled by a
		 * software over-ride.
		 */
		mii_autoneg_adv_reg &= ~(NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);
		break;
	case E1000_FC_RX_PAUSE:	/* 1 */
		/* RX Flow control is enabled, and TX Flow control is
		 * disabled, by a software over-ride.
		 */
		/* Since there really isn't a way to advertise that we are
		 * capable of RX Pause ONLY, we will advertise that we
		 * support both symmetric and asymmetric RX PAUSE.  Later
		 * (in e1000_config_fc_after_link_up) we will disable the
		 * hw's ability to send PAUSE frames.
		 */
		mii_autoneg_adv_reg |= (NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);
		break;
	case E1000_FC_TX_PAUSE:	/* 2 */
		/* TX Flow control is enabled, and RX Flow control is
		 * disabled, by a software over-ride.
		 */
		mii_autoneg_adv_reg |= NWAY_AR_ASM_DIR;
		mii_autoneg_adv_reg &= ~NWAY_AR_PAUSE;
		break;
	case E1000_FC_FULL:	/* 3 */
		/* Flow control (both RX and TX) is enabled by a software
		 * over-ride.
		 */
		mii_autoneg_adv_reg |= (NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);
		break;
	default:
		e_dbg("Flow control param set incorrectly\n");
		return -E1000_ERR_CONFIG;
	}

	ret_val = e1000_write_phy_reg(hw, PHY_AUTONEG_ADV, mii_autoneg_adv_reg);
	if (ret_val)
		return ret_val;

	e_dbg("Auto-Neg Advertising %x\n", mii_autoneg_adv_reg);

	if (hw->phy_type == e1000_phy_8201) {
		mii_1000t_ctrl_reg = 0;
	} else {
		ret_val = e1000_write_phy_reg(hw, PHY_1000T_CTRL,
					      mii_1000t_ctrl_reg);
		if (ret_val)
			return ret_val;
	}

	return E1000_SUCCESS;
}

/**
 * e1000_phy_force_speed_duplex - force link settings
 * @hw: Struct containing variables accessed by shared code
 *
 * Force PHY speed and duplex settings to hw->forced_speed_duplex
 */
static s32 e1000_phy_force_speed_duplex(struct e1000_hw *hw)
{
	u32 ctrl;
	s32 ret_val;
	u16 mii_ctrl_reg;
	u16 mii_status_reg;
	u16 phy_data;
	u16 i;

	/* Turn off Flow control if we are forcing speed and duplex. */
	hw->fc = E1000_FC_NONE;

	e_dbg("hw->fc = %d\n", hw->fc);

	/* Read the Device Control Register. */
	ctrl = er32(CTRL);

	/* Set the bits to Force Speed and Duplex in the Device Ctrl Reg. */
	ctrl |= (E1000_CTRL_FRCSPD | E1000_CTRL_FRCDPX);
	ctrl &= ~(DEVICE_SPEED_MASK);

	/* Clear the Auto Speed Detect Enable bit. */
	ctrl &= ~E1000_CTRL_ASDE;

	/* Read the MII Control Register. */
	ret_val = e1000_read_phy_reg(hw, PHY_CTRL, &mii_ctrl_reg);
	if (ret_val)
		return ret_val;

	/* We need to disable autoneg in order to force link and duplex. */

	mii_ctrl_reg &= ~MII_CR_AUTO_NEG_EN;

	/* Are we forcing Full or Half Duplex? */
	if (hw->forced_speed_duplex == e1000_100_full ||
	    hw->forced_speed_duplex == e1000_10_full) {
		/* We want to force full duplex so we SET the full duplex bits
		 * in the Device and MII Control Registers.
		 */
		ctrl |= E1000_CTRL_FD;
		mii_ctrl_reg |= MII_CR_FULL_DUPLEX;
		e_dbg("Full Duplex\n");
	} else {
		/* We want to force half duplex so we CLEAR the full duplex bits
		 * in the Device and MII Control Registers.
		 */
		ctrl &= ~E1000_CTRL_FD;
		mii_ctrl_reg &= ~MII_CR_FULL_DUPLEX;
		e_dbg("Half Duplex\n");
	}

	/* Are we forcing 100Mbps??? */
	if (hw->forced_speed_duplex == e1000_100_full ||
	    hw->forced_speed_duplex == e1000_100_half) {
		/* Set the 100Mb bit and turn off the 1000Mb and 10Mb bits. */
		ctrl |= E1000_CTRL_SPD_100;
		mii_ctrl_reg |= MII_CR_SPEED_100;
		mii_ctrl_reg &= ~(MII_CR_SPEED_1000 | MII_CR_SPEED_10);
		e_dbg("Forcing 100mb ");
	} else {
		/* Set the 10Mb bit and turn off the 1000Mb and 100Mb bits. */
		ctrl &= ~(E1000_CTRL_SPD_1000 | E1000_CTRL_SPD_100);
		mii_ctrl_reg |= MII_CR_SPEED_10;
		mii_ctrl_reg &= ~(MII_CR_SPEED_1000 | MII_CR_SPEED_100);
		e_dbg("Forcing 10mb ");
	}

	e1000_config_collision_dist(hw);

	/* Write the configured values back to the Device Control Reg. */
	ew32(CTRL, ctrl);

	if (hw->phy_type == e1000_phy_m88) {
		ret_val =
		    e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_data);
		if (ret_val)
			return ret_val;

		/* Clear Auto-Crossover to force MDI manually. M88E1000 requires
		 * MDI forced whenever speed are duplex are forced.
		 */
		phy_data &= ~M88E1000_PSCR_AUTO_X_MODE;
		ret_val =
		    e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, phy_data);
		if (ret_val)
			return ret_val;

		e_dbg("M88E1000 PSCR: %x\n", phy_data);

		/* Need to reset the PHY or these changes will be ignored */
		mii_ctrl_reg |= MII_CR_RESET;

		/* Disable MDI-X support for 10/100 */
	} else {
		/* Clear Auto-Crossover to force MDI manually.  IGP requires MDI
		 * forced whenever speed or duplex are forced.
		 */
		ret_val =
		    e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_CTRL, &phy_data);
		if (ret_val)
			return ret_val;

		phy_data &= ~IGP01E1000_PSCR_AUTO_MDIX;
		phy_data &= ~IGP01E1000_PSCR_FORCE_MDI_MDIX;

		ret_val =
		    e1000_write_phy_reg(hw, IGP01E1000_PHY_PORT_CTRL, phy_data);
		if (ret_val)
			return ret_val;
	}

	/* Write back the modified PHY MII control register. */
	ret_val = e1000_write_phy_reg(hw, PHY_CTRL, mii_ctrl_reg);
	if (ret_val)
		return ret_val;

	udelay(1);

	/* The wait_autoneg_complete flag may be a little misleading here.
	 * Since we are forcing speed and duplex, Auto-Neg is not enabled.
	 * But we do want to delay for a period while forcing only so we
	 * don't generate false No Link messages.  So we will wait here
	 * only if the user has set wait_autoneg_complete to 1, which is
	 * the default.
	 */
	if (hw->wait_autoneg_complete) {
		/* We will wait for autoneg to complete. */
		e_dbg("Waiting for forced speed/duplex link.\n");
		mii_status_reg = 0;

		/* Wait for autoneg to complete or 4.5 seconds to expire */
		for (i = PHY_FORCE_TIME; i > 0; i--) {
			/* Read the MII Status Register and wait for Auto-Neg
			 * Complete bit to be set.
			 */
			ret_val =
			    e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg);
			if (ret_val)
				return ret_val;

			ret_val =
			    e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg);
			if (ret_val)
				return ret_val;

			if (mii_status_reg & MII_SR_LINK_STATUS)
				break;
			msleep(100);
		}
		if ((i == 0) && (hw->phy_type == e1000_phy_m88)) {
			/* We didn't get link.  Reset the DSP and wait again
			 * for link.
			 */
			ret_val = e1000_phy_reset_dsp(hw);
			if (ret_val) {
				e_dbg("Error Resetting PHY DSP\n");
				return ret_val;
			}
		}
		/* This loop will early-out if the link condition has been
		 * met
		 */
		for (i = PHY_FORCE_TIME; i > 0; i--) {
			if (mii_status_reg & MII_SR_LINK_STATUS)
				break;
			msleep(100);
			/* Read the MII Status Register and wait for Auto-Neg
			 * Complete bit to be set.
			 */
			ret_val =
			    e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg);
			if (ret_val)
				return ret_val;

			ret_val =
			    e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg);
			if (ret_val)
				return ret_val;
		}
	}

	if (hw->phy_type == e1000_phy_m88) {
		/* Because we reset the PHY above, we need to re-force TX_CLK in
		 * the Extended PHY Specific Control Register to 25MHz clock.
		 * This value defaults back to a 2.5MHz clock when the PHY is
		 * reset.
		 */
		ret_val =
		    e1000_read_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL,
				       &phy_data);
		if (ret_val)
			return ret_val;

		phy_data |= M88E1000_EPSCR_TX_CLK_25;
		ret_val =
		    e1000_write_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL,
					phy_data);
		if (ret_val)
			return ret_val;

		/* In addition, because of the s/w reset above, we need to
		 * enable CRS on Tx.  This must be set for both full and half
		 * duplex operation.
		 */
		ret_val =
		    e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_data);
		if (ret_val)
			return ret_val;

		phy_data |= M88E1000_PSCR_ASSERT_CRS_ON_TX;
		ret_val =
		    e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, phy_data);
		if (ret_val)
			return ret_val;

	}
	return E1000_SUCCESS;
}

/**
 * e1000_config_collision_dist - set collision distance register
 * @hw: Struct containing variables accessed by shared code
 *
 * Sets the collision distance in the Transmit Control register.
 * Link should have been established previously. Reads the speed and duplex
 * information from the Device Status register.
 */
void e1000_config_collision_dist(struct e1000_hw *hw)
{
	u32 tctl, coll_dist;


	coll_dist = E1000_COLLISION_DISTANCE;

	tctl = er32(TCTL);

	tctl &= ~E1000_TCTL_COLD;
	tctl |= coll_dist << E1000_COLD_SHIFT;

	ew32(TCTL, tctl);
	E1000_WRITE_FLUSH();
}



/**
 * e1000_force_mac_fc - force flow control settings
 * @hw: Struct containing variables accessed by shared code
 *
 * Forces the MAC's flow control settings.
 * Sets the TFCE and RFCE bits in the device control register to reflect
 * the adapter settings. TFCE and RFCE need to be explicitly set by
 * software when a Copper PHY is used because autonegotiation is managed
 * by the PHY rather than the MAC. Software must also configure these
 * bits when link is forced on a fiber connection.
 */
s32 e1000_force_mac_fc(struct e1000_hw *hw)
{
	u32 ctrl;

	/* Get the current configuration of the Device Control Register */
	ctrl = er32(CTRL);

	/* Because we didn't get link via the internal auto-negotiation
	 * mechanism (we either forced link or we got link via PHY
	 * auto-neg), we have to manually enable/disable transmit an
	 * receive flow control.
	 *
	 * The "Case" statement below enables/disable flow control
	 * according to the "hw->fc" parameter.
	 *
	 * The possible values of the "fc" parameter are:
	 *      0:  Flow control is completely disabled
	 *      1:  Rx flow control is enabled (we can receive pause
	 *          frames but not send pause frames).
	 *      2:  Tx flow control is enabled (we can send pause frames
	 *          frames but we do not receive pause frames).
	 *      3:  Both Rx and TX flow control (symmetric) is enabled.
	 *  other:  No other values should be possible at this point.
	 */

	switch (hw->fc) {
	case E1000_FC_NONE:
		ctrl &= (~(E1000_CTRL_TFCE | E1000_CTRL_RFCE));
		break;
	case E1000_FC_RX_PAUSE:
		ctrl &= (~E1000_CTRL_TFCE);
		ctrl |= E1000_CTRL_RFCE;
		break;
	case E1000_FC_TX_PAUSE:
		ctrl &= (~E1000_CTRL_RFCE);
		ctrl |= E1000_CTRL_TFCE;
		break;
	case E1000_FC_FULL:
		ctrl |= (E1000_CTRL_TFCE | E1000_CTRL_RFCE);
		break;
	default:
		e_dbg("Flow control param set incorrectly\n");
		return -E1000_ERR_CONFIG;
	}


	ew32(CTRL, ctrl);
	return E1000_SUCCESS;
}

/**
 * e1000_config_fc_after_link_up - configure flow control after autoneg
 * @hw: Struct containing variables accessed by shared code
 *
 * Configures flow control settings after link is established
 * Should be called immediately after a valid link has been established.
 * Forces MAC flow control settings if link was forced. When in MII/GMII mode
 * and autonegotiation is enabled, the MAC flow control settings will be set
 * based on the flow control negotiated by the PHY. In TBI mode, the TFCE
 * and RFCE bits will be automatically set to the negotiated flow control mode.
 */
static s32 e1000_config_fc_after_link_up(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 mii_status_reg;
	u16 mii_nway_adv_reg;
	u16 mii_nway_lp_ability_reg;
	u16 speed;
	u16 duplex;

	/* Check for the case where we have fiber media and auto-neg failed
	 * so we had to force link.  In this case, we need to force the
	 * configuration of the MAC to match the "fc" parameter.
	 */
	if (((hw->media_type == e1000_media_type_fiber) &&
	     (hw->autoneg_failed)) ||
	    ((hw->media_type == e1000_media_type_internal_serdes) &&
	     (hw->autoneg_failed)) ||
	    ((hw->media_type == e1000_media_type_copper) &&
	     (!hw->autoneg))) {
		ret_val = e1000_force_mac_fc(hw);
		if (ret_val) {
			e_dbg("Error forcing flow control settings\n");
			return ret_val;
		}
	}

	/* Check for the case where we have copper media and auto-neg is
	 * enabled.  In this case, we need to check and see if Auto-Neg
	 * has completed, and if so, how the PHY and link partner has
	 * flow control configured.
	 */
	if ((hw->media_type == e1000_media_type_copper) && hw->autoneg) {
		/* Read the MII Status Register and check to see if AutoNeg
		 * has completed.  We read this twice because this reg has
		 * some "sticky" (latched) bits.
		 */
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg);
		if (ret_val)
			return ret_val;
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg);
		if (ret_val)
			return ret_val;

		if (mii_status_reg & MII_SR_AUTONEG_COMPLETE) {
			/* The AutoNeg process has completed, so we now need to
			 * read both the Auto Negotiation Advertisement Register
			 * (Address 4) and the Auto_Negotiation Base Page
			 * Ability Register (Address 5) to determine how flow
			 * control was negotiated.
			 */
			ret_val = e1000_read_phy_reg(hw, PHY_AUTONEG_ADV,
						     &mii_nway_adv_reg);
			if (ret_val)
				return ret_val;
			ret_val = e1000_read_phy_reg(hw, PHY_LP_ABILITY,
						     &mii_nway_lp_ability_reg);
			if (ret_val)
				return ret_val;

			/* Two bits in the Auto Negotiation Advertisement
			 * Register (Address 4) and two bits in the Auto
			 * Negotiation Base Page Ability Register (Address 5)
			 * determine flow control for both the PHY and the link
			 * partner.  The following table, taken out of the IEEE
			 * 802.3ab/D6.0 dated March 25, 1999, describes these
			 * PAUSE resolution bits and how flow control is
			 * determined based upon these settings.
			 * NOTE:  DC = Don't Care
			 *
			 *   LOCAL DEVICE  |   LINK PARTNER
			 * PAUSE | ASM_DIR | PAUSE | ASM_DIR | NIC Resolution
			 *-------|---------|-------|---------|------------------
			 *   0   |    0    |  DC   |   DC    | E1000_FC_NONE
			 *   0   |    1    |   0   |   DC    | E1000_FC_NONE
			 *   0   |    1    |   1   |    0    | E1000_FC_NONE
			 *   0   |    1    |   1   |    1    | E1000_FC_TX_PAUSE
			 *   1   |    0    |   0   |   DC    | E1000_FC_NONE
			 *   1   |   DC    |   1   |   DC    | E1000_FC_FULL
			 *   1   |    1    |   0   |    0    | E1000_FC_NONE
			 *   1   |    1    |   0   |    1    | E1000_FC_RX_PAUSE
			 *
			 */
			/* Are both PAUSE bits set to 1?  If so, this implies
			 * Symmetric Flow Control is enabled at both ends.  The
			 * ASM_DIR bits are irrelevant per the spec.
			 *
			 * For Symmetric Flow Control:
			 *
			 *   LOCAL DEVICE  |   LINK PARTNER
			 * PAUSE | ASM_DIR | PAUSE | ASM_DIR | Result
			 *-------|---------|-------|---------|------------------
			 *   1   |   DC    |   1   |   DC    | E1000_FC_FULL
			 *
			 */
			if ((mii_nway_adv_reg & NWAY_AR_PAUSE) &&
			    (mii_nway_lp_ability_reg & NWAY_LPAR_PAUSE)) {
				/* Now we need to check if the user selected Rx
				 * ONLY of pause frames.  In this case, we had
				 * to advertise FULL flow control because we
				 * could not advertise Rx ONLY. Hence, we must
				 * now check to see if we need to turn OFF the
				 * TRANSMISSION of PAUSE frames.
				 */
				if (hw->original_fc == E1000_FC_FULL) {
					hw->fc = E1000_FC_FULL;
					e_dbg("Flow Control = FULL.\n");
				} else {
					hw->fc = E1000_FC_RX_PAUSE;
					e_dbg
					    ("Flow Control = RX PAUSE frames only.\n");
				}
			}
			/* For receiving PAUSE frames ONLY.
			 *
			 *   LOCAL DEVICE  |   LINK PARTNER
			 * PAUSE | ASM_DIR | PAUSE | ASM_DIR | Result
			 *-------|---------|-------|---------|------------------
			 *   0   |    1    |   1   |    1    | E1000_FC_TX_PAUSE
			 *
			 */
			else if (!(mii_nway_adv_reg & NWAY_AR_PAUSE) &&
				 (mii_nway_adv_reg & NWAY_AR_ASM_DIR) &&
				 (mii_nway_lp_ability_reg & NWAY_LPAR_PAUSE) &&
				 (mii_nway_lp_ability_reg & NWAY_LPAR_ASM_DIR)) {
				hw->fc = E1000_FC_TX_PAUSE;
				e_dbg
				    ("Flow Control = TX PAUSE frames only.\n");
			}
			/* For transmitting PAUSE frames ONLY.
			 *
			 *   LOCAL DEVICE  |   LINK PARTNER
			 * PAUSE | ASM_DIR | PAUSE | ASM_DIR | Result
			 *-------|---------|-------|---------|------------------
			 *   1   |    1    |   0   |    1    | E1000_FC_RX_PAUSE
			 *
			 */
			else if ((mii_nway_adv_reg & NWAY_AR_PAUSE) &&
				 (mii_nway_adv_reg & NWAY_AR_ASM_DIR) &&
				 !(mii_nway_lp_ability_reg & NWAY_LPAR_PAUSE) &&
				 (mii_nway_lp_ability_reg & NWAY_LPAR_ASM_DIR)) {
				hw->fc = E1000_FC_RX_PAUSE;
				e_dbg
				    ("Flow Control = RX PAUSE frames only.\n");
			}
			/* Per the IEEE spec, at this point flow control should
			 * be disabled.  However, we want to consider that we
			 * could be connected to a legacy switch that doesn't
			 * advertise desired flow control, but can be forced on
			 * the link partner.  So if we advertised no flow
			 * control, that is what we will resolve to.  If we
			 * advertised some kind of receive capability (Rx Pause
			 * Only or Full Flow Control) and the link partner
			 * advertised none, we will configure ourselves to
			 * enable Rx Flow Control only.  We can do this safely
			 * for two reasons:  If the link partner really
			 * didn't want flow control enabled, and we enable Rx,
			 * no harm done since we won't be receiving any PAUSE
			 * frames anyway.  If the intent on the link partner was
			 * to have flow control enabled, then by us enabling Rx
			 * only, we can at least receive pause frames and
			 * process them. This is a good idea because in most
			 * cases, since we are predominantly a server NIC, more
			 * times than not we will be asked to delay transmission
			 * of packets than asking our link partner to pause
			 * transmission of frames.
			 */
			else if ((hw->original_fc == E1000_FC_NONE ||
				  hw->original_fc == E1000_FC_TX_PAUSE) ||
				 hw->fc_strict_ieee) {
				hw->fc = E1000_FC_NONE;
				e_dbg("Flow Control = NONE.\n");
			} else {
				hw->fc = E1000_FC_RX_PAUSE;
				e_dbg
				    ("Flow Control = RX PAUSE frames only.\n");
			}

			/* Now we need to do one last check...  If we auto-
			 * negotiated to HALF DUPLEX, flow control should not be
			 * enabled per IEEE 802.3 spec.
			 */
			ret_val =
			    e1000_get_speed_and_duplex(hw, &speed, &duplex);
			if (ret_val) {
				e_dbg
				    ("Error getting link speed and duplex\n");
				return ret_val;
			}

			if (duplex == HALF_DUPLEX)
				hw->fc = E1000_FC_NONE;

			/* Now we call a subroutine to actually force the MAC
			 * controller to use the correct flow control settings.
			 */
			ret_val = e1000_force_mac_fc(hw);
			if (ret_val) {
				e_dbg
				    ("Error forcing flow control settings\n");
				return ret_val;
			}
		} else {
			e_dbg
			    ("Copper PHY and Auto Neg has not completed.\n");
		}
	}
	return E1000_SUCCESS;
}

/**
 * e1000_check_for_serdes_link_generic - Check for link (Serdes)
 * @hw: pointer to the HW structure
 *
 * Checks for link up on the hardware.  If link is not up and we have
 * a signal, then we need to force link up.
 */
static s32 e1000_check_for_serdes_link_generic(struct e1000_hw *hw)
{
	u32 rxcw;
	u32 ctrl;
	u32 status;
	s32 ret_val = E1000_SUCCESS;

	ctrl = er32(CTRL);
	status = er32(STATUS);
	rxcw = er32(RXCW);

	/* If we don't have link (auto-negotiation failed or link partner
	 * cannot auto-negotiate), and our link partner is not trying to
	 * auto-negotiate with us (we are receiving idles or data),
	 * we need to force link up. We also need to give auto-negotiation
	 * time to complete.
	 */
	/* (ctrl & E1000_CTRL_SWDPIN1) == 1 == have signal */
	if ((!(status & E1000_STATUS_LU)) && (!(rxcw & E1000_RXCW_C))) {
		if (hw->autoneg_failed == 0) {
			hw->autoneg_failed = 1;
			goto out;
		}
		e_dbg("NOT RXing /C/, disable AutoNeg and force link.\n");

		/* Disable auto-negotiation in the TXCW register */
		ew32(TXCW, (hw->txcw & ~E1000_TXCW_ANE));

		/* Force link-up and also force full-duplex. */
		ctrl = er32(CTRL);
		ctrl |= (E1000_CTRL_SLU | E1000_CTRL_FD);
		ew32(CTRL, ctrl);

		/* Configure Flow Control after forcing link up. */
		ret_val = e1000_config_fc_after_link_up(hw);
		if (ret_val) {
			e_dbg("Error configuring flow control\n");
			goto out;
		}
	} else if ((ctrl & E1000_CTRL_SLU) && (rxcw & E1000_RXCW_C)) {
		/* If we are forcing link and we are receiving /C/ ordered
		 * sets, re-enable auto-negotiation in the TXCW register
		 * and disable forced link in the Device Control register
		 * in an attempt to auto-negotiate with our link partner.
		 */
		e_dbg("RXing /C/, enable AutoNeg and stop forcing link.\n");
		ew32(TXCW, hw->txcw);
		ew32(CTRL, (ctrl & ~E1000_CTRL_SLU));

		hw->serdes_has_link = true;
	} else if (!(E1000_TXCW_ANE & er32(TXCW))) {
		/* If we force link for non-auto-negotiation switch, check
		 * link status based on MAC synchronization for internal
		 * serdes media type.
		 */
		/* SYNCH bit and IV bit are sticky. */
		udelay(10);
		rxcw = er32(RXCW);
		if (rxcw & E1000_RXCW_SYNCH) {
			if (!(rxcw & E1000_RXCW_IV)) {
				hw->serdes_has_link = true;
				e_dbg("SERDES: Link up - forced.\n");
			}
		} else {
			hw->serdes_has_link = false;
			e_dbg("SERDES: Link down - force failed.\n");
		}
	}

	if (E1000_TXCW_ANE & er32(TXCW)) {
		status = er32(STATUS);
		if (status & E1000_STATUS_LU) {
			/* SYNCH bit and IV bit are sticky, so reread rxcw. */
			udelay(10);
			rxcw = er32(RXCW);
			if (rxcw & E1000_RXCW_SYNCH) {
				if (!(rxcw & E1000_RXCW_IV)) {
					hw->serdes_has_link = true;
					e_dbg("SERDES: Link up - autoneg "
						 "completed successfully.\n");
				} else {
					hw->serdes_has_link = false;
					e_dbg("SERDES: Link down - invalid"
						 "codewords detected in autoneg.\n");
				}
			} else {
				hw->serdes_has_link = false;
				e_dbg("SERDES: Link down - no sync.\n");
			}
		} else {
			hw->serdes_has_link = false;
			e_dbg("SERDES: Link down - autoneg failed\n");
		}
	}

      out:
	return ret_val;
}

/**
 * e1000_check_for_link
 * @hw: Struct containing variables accessed by shared code
 *
 * Checks to see if the link status of the hardware has changed.
 * Called by any function that needs to check the link status of the adapter.
 */
s32 e1000_check_for_link(struct e1000_hw *hw)
{
	u32 status;
	u32 rctl;
	s32 ret_val;
	u16 phy_data;

	er32(CTRL);
	status = er32(STATUS);

	/* On adapters with a MAC newer than 82544, SW Definable pin 1 will be
	 * set when the optics detect a signal. On older adapters, it will be
	 * cleared when there is a signal.  This applies to fiber media only.
	 */
	if ((hw->media_type == e1000_media_type_fiber) ||
	    (hw->media_type == e1000_media_type_internal_serdes)) {
		er32(RXCW);

		if (hw->media_type == e1000_media_type_fiber) {
			if (status & E1000_STATUS_LU)
				hw->get_link_status = false;
		}
	}

	/* If we have a copper PHY then we only want to go out to the PHY
	 * registers to see if Auto-Neg has completed and/or if our link
	 * status has changed.  The get_link_status flag will be set if we
	 * receive a Link Status Change interrupt or we have Rx Sequence
	 * Errors.
	 */
	if ((hw->media_type == e1000_media_type_copper) && hw->get_link_status) {
		/* First we want to see if the MII Status Register reports
		 * link.  If so, then we want to get the current speed/duplex
		 * of the PHY.
		 * Read the register twice since the link bit is sticky.
		 */
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &phy_data);
		if (ret_val)
			return ret_val;
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &phy_data);
		if (ret_val)
			return ret_val;

		if (phy_data & MII_SR_LINK_STATUS) {
			hw->get_link_status = false;
			/* Check if there was DownShift, must be checked
			 * immediately after link-up
			 */
			e1000_check_downshift(hw);

		} else {
			/* No link detected */
			e1000_config_dsp_after_link_change(hw, false);
			return 0;
		}

		/* If we are forcing speed/duplex, then we simply return since
		 * we have already determined whether we have link or not.
		 */
		if (!hw->autoneg)
			return -E1000_ERR_CONFIG;

		/* optimize the dsp settings for the igp phy */
		e1000_config_dsp_after_link_change(hw, true);

		/* We have a M88E1000 PHY and Auto-Neg is enabled.  If we
		 * have Si on board that is 82544 or newer, Auto
		 * Speed Detection takes care of MAC speed/duplex
		 * configuration.  So we only need to configure Collision
		 * Distance in the MAC.  Otherwise, we need to force
		 * speed/duplex on the MAC to the current PHY speed/duplex
		 * settings.
		 */
		e1000_config_collision_dist(hw);

		/* Configure Flow Control now that Auto-Neg has completed.
		 * First, we need to restore the desired flow control settings
		 * because we may have had to re-autoneg with a different link
		 * partner.
		 */
		ret_val = e1000_config_fc_after_link_up(hw);
		if (ret_val) {
			e_dbg("Error configuring flow control\n");
			return ret_val;
		}

		/* At this point we know that we are on copper and we have
		 * auto-negotiated link.  These are conditions for checking the
		 * link partner capability register.  We use the link speed to
		 * determine if TBI compatibility needs to be turned on or off.
		 * If the link is not at gigabit speed, then TBI compatibility
		 * is not needed.  If we are at gigabit speed, we turn on TBI
		 * compatibility.
		 */
		if (hw->tbi_compatibility_en) {
			u16 speed, duplex;

			ret_val =
			    e1000_get_speed_and_duplex(hw, &speed, &duplex);

			if (ret_val) {
				e_dbg
				    ("Error getting link speed and duplex\n");
				return ret_val;
			}
			if (speed != SPEED_1000) {
				/* If link speed is not set to gigabit speed, we
				 * do not need to enable TBI compatibility.
				 */
				if (hw->tbi_compatibility_on) {
					/* If we previously were in the mode,
					 * turn it off.
					 */
					rctl = er32(RCTL);
					rctl &= ~E1000_RCTL_SBP;
					ew32(RCTL, rctl);
					hw->tbi_compatibility_on = false;
				}
			} else {
				/* If TBI compatibility is was previously off,
				 * turn it on. For compatibility with a TBI link
				 * partner, we will store bad packets. Some
				 * frames have an additional byte on the end and
				 * will look like CRC errors to to the hardware.
				 */
				if (!hw->tbi_compatibility_on) {
					hw->tbi_compatibility_on = true;
					rctl = er32(RCTL);
					rctl |= E1000_RCTL_SBP;
					ew32(RCTL, rctl);
				}
			}
		}
	}

	if ((hw->media_type == e1000_media_type_fiber) ||
	    (hw->media_type == e1000_media_type_internal_serdes))
		e1000_check_for_serdes_link_generic(hw);

	return E1000_SUCCESS;
}

/**
 * e1000_get_speed_and_duplex
 * @hw: Struct containing variables accessed by shared code
 * @speed: Speed of the connection
 * @duplex: Duplex setting of the connection
 *
 * Detects the current speed and duplex settings of the hardware.
 */
s32 e1000_get_speed_and_duplex(struct e1000_hw *hw, u16 *speed, u16 *duplex)
{
	u32 status;
	s32 ret_val;
	u16 phy_data;

	status = er32(STATUS);
	if (status & E1000_STATUS_SPEED_1000) {
		*speed = SPEED_1000;
		e_dbg("1000 Mbs, ");
	} else if (status & E1000_STATUS_SPEED_100) {
		*speed = SPEED_100;
		e_dbg("100 Mbs, ");
	} else {
		*speed = SPEED_10;
		e_dbg("10 Mbs, ");
	}

	if (status & E1000_STATUS_FD) {
		*duplex = FULL_DUPLEX;
		e_dbg("Full Duplex\n");
	} else {
		*duplex = HALF_DUPLEX;
		e_dbg(" Half Duplex\n");
	}

	/* IGP01 PHY may advertise full duplex operation after speed downgrade
	 * even if it is operating at half duplex.  Here we set the duplex
	 * settings to match the duplex in the link partner's capabilities.
	 */
	if (hw->phy_type == e1000_phy_igp && hw->speed_downgraded) {
		ret_val = e1000_read_phy_reg(hw, PHY_AUTONEG_EXP, &phy_data);
		if (ret_val)
			return ret_val;

		if (!(phy_data & NWAY_ER_LP_NWAY_CAPS))
			*duplex = HALF_DUPLEX;
		else {
			ret_val =
			    e1000_read_phy_reg(hw, PHY_LP_ABILITY, &phy_data);
			if (ret_val)
				return ret_val;
			if ((*speed == SPEED_100 &&
			     !(phy_data & NWAY_LPAR_100TX_FD_CAPS)) ||
			    (*speed == SPEED_10 &&
			     !(phy_data & NWAY_LPAR_10T_FD_CAPS)))
				*duplex = HALF_DUPLEX;
		}
	}

	return E1000_SUCCESS;
}

/**
 * e1000_wait_autoneg
 * @hw: Struct containing variables accessed by shared code
 *
 * Blocks until autoneg completes or times out (~4.5 seconds)
 */
static s32 e1000_wait_autoneg(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 i;
	u16 phy_data;

	e_dbg("Waiting for Auto-Neg to complete.\n");

	/* We will wait for autoneg to complete or 4.5 seconds to expire. */
	for (i = PHY_AUTO_NEG_TIME; i > 0; i--) {
		/* Read the MII Status Register and wait for Auto-Neg
		 * Complete bit to be set.
		 */
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &phy_data);
		if (ret_val)
			return ret_val;
		ret_val = e1000_read_phy_reg(hw, PHY_STATUS, &phy_data);
		if (ret_val)
			return ret_val;
		if (phy_data & MII_SR_AUTONEG_COMPLETE)
			return E1000_SUCCESS;

		msleep(100);
	}
	return E1000_SUCCESS;
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
	return e1000_get_phy_cfg_done(hw);
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

	switch (hw->phy_type) {
	case e1000_phy_igp:
		ret_val = e1000_phy_hw_reset(hw);
		if (ret_val)
			return ret_val;
		break;
	default:
		ret_val = e1000_read_phy_reg(hw, PHY_CTRL, &phy_data);
		if (ret_val)
			return ret_val;

		phy_data |= MII_CR_RESET;
		ret_val = e1000_write_phy_reg(hw, PHY_CTRL, phy_data);
		if (ret_val)
			return ret_val;

		udelay(1);
		break;
	}

	if (hw->phy_type == e1000_phy_igp)
		e1000_phy_init_script(hw);

	return E1000_SUCCESS;
}

/**
 * e1000_detect_gig_phy - check the phy type
 * @hw: Struct containing variables accessed by shared code
 *
 * Probes the expected PHY address for known PHY IDs
 */
static s32 e1000_detect_gig_phy(struct e1000_hw *hw)
{
	s32 phy_init_status, ret_val;
	u16 phy_id_high, phy_id_low;
	bool match = false;

	if (hw->phy_id != 0)
		return E1000_SUCCESS;

	/* Read the PHY ID Registers to identify which PHY is onboard. */
	ret_val = e1000_read_phy_reg(hw, PHY_ID1, &phy_id_high);
	if (ret_val)
		return ret_val;

	hw->phy_id = (u32)(phy_id_high << 16);
	udelay(20);
	ret_val = e1000_read_phy_reg(hw, PHY_ID2, &phy_id_low);
	if (ret_val)
		return ret_val;

	hw->phy_id |= (u32)(phy_id_low & PHY_REVISION_MASK);
	hw->phy_revision = (u32)phy_id_low & ~PHY_REVISION_MASK;

	switch (hw->mac_type) {
	case e1000_82545:
	case e1000_82545_rev_3:
		if (hw->phy_id == M88E1011_I_PHY_ID)
			match = true;
		break;
	default:
		e_dbg("Invalid MAC type %d\n", hw->mac_type);
		return -E1000_ERR_CONFIG;
	}
	phy_init_status = e1000_set_phy_type(hw);

	if ((match) && (phy_init_status == E1000_SUCCESS)) {
		e_dbg("PHY ID 0x%X detected\n", hw->phy_id);
		return E1000_SUCCESS;
	}
	e_dbg("Invalid PHY ID 0x%X\n", hw->phy_id);
	return -E1000_ERR_PHY;
}

/**
 * e1000_phy_reset_dsp - reset DSP
 * @hw: Struct containing variables accessed by shared code
 *
 * Resets the PHY's DSP
 */
static s32 e1000_phy_reset_dsp(struct e1000_hw *hw)
{
	s32 ret_val;

	do {
		ret_val = e1000_write_phy_reg(hw, 29, 0x001d);
		if (ret_val)
			break;
		ret_val = e1000_write_phy_reg(hw, 30, 0x00c1);
		if (ret_val)
			break;
		ret_val = e1000_write_phy_reg(hw, 30, 0x0000);
		if (ret_val)
			break;
		ret_val = E1000_SUCCESS;
	} while (0);

	return ret_val;
}






s32 e1000_validate_mdi_setting(struct e1000_hw *hw)
{
	if (!hw->autoneg && (hw->mdix == 0 || hw->mdix == 3)) {
		e_dbg("Invalid MDI setting detected\n");
		hw->mdix = 1;
		return -E1000_ERR_CONFIG;
	}
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
	u16 eeprom_size;

	switch (hw->mac_type) {
	case e1000_82545:
	case e1000_82545_rev_3:
		eeprom->type = e1000_eeprom_microwire;
		eeprom->opcode_bits = 3;
		eeprom->delay_usec = 50;
		if (eecd & E1000_EECD_SIZE) {
			eeprom->word_size = 256;
			eeprom->address_bits = 8;
		} else {
			eeprom->word_size = 64;
			eeprom->address_bits = 6;
		}
		break;
	default:
		break;
	}

	if (eeprom->type == e1000_eeprom_spi) {
		/* eeprom_size will be an enum [0..8] that maps to eeprom sizes
		 * 128B to 32KB (incremented by powers of 2).
		 */
		/* Set to default value for initial eeprom read. */
		eeprom->word_size = 64;
		ret_val = e1000_read_eeprom(hw, EEPROM_CFG, 1, &eeprom_size);
		if (ret_val)
			return ret_val;
		eeprom_size =
		    (eeprom_size & EEPROM_SIZE_MASK) >> EEPROM_SIZE_SHIFT;
		/* 256B eeprom size was not supported in earlier hardware, so we
		 * bump eeprom_size up one to ensure that "1" (which maps to
		 * 256B) is never the result used in the shifting logic below.
		 */
		if (eeprom_size)
			eeprom_size++;

		eeprom->word_size = 1 << (eeprom_size + EEPROM_WORD_SIZE_SHIFT);
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
	if (eeprom->type == e1000_eeprom_microwire)
		eecd &= ~E1000_EECD_DO;
	else if (eeprom->type == e1000_eeprom_spi)
		eecd |= E1000_EECD_DO;

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
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
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

	if (eeprom->type == e1000_eeprom_microwire) {
		/* Clear SK and DI */
		eecd &= ~(E1000_EECD_DI | E1000_EECD_SK);
		ew32(EECD, eecd);

		/* Set CS */
		eecd |= E1000_EECD_CS;
		ew32(EECD, eecd);
	} else if (eeprom->type == e1000_eeprom_spi) {
		/* Clear SK and CS */
		eecd &= ~(E1000_EECD_CS | E1000_EECD_SK);
		ew32(EECD, eecd);
		E1000_WRITE_FLUSH();
		udelay(1);
	}

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

	if (eeprom->type == e1000_eeprom_microwire) {
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
	} else if (eeprom->type == e1000_eeprom_spi) {
		/* Toggle CS to flush commands */
		eecd |= E1000_EECD_CS;
		ew32(EECD, eecd);
		E1000_WRITE_FLUSH();
		udelay(eeprom->delay_usec);
		eecd &= ~E1000_EECD_CS;
		ew32(EECD, eecd);
		E1000_WRITE_FLUSH();
		udelay(eeprom->delay_usec);
	}
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

	if (hw->eeprom.type == e1000_eeprom_spi) {
		eecd |= E1000_EECD_CS;	/* Pull CS high */
		eecd &= ~E1000_EECD_SK;	/* Lower SCK */

		ew32(EECD, eecd);
		E1000_WRITE_FLUSH();

		udelay(hw->eeprom.delay_usec);
	} else if (hw->eeprom.type == e1000_eeprom_microwire) {
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
	}

	/* Stop requesting EEPROM access */
	eecd &= ~E1000_EECD_REQ;
	ew32(EECD, eecd);
}

/**
 * e1000_spi_eeprom_ready - Reads a 16 bit word from the EEPROM.
 * @hw: Struct containing variables accessed by shared code
 */
static s32 e1000_spi_eeprom_ready(struct e1000_hw *hw)
{
	u16 retry_count = 0;
	u8 spi_stat_reg;

	/* Read "Status Register" repeatedly until the LSB is cleared.  The
	 * EEPROM will signal that the command has been completed by clearing
	 * bit 0 of the internal status register.  If it's not cleared within
	 * 5 milliseconds, then error out.
	 */
	retry_count = 0;
	do {
		e1000_shift_out_ee_bits(hw, EEPROM_RDSR_OPCODE_SPI,
					hw->eeprom.opcode_bits);
		spi_stat_reg = (u8)e1000_shift_in_ee_bits(hw, 8);
		if (!(spi_stat_reg & EEPROM_STATUS_RDY_SPI))
			break;

		udelay(5);
		retry_count += 5;

		e1000_standby_eeprom(hw);
	} while (retry_count < EEPROM_MAX_RETRY_SPI);

	/* ATMEL SPI write time could vary from 0-20mSec on 3.3V devices (and
	 * only 0-5mSec on 5V devices)
	 */
	if (retry_count >= EEPROM_MAX_RETRY_SPI) {
		e_dbg("SPI EEPROM Status error\n");
		return -E1000_ERR_EEPROM;
	}

	return E1000_SUCCESS;
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

	/* Set up the SPI or Microwire EEPROM for bit-bang reading.  We have
	 * acquired the EEPROM at this point, so any returns should release it
	 */
	if (eeprom->type == e1000_eeprom_spi) {
		u16 word_in;
		u8 read_opcode = EEPROM_READ_OPCODE_SPI;

		if (e1000_spi_eeprom_ready(hw)) {
			e1000_release_eeprom(hw);
			return -E1000_ERR_EEPROM;
		}

		e1000_standby_eeprom(hw);

		/* Some SPI eeproms use the 8th address bit embedded in the
		 * opcode
		 */
		if ((eeprom->address_bits == 8) && (offset >= 128))
			read_opcode |= EEPROM_A8_OPCODE_SPI;

		/* Send the READ command (opcode + addr)  */
		e1000_shift_out_ee_bits(hw, read_opcode, eeprom->opcode_bits);
		e1000_shift_out_ee_bits(hw, (u16)(offset * 2),
					eeprom->address_bits);

		/* Read the data.  The address of the eeprom internally
		 * increments with each byte (spi) being read, saving on the
		 * overhead of eeprom setup and tear-down.  The address counter
		 * will roll over if reading beyond the size of the eeprom, thus
		 * allowing the entire memory to be read starting from any
		 * offset.
		 */
		for (i = 0; i < words; i++) {
			word_in = e1000_shift_in_ee_bits(hw, 16);
			data[i] = (word_in >> 8) | (word_in << 8);
		}
	} else if (eeprom->type == e1000_eeprom_microwire) {
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

	if (eeprom->type == e1000_eeprom_microwire) {
		status = e1000_write_eeprom_microwire(hw, offset, words, data);
	} else {
		status = e1000_write_eeprom_spi(hw, offset, words, data);
		msleep(10);
	}

	/* Done with writing */
	e1000_release_eeprom(hw);

	return status;
}

/**
 * e1000_write_eeprom_spi - Writes a 16 bit word to a given offset in an SPI EEPROM.
 * @hw: Struct containing variables accessed by shared code
 * @offset: offset within the EEPROM to be written to
 * @words: number of words to write
 * @data: pointer to array of 8 bit words to be written to the EEPROM
 */
static s32 e1000_write_eeprom_spi(struct e1000_hw *hw, u16 offset, u16 words,
				  u16 *data)
{
	struct e1000_eeprom_info *eeprom = &hw->eeprom;
	u16 widx = 0;

	while (widx < words) {
		u8 write_opcode = EEPROM_WRITE_OPCODE_SPI;

		if (e1000_spi_eeprom_ready(hw))
			return -E1000_ERR_EEPROM;

		e1000_standby_eeprom(hw);
		cond_resched();

		/*  Send the WRITE ENABLE command (8 bit opcode )  */
		e1000_shift_out_ee_bits(hw, EEPROM_WREN_OPCODE_SPI,
					eeprom->opcode_bits);

		e1000_standby_eeprom(hw);

		/* Some SPI eeproms use the 8th address bit embedded in the
		 * opcode
		 */
		if ((eeprom->address_bits == 8) && (offset >= 128))
			write_opcode |= EEPROM_A8_OPCODE_SPI;

		/* Send the Write command (8-bit opcode + addr) */
		e1000_shift_out_ee_bits(hw, write_opcode, eeprom->opcode_bits);

		e1000_shift_out_ee_bits(hw, (u16)((offset + widx) * 2),
					eeprom->address_bits);

		/* Send the data */

		/* Loop to allow for up to whole page write (32 bytes) of
		 * eeprom
		 */
		while (widx < words) {
			u16 word_out = data[widx];

			word_out = (word_out >> 8) | (word_out << 8);
			e1000_shift_out_ee_bits(hw, word_out, 16);
			widx++;

			/* Some larger eeprom sizes are capable of a 32-byte
			 * PAGE WRITE operation, while the smaller eeproms are
			 * capable of an 8-byte PAGE WRITE operation.  Break the
			 * inner loop to pass new address
			 */
			if ((((offset + widx) * 2) % eeprom->page_size) == 0) {
				e1000_standby_eeprom(hw);
				break;
			}
		}
	}

	return E1000_SUCCESS;
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
 * e1000_clear_vfta - Clears the VLAN filer table
 * @hw: Struct containing variables accessed by shared code
 */
static void e1000_clear_vfta(struct e1000_hw *hw)
{
	u32 offset;

	for (offset = 0; offset < E1000_VLAN_FILTER_TBL_SIZE; offset++) {
		E1000_WRITE_REG_ARRAY(hw, VFTA, offset, 0);
		E1000_WRITE_FLUSH();
	}
}




/**
 * e1000_clear_hw_cntrs - Clears all hardware statistics counters.
 * @hw: Struct containing variables accessed by shared code
 */
static void e1000_clear_hw_cntrs(struct e1000_hw *hw)
{
	er32(CRCERRS);
	er32(SYMERRS);
	er32(MPC);
	er32(SCC);
	er32(ECOL);
	er32(MCC);
	er32(LATECOL);
	er32(COLC);
	er32(DC);
	er32(SEC);
	er32(RLEC);
	er32(XONRXC);
	er32(XONTXC);
	er32(XOFFRXC);
	er32(XOFFTXC);
	er32(FCRUC);

	er32(PRC64);
	er32(PRC127);
	er32(PRC255);
	er32(PRC511);
	er32(PRC1023);
	er32(PRC1522);

	er32(GPRC);
	er32(BPRC);
	er32(MPRC);
	er32(GPTC);
	er32(GORCL);
	er32(GORCH);
	er32(GOTCL);
	er32(GOTCH);
	er32(RNBC);
	er32(RUC);
	er32(RFC);
	er32(ROC);
	er32(RJC);
	er32(TORL);
	er32(TORH);
	er32(TOTL);
	er32(TOTH);
	er32(TPR);
	er32(TPT);

	er32(PTC64);
	er32(PTC127);
	er32(PTC255);
	er32(PTC511);
	er32(PTC1023);
	er32(PTC1522);

	er32(MPTC);
	er32(BPTC);

	er32(ALGNERRC);
	er32(RXERRC);
	er32(TNCRS);
	er32(CEXTERR);
	er32(TSCTC);
	er32(TSCTFC);

	er32(MGTPRC);
	er32(MGTPDC);
	er32(MGTPTC);
}

/**
 * e1000_reset_adaptive - Resets Adaptive IFS to its default state.
 * @hw: Struct containing variables accessed by shared code
 *
 * Call this after e1000_init_hw. You may override the IFS defaults by setting
 * hw->ifs_params_forced to true. However, you must initialize hw->
 * current_ifs_val, ifs_min_val, ifs_max_val, ifs_step_size, and ifs_ratio
 * before calling this function.
 */
void e1000_reset_adaptive(struct e1000_hw *hw)
{
	if (hw->adaptive_ifs) {
		if (!hw->ifs_params_forced) {
			hw->current_ifs_val = 0;
			hw->ifs_min_val = IFS_MIN;
			hw->ifs_max_val = IFS_MAX;
			hw->ifs_step_size = IFS_STEP;
			hw->ifs_ratio = IFS_RATIO;
		}
		hw->in_ifs_mode = false;
		ew32(AIT, 0);
	} else {
		e_dbg("Not in Adaptive IFS mode!\n");
	}
}

/**
 * e1000_update_adaptive - update adaptive IFS
 * @hw: Struct containing variables accessed by shared code
 *
 * Called during the callback/watchdog routine to update IFS value based on
 * the ratio of transmits to collisions.
 */
void e1000_update_adaptive(struct e1000_hw *hw)
{
	if (hw->adaptive_ifs) {
		if ((hw->collision_delta * hw->ifs_ratio) > hw->tx_packet_delta) {
			if (hw->tx_packet_delta > MIN_NUM_XMITS) {
				hw->in_ifs_mode = true;
				if (hw->current_ifs_val < hw->ifs_max_val) {
					if (hw->current_ifs_val == 0)
						hw->current_ifs_val =
						    hw->ifs_min_val;
					else
						hw->current_ifs_val +=
						    hw->ifs_step_size;
					ew32(AIT, hw->current_ifs_val);
				}
			}
		} else {
			if (hw->in_ifs_mode &&
			    (hw->tx_packet_delta <= MIN_NUM_XMITS)) {
				hw->current_ifs_val = 0;
				hw->in_ifs_mode = false;
				ew32(AIT, 0);
			}
		}
	} else {
		e_dbg("Not in Adaptive IFS mode!\n");
	}
}

/**
 * e1000_get_bus_info
 * @hw: Struct containing variables accessed by shared code
 *
 * Gets the current PCI bus type, speed, and width of the hardware
 */
void e1000_get_bus_info(struct e1000_hw *hw)
{
	u32 status;

	status = er32(STATUS);
	hw->bus_type = (status & E1000_STATUS_PCIX_MODE) ?
		e1000_bus_type_pcix : e1000_bus_type_pci;

	if (hw->bus_type == e1000_bus_type_pci) {
		hw->bus_speed = (status & E1000_STATUS_PCI66) ?
			e1000_bus_speed_66 : e1000_bus_speed_33;
	} else {
		switch (status & E1000_STATUS_PCIX_SPEED) {
		case E1000_STATUS_PCIX_SPEED_66:
			hw->bus_speed = e1000_bus_speed_66;
			break;
		case E1000_STATUS_PCIX_SPEED_100:
			hw->bus_speed = e1000_bus_speed_100;
			break;
		case E1000_STATUS_PCIX_SPEED_133:
			hw->bus_speed = e1000_bus_speed_133;
			break;
		default:
			hw->bus_speed = e1000_bus_speed_reserved;
			break;
		}
	}
	hw->bus_width = (status & E1000_STATUS_BUS64) ?
		e1000_bus_width_64 : e1000_bus_width_32;
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



/**
 * e1000_check_downshift - Check if Downshift occurred
 * @hw: Struct containing variables accessed by shared code
 *
 * returns: - E1000_ERR_XXX
 *            E1000_SUCCESS
 *
 * For phy's older than IGP, this function reads the Downshift bit in the Phy
 * Specific Status register.  For IGP phy's, it reads the Downgrade bit in the
 * Link Health register.  In IGP this bit is latched high, so the driver must
 * read it immediately after link is established.
 */
static s32 e1000_check_downshift(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 phy_data;

	if (hw->phy_type == e1000_phy_igp) {
		ret_val = e1000_read_phy_reg(hw, IGP01E1000_PHY_LINK_HEALTH,
					     &phy_data);
		if (ret_val)
			return ret_val;

		hw->speed_downgraded =
		    (phy_data & IGP01E1000_PLHR_SS_DOWNGRADE) ? 1 : 0;
	} else if (hw->phy_type == e1000_phy_m88) {
		ret_val = e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_STATUS,
					     &phy_data);
		if (ret_val)
			return ret_val;

		hw->speed_downgraded = (phy_data & M88E1000_PSSR_DOWNSHIFT) >>
		    M88E1000_PSSR_DOWNSHIFT_SHIFT;
	}

	return E1000_SUCCESS;
}

static const u16 dsp_reg_array[IGP01E1000_PHY_CHANNEL_NUM] = {
	IGP01E1000_PHY_AGC_PARAM_A,
	IGP01E1000_PHY_AGC_PARAM_B,
	IGP01E1000_PHY_AGC_PARAM_C,
	IGP01E1000_PHY_AGC_PARAM_D
};


/**
 * e1000_config_dsp_after_link_change
 * @hw: Struct containing variables accessed by shared code
 * @link_up: was link up at the time this was called
 *
 * returns: - E1000_ERR_PHY if fail to read/write the PHY
 *            E1000_SUCCESS at any other case.
 *
 * 82541_rev_2 & 82547_rev_2 have the capability to configure the DSP when a
 * gigabit link is achieved to improve link quality.
 */

static s32 e1000_config_dsp_after_link_change(struct e1000_hw *hw, bool link_up)
{
	s32 ret_val;
	u16 phy_data, phy_saved_data, speed, duplex, i;

	if (hw->phy_type != e1000_phy_igp)
		return E1000_SUCCESS;

	if (link_up) {
		ret_val = e1000_get_speed_and_duplex(hw, &speed, &duplex);
		if (ret_val) {
			e_dbg("Error getting link speed and duplex\n");
			return ret_val;
		}

		// if (speed == SPEED_1000) {
		// 	ret_val = e1000_1000Mb_check_cable_length(hw);
		// 	if (ret_val)
		// 		return ret_val;
		// }

	} else {
		if (hw->dsp_config_state == e1000_dsp_config_activated) {
			/* Save off the current value of register 0x2F5B to be
			 * restored at the end of the routines.
			 */
			ret_val =
			    e1000_read_phy_reg(hw, 0x2F5B, &phy_saved_data);

			if (ret_val)
				return ret_val;

			/* Disable the PHY transmitter */
			ret_val = e1000_write_phy_reg(hw, 0x2F5B, 0x0003);

			if (ret_val)
				return ret_val;

			msleep(20);

			ret_val = e1000_write_phy_reg(hw, 0x0000,
						      IGP01E1000_IEEE_FORCE_GIGA);
			if (ret_val)
				return ret_val;
			for (i = 0; i < IGP01E1000_PHY_CHANNEL_NUM; i++) {
				ret_val =
				    e1000_read_phy_reg(hw, dsp_reg_array[i],
						       &phy_data);
				if (ret_val)
					return ret_val;

				phy_data &= ~IGP01E1000_PHY_EDAC_MU_INDEX;
				phy_data |= IGP01E1000_PHY_EDAC_SIGN_EXT_9_BITS;

				ret_val =
				    e1000_write_phy_reg(hw, dsp_reg_array[i],
							phy_data);
				if (ret_val)
					return ret_val;
			}

			ret_val = e1000_write_phy_reg(hw, 0x0000,
						      IGP01E1000_IEEE_RESTART_AUTONEG);
			if (ret_val)
				return ret_val;

			msleep(20);

			/* Now enable the transmitter */
			ret_val =
			    e1000_write_phy_reg(hw, 0x2F5B, phy_saved_data);

			if (ret_val)
				return ret_val;

			hw->dsp_config_state = e1000_dsp_config_enabled;
		}

		if (hw->ffe_config_state == e1000_ffe_config_active) {
			/* Save off the current value of register 0x2F5B to be
			 * restored at the end of the routines.
			 */
			ret_val =
			    e1000_read_phy_reg(hw, 0x2F5B, &phy_saved_data);

			if (ret_val)
				return ret_val;

			/* Disable the PHY transmitter */
			ret_val = e1000_write_phy_reg(hw, 0x2F5B, 0x0003);

			if (ret_val)
				return ret_val;

			msleep(20);

			ret_val = e1000_write_phy_reg(hw, 0x0000,
						      IGP01E1000_IEEE_FORCE_GIGA);
			if (ret_val)
				return ret_val;
			ret_val =
			    e1000_write_phy_reg(hw, IGP01E1000_PHY_DSP_FFE,
						IGP01E1000_PHY_DSP_FFE_DEFAULT);
			if (ret_val)
				return ret_val;

			ret_val = e1000_write_phy_reg(hw, 0x0000,
						      IGP01E1000_IEEE_RESTART_AUTONEG);
			if (ret_val)
				return ret_val;

			msleep(20);

			/* Now enable the transmitter */
			ret_val =
			    e1000_write_phy_reg(hw, 0x2F5B, phy_saved_data);

			if (ret_val)
				return ret_val;

			hw->ffe_config_state = e1000_ffe_config_enabled;
		}
	}
	return E1000_SUCCESS;
}

/**
 * e1000_set_phy_mode - Set PHY to class A mode
 * @hw: Struct containing variables accessed by shared code
 *
 * Assumes the following operations will follow to enable the new class mode.
 *  1. Do a PHY soft reset
 *  2. Restart auto-negotiation or force link.
 */
static s32 e1000_set_phy_mode(struct e1000_hw *hw)
{
	s32 ret_val;
	u16 eeprom_data;

	if ((hw->mac_type == e1000_82545_rev_3) &&
	    (hw->media_type == e1000_media_type_copper)) {
		ret_val =
		    e1000_read_eeprom(hw, EEPROM_PHY_CLASS_WORD, 1,
				      &eeprom_data);
		if (ret_val)
			return ret_val;

		if ((eeprom_data != EEPROM_RESERVED_WORD) &&
		    (eeprom_data & EEPROM_PHY_CLASS_A)) {
			ret_val =
			    e1000_write_phy_reg(hw, M88E1000_PHY_PAGE_SELECT,
						0x000B);
			if (ret_val)
				return ret_val;
			ret_val =
			    e1000_write_phy_reg(hw, M88E1000_PHY_GEN_CONTROL,
						0x8104);
			if (ret_val)
				return ret_val;

			hw->phy_reset_disable = false;
		}
	}

	return E1000_SUCCESS;
}

/**
 * e1000_set_d3_lplu_state - set d3 link power state
 * @hw: Struct containing variables accessed by shared code
 * @active: true to enable lplu false to disable lplu.
 *
 * This function sets the lplu state according to the active flag.  When
 * activating lplu this function also disables smart speed and vise versa.
 * lplu will not be activated unless the device autonegotiation advertisement
 * meets standards of either 10 or 10/100 or 10/100/1000 at all duplexes.
 *
 * returns: - E1000_ERR_PHY if fail to read/write the PHY
 *            E1000_SUCCESS at any other case.
 */
static s32 e1000_set_d3_lplu_state(struct e1000_hw *hw, bool active)
{
	s32 ret_val;
	u16 phy_data;

	if (hw->phy_type != e1000_phy_igp)
		return E1000_SUCCESS;


	if (!active) {

		/* LPLU and SmartSpeed are mutually exclusive.  LPLU is used
		 * during Dx states where the power conservation is most
		 * important.  During driver activity we should enable
		 * SmartSpeed, so performance is maintained.
		 */
		if (hw->smart_speed == e1000_smart_speed_on) {
			ret_val =
			    e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
					       &phy_data);
			if (ret_val)
				return ret_val;

			phy_data |= IGP01E1000_PSCFR_SMART_SPEED;
			ret_val =
			    e1000_write_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
						phy_data);
			if (ret_val)
				return ret_val;
		} else if (hw->smart_speed == e1000_smart_speed_off) {
			ret_val =
			    e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
					       &phy_data);
			if (ret_val)
				return ret_val;

			phy_data &= ~IGP01E1000_PSCFR_SMART_SPEED;
			ret_val =
			    e1000_write_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
						phy_data);
			if (ret_val)
				return ret_val;
		}
	} else if ((hw->autoneg_advertised == AUTONEG_ADVERTISE_SPEED_DEFAULT) ||
		   (hw->autoneg_advertised == AUTONEG_ADVERTISE_10_ALL) ||
		   (hw->autoneg_advertised == AUTONEG_ADVERTISE_10_100_ALL)) {

		/* When LPLU is enabled we should disable SmartSpeed */
		ret_val =
		    e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
				       &phy_data);
		if (ret_val)
			return ret_val;

		phy_data &= ~IGP01E1000_PSCFR_SMART_SPEED;
		ret_val =
		    e1000_write_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG,
					phy_data);
		if (ret_val)
			return ret_val;
	}
	return E1000_SUCCESS;
}


/**
 * e1000_enable_mng_pass_thru - check for bmc pass through
 * @hw: Struct containing variables accessed by shared code
 *
 * Verifies the hardware needs to allow ARPs to be processed by the host
 * returns: - true/false
 */
u32 e1000_enable_mng_pass_thru(struct e1000_hw *hw)
{
	u32 manc;

	if (hw->asf_firmware_present) {
		manc = er32(MANC);

		if (!(manc & E1000_MANC_RCV_TCO_EN) ||
		    !(manc & E1000_MANC_EN_MAC_ADDR_FILTER))
			return false;
		if ((manc & E1000_MANC_SMBUS_EN) && !(manc & E1000_MANC_ASF_EN))
			return true;
	}
	return false;
}



/**
 * e1000_get_auto_rd_done
 * @hw: Struct containing variables accessed by shared code
 *
 * Check for EEPROM Auto Read bit done.
 * returns: - E1000_ERR_RESET if fail to reset MAC
 *            E1000_SUCCESS at any other case.
 */
static s32 e1000_get_auto_rd_done(struct e1000_hw *hw)
{
	msleep(5);
	return E1000_SUCCESS;
}

/**
 * e1000_get_phy_cfg_done
 * @hw: Struct containing variables accessed by shared code
 *
 * Checks if the PHY configuration is done
 * returns: - E1000_ERR_RESET if fail to reset MAC
 *            E1000_SUCCESS at any other case.
 */
static s32 e1000_get_phy_cfg_done(struct e1000_hw *hw)
{
	msleep(10);
	return E1000_SUCCESS;
}