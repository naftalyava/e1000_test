/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 1999 - 2006 Intel Corporation. */

/* e1000_hw.h
 * Structures, enums, and macros for the MAC
 */

#ifndef _E1000_HW_H_
#define _E1000_HW_H_


#define DEFAULT_TIDV                   8
#define DEFAULT_TADV                  32
#define DEFAULT_RDTR                   0
#define DEFAULT_RADV                   8
#define DEFAULT_ITR                    3


#define er32(reg)							\
	(readl(hw->hw_addr + E1000_##reg))

#define ew32(reg, value)						\
	(writel((value), (hw->hw_addr + E1000_##reg)))

#define E1000_WRITE_REG_ARRAY(a, reg, offset, value) ( \
    writel((value), ((a)->hw_addr + E1000_##reg)))

#define E1000_READ_REG_ARRAY(a, reg, offset) ( \
    readl((a)->hw_addr + E1000_##reg))


#define E1000_WRITE_FLUSH() er32(STATUS)



/* Forward declarations of structures used by the shared code */
struct e1000_hw;
struct e1000_hw_stats;

/* Enumerated types specific to the e1000 hardware */
/* Media Access Controllers */
typedef enum {
	e1000_undefined = 0,
	e1000_82545,
	e1000_82545_rev_3,
	e1000_num_macs
} e1000_mac_type;



/* Media Types */
typedef enum {
	e1000_media_type_copper = 0,
	e1000_media_type_fiber = 1,
	e1000_media_type_internal_serdes = 2,
	e1000_num_media_types
} e1000_media_type;

typedef enum {
	e1000_10_half = 0,
	e1000_10_full = 1,
	e1000_100_half = 2,
	e1000_100_full = 3
} e1000_speed_duplex_type;

/* Flow Control Settings */
typedef enum {
	E1000_FC_NONE = 0,
	E1000_FC_RX_PAUSE = 1,
	E1000_FC_TX_PAUSE = 2,
	E1000_FC_FULL = 3,
	E1000_FC_DEFAULT = 0xFF
} e1000_fc_type;

struct e1000_shadow_ram {
	u16 eeprom_word;
	bool modified;
};

/* PCI bus types */
typedef enum {
	e1000_bus_type_unknown = 0,
	e1000_bus_type_pci,
	e1000_bus_type_pcix,
	e1000_bus_type_reserved
} e1000_bus_type;

/* PCI bus speeds */
typedef enum {
	e1000_bus_speed_unknown = 0,
	e1000_bus_speed_33,
	e1000_bus_speed_66,
	e1000_bus_speed_100,
	e1000_bus_speed_120,
	e1000_bus_speed_133,
	e1000_bus_speed_reserved
} e1000_bus_speed;

/* PCI bus widths */
typedef enum {
	e1000_bus_width_unknown = 0,
	e1000_bus_width_32,
	e1000_bus_width_64,
	e1000_bus_width_reserved
} e1000_bus_width;



typedef enum {
	e1000_polarity_reversal_enabled = 0,
	e1000_polarity_reversal_disabled,
	e1000_polarity_reversal_undefined = 0xFF
} e1000_polarity_reversal;

typedef enum {
	e1000_auto_x_mode_manual_mdi = 0,
	e1000_auto_x_mode_manual_mdix,
	e1000_auto_x_mode_auto1,
	e1000_auto_x_mode_auto2,
	e1000_auto_x_mode_undefined = 0xFF
} e1000_auto_x_mode;

typedef enum {
	e1000_1000t_rx_status_not_ok = 0,
	e1000_1000t_rx_status_ok,
	e1000_1000t_rx_status_undefined = 0xFF
} e1000_1000t_rx_status;

typedef enum {
	e1000_phy_m88 = 0,
	e1000_phy_igp,
	e1000_phy_8211,
	e1000_phy_8201,
	e1000_phy_undefined = 0xFF
} e1000_phy_type;

typedef enum {
	e1000_ms_hw_default = 0,
	e1000_ms_force_master,
	e1000_ms_force_slave,
	e1000_ms_auto
} e1000_ms_type;




struct e1000_eeprom_info {
	u16 word_size;
	u16 opcode_bits;
	u16 address_bits;
	u16 delay_usec;
	u16 page_size;
};


typedef enum {
	e1000_byte_align = 0,
	e1000_word_align = 1,
	e1000_dword_align = 2
} e1000_align_type;

/* Error Codes */
#define E1000_SUCCESS      0
#define E1000_ERR_EEPROM   1
#define E1000_ERR_PHY      2
#define E1000_ERR_CONFIG   3
#define E1000_ERR_PARAM    4
#define E1000_ERR_MAC_TYPE 5
#define E1000_ERR_PHY_TYPE 6
#define E1000_ERR_RESET   9
#define E1000_ERR_MASTER_REQUESTS_PENDING 10
#define E1000_ERR_HOST_INTERFACE_COMMAND 11
#define E1000_BLK_PHY_RESET   12

#define E1000_BYTE_SWAP_WORD(_value) ((((_value) & 0x00ff) << 8) | \
                                     (((_value) & 0xff00) >> 8))

/* Function prototypes */
/* Initialization */
s32 e1000_reset_hw(struct e1000_hw *hw);
s32 e1000_init_hw(struct e1000_hw *hw);





/* PHY */
s32 e1000_read_phy_reg(struct e1000_hw *hw, u32 reg_addr, u16 * phy_data);
s32 e1000_write_phy_reg(struct e1000_hw *hw, u32 reg_addr, u16 data);
s32 e1000_phy_hw_reset(struct e1000_hw *hw);
s32 e1000_phy_reset(struct e1000_hw *hw);
s32 e1000_validate_mdi_setting(struct e1000_hw *hw);

/* EEPROM Functions */
s32 e1000_init_eeprom_params(struct e1000_hw *hw);



struct e1000_host_mng_command_header {
	u8 command_id;
	u8 checksum;
	u16 reserved1;
	u16 reserved2;
	u16 command_length;
};


struct e1000_host_mng_dhcp_cookie {
	u32 signature;
	u8 status;
	u8 reserved0;
	u16 vlan_id;
	u32 reserved1;
	u16 reserved2;
	u8 reserved3;
	u8 checksum;
};


bool e1000_check_mng_mode(struct e1000_hw *hw);
s32 e1000_read_eeprom(struct e1000_hw *hw, u16 reg, u16 words, u16 * data);
s32 e1000_validate_eeprom_checksum(struct e1000_hw *hw);
s32 e1000_update_eeprom_checksum(struct e1000_hw *hw);
s32 e1000_write_eeprom(struct e1000_hw *hw, u16 reg, u16 words, u16 * data);
s32 e1000_read_mac_addr(struct e1000_hw *hw);

/* Filters (multicast, vlan, receive) */
u32 e1000_hash_mc_addr(struct e1000_hw *hw, u8 * mc_addr);
void e1000_mta_set(struct e1000_hw *hw, u32 hash_value);
void e1000_rar_set(struct e1000_hw *hw, u8 * mc_addr, u32 rar_index);



/* Port I/O is only supported on 82544 and newer */
void e1000_io_write(struct e1000_hw *hw, unsigned long port, u32 value);

#define E1000_READ_REG_IO(a, reg) \
    e1000_read_reg_io((a), E1000_##reg)
#define E1000_WRITE_REG_IO(a, reg, val) \
    e1000_write_reg_io((a), E1000_##reg, val)

/* PCI Device IDs */
#define E1000_DEV_ID_82545EM_COPPER      0x100F

#define NODE_ADDRESS_SIZE 6

/* MAC decode size is 128K - This is the size of BAR0 */
#define MAC_DECODE_SIZE (128 * 1024)

#define E1000_82542_2_0_REV_ID 2
#define E1000_82542_2_1_REV_ID 3
#define E1000_REVISION_0       0
#define E1000_REVISION_1       1
#define E1000_REVISION_2       2
#define E1000_REVISION_3       3


#define HALF_DUPLEX 1
#define FULL_DUPLEX 2

/* The sizes (in bytes) of a ethernet packet */
#define ENET_HEADER_SIZE             14
#define MINIMUM_ETHERNET_FRAME_SIZE  64	/* With FCS */
#define ETHERNET_FCS_SIZE            4
#define MINIMUM_ETHERNET_PACKET_SIZE \
    (MINIMUM_ETHERNET_FRAME_SIZE - ETHERNET_FCS_SIZE)
#define CRC_LENGTH                   ETHERNET_FCS_SIZE
#define MAX_JUMBO_FRAME_SIZE         0x3F00

/* 802.1q VLAN Packet Sizes */
#define VLAN_TAG_SIZE  4	/* 802.3ac tag (not DMAed) */

/* Ethertype field values */
#define ETHERNET_IEEE_VLAN_TYPE 0x8100	/* 802.3ac packet */
#define ETHERNET_IP_TYPE        0x0800	/* IP packets */
#define ETHERNET_ARP_TYPE       0x0806	/* Address Resolution Protocol (ARP) */

/* Packet Header defines */
#define IP_PROTOCOL_TCP    6
#define IP_PROTOCOL_UDP    0x11

/* This defines the bits that are set in the Interrupt Mask
 * Set/Read Register.  Each bit is documented below:
 *   o RXDMT0 = Receive Descriptor Minimum Threshold hit (ring 0)
 *   o RXSEQ  = Receive Sequence Error
 */
#define POLL_IMS_ENABLE_MASK ( \
    E1000_IMS_RXDMT0 |         \
    E1000_IMS_RXSEQ)

/* This defines the bits that are set in the Interrupt Mask
 * Set/Read Register.  Each bit is documented below:
 *   o RXT0   = Receiver Timer Interrupt (ring 0)
 *   o TXDW   = Transmit Descriptor Written Back
 *   o RXDMT0 = Receive Descriptor Minimum Threshold hit (ring 0)
 *   o RXSEQ  = Receive Sequence Error
 *   o LSC    = Link Status Change
 */
#define IMS_ENABLE_MASK ( \
    E1000_IMS_RXT0   |    \
    E1000_IMS_TXDW   |    \
    E1000_IMS_RXDMT0 |    \
    E1000_IMS_RXSEQ  |    \
    E1000_IMS_LSC)

/* Number of high/low register pairs in the RAR. The RAR (Receive Address
 * Registers) holds the directed and multicast addresses that we monitor. We
 * reserve one of these spots for our directed address, allowing us room for
 * E1000_RAR_ENTRIES - 1 multicast addresses.
 */
#define E1000_RAR_ENTRIES 15

#define MIN_NUMBER_OF_DESCRIPTORS  8
#define MAX_NUMBER_OF_DESCRIPTORS  0xFFF8

/* Receive Descriptor */
struct e1000_rx_desc {
	__le64 buffer_addr;	/* Address of the descriptor's data buffer */
	__le16 length;		/* Length of data DMAed into data buffer */
	__le16 csum;		/* Packet checksum */
	u8 status;		/* Descriptor status */
	u8 errors;		/* Descriptor Errors */
	__le16 special;
};

/* Receive Descriptor - Extended */
union e1000_rx_desc_extended {
	struct {
		__le64 buffer_addr;
		__le64 reserved;
	} read;
	struct {
		struct {
			__le32 mrq;	/* Multiple Rx Queues */
			union {
				__le32 rss;	/* RSS Hash */
				struct {
					__le16 ip_id;	/* IP id */
					__le16 csum;	/* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error;	/* ext status/error */
			__le16 length;
			__le16 vlan;	/* VLAN tag */
		} upper;
	} wb;			/* writeback */
};

#define MAX_PS_BUFFERS 4
/* Receive Descriptor - Packet Split */
union e1000_rx_desc_packet_split {
	struct {
		/* one buffer for protocol header(s), three data buffers */
		__le64 buffer_addr[MAX_PS_BUFFERS];
	} read;
	struct {
		struct {
			__le32 mrq;	/* Multiple Rx Queues */
			union {
				__le32 rss;	/* RSS Hash */
				struct {
					__le16 ip_id;	/* IP id */
					__le16 csum;	/* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error;	/* ext status/error */
			__le16 length0;	/* length of buffer 0 */
			__le16 vlan;	/* VLAN tag */
		} middle;
		struct {
			__le16 header_status;
			__le16 length[3];	/* length of buffers 1-3 */
		} upper;
		__le64 reserved;
	} wb;			/* writeback */
};

/* Receive Descriptor bit definitions */
#define E1000_RXD_STAT_DD       0x01	/* Descriptor Done */
#define E1000_RXD_STAT_EOP      0x02	/* End of Packet */
#define E1000_RXD_STAT_IXSM     0x04	/* Ignore checksum */
#define E1000_RXD_STAT_VP       0x08	/* IEEE VLAN Packet */
#define E1000_RXD_STAT_UDPCS    0x10	/* UDP xsum calculated */
#define E1000_RXD_STAT_TCPCS    0x20	/* TCP xsum calculated */
#define E1000_RXD_STAT_IPCS     0x40	/* IP xsum calculated */
#define E1000_RXD_STAT_PIF      0x80	/* passed in-exact filter */
#define E1000_RXD_STAT_IPIDV    0x200	/* IP identification valid */
#define E1000_RXD_STAT_UDPV     0x400	/* Valid UDP checksum */
#define E1000_RXD_STAT_ACK      0x8000	/* ACK Packet indication */
#define E1000_RXD_ERR_CE        0x01	/* CRC Error */
#define E1000_RXD_ERR_SE        0x02	/* Symbol Error */
#define E1000_RXD_ERR_SEQ       0x04	/* Sequence Error */
#define E1000_RXD_ERR_CXE       0x10	/* Carrier Extension Error */
#define E1000_RXD_ERR_TCPE      0x20	/* TCP/UDP Checksum Error */
#define E1000_RXD_ERR_IPE       0x40	/* IP Checksum Error */
#define E1000_RXD_ERR_RXE       0x80	/* Rx Data Error */
#define E1000_RXD_SPC_VLAN_MASK 0x0FFF	/* VLAN ID is in lower 12 bits */
#define E1000_RXD_SPC_PRI_MASK  0xE000	/* Priority is in upper 3 bits */
#define E1000_RXD_SPC_PRI_SHIFT 13
#define E1000_RXD_SPC_CFI_MASK  0x1000	/* CFI is bit 12 */
#define E1000_RXD_SPC_CFI_SHIFT 12




/* mask to determine if packets should be dropped due to frame errors */
#define E1000_RXD_ERR_FRAME_ERR_MASK ( \
    E1000_RXD_ERR_CE  |                \
    E1000_RXD_ERR_SE  |                \
    E1000_RXD_ERR_SEQ |                \
    E1000_RXD_ERR_CXE |                \
    E1000_RXD_ERR_RXE)

/* Same mask, but for extended and packet split descriptors */
#define E1000_RXDEXT_ERR_FRAME_ERR_MASK ( \
    E1000_RXDEXT_STATERR_CE  |            \
    E1000_RXDEXT_STATERR_SE  |            \
    E1000_RXDEXT_STATERR_SEQ |            \
    E1000_RXDEXT_STATERR_CXE |            \
    E1000_RXDEXT_STATERR_RXE)

/* Transmit Descriptor */
struct e1000_tx_desc {
	__le64 buffer_addr;	/* Address of the descriptor's data buffer */
	union {
		__le32 data;
		struct {
			__le16 length;	/* Data buffer length */
			u8 cso;	/* Checksum offset */
			u8 cmd;	/* Descriptor control */
		} flags;
	} lower;
	union {
		__le32 data;
		struct {
			u8 status;	/* Descriptor status */
			u8 css;	/* Checksum start */
			__le16 special;
		} fields;
	} upper;
};

/* Transmit Descriptor bit definitions */
#define E1000_TXD_DTYP_D     0x00100000	/* Data Descriptor */
#define E1000_TXD_DTYP_C     0x00000000	/* Context Descriptor */
#define E1000_TXD_POPTS_IXSM 0x01	/* Insert IP checksum */
#define E1000_TXD_POPTS_TXSM 0x02	/* Insert TCP/UDP checksum */
#define E1000_TXD_CMD_EOP    0x01000000	/* End of Packet */
#define E1000_TXD_CMD_IFCS   0x02000000	/* Insert FCS (Ethernet CRC) */
#define E1000_TXD_CMD_IC     0x04000000	/* Insert Checksum */
#define E1000_TXD_CMD_RS     0x08000000	/* Report Status */
#define E1000_TXD_CMD_RPS    0x10000000	/* Report Packet Sent */
#define E1000_TXD_CMD_DEXT   0x20000000	/* Descriptor extension (0 = legacy) */
#define E1000_TXD_CMD_VLE    0x40000000	/* Add VLAN tag */
#define E1000_TXD_CMD_IDE    0x80000000	/* Enable Tidv register */
#define E1000_TXD_STAT_DD    0x00000001	/* Descriptor Done */
#define E1000_TXD_STAT_EC    0x00000002	/* Excess Collisions */
#define E1000_TXD_STAT_LC    0x00000004	/* Late Collisions */
#define E1000_TXD_STAT_TU    0x00000008	/* Transmit underrun */
#define E1000_TXD_CMD_TCP    0x01000000	/* TCP packet */
#define E1000_TXD_CMD_IP     0x02000000	/* IP packet */
#define E1000_TXD_CMD_TSE    0x04000000	/* TCP Seg enable */
#define E1000_TXD_STAT_TC    0x00000004	/* Tx Underrun */

/* Offload Context Descriptor */
struct e1000_context_desc {
	union {
		__le32 ip_config;
		struct {
			u8 ipcss;	/* IP checksum start */
			u8 ipcso;	/* IP checksum offset */
			__le16 ipcse;	/* IP checksum end */
		} ip_fields;
	} lower_setup;
	union {
		__le32 tcp_config;
		struct {
			u8 tucss;	/* TCP checksum start */
			u8 tucso;	/* TCP checksum offset */
			__le16 tucse;	/* TCP checksum end */
		} tcp_fields;
	} upper_setup;
	__le32 cmd_and_length;	/* */
	union {
		__le32 data;
		struct {
			u8 status;	/* Descriptor status */
			u8 hdr_len;	/* Header length */
			__le16 mss;	/* Maximum segment size */
		} fields;
	} tcp_seg_setup;
};

/* Offload data descriptor */
struct e1000_data_desc {
	__le64 buffer_addr;	/* Address of the descriptor's buffer address */
	union {
		__le32 data;
		struct {
			__le16 length;	/* Data buffer length */
			u8 typ_len_ext;	/* */
			u8 cmd;	/* */
		} flags;
	} lower;
	union {
		__le32 data;
		struct {
			u8 status;	/* Descriptor status */
			u8 popts;	/* Packet Options */
			__le16 special;	/* */
		} fields;
	} upper;
};

/* Filters */
#define E1000_NUM_UNICAST          16	/* Unicast filter entries */
#define E1000_MC_TBL_SIZE          128	/* Multicast Filter Table (4096 bits) */
#define E1000_VLAN_FILTER_TBL_SIZE 128	/* VLAN Filter Table (4096 bits) */

/* Receive Address Register */
struct e1000_rar {
	volatile __le32 low;	/* receive address low */
	volatile __le32 high;	/* receive address high */
};

/* Number of entries in the Multicast Table Array (MTA). */
#define E1000_NUM_MTA_REGISTERS 128

/* IPv4 Address Table Entry */
struct e1000_ipv4_at_entry {
	volatile u32 ipv4_addr;	/* IP Address (RW) */
	volatile u32 reserved;
};

/* Four wakeup IP addresses are supported */
#define E1000_WAKEUP_IP_ADDRESS_COUNT_MAX 4
#define E1000_IP4AT_SIZE                  E1000_WAKEUP_IP_ADDRESS_COUNT_MAX
#define E1000_IP6AT_SIZE                  1

/* IPv6 Address Table Entry */
struct e1000_ipv6_at_entry {
	volatile u8 ipv6_addr[16];
};

/* Flexible Filter Length Table Entry */
struct e1000_fflt_entry {
	volatile u32 length;	/* Flexible Filter Length (RW) */
	volatile u32 reserved;
};

/* Flexible Filter Mask Table Entry */
struct e1000_ffmt_entry {
	volatile u32 mask;	/* Flexible Filter Mask (RW) */
	volatile u32 reserved;
};

/* Flexible Filter Value Table Entry */
struct e1000_ffvt_entry {
	volatile u32 value;	/* Flexible Filter Value (RW) */
	volatile u32 reserved;
};




/* Register Set. (82543, 82544)
 *
 * Registers are defined to be 32 bits and  should be accessed as 32 bit values.
 * These registers are physically located on the NIC, but are mapped into the
 * host memory address space.
 *
 * RW - register is both readable and writable
 * RO - register is read only
 * WO - register is write only
 * R/clr - register is read only and is cleared when read
 * A - register array
 */
#define E1000_CTRL     0x00000	/* Device Control - RW */
#define E1000_CTRL_DUP 0x00004	/* Device Control Duplicate (Shadow) - RW */
#define E1000_STATUS   0x00008	/* Device Status - RO */
#define E1000_EECD     0x00010	/* EEPROM/Flash Control - RW */
#define E1000_EERD     0x00014	/* EEPROM Read - RW */
#define E1000_CTRL_EXT 0x00018	/* Extended Device Control - RW */
#define E1000_FLA      0x0001C	/* Flash Access - RW */
#define E1000_MDIC     0x00020	/* MDI Control - RW */

#define INTEL_CE_GBE_MDIO_RCOMP_BASE    (hw->ce4100_gbe_mdio_base_virt)
#define E1000_MDIO_STS  (INTEL_CE_GBE_MDIO_RCOMP_BASE + 0)
#define E1000_MDIO_CMD  (INTEL_CE_GBE_MDIO_RCOMP_BASE + 4)
#define E1000_MDIO_DRV  (INTEL_CE_GBE_MDIO_RCOMP_BASE + 8)
#define E1000_MDC_CMD   (INTEL_CE_GBE_MDIO_RCOMP_BASE + 0xC)
#define E1000_RCOMP_CTL (INTEL_CE_GBE_MDIO_RCOMP_BASE + 0x20)
#define E1000_RCOMP_STS (INTEL_CE_GBE_MDIO_RCOMP_BASE + 0x24)

#define E1000_SCTL     0x00024	/* SerDes Control - RW */
#define E1000_FEXTNVM  0x00028	/* Future Extended NVM register */
#define E1000_FCAL     0x00028	/* Flow Control Address Low - RW */
#define E1000_FCAH     0x0002C	/* Flow Control Address High -RW */
#define E1000_FCT      0x00030	/* Flow Control Type - RW */
#define E1000_VET      0x00038	/* VLAN Ether Type - RW */
#define E1000_ICR      0x000C0	/* Interrupt Cause Read - R/clr */
#define E1000_ITR      0x000C4	/* Interrupt Throttling Rate - RW */
#define E1000_ICS      0x000C8	/* Interrupt Cause Set - WO */
#define E1000_IMS      0x000D0	/* Interrupt Mask Set - RW */
#define E1000_IMC      0x000D8	/* Interrupt Mask Clear - WO */
#define E1000_IAM      0x000E0	/* Interrupt Acknowledge Auto Mask */

/* Auxiliary Control Register. This register is CE4100 specific,
 * RMII/RGMII function is switched by this register - RW
 * Following are bits definitions of the Auxiliary Control Register
 */
#define E1000_CTL_AUX  0x000E0
#define E1000_CTL_AUX_END_SEL_SHIFT     10
#define E1000_CTL_AUX_ENDIANESS_SHIFT   8
#define E1000_CTL_AUX_RGMII_RMII_SHIFT  0

/* descriptor and packet transfer use CTL_AUX.ENDIANESS */
#define E1000_CTL_AUX_DES_PKT   (0x0 << E1000_CTL_AUX_END_SEL_SHIFT)
/* descriptor use CTL_AUX.ENDIANESS, packet use default */
#define E1000_CTL_AUX_DES       (0x1 << E1000_CTL_AUX_END_SEL_SHIFT)
/* descriptor use default, packet use CTL_AUX.ENDIANESS */
#define E1000_CTL_AUX_PKT       (0x2 << E1000_CTL_AUX_END_SEL_SHIFT)
/* all use CTL_AUX.ENDIANESS */
#define E1000_CTL_AUX_ALL       (0x3 << E1000_CTL_AUX_END_SEL_SHIFT)

#define E1000_CTL_AUX_RGMII     (0x0 << E1000_CTL_AUX_RGMII_RMII_SHIFT)
#define E1000_CTL_AUX_RMII      (0x1 << E1000_CTL_AUX_RGMII_RMII_SHIFT)

/* LW little endian, Byte big endian */
#define E1000_CTL_AUX_LWLE_BBE  (0x0 << E1000_CTL_AUX_ENDIANESS_SHIFT)
#define E1000_CTL_AUX_LWLE_BLE  (0x1 << E1000_CTL_AUX_ENDIANESS_SHIFT)
#define E1000_CTL_AUX_LWBE_BBE  (0x2 << E1000_CTL_AUX_ENDIANESS_SHIFT)
#define E1000_CTL_AUX_LWBE_BLE  (0x3 << E1000_CTL_AUX_ENDIANESS_SHIFT)

#define E1000_RCTL     0x00100	/* RX Control - RW */
#define E1000_RDTR1    0x02820	/* RX Delay Timer (1) - RW */
#define E1000_RDBAL1   0x02900	/* RX Descriptor Base Address Low (1) - RW */
#define E1000_RDBAH1   0x02904	/* RX Descriptor Base Address High (1) - RW */
#define E1000_RDLEN1   0x02908	/* RX Descriptor Length (1) - RW */
#define E1000_RDH1     0x02910	/* RX Descriptor Head (1) - RW */
#define E1000_RDT1     0x02918	/* RX Descriptor Tail (1) - RW */
#define E1000_FCTTV    0x00170	/* Flow Control Transmit Timer Value - RW */
#define E1000_TXCW     0x00178	/* TX Configuration Word - RW */
#define E1000_RXCW     0x00180	/* RX Configuration Word - RO */
#define E1000_TCTL     0x00400	/* TX Control - RW */
#define E1000_TCTL_EXT 0x00404	/* Extended TX Control - RW */
#define E1000_TIPG     0x00410	/* TX Inter-packet gap -RW */
#define E1000_TBT      0x00448	/* TX Burst Timer - RW */
#define E1000_AIT      0x00458	/* Adaptive Interframe Spacing Throttle - RW */
#define E1000_LEDCTL   0x00E00	/* LED Control - RW */
#define E1000_EXTCNF_CTRL  0x00F00	/* Extended Configuration Control */
#define E1000_EXTCNF_SIZE  0x00F08	/* Extended Configuration Size */
#define E1000_PHY_CTRL     0x00F10	/* PHY Control Register in CSR */
#define FEXTNVM_SW_CONFIG  0x0001
#define E1000_PBA      0x01000	/* Packet Buffer Allocation - RW */
#define E1000_PBS      0x01008	/* Packet Buffer Size */
#define E1000_EEMNGCTL 0x01010	/* MNG EEprom Control */
#define E1000_FLASH_UPDATES 1000
#define E1000_EEARBC   0x01024	/* EEPROM Auto Read Bus Control */
#define E1000_FLASHT   0x01028	/* FLASH Timer Register */
#define E1000_EEWR     0x0102C	/* EEPROM Write Register - RW */
#define E1000_FLSWCTL  0x01030	/* FLASH control register */
#define E1000_FLSWDATA 0x01034	/* FLASH data register */
#define E1000_FLSWCNT  0x01038	/* FLASH Access Counter */
#define E1000_FLOP     0x0103C	/* FLASH Opcode Register */
#define E1000_ERT      0x02008	/* Early Rx Threshold - RW */
#define E1000_FCRTL    0x02160	/* Flow Control Receive Threshold Low - RW */
#define E1000_FCRTH    0x02168	/* Flow Control Receive Threshold High - RW */
#define E1000_PSRCTL   0x02170	/* Packet Split Receive Control - RW */
#define E1000_RDFH     0x02410  /* RX Data FIFO Head - RW */
#define E1000_RDFT     0x02418  /* RX Data FIFO Tail - RW */
#define E1000_RDFHS    0x02420  /* RX Data FIFO Head Saved - RW */
#define E1000_RDFTS    0x02428  /* RX Data FIFO Tail Saved - RW */
#define E1000_RDFPC    0x02430  /* RX Data FIFO Packet Count - RW */
#define E1000_RDBAL    0x02800	/* RX Descriptor Base Address Low - RW */
#define E1000_RDBAH    0x02804	/* RX Descriptor Base Address High - RW */
#define E1000_RDLEN    0x02808	/* RX Descriptor Length - RW */
#define E1000_RDH      0x02810	/* RX Descriptor Head - RW */
#define E1000_RDT      0x02818	/* RX Descriptor Tail - RW */
#define E1000_RDTR     0x02820	/* RX Delay Timer - RW */
#define E1000_RDBAL0   E1000_RDBAL	/* RX Desc Base Address Low (0) - RW */
#define E1000_RDBAH0   E1000_RDBAH	/* RX Desc Base Address High (0) - RW */
#define E1000_RDLEN0   E1000_RDLEN	/* RX Desc Length (0) - RW */
#define E1000_RDH0     E1000_RDH	/* RX Desc Head (0) - RW */
#define E1000_RDT0     E1000_RDT	/* RX Desc Tail (0) - RW */
#define E1000_RDTR0    E1000_RDTR	/* RX Delay Timer (0) - RW */
#define E1000_RXDCTL   0x02828	/* RX Descriptor Control queue 0 - RW */
#define E1000_RXDCTL1  0x02928	/* RX Descriptor Control queue 1 - RW */
#define E1000_RADV     0x0282C	/* RX Interrupt Absolute Delay Timer - RW */
#define E1000_RSRPD    0x02C00	/* RX Small Packet Detect - RW */
#define E1000_RAID     0x02C08	/* Receive Ack Interrupt Delay - RW */
#define E1000_TXDMAC   0x03000	/* TX DMA Control - RW */
#define E1000_KABGTXD  0x03004	/* AFE Band Gap Transmit Ref Data */
#define E1000_TDFH     0x03410	/* TX Data FIFO Head - RW */
#define E1000_TDFT     0x03418	/* TX Data FIFO Tail - RW */
#define E1000_TDFHS    0x03420	/* TX Data FIFO Head Saved - RW */
#define E1000_TDFTS    0x03428	/* TX Data FIFO Tail Saved - RW */
#define E1000_TDFPC    0x03430	/* TX Data FIFO Packet Count - RW */
#define E1000_TDBAL    0x03800	/* TX Descriptor Base Address Low - RW */
#define E1000_TDBAH    0x03804	/* TX Descriptor Base Address High - RW */
#define E1000_TDLEN    0x03808	/* TX Descriptor Length - RW */
#define E1000_TDH      0x03810	/* TX Descriptor Head - RW */
#define E1000_TDT      0x03818	/* TX Descripotr Tail - RW */
#define E1000_TIDV     0x03820	/* TX Interrupt Delay Value - RW */
#define E1000_TXDCTL   0x03828	/* TX Descriptor Control - RW */
#define E1000_TADV     0x0382C	/* TX Interrupt Absolute Delay Val - RW */
#define E1000_TSPMT    0x03830	/* TCP Segmentation PAD & Min Threshold - RW */
#define E1000_TARC0    0x03840	/* TX Arbitration Count (0) */
#define E1000_TDBAL1   0x03900	/* TX Desc Base Address Low (1) - RW */
#define E1000_TDBAH1   0x03904	/* TX Desc Base Address High (1) - RW */
#define E1000_TDLEN1   0x03908	/* TX Desc Length (1) - RW */
#define E1000_TDH1     0x03910	/* TX Desc Head (1) - RW */
#define E1000_TDT1     0x03918	/* TX Desc Tail (1) - RW */
#define E1000_TXDCTL1  0x03928	/* TX Descriptor Control (1) - RW */
#define E1000_TARC1    0x03940	/* TX Arbitration Count (1) */
#define E1000_CRCERRS  0x04000	/* CRC Error Count - R/clr */
#define E1000_ALGNERRC 0x04004	/* Alignment Error Count - R/clr */
#define E1000_SYMERRS  0x04008	/* Symbol Error Count - R/clr */
#define E1000_RXERRC   0x0400C	/* Receive Error Count - R/clr */
#define E1000_MPC      0x04010	/* Missed Packet Count - R/clr */
#define E1000_SCC      0x04014	/* Single Collision Count - R/clr */
#define E1000_ECOL     0x04018	/* Excessive Collision Count - R/clr */
#define E1000_MCC      0x0401C	/* Multiple Collision Count - R/clr */
#define E1000_LATECOL  0x04020	/* Late Collision Count - R/clr */
#define E1000_COLC     0x04028	/* Collision Count - R/clr */
#define E1000_DC       0x04030	/* Defer Count - R/clr */
#define E1000_TNCRS    0x04034	/* TX-No CRS - R/clr */
#define E1000_SEC      0x04038	/* Sequence Error Count - R/clr */
#define E1000_CEXTERR  0x0403C	/* Carrier Extension Error Count - R/clr */
#define E1000_RLEC     0x04040	/* Receive Length Error Count - R/clr */
#define E1000_XONRXC   0x04048	/* XON RX Count - R/clr */
#define E1000_XONTXC   0x0404C	/* XON TX Count - R/clr */
#define E1000_XOFFRXC  0x04050	/* XOFF RX Count - R/clr */
#define E1000_XOFFTXC  0x04054	/* XOFF TX Count - R/clr */
#define E1000_FCRUC    0x04058	/* Flow Control RX Unsupported Count- R/clr */
#define E1000_PRC64    0x0405C	/* Packets RX (64 bytes) - R/clr */
#define E1000_PRC127   0x04060	/* Packets RX (65-127 bytes) - R/clr */
#define E1000_PRC255   0x04064	/* Packets RX (128-255 bytes) - R/clr */
#define E1000_PRC511   0x04068	/* Packets RX (255-511 bytes) - R/clr */
#define E1000_PRC1023  0x0406C	/* Packets RX (512-1023 bytes) - R/clr */
#define E1000_PRC1522  0x04070	/* Packets RX (1024-1522 bytes) - R/clr */
#define E1000_GPRC     0x04074	/* Good Packets RX Count - R/clr */
#define E1000_BPRC     0x04078	/* Broadcast Packets RX Count - R/clr */
#define E1000_MPRC     0x0407C	/* Multicast Packets RX Count - R/clr */
#define E1000_GPTC     0x04080	/* Good Packets TX Count - R/clr */
#define E1000_GORCL    0x04088	/* Good Octets RX Count Low - R/clr */
#define E1000_GORCH    0x0408C	/* Good Octets RX Count High - R/clr */
#define E1000_GOTCL    0x04090	/* Good Octets TX Count Low - R/clr */
#define E1000_GOTCH    0x04094	/* Good Octets TX Count High - R/clr */
#define E1000_RNBC     0x040A0	/* RX No Buffers Count - R/clr */
#define E1000_RUC      0x040A4	/* RX Undersize Count - R/clr */
#define E1000_RFC      0x040A8	/* RX Fragment Count - R/clr */
#define E1000_ROC      0x040AC	/* RX Oversize Count - R/clr */
#define E1000_RJC      0x040B0	/* RX Jabber Count - R/clr */
#define E1000_MGTPRC   0x040B4	/* Management Packets RX Count - R/clr */
#define E1000_MGTPDC   0x040B8	/* Management Packets Dropped Count - R/clr */
#define E1000_MGTPTC   0x040BC	/* Management Packets TX Count - R/clr */
#define E1000_TORL     0x040C0	/* Total Octets RX Low - R/clr */
#define E1000_TORH     0x040C4	/* Total Octets RX High - R/clr */
#define E1000_TOTL     0x040C8	/* Total Octets TX Low - R/clr */
#define E1000_TOTH     0x040CC	/* Total Octets TX High - R/clr */
#define E1000_TPR      0x040D0	/* Total Packets RX - R/clr */
#define E1000_TPT      0x040D4	/* Total Packets TX - R/clr */
#define E1000_PTC64    0x040D8	/* Packets TX (64 bytes) - R/clr */
#define E1000_PTC127   0x040DC	/* Packets TX (65-127 bytes) - R/clr */
#define E1000_PTC255   0x040E0	/* Packets TX (128-255 bytes) - R/clr */
#define E1000_PTC511   0x040E4	/* Packets TX (256-511 bytes) - R/clr */
#define E1000_PTC1023  0x040E8	/* Packets TX (512-1023 bytes) - R/clr */
#define E1000_PTC1522  0x040EC	/* Packets TX (1024-1522 Bytes) - R/clr */
#define E1000_MPTC     0x040F0	/* Multicast Packets TX Count - R/clr */
#define E1000_BPTC     0x040F4	/* Broadcast Packets TX Count - R/clr */
#define E1000_TSCTC    0x040F8	/* TCP Segmentation Context TX - R/clr */
#define E1000_TSCTFC   0x040FC	/* TCP Segmentation Context TX Fail - R/clr */
#define E1000_IAC      0x04100	/* Interrupt Assertion Count */
#define E1000_ICRXPTC  0x04104	/* Interrupt Cause Rx Packet Timer Expire Count */
#define E1000_ICRXATC  0x04108	/* Interrupt Cause Rx Absolute Timer Expire Count */
#define E1000_ICTXPTC  0x0410C	/* Interrupt Cause Tx Packet Timer Expire Count */
#define E1000_ICTXATC  0x04110	/* Interrupt Cause Tx Absolute Timer Expire Count */
#define E1000_ICTXQEC  0x04118	/* Interrupt Cause Tx Queue Empty Count */
#define E1000_ICTXQMTC 0x0411C	/* Interrupt Cause Tx Queue Minimum Threshold Count */
#define E1000_ICRXDMTC 0x04120	/* Interrupt Cause Rx Descriptor Minimum Threshold Count */
#define E1000_ICRXOC   0x04124	/* Interrupt Cause Receiver Overrun Count */
#define E1000_RXCSUM   0x05000	/* RX Checksum Control - RW */
#define E1000_RFCTL    0x05008	/* Receive Filter Control */
#define E1000_MTA      0x05200	/* Multicast Table Array - RW Array */
#define E1000_RA       0x05400	/* Receive Address - RW Array */
#define E1000_VFTA     0x05600	/* VLAN Filter Table Array - RW Array */
#define E1000_WUC      0x05800	/* Wakeup Control - RW */
#define E1000_WUFC     0x05808	/* Wakeup Filter Control - RW */
#define E1000_WUS      0x05810	/* Wakeup Status - RO */
#define E1000_MANC     0x05820	/* Management Control - RW */
#define E1000_IPAV     0x05838	/* IP Address Valid - RW */
#define E1000_IP4AT    0x05840	/* IPv4 Address Table - RW Array */
#define E1000_IP6AT    0x05880	/* IPv6 Address Table - RW Array */
#define E1000_WUPL     0x05900	/* Wakeup Packet Length - RW */
#define E1000_WUPM     0x05A00	/* Wakeup Packet Memory - RO A */
#define E1000_FFLT     0x05F00	/* Flexible Filter Length Table - RW Array */
#define E1000_HOST_IF  0x08800	/* Host Interface */
#define E1000_FFMT     0x09000	/* Flexible Filter Mask Table - RW Array */
#define E1000_FFVT     0x09800	/* Flexible Filter Value Table - RW Array */

#define E1000_KUMCTRLSTA 0x00034	/* MAC-PHY interface - RW */
#define E1000_MDPHYA     0x0003C	/* PHY address - RW */
#define E1000_MANC2H     0x05860	/* Management Control To Host - RW */
#define E1000_SW_FW_SYNC 0x05B5C	/* Software-Firmware Synchronization - RW */

#define E1000_GCR       0x05B00	/* PCI-Ex Control */
#define E1000_GSCL_1    0x05B10	/* PCI-Ex Statistic Control #1 */
#define E1000_GSCL_2    0x05B14	/* PCI-Ex Statistic Control #2 */
#define E1000_GSCL_3    0x05B18	/* PCI-Ex Statistic Control #3 */
#define E1000_GSCL_4    0x05B1C	/* PCI-Ex Statistic Control #4 */
#define E1000_FACTPS    0x05B30	/* Function Active and Power State to MNG */
#define E1000_SWSM      0x05B50	/* SW Semaphore */
#define E1000_FWSM      0x05B54	/* FW Semaphore */
#define E1000_FFLT_DBG  0x05F04	/* Debug Register */
#define E1000_HICR      0x08F00	/* Host Interface Control */



/* Structure containing variables used by the shared code (e1000_hw.c) */
struct e1000_hw {
	u8 __iomem *hw_addr;
	u8 __iomem *flash_address;
	void __iomem *ce4100_gbe_mdio_base_virt;
	e1000_mac_type mac_type;
	e1000_phy_type phy_type;
	e1000_media_type media_type;
	void *back;
	struct e1000_shadow_ram *eeprom_shadow_ram;
	u32 flash_bank_size;
	u32 flash_base_addr;
	e1000_bus_speed bus_speed;
	e1000_bus_width bus_width;
	e1000_bus_type bus_type;
	struct e1000_eeprom_info eeprom;
	e1000_ms_type master_slave;
	e1000_ms_type original_master_slave;
	u32 asf_firmware_present;
	u32 eeprom_semaphore_present;
	unsigned long io_base;
	u32 phy_id;
	u32 phy_revision;
	u32 phy_addr;
	u32 original_fc;
	u32 txcw;
	u32 autoneg_failed;
	u32 max_frame_size;
	u32 min_frame_size;
	u32 mc_filter_type;
	u32 num_mc_addrs;
	u32 collision_delta;
	u32 tx_packet_delta;
	u32 ledctl_default;
	u32 ledctl_mode1;
	u32 ledctl_mode2;
	bool tx_pkt_filtering;
	struct e1000_host_mng_dhcp_cookie mng_cookie;
	u16 phy_spd_default;
	u16 autoneg_advertised;
	u16 pci_cmd_word;
	u16 current_ifs_val;
	u16 device_id;
	u16 vendor_id;
	u16 subsystem_id;
	u16 subsystem_vendor_id;
	u8 revision_id;
	u8 autoneg;
	u8 mdix;
	u8 forced_speed_duplex;
	u8 wait_autoneg_complete;
	u8 dma_fairness;
	u8 mac_addr[NODE_ADDRESS_SIZE];
	u8 perm_mac_addr[NODE_ADDRESS_SIZE];
	bool get_link_status;
	bool phy_reset_disable;
	bool initialize_hw_bits_disable;

};


/* Register Bit Masks */
/* Device Control */
#define E1000_CTRL_FD       0x00000001	/* Full duplex.0=half; 1=full */
#define E1000_CTRL_BEM      0x00000002	/* Endian Mode.0=little,1=big */
#define E1000_CTRL_PRIOR    0x00000004	/* Priority on PCI. 0=rx,1=fair */
#define E1000_CTRL_GIO_MASTER_DISABLE 0x00000004	/*Blocks new Master requests */
#define E1000_CTRL_LRST     0x00000008	/* Link reset. 0=normal,1=reset */
#define E1000_CTRL_TME      0x00000010	/* Test mode. 0=normal,1=test */
#define E1000_CTRL_SLE      0x00000020	/* Serial Link on 0=dis,1=en */
#define E1000_CTRL_ASDE     0x00000020	/* Auto-speed detect enable */
#define E1000_CTRL_SLU      0x00000040	/* Set link up (Force Link) */
#define E1000_CTRL_ILOS     0x00000080	/* Invert Loss-Of Signal */
#define E1000_CTRL_SPD_SEL  0x00000300	/* Speed Select Mask */
#define E1000_CTRL_SPD_10   0x00000000	/* Force 10Mb */
#define E1000_CTRL_SPD_100  0x00000100	/* Force 100Mb */
#define E1000_CTRL_SPD_1000 0x00000200	/* Force 1Gb */
#define E1000_CTRL_BEM32    0x00000400	/* Big Endian 32 mode */
#define E1000_CTRL_FRCSPD   0x00000800	/* Force Speed */
#define E1000_CTRL_FRCDPX   0x00001000	/* Force Duplex */
#define E1000_CTRL_D_UD_EN  0x00002000	/* Dock/Undock enable */
#define E1000_CTRL_D_UD_POLARITY 0x00004000	/* Defined polarity of Dock/Undock indication in SDP[0] */
#define E1000_CTRL_FORCE_PHY_RESET 0x00008000	/* Reset both PHY ports, through PHYRST_N pin */
#define E1000_CTRL_EXT_LINK_EN 0x00010000	/* enable link status from external LINK_0 and LINK_1 pins */
#define E1000_CTRL_SWDPIN0  0x00040000	/* SWDPIN 0 value */
#define E1000_CTRL_SWDPIN1  0x00080000	/* SWDPIN 1 value */
#define E1000_CTRL_SWDPIN2  0x00100000	/* SWDPIN 2 value */
#define E1000_CTRL_SWDPIN3  0x00200000	/* SWDPIN 3 value */
#define E1000_CTRL_SWDPIO0  0x00400000	/* SWDPIN 0 Input or output */
#define E1000_CTRL_SWDPIO1  0x00800000	/* SWDPIN 1 input or output */
#define E1000_CTRL_SWDPIO2  0x01000000	/* SWDPIN 2 input or output */
#define E1000_CTRL_SWDPIO3  0x02000000	/* SWDPIN 3 input or output */
#define E1000_CTRL_RST      0x04000000	/* Global reset */
#define E1000_CTRL_RFCE     0x08000000	/* Receive Flow Control enable */
#define E1000_CTRL_TFCE     0x10000000	/* Transmit flow control enable */
#define E1000_CTRL_RTE      0x20000000	/* Routing tag enable */
#define E1000_CTRL_VME      0x40000000	/* IEEE VLAN mode enable */
#define E1000_CTRL_PHY_RST  0x80000000	/* PHY Reset */
#define E1000_CTRL_SW2FW_INT 0x02000000	/* Initiate an interrupt to manageability engine */

/* Device Status */
#define E1000_STATUS_FD         0x00000001	/* Full duplex.0=half,1=full */
#define E1000_STATUS_LU         0x00000002	/* Link up.0=no,1=link */
#define E1000_STATUS_FUNC_MASK  0x0000000C	/* PCI Function Mask */
#define E1000_STATUS_FUNC_SHIFT 2
#define E1000_STATUS_FUNC_0     0x00000000	/* Function 0 */
#define E1000_STATUS_FUNC_1     0x00000004	/* Function 1 */
#define E1000_STATUS_TXOFF      0x00000010	/* transmission paused */
#define E1000_STATUS_TBIMODE    0x00000020	/* TBI mode */
#define E1000_STATUS_SPEED_MASK 0x000000C0
#define E1000_STATUS_SPEED_10   0x00000000	/* Speed 10Mb/s */
#define E1000_STATUS_SPEED_100  0x00000040	/* Speed 100Mb/s */
#define E1000_STATUS_SPEED_1000 0x00000080	/* Speed 1000Mb/s */
#define E1000_STATUS_LAN_INIT_DONE 0x00000200	/* Lan Init Completion
						   by EEPROM/Flash */
#define E1000_STATUS_ASDV       0x00000300	/* Auto speed detect value */
#define E1000_STATUS_DOCK_CI    0x00000800	/* Change in Dock/Undock state. Clear on write '0'. */
#define E1000_STATUS_GIO_MASTER_ENABLE 0x00080000	/* Status of Master requests. */
#define E1000_STATUS_MTXCKOK    0x00000400	/* MTX clock running OK */
#define E1000_STATUS_PCI66      0x00000800	/* In 66Mhz slot */
#define E1000_STATUS_BUS64      0x00001000	/* In 64 bit slot */
#define E1000_STATUS_PCIX_MODE  0x00002000	/* PCI-X mode */
#define E1000_STATUS_PCIX_SPEED 0x0000C000	/* PCI-X bus speed */
#define E1000_STATUS_BMC_SKU_0  0x00100000	/* BMC USB redirect disabled */
#define E1000_STATUS_BMC_SKU_1  0x00200000	/* BMC SRAM disabled */
#define E1000_STATUS_BMC_SKU_2  0x00400000	/* BMC SDRAM disabled */
#define E1000_STATUS_BMC_CRYPTO 0x00800000	/* BMC crypto disabled */
#define E1000_STATUS_BMC_LITE   0x01000000	/* BMC external code execution disabled */
#define E1000_STATUS_RGMII_ENABLE 0x02000000	/* RGMII disabled */
#define E1000_STATUS_FUSE_8       0x04000000
#define E1000_STATUS_FUSE_9       0x08000000
#define E1000_STATUS_SERDES0_DIS  0x10000000	/* SERDES disabled on port 0 */
#define E1000_STATUS_SERDES1_DIS  0x20000000	/* SERDES disabled on port 1 */

/* Constants used to interpret the masked PCI-X bus speed. */
#define E1000_STATUS_PCIX_SPEED_66  0x00000000	/* PCI-X bus speed  50-66 MHz */
#define E1000_STATUS_PCIX_SPEED_100 0x00004000	/* PCI-X bus speed  66-100 MHz */
#define E1000_STATUS_PCIX_SPEED_133 0x00008000	/* PCI-X bus speed 100-133 MHz */

/* EEPROM/Flash Control */
#define E1000_EECD_SK        0x00000001	/* EEPROM Clock */
#define E1000_EECD_CS        0x00000002	/* EEPROM Chip Select */
#define E1000_EECD_DI        0x00000004	/* EEPROM Data In */
#define E1000_EECD_DO        0x00000008	/* EEPROM Data Out */
#define E1000_EECD_FWE_MASK  0x00000030
#define E1000_EECD_FWE_DIS   0x00000010	/* Disable FLASH writes */
#define E1000_EECD_FWE_EN    0x00000020	/* Enable FLASH writes */
#define E1000_EECD_FWE_SHIFT 4
#define E1000_EECD_REQ       0x00000040	/* EEPROM Access Request */
#define E1000_EECD_GNT       0x00000080	/* EEPROM Access Grant */
#define E1000_EECD_PRES      0x00000100	/* EEPROM Present */
#define E1000_EECD_SIZE      0x00000200	/* EEPROM Size (0=64 word 1=256 word) */
#define E1000_EECD_ADDR_BITS 0x00000400	/* EEPROM Addressing bits based on type
					 * (0-small, 1-large) */
#define E1000_EECD_TYPE      0x00002000	/* EEPROM Type (1-SPI, 0-Microwire) */
#ifndef E1000_EEPROM_GRANT_ATTEMPTS
#define E1000_EEPROM_GRANT_ATTEMPTS 1000	/* EEPROM # attempts to gain grant */
#endif
#define E1000_EECD_AUTO_RD          0x00000200	/* EEPROM Auto Read done */
#define E1000_EECD_SIZE_EX_MASK     0x00007800	/* EEprom Size */
#define E1000_EECD_SIZE_EX_SHIFT    11
#define E1000_EECD_NVADDS    0x00018000	/* NVM Address Size */
#define E1000_EECD_SELSHAD   0x00020000	/* Select Shadow RAM */
#define E1000_EECD_INITSRAM  0x00040000	/* Initialize Shadow RAM */
#define E1000_EECD_FLUPD     0x00080000	/* Update FLASH */
#define E1000_EECD_AUPDEN    0x00100000	/* Enable Autonomous FLASH update */
#define E1000_EECD_SHADV     0x00200000	/* Shadow RAM Data Valid */
#define E1000_EECD_SEC1VAL   0x00400000	/* Sector One Valid */
#define E1000_EECD_SECVAL_SHIFT      22
#define E1000_STM_OPCODE     0xDB00
#define E1000_HICR_FW_RESET  0xC0

#define E1000_SHADOW_RAM_WORDS     2048
#define E1000_ICH_NVM_SIG_WORD     0x13
#define E1000_ICH_NVM_SIG_MASK     0xC0

/* EEPROM Read */
#define E1000_EERD_START      0x00000001	/* Start Read */
#define E1000_EERD_DONE       0x00000010	/* Read Done */
#define E1000_EERD_ADDR_SHIFT 8
#define E1000_EERD_ADDR_MASK  0x0000FF00	/* Read Address */
#define E1000_EERD_DATA_SHIFT 16
#define E1000_EERD_DATA_MASK  0xFFFF0000	/* Read Data */


/* Extended Device Control */

#define E1000_CTRL_EXT_SDP7_DATA 0x00000080	/* Value of SW Defineable Pin 7 */

/* MDI Control */
#define E1000_MDIC_DATA_MASK 0x0000FFFF
#define E1000_MDIC_REG_MASK  0x001F0000
#define E1000_MDIC_REG_SHIFT 16
#define E1000_MDIC_PHY_MASK  0x03E00000
#define E1000_MDIC_PHY_SHIFT 21
#define E1000_MDIC_OP_WRITE  0x04000000
#define E1000_MDIC_OP_READ   0x08000000
#define E1000_MDIC_READY     0x10000000
#define E1000_MDIC_INT_EN    0x20000000
#define E1000_MDIC_ERROR     0x40000000



/* FIFO Control */
#define E1000_KUMCTRLSTA_FIFO_CTRL_RX_BYPASS   0x00000008
#define E1000_KUMCTRLSTA_FIFO_CTRL_TX_BYPASS   0x00000800





/* Receive Address */
#define E1000_RAH_AV  0x80000000	/* Receive descriptor valid */

/* Interrupt Cause Read */
#define E1000_ICR_TXDW          0x00000001	/* Transmit desc written back */
#define E1000_ICR_TXQE          0x00000002	/* Transmit Queue empty */
#define E1000_ICR_LSC           0x00000004	/* Link Status Change */
#define E1000_ICR_RXSEQ         0x00000008	/* rx sequence error */
#define E1000_ICR_RXDMT0        0x00000010	/* rx desc min. threshold (0) */
#define E1000_ICR_RXO           0x00000040	/* rx overrun */
#define E1000_ICR_RXT0          0x00000080	/* rx timer intr (ring 0) */
#define E1000_ICR_MDAC          0x00000200	/* MDIO access complete */
#define E1000_ICR_RXCFG         0x00000400	/* RX /c/ ordered set */
#define E1000_ICR_GPI_EN0       0x00000800	/* GP Int 0 */
#define E1000_ICR_GPI_EN1       0x00001000	/* GP Int 1 */
#define E1000_ICR_GPI_EN2       0x00002000	/* GP Int 2 */
#define E1000_ICR_GPI_EN3       0x00004000	/* GP Int 3 */
#define E1000_ICR_TXD_LOW       0x00008000
#define E1000_ICR_SRPD          0x00010000
#define E1000_ICR_ACK           0x00020000	/* Receive Ack frame */
#define E1000_ICR_MNG           0x00040000	/* Manageability event */
#define E1000_ICR_DOCK          0x00080000	/* Dock/Undock */
#define E1000_ICR_INT_ASSERTED  0x80000000	/* If this bit asserted, the driver should claim the interrupt */
#define E1000_ICR_RXD_FIFO_PAR0 0x00100000	/* queue 0 Rx descriptor FIFO parity error */
#define E1000_ICR_TXD_FIFO_PAR0 0x00200000	/* queue 0 Tx descriptor FIFO parity error */
#define E1000_ICR_HOST_ARB_PAR  0x00400000	/* host arb read buffer parity error */
#define E1000_ICR_PB_PAR        0x00800000	/* packet buffer parity error */
#define E1000_ICR_RXD_FIFO_PAR1 0x01000000	/* queue 1 Rx descriptor FIFO parity error */
#define E1000_ICR_TXD_FIFO_PAR1 0x02000000	/* queue 1 Tx descriptor FIFO parity error */
#define E1000_ICR_ALL_PARITY    0x03F00000	/* all parity error bits */
#define E1000_ICR_DSW           0x00000020	/* FW changed the status of DISSW bit in the FWSM */
#define E1000_ICR_PHYINT        0x00001000	/* LAN connected device generates an interrupt */
#define E1000_ICR_EPRST         0x00100000	/* ME hardware reset occurs */

/* Interrupt Cause Set */
#define E1000_ICS_TXDW      E1000_ICR_TXDW	/* Transmit desc written back */
#define E1000_ICS_TXQE      E1000_ICR_TXQE	/* Transmit Queue empty */
#define E1000_ICS_LSC       E1000_ICR_LSC	/* Link Status Change */
#define E1000_ICS_RXSEQ     E1000_ICR_RXSEQ	/* rx sequence error */
#define E1000_ICS_RXDMT0    E1000_ICR_RXDMT0	/* rx desc min. threshold */
#define E1000_ICS_RXO       E1000_ICR_RXO	/* rx overrun */
#define E1000_ICS_RXT0      E1000_ICR_RXT0	/* rx timer intr */
#define E1000_ICS_MDAC      E1000_ICR_MDAC	/* MDIO access complete */
#define E1000_ICS_RXCFG     E1000_ICR_RXCFG	/* RX /c/ ordered set */
#define E1000_ICS_GPI_EN0   E1000_ICR_GPI_EN0	/* GP Int 0 */
#define E1000_ICS_GPI_EN1   E1000_ICR_GPI_EN1	/* GP Int 1 */
#define E1000_ICS_GPI_EN2   E1000_ICR_GPI_EN2	/* GP Int 2 */
#define E1000_ICS_GPI_EN3   E1000_ICR_GPI_EN3	/* GP Int 3 */
#define E1000_ICS_TXD_LOW   E1000_ICR_TXD_LOW
#define E1000_ICS_SRPD      E1000_ICR_SRPD
#define E1000_ICS_ACK       E1000_ICR_ACK	/* Receive Ack frame */
#define E1000_ICS_MNG       E1000_ICR_MNG	/* Manageability event */
#define E1000_ICS_DOCK      E1000_ICR_DOCK	/* Dock/Undock */
#define E1000_ICS_RXD_FIFO_PAR0 E1000_ICR_RXD_FIFO_PAR0	/* queue 0 Rx descriptor FIFO parity error */
#define E1000_ICS_TXD_FIFO_PAR0 E1000_ICR_TXD_FIFO_PAR0	/* queue 0 Tx descriptor FIFO parity error */
#define E1000_ICS_HOST_ARB_PAR  E1000_ICR_HOST_ARB_PAR	/* host arb read buffer parity error */
#define E1000_ICS_PB_PAR        E1000_ICR_PB_PAR	/* packet buffer parity error */
#define E1000_ICS_RXD_FIFO_PAR1 E1000_ICR_RXD_FIFO_PAR1	/* queue 1 Rx descriptor FIFO parity error */
#define E1000_ICS_TXD_FIFO_PAR1 E1000_ICR_TXD_FIFO_PAR1	/* queue 1 Tx descriptor FIFO parity error */
#define E1000_ICS_DSW       E1000_ICR_DSW
#define E1000_ICS_PHYINT    E1000_ICR_PHYINT
#define E1000_ICS_EPRST     E1000_ICR_EPRST

/* Interrupt Mask Set */
#define E1000_IMS_TXDW      E1000_ICR_TXDW	/* Transmit desc written back */
#define E1000_IMS_TXQE      E1000_ICR_TXQE	/* Transmit Queue empty */
#define E1000_IMS_LSC       E1000_ICR_LSC	/* Link Status Change */
#define E1000_IMS_RXSEQ     E1000_ICR_RXSEQ	/* rx sequence error */
#define E1000_IMS_RXDMT0    E1000_ICR_RXDMT0	/* rx desc min. threshold */
#define E1000_IMS_RXO       E1000_ICR_RXO	/* rx overrun */
#define E1000_IMS_RXT0      E1000_ICR_RXT0	/* rx timer intr */
#define E1000_IMS_MDAC      E1000_ICR_MDAC	/* MDIO access complete */
#define E1000_IMS_RXCFG     E1000_ICR_RXCFG	/* RX /c/ ordered set */
#define E1000_IMS_GPI_EN0   E1000_ICR_GPI_EN0	/* GP Int 0 */
#define E1000_IMS_GPI_EN1   E1000_ICR_GPI_EN1	/* GP Int 1 */
#define E1000_IMS_GPI_EN2   E1000_ICR_GPI_EN2	/* GP Int 2 */
#define E1000_IMS_GPI_EN3   E1000_ICR_GPI_EN3	/* GP Int 3 */
#define E1000_IMS_TXD_LOW   E1000_ICR_TXD_LOW
#define E1000_IMS_SRPD      E1000_ICR_SRPD
#define E1000_IMS_ACK       E1000_ICR_ACK	/* Receive Ack frame */
#define E1000_IMS_MNG       E1000_ICR_MNG	/* Manageability event */
#define E1000_IMS_DOCK      E1000_ICR_DOCK	/* Dock/Undock */
#define E1000_IMS_RXD_FIFO_PAR0 E1000_ICR_RXD_FIFO_PAR0	/* queue 0 Rx descriptor FIFO parity error */
#define E1000_IMS_TXD_FIFO_PAR0 E1000_ICR_TXD_FIFO_PAR0	/* queue 0 Tx descriptor FIFO parity error */
#define E1000_IMS_HOST_ARB_PAR  E1000_ICR_HOST_ARB_PAR	/* host arb read buffer parity error */
#define E1000_IMS_PB_PAR        E1000_ICR_PB_PAR	/* packet buffer parity error */
#define E1000_IMS_RXD_FIFO_PAR1 E1000_ICR_RXD_FIFO_PAR1	/* queue 1 Rx descriptor FIFO parity error */
#define E1000_IMS_TXD_FIFO_PAR1 E1000_ICR_TXD_FIFO_PAR1	/* queue 1 Tx descriptor FIFO parity error */
#define E1000_IMS_DSW       E1000_ICR_DSW
#define E1000_IMS_PHYINT    E1000_ICR_PHYINT
#define E1000_IMS_EPRST     E1000_ICR_EPRST

/* Interrupt Mask Clear */
#define E1000_IMC_TXDW      E1000_ICR_TXDW	/* Transmit desc written back */
#define E1000_IMC_TXQE      E1000_ICR_TXQE	/* Transmit Queue empty */
#define E1000_IMC_LSC       E1000_ICR_LSC	/* Link Status Change */
#define E1000_IMC_RXSEQ     E1000_ICR_RXSEQ	/* rx sequence error */
#define E1000_IMC_RXDMT0    E1000_ICR_RXDMT0	/* rx desc min. threshold */
#define E1000_IMC_RXO       E1000_ICR_RXO	/* rx overrun */
#define E1000_IMC_RXT0      E1000_ICR_RXT0	/* rx timer intr */
#define E1000_IMC_MDAC      E1000_ICR_MDAC	/* MDIO access complete */
#define E1000_IMC_RXCFG     E1000_ICR_RXCFG	/* RX /c/ ordered set */
#define E1000_IMC_GPI_EN0   E1000_ICR_GPI_EN0	/* GP Int 0 */
#define E1000_IMC_GPI_EN1   E1000_ICR_GPI_EN1	/* GP Int 1 */
#define E1000_IMC_GPI_EN2   E1000_ICR_GPI_EN2	/* GP Int 2 */
#define E1000_IMC_GPI_EN3   E1000_ICR_GPI_EN3	/* GP Int 3 */
#define E1000_IMC_TXD_LOW   E1000_ICR_TXD_LOW
#define E1000_IMC_SRPD      E1000_ICR_SRPD
#define E1000_IMC_ACK       E1000_ICR_ACK	/* Receive Ack frame */
#define E1000_IMC_MNG       E1000_ICR_MNG	/* Manageability event */
#define E1000_IMC_DOCK      E1000_ICR_DOCK	/* Dock/Undock */
#define E1000_IMC_RXD_FIFO_PAR0 E1000_ICR_RXD_FIFO_PAR0	/* queue 0 Rx descriptor FIFO parity error */
#define E1000_IMC_TXD_FIFO_PAR0 E1000_ICR_TXD_FIFO_PAR0	/* queue 0 Tx descriptor FIFO parity error */
#define E1000_IMC_HOST_ARB_PAR  E1000_ICR_HOST_ARB_PAR	/* host arb read buffer parity error */
#define E1000_IMC_PB_PAR        E1000_ICR_PB_PAR	/* packet buffer parity error */
#define E1000_IMC_RXD_FIFO_PAR1 E1000_ICR_RXD_FIFO_PAR1	/* queue 1 Rx descriptor FIFO parity error */
#define E1000_IMC_TXD_FIFO_PAR1 E1000_ICR_TXD_FIFO_PAR1	/* queue 1 Tx descriptor FIFO parity error */
#define E1000_IMC_DSW       E1000_ICR_DSW
#define E1000_IMC_PHYINT    E1000_ICR_PHYINT
#define E1000_IMC_EPRST     E1000_ICR_EPRST

/* Receive Control */
#define E1000_RCTL_RST            0x00000001	/* Software reset */
#define E1000_RCTL_EN             0x00000002	/* enable */
#define E1000_RCTL_SBP            0x00000004	/* store bad packet */
#define E1000_RCTL_UPE            0x00000008	/* unicast promiscuous enable */
#define E1000_RCTL_MPE            0x00000010	/* multicast promiscuous enab */
#define E1000_RCTL_LPE            0x00000020	/* long packet enable */
#define E1000_RCTL_LBM_NO         0x00000000	/* no loopback mode */
#define E1000_RCTL_LBM_MAC        0x00000040	/* MAC loopback mode */
#define E1000_RCTL_LBM_SLP        0x00000080	/* serial link loopback mode */
#define E1000_RCTL_LBM_TCVR       0x000000C0	/* tcvr loopback mode */
#define E1000_RCTL_DTYP_MASK      0x00000C00	/* Descriptor type mask */
#define E1000_RCTL_DTYP_PS        0x00000400	/* Packet Split descriptor */
#define E1000_RCTL_RDMTS_HALF     0x00000000	/* rx desc min threshold size */
#define E1000_RCTL_RDMTS_QUAT     0x00000100	/* rx desc min threshold size */
#define E1000_RCTL_RDMTS_EIGTH    0x00000200	/* rx desc min threshold size */
#define E1000_RCTL_MO_SHIFT       12	/* multicast offset shift */
#define E1000_RCTL_MO_0           0x00000000	/* multicast offset 11:0 */
#define E1000_RCTL_MO_1           0x00001000	/* multicast offset 12:1 */
#define E1000_RCTL_MO_2           0x00002000	/* multicast offset 13:2 */
#define E1000_RCTL_MO_3           0x00003000	/* multicast offset 15:4 */
#define E1000_RCTL_MDR            0x00004000	/* multicast desc ring 0 */
#define E1000_RCTL_BAM            0x00008000	/* broadcast enable */
/* these buffer sizes are valid if E1000_RCTL_BSEX is 0 */
#define E1000_RCTL_SZ_2048        0x00000000	/* rx buffer size 2048 */
#define E1000_RCTL_SZ_1024        0x00010000	/* rx buffer size 1024 */
#define E1000_RCTL_SZ_512         0x00020000	/* rx buffer size 512 */
#define E1000_RCTL_SZ_256         0x00030000	/* rx buffer size 256 */
/* these buffer sizes are valid if E1000_RCTL_BSEX is 1 */
#define E1000_RCTL_SZ_16384       0x00010000	/* rx buffer size 16384 */
#define E1000_RCTL_SZ_8192        0x00020000	/* rx buffer size 8192 */
#define E1000_RCTL_SZ_4096        0x00030000	/* rx buffer size 4096 */
#define E1000_RCTL_VFE            0x00040000	/* vlan filter enable */
#define E1000_RCTL_CFIEN          0x00080000	/* canonical form enable */
#define E1000_RCTL_CFI            0x00100000	/* canonical form indicator */
#define E1000_RCTL_DPF            0x00400000	/* discard pause frames */
#define E1000_RCTL_PMCF           0x00800000	/* pass MAC control frames */
#define E1000_RCTL_BSEX           0x02000000	/* Buffer size extension */
#define E1000_RCTL_SECRC          0x04000000	/* Strip Ethernet CRC */
#define E1000_RCTL_FLXBUF_MASK    0x78000000	/* Flexible buffer size */
#define E1000_RCTL_FLXBUF_SHIFT   27	/* Flexible buffer shift */




/* Receive Descriptor */
#define E1000_RDT_DELAY 0x0000ffff	/* Delay timer (1=1024us) */
#define E1000_RDT_FPDB  0x80000000	/* Flush descriptor block */
#define E1000_RDLEN_LEN 0x0007ff80	/* descriptor length */
#define E1000_RDH_RDH   0x0000ffff	/* receive descriptor head */
#define E1000_RDT_RDT   0x0000ffff	/* receive descriptor tail */

/* Flow Control */
#define E1000_FCRTH_RTH  0x0000FFF8	/* Mask Bits[15:3] for RTH */
#define E1000_FCRTH_XFCE 0x80000000	/* External Flow Control Enable */
#define E1000_FCRTL_RTL  0x0000FFF8	/* Mask Bits[15:3] for RTL */
#define E1000_FCRTL_XONE 0x80000000	/* Enable XON frame transmission */



/* Receive Descriptor Control */
#define E1000_RXDCTL_PTHRESH 0x0000003F	/* RXDCTL Prefetch Threshold */
#define E1000_RXDCTL_HTHRESH 0x00003F00	/* RXDCTL Host Threshold */
#define E1000_RXDCTL_WTHRESH 0x003F0000	/* RXDCTL Writeback Threshold */
#define E1000_RXDCTL_GRAN    0x01000000	/* RXDCTL Granularity */

/* Transmit Descriptor Control */
#define E1000_TXDCTL_PTHRESH 0x0000003F	/* TXDCTL Prefetch Threshold */
#define E1000_TXDCTL_HTHRESH 0x00003F00	/* TXDCTL Host Threshold */
#define E1000_TXDCTL_WTHRESH 0x003F0000	/* TXDCTL Writeback Threshold */
#define E1000_TXDCTL_GRAN    0x01000000	/* TXDCTL Granularity */
#define E1000_TXDCTL_LWTHRESH 0xFE000000	/* TXDCTL Low Threshold */
#define E1000_TXDCTL_FULL_TX_DESC_WB 0x01010000	/* GRAN=1, WTHRESH=1 */
#define E1000_TXDCTL_COUNT_DESC 0x00400000	/* Enable the counting of desc.
						   still to be processed. */
/* Transmit Configuration Word */
#define E1000_TXCW_FD         0x00000020	/* TXCW full duplex */
#define E1000_TXCW_HD         0x00000040	/* TXCW half duplex */
#define E1000_TXCW_PAUSE      0x00000080	/* TXCW sym pause request */
#define E1000_TXCW_ASM_DIR    0x00000100	/* TXCW astm pause direction */
#define E1000_TXCW_PAUSE_MASK 0x00000180	/* TXCW pause request mask */
#define E1000_TXCW_RF         0x00003000	/* TXCW remote fault */
#define E1000_TXCW_NP         0x00008000	/* TXCW next page */
#define E1000_TXCW_CW         0x0000ffff	/* TxConfigWord mask */
#define E1000_TXCW_TXC        0x40000000	/* Transmit Config control */
#define E1000_TXCW_ANE        0x80000000	/* Auto-neg enable */

/* Receive Configuration Word */
#define E1000_RXCW_CW    0x0000ffff	/* RxConfigWord mask */
#define E1000_RXCW_NC    0x04000000	/* Receive config no carrier */
#define E1000_RXCW_IV    0x08000000	/* Receive config invalid */
#define E1000_RXCW_CC    0x10000000	/* Receive config change */
#define E1000_RXCW_C     0x20000000	/* Receive config */
#define E1000_RXCW_SYNCH 0x40000000	/* Receive config synch */
#define E1000_RXCW_ANC   0x80000000	/* Auto-neg complete */

/* Transmit Control */
#define E1000_TCTL_RST    0x00000001	/* software reset */
#define E1000_TCTL_EN     0x00000002	/* enable tx */
#define E1000_TCTL_BCE    0x00000004	/* busy check enable */
#define E1000_TCTL_PSP    0x00000008	/* pad short packets */
#define E1000_TCTL_CT     0x00000ff0	/* collision threshold */
#define E1000_TCTL_COLD   0x003ff000	/* collision distance */
#define E1000_TCTL_SWXOFF 0x00400000	/* SW Xoff transmission */
#define E1000_TCTL_PBE    0x00800000	/* Packet Burst Enable */
#define E1000_TCTL_RTLC   0x01000000	/* Re-transmit on late collision */
#define E1000_TCTL_NRTU   0x02000000	/* No Re-transmit on underrun */
#define E1000_TCTL_MULR   0x10000000	/* Multiple request support */
/* Extended Transmit Control */
#define E1000_TCTL_EXT_BST_MASK  0x000003FF	/* Backoff Slot Time */
#define E1000_TCTL_EXT_GCEX_MASK 0x000FFC00	/* Gigabit Carry Extend Padding */

/* Receive Checksum Control */
#define E1000_RXCSUM_PCSS_MASK 0x000000FF	/* Packet Checksum Start */
#define E1000_RXCSUM_IPOFL     0x00000100	/* IPv4 checksum offload */
#define E1000_RXCSUM_TUOFL     0x00000200	/* TCP / UDP checksum offload */
#define E1000_RXCSUM_IPV6OFL   0x00000400	/* IPv6 checksum offload */
#define E1000_RXCSUM_IPPCSE    0x00001000	/* IP payload checksum enable */
#define E1000_RXCSUM_PCSD      0x00002000	/* packet checksum disabled */

/* Multiple Receive Queue Control */
#define E1000_MRQC_ENABLE_MASK              0x00000003
#define E1000_MRQC_ENABLE_RSS_2Q            0x00000001
#define E1000_MRQC_ENABLE_RSS_INT           0x00000004
#define E1000_MRQC_RSS_FIELD_MASK           0xFFFF0000
#define E1000_MRQC_RSS_FIELD_IPV4_TCP       0x00010000
#define E1000_MRQC_RSS_FIELD_IPV4           0x00020000
#define E1000_MRQC_RSS_FIELD_IPV6_TCP_EX    0x00040000
#define E1000_MRQC_RSS_FIELD_IPV6_EX        0x00080000
#define E1000_MRQC_RSS_FIELD_IPV6           0x00100000
#define E1000_MRQC_RSS_FIELD_IPV6_TCP       0x00200000

/* Definitions for power management and wakeup registers */
/* Wake Up Control */
#define E1000_WUC_APME       0x00000001	/* APM Enable */
#define E1000_WUC_PME_EN     0x00000002	/* PME Enable */
#define E1000_WUC_PME_STATUS 0x00000004	/* PME Status */
#define E1000_WUC_APMPME     0x00000008	/* Assert PME on APM Wakeup */
#define E1000_WUC_SPM        0x80000000	/* Enable SPM */

/* Wake Up Filter Control */
#define E1000_WUFC_LNKC 0x00000001	/* Link Status Change Wakeup Enable */
#define E1000_WUFC_MAG  0x00000002	/* Magic Packet Wakeup Enable */
#define E1000_WUFC_EX   0x00000004	/* Directed Exact Wakeup Enable */
#define E1000_WUFC_MC   0x00000008	/* Directed Multicast Wakeup Enable */
#define E1000_WUFC_BC   0x00000010	/* Broadcast Wakeup Enable */
#define E1000_WUFC_ARP  0x00000020	/* ARP Request Packet Wakeup Enable */
#define E1000_WUFC_IPV4 0x00000040	/* Directed IPv4 Packet Wakeup Enable */
#define E1000_WUFC_IPV6 0x00000080	/* Directed IPv6 Packet Wakeup Enable */
#define E1000_WUFC_IGNORE_TCO      0x00008000	/* Ignore WakeOn TCO packets */
#define E1000_WUFC_FLX0 0x00010000	/* Flexible Filter 0 Enable */
#define E1000_WUFC_FLX1 0x00020000	/* Flexible Filter 1 Enable */
#define E1000_WUFC_FLX2 0x00040000	/* Flexible Filter 2 Enable */
#define E1000_WUFC_FLX3 0x00080000	/* Flexible Filter 3 Enable */
#define E1000_WUFC_ALL_FILTERS 0x000F00FF	/* Mask for all wakeup filters */
#define E1000_WUFC_FLX_OFFSET 16	/* Offset to the Flexible Filters bits */
#define E1000_WUFC_FLX_FILTERS 0x000F0000	/* Mask for the 4 flexible filters */



/* Management Control */
#define E1000_MANC_SMBUS_EN      0x00000001	/* SMBus Enabled - RO */
#define E1000_MANC_ASF_EN        0x00000002	/* ASF Enabled - RO */
#define E1000_MANC_R_ON_FORCE    0x00000004	/* Reset on Force TCO - RO */
#define E1000_MANC_RMCP_EN       0x00000100	/* Enable RCMP 026Fh Filtering */
#define E1000_MANC_0298_EN       0x00000200	/* Enable RCMP 0298h Filtering */
#define E1000_MANC_IPV4_EN       0x00000400	/* Enable IPv4 */
#define E1000_MANC_IPV6_EN       0x00000800	/* Enable IPv6 */
#define E1000_MANC_SNAP_EN       0x00001000	/* Accept LLC/SNAP */
#define E1000_MANC_ARP_EN        0x00002000	/* Enable ARP Request Filtering */
#define E1000_MANC_NEIGHBOR_EN   0x00004000	/* Enable Neighbor Discovery
						 * Filtering */
#define E1000_MANC_ARP_RES_EN    0x00008000	/* Enable ARP response Filtering */
#define E1000_MANC_TCO_RESET     0x00010000	/* TCO Reset Occurred */
#define E1000_MANC_RCV_TCO_EN    0x00020000	/* Receive TCO Packets Enabled */
#define E1000_MANC_REPORT_STATUS 0x00040000	/* Status Reporting Enabled */
#define E1000_MANC_RCV_ALL       0x00080000	/* Receive All Enabled */
#define E1000_MANC_BLK_PHY_RST_ON_IDE   0x00040000	/* Block phy resets */
#define E1000_MANC_EN_MAC_ADDR_FILTER   0x00100000	/* Enable MAC address
							 * filtering */
#define E1000_MANC_EN_MNG2HOST   0x00200000	/* Enable MNG packets to host
						 * memory */
#define E1000_MANC_EN_IP_ADDR_FILTER    0x00400000	/* Enable IP address
							 * filtering */
#define E1000_MANC_EN_XSUM_FILTER   0x00800000	/* Enable checksum filtering */
#define E1000_MANC_BR_EN         0x01000000	/* Enable broadcast filtering */
#define E1000_MANC_SMB_REQ       0x01000000	/* SMBus Request */
#define E1000_MANC_SMB_GNT       0x02000000	/* SMBus Grant */
#define E1000_MANC_SMB_CLK_IN    0x04000000	/* SMBus Clock In */
#define E1000_MANC_SMB_DATA_IN   0x08000000	/* SMBus Data In */
#define E1000_MANC_SMB_DATA_OUT  0x10000000	/* SMBus Data Out */
#define E1000_MANC_SMB_CLK_OUT   0x20000000	/* SMBus Clock Out */

#define E1000_MANC_SMB_DATA_OUT_SHIFT  28	/* SMBus Data Out Shift */
#define E1000_MANC_SMB_CLK_OUT_SHIFT   29	/* SMBus Clock Out Shift */








/* Host Interface Control Register */
#define E1000_HICR_EN           0x00000001	/* Enable Bit - RO */
#define E1000_HICR_C            0x00000002	/* Driver sets this bit when done
						 * to put command in RAM */
#define E1000_HICR_SV           0x00000004	/* Status Validity */
#define E1000_HICR_FWR          0x00000080	/* FW reset. Set by the Host */

/* Host Interface Command Interface - Address range 0x8800-0x8EFF */
#define E1000_HI_MAX_DATA_LENGTH         252	/* Host Interface data length */
#define E1000_HI_MAX_BLOCK_BYTE_LENGTH  1792	/* Number of bytes in range */
#define E1000_HI_MAX_BLOCK_DWORD_LENGTH  448	/* Number of dwords in range */
#define E1000_HI_COMMAND_TIMEOUT         500	/* Time in ms to process HI command */

struct e1000_host_command_header {
	u8 command_id;
	u8 command_length;
	u8 command_options;	/* I/F bits for command, status for return */
	u8 checksum;
};
struct e1000_host_command_info {
	struct e1000_host_command_header command_header;	/* Command Head/Command Result Head has 4 bytes */
	u8 command_data[E1000_HI_MAX_DATA_LENGTH];	/* Command data can length 0..252 */
};


/* Host SMB register #1 */
#define E1000_HSMC1R_CLKIN      E1000_HSMC0R_CLKIN
#define E1000_HSMC1R_DATAIN     E1000_HSMC0R_DATAIN
#define E1000_HSMC1R_DATAOUT    E1000_HSMC0R_DATAOUT
#define E1000_HSMC1R_CLKOUT     E1000_HSMC0R_CLKOUT

/* FW Status Register */
#define E1000_FWSTS_FWS_MASK    0x000000FF	/* FW Status */

/* Wake Up Packet Length */
#define E1000_WUPL_LENGTH_MASK 0x0FFF	/* Only the lower 12 bits are valid */

#define E1000_MDALIGN          4096





/* EEPROM Commands - Microwire */
#define EEPROM_READ_OPCODE_MICROWIRE  0x6	/* EEPROM read opcode */
#define EEPROM_WRITE_OPCODE_MICROWIRE 0x5	/* EEPROM write opcode */
#define EEPROM_ERASE_OPCODE_MICROWIRE 0x7	/* EEPROM erase opcode */
#define EEPROM_EWEN_OPCODE_MICROWIRE  0x13	/* EEPROM erase/write enable */
#define EEPROM_EWDS_OPCODE_MICROWIRE  0x10	/* EEPROM erase/write disable */


/* EEPROM Size definitions */
#define EEPROM_WORD_SIZE_SHIFT  6
#define EEPROM_SIZE_SHIFT       10
#define EEPROM_SIZE_MASK        0x1C00

/* EEPROM Word Offsets */
#define EEPROM_COMPAT                 0x0003
#define EEPROM_ID_LED_SETTINGS        0x0004
#define EEPROM_VERSION                0x0005
#define EEPROM_SERDES_AMPLITUDE       0x0006	/* For SERDES output amplitude adjustment. */
#define EEPROM_PHY_CLASS_WORD         0x0007
#define EEPROM_INIT_CONTROL1_REG      0x000A
#define EEPROM_INIT_CONTROL2_REG      0x000F
#define EEPROM_SWDEF_PINS_CTRL_PORT_1 0x0010
#define EEPROM_INIT_CONTROL3_PORT_B   0x0014
#define EEPROM_INIT_3GIO_3            0x001A
#define EEPROM_SWDEF_PINS_CTRL_PORT_0 0x0020
#define EEPROM_INIT_CONTROL3_PORT_A   0x0024
#define EEPROM_CFG                    0x0012
#define EEPROM_FLASH_VERSION          0x0032
#define EEPROM_CHECKSUM_REG           0x003F




/* Mask bit for PHY class in Word 7 of the EEPROM */
#define EEPROM_PHY_CLASS_A   0x8000

/* Mask bits for fields in Word 0x0a of the EEPROM */
#define EEPROM_WORD0A_ILOS   0x0010
#define EEPROM_WORD0A_SWDPIO 0x01E0
#define EEPROM_WORD0A_LRST   0x0200
#define EEPROM_WORD0A_FD     0x0400
#define EEPROM_WORD0A_66MHZ  0x0800

/* Mask bits for fields in Word 0x0f of the EEPROM */
#define EEPROM_WORD0F_PAUSE_MASK 0x3000
#define EEPROM_WORD0F_PAUSE      0x1000
#define EEPROM_WORD0F_ASM_DIR    0x2000
#define EEPROM_WORD0F_ANE        0x0800
#define EEPROM_WORD0F_SWPDIO_EXT 0x00F0
#define EEPROM_WORD0F_LPLU       0x0001

/* Mask bits for fields in Word 0x10/0x20 of the EEPROM */
#define EEPROM_WORD1020_GIGA_DISABLE         0x0010
#define EEPROM_WORD1020_GIGA_DISABLE_NON_D0A 0x0008

/* Mask bits for fields in Word 0x1a of the EEPROM */
#define EEPROM_WORD1A_ASPM_MASK  0x000C

/* For checksumming, the sum of all words in the EEPROM should equal 0xBABA. */
#define EEPROM_SUM 0xBABA

/* EEPROM Map defines (WORD OFFSETS)*/
#define EEPROM_NODE_ADDRESS_BYTE_0 0
#define EEPROM_PBA_BYTE_1          8

#define EEPROM_RESERVED_WORD          0xFFFF

/* EEPROM Map Sizes (Byte Counts) */
#define PBA_SIZE 4

/* Collision related configuration parameters */
#define E1000_COLLISION_THRESHOLD       15
#define E1000_CT_SHIFT                  4
/* Collision distance is a 0-based value that applies to
 * half-duplex-capable hardware only. */
#define E1000_COLLISION_DISTANCE        63
#define E1000_COLLISION_DISTANCE_82542  64
#define E1000_FDX_COLLISION_DISTANCE    E1000_COLLISION_DISTANCE
#define E1000_HDX_COLLISION_DISTANCE    E1000_COLLISION_DISTANCE
#define E1000_COLD_SHIFT                12

/* Number of Transmit and Receive Descriptors must be a multiple of 8 */
#define REQ_TX_DESCRIPTOR_MULTIPLE  8
#define REQ_RX_DESCRIPTOR_MULTIPLE  8

/* Default values for the transmit IPG register */
#define DEFAULT_82542_TIPG_IPGT        10
#define DEFAULT_82543_TIPG_IPGT_FIBER  9
#define DEFAULT_82543_TIPG_IPGT_COPPER 8


#define DEFAULT_82542_TIPG_IPGR1 2
#define DEFAULT_82543_TIPG_IPGR1 8
#define E1000_TIPG_IPGR1_SHIFT  10

#define DEFAULT_82542_TIPG_IPGR2 10
#define DEFAULT_82543_TIPG_IPGR2 6
#define E1000_TIPG_IPGR2_SHIFT  20

#define E1000_TXDMAC_DPP 0x00000001





/* PBA constants */
#define E1000_PBA_8K 0x0008	/* 8KB, default Rx allocation */
#define E1000_PBA_12K 0x000C	/* 12KB, default Rx allocation */
#define E1000_PBA_16K 0x0010	/* 16KB, default TX allocation */
#define E1000_PBA_20K 0x0014
#define E1000_PBA_22K 0x0016
#define E1000_PBA_24K 0x0018
#define E1000_PBA_30K 0x001E
#define E1000_PBA_32K 0x0020
#define E1000_PBA_34K 0x0022
#define E1000_PBA_38K 0x0026
#define E1000_PBA_40K 0x0028
#define E1000_PBA_48K 0x0030	/* 48KB, default RX allocation */

#define E1000_PBS_16K E1000_PBA_16K

/* Flow Control Constants */
#define FLOW_CONTROL_ADDRESS_LOW  0x00C28001
#define FLOW_CONTROL_ADDRESS_HIGH 0x00000100
#define FLOW_CONTROL_TYPE         0x8808





/* Number of bits required to shift right the "pause" bits from the
 * EEPROM (bits 13:12) to the "pause" (bits 8:7) field in the TXCW register.
 */
#define PAUSE_SHIFT 5

/* Number of bits required to shift left the "SWDPIO" bits from the
 * EEPROM (bits 8:5) to the "SWDPIO" (bits 25:22) field in the CTRL register.
 */
#define SWDPIO_SHIFT 17

/* Number of bits required to shift left the "SWDPIO_EXT" bits from the
 * EEPROM word F (bits 7:4) to the bits 11:8 of The Extended CTRL register.
 */
#define SWDPIO__EXT_SHIFT 4

/* Number of bits required to shift left the "ILOS" bit from the EEPROM
 * (bit 4) to the "ILOS" (bit 7) field in the CTRL register.
 */
#define ILOS_SHIFT  3

#define RECEIVE_BUFFER_ALIGN_SIZE  (256)

/* Number of milliseconds we wait for auto-negotiation to complete */
#define LINK_UP_TIMEOUT             500

/* Number of milliseconds we wait for Eeprom auto read bit done after MAC reset */
#define AUTO_READ_DONE_TIMEOUT      10
/* Number of milliseconds we wait for PHY configuration done after MAC reset */
#define PHY_CFG_TIMEOUT             100

#define E1000_TX_BUFFER_SIZE ((u32)1514)




/* Structures, enums, and macros for the PHY */



/* PHY 1000 MII Register/Bit Definitions */
/* PHY Registers defined by IEEE */
#define PHY_CTRL         0x00	/* Control Register */
#define PHY_STATUS       0x01	/* Status Register */
#define PHY_ID1          0x02	/* Phy Id Reg (word 1) */
#define PHY_ID2          0x03	/* Phy Id Reg (word 2) */
#define PHY_AUTONEG_ADV  0x04	/* Autoneg Advertisement */
#define PHY_LP_ABILITY   0x05	/* Link Partner Ability (Base Page) */
#define PHY_AUTONEG_EXP  0x06	/* Autoneg Expansion Reg */
#define PHY_NEXT_PAGE_TX 0x07	/* Next Page TX */
#define PHY_LP_NEXT_PAGE 0x08	/* Link Partner Next Page */
#define PHY_1000T_CTRL   0x09	/* 1000Base-T Control Reg */
#define PHY_1000T_STATUS 0x0A	/* 1000Base-T Status Reg */
#define PHY_EXT_STATUS   0x0F	/* Extended Status Reg */

#define MAX_PHY_REG_ADDRESS        0x1F	/* 5 bit address bus (0-0x1F) */
#define MAX_PHY_MULTI_PAGE_REG     0xF	/* Registers equal on all pages */

/* M88E1000 Specific Registers */
#define M88E1000_PHY_SPEC_CTRL     0x10	/* PHY Specific Control Register */
#define M88E1000_PHY_SPEC_STATUS   0x11	/* PHY Specific Status Register */
#define M88E1000_INT_ENABLE        0x12	/* Interrupt Enable Register */
#define M88E1000_INT_STATUS        0x13	/* Interrupt Status Register */
#define M88E1000_EXT_PHY_SPEC_CTRL 0x14	/* Extended PHY Specific Control */
#define M88E1000_RX_ERR_CNTR       0x15	/* Receive Error Counter */

#define M88E1000_PHY_EXT_CTRL      0x1A	/* PHY extend control register */
#define M88E1000_PHY_PAGE_SELECT   0x1D	/* Reg 29 for page number setting */
#define M88E1000_PHY_GEN_CONTROL   0x1E	/* Its meaning depends on reg 29 */
#define M88E1000_PHY_VCO_REG_BIT8  0x100	/* Bits 8 & 11 are adjusted for */
#define M88E1000_PHY_VCO_REG_BIT11 0x800	/* improved BER performance */

#define IGP01E1000_IEEE_REGS_PAGE  0x0000
#define IGP01E1000_IEEE_RESTART_AUTONEG 0x3300
#define IGP01E1000_IEEE_FORCE_GIGA      0x0140

/* IGP01E1000 Specific Registers */
#define IGP01E1000_PHY_PORT_CONFIG 0x10	/* PHY Specific Port Config Register */
#define IGP01E1000_PHY_PORT_STATUS 0x11	/* PHY Specific Status Register */
#define IGP01E1000_PHY_PORT_CTRL   0x12	/* PHY Specific Control Register */
#define IGP01E1000_PHY_LINK_HEALTH 0x13	/* PHY Link Health Register */
#define IGP01E1000_GMII_FIFO       0x14	/* GMII FIFO Register */
#define IGP01E1000_PHY_CHANNEL_QUALITY 0x15	/* PHY Channel Quality Register */
#define IGP02E1000_PHY_POWER_MGMT      0x19
#define IGP01E1000_PHY_PAGE_SELECT     0x1F	/* PHY Page Select Core Register */





#define IGP01E1000_PHY_DSP_FFE_DEFAULT      0x002A
/* IGP01E1000 PCS Initialization register - stores the polarity status when
 * speed = 1000 Mbps. */
#define IGP01E1000_PHY_PCS_INIT_REG  0x00B4
#define IGP01E1000_PHY_PCS_CTRL_REG  0x00B5

#define IGP01E1000_ANALOG_REGS_PAGE  0x20C0

/* PHY Control Register */
#define MII_CR_SPEED_SELECT_MSB 0x0040	/* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_COLL_TEST_ENABLE 0x0080	/* Collision test enable */
#define MII_CR_FULL_DUPLEX      0x0100	/* FDX =1, half duplex =0 */
#define MII_CR_RESTART_AUTO_NEG 0x0200	/* Restart auto negotiation */
#define MII_CR_ISOLATE          0x0400	/* Isolate PHY from MII */
#define MII_CR_POWER_DOWN       0x0800	/* Power down */
#define MII_CR_AUTO_NEG_EN      0x1000	/* Auto Neg Enable */
#define MII_CR_SPEED_SELECT_LSB 0x2000	/* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_LOOPBACK         0x4000	/* 0 = normal, 1 = loopback */
#define MII_CR_RESET            0x8000	/* 0 = normal, 1 = PHY reset */







/* Miscellaneous PHY bit definitions. */


#define ADVERTISE_1000_FULL 0x0020


#endif /* _E1000_HW_H_ */
