/* vendor and product id */
#define MCBA_MODULE_NAME    "mcba_usb"
#define MCBA_VENDOR_ID      0x04d8
#define MCBA_PRODUCT_ID     0x0a30

/* driver constants */
#define MCBA_MAX_RX_URBS         20
#define MCBA_MAX_TX_URBS         20
#define MCBA_CTX_FREE            MCBA_MAX_TX_URBS

/* RX buffer must be bigger than msg size since at the
 * beggining USB messages are stacked.
*/
#define MCBA_USB_RX_BUFF_SIZE    64
#define MCBA_USB_TX_BUFF_SIZE    (sizeof(struct mcba_usb_msg))

#define MCBA_USB_EP_IN      1
#define MCBA_USB_EP_OUT     1

/* Not required by driver itself as CANBUS is USB based */
/* Used internally by candev for bitrate calculation    */
#define MCBA_CAN_CLOCK      40000000

/* Microchip command id */
////////////////////////////////
//Send to USB
#define MBCA_CMD_CHANGE_BIT_RATE_RSP            0xE1
#define MBCA_CMD_TRANSMIT_MESSAGE_RSP           0xE2
#define MBCA_CMD_RECEIVE_MESSAGE                0xE3
#define MBCA_CMD_TRIGGER_EVENT                  0xE4
#define MBCA_CMD_SETUP_TRIGGER_RSP              0xE5
#define MBCA_CMD_ERROR_COUNT_STATUS             0xE6
#define MBCA_CMD_READ_REGISTER_DIRECTLY_RSP     0xE7
#define MBCA_CMD_READ_FW_VERSION_RSP            0xE8
#define MBCA_CMD_DEBUG_MODE_RSP                 0xF0
#define MBCA_CMD_I_AM_ALIVE_FROM_CAN            0xF5
#define MBCA_CMD_I_AM_ALIVE_FROM_USB            0xF7

//Recieve from USB USB host, send down to CAN micro
#define MBCA_CMD_CHANGE_BIT_RATE                0xA1
#define MBCA_CMD_TRANSMIT_MESSAGE_PD            0xA2
#define MBCA_CMD_TRANSMIT_MESSAGE_EV            0xA3
#define MBCA_CMD_SETUP_TRIGGER                  0xA4
#define MBCA_CMD_WRITE_REGISTER_DIRECTLY        0xA5
#define MBCA_CMD_READ_REGISTER_DIRECTLY         0xA6
#define MBCA_CMD_TOGGLE_LED                     0xA7
#define MBCA_CMD_SETUP_TERMINATION_RESISTANCE   0xA8
#define MBCA_CMD_READ_FW_VERSION                0xA9
#define MBCA_CMD_CAN_RESET                      0xAA
#define MBCA_CMD_CHANGE_CAN_MODE                0xAB
#define MBCA_CMD_SETUP_HW_FILTER                0xAC
#define MBCA_CMD_DEBUG_MODE                     0xB0

/* supported bitrates */
#define MCBA_BITRATE_20_KBPS_40MHZ      19940
#define MCBA_BITRATE_33_3KBPS_40MHZ     33333
#define MCBA_BITRATE_50KBPS_40MHZ       50000
#define MCBA_BITRATE_80KBPS_40MHZ       80000
#define MCBA_BITRATE_83_3KBPS_40MHZ     83333
#define MCBA_BITRATE_100KBPS_40MHZ      100000
#define MCBA_BITRATE_125KBPS_40MHZ      125000
#define MCBA_BITRATE_150KBPS_40MHZ      150375
#define MCBA_BITRATE_175KBPS_40MHZ      175438
#define MCBA_BITRATE_200KBPS_40MHZ      200000
#define MCBA_BITRATE_225KBPS_40MHZ      227272
#define MCBA_BITRATE_250KBPS_40MHZ      250000
#define MCBA_BITRATE_275KBPS_40MHZ      277777
#define MCBA_BITRATE_300KBPS_40MHZ      303030
#define MCBA_BITRATE_500KBPS_40MHZ      500000
#define MCBA_BITRATE_625KBPS_40MHZ      625000
#define MCBA_BITRATE_800KBPS_40MHZ      800000
#define MCBA_BITRATE_1000KBPS_40MHZ     1000000

#define MCBA_PARAM_DEBUG_DISABLE  0
#define MCBA_PARAM_DEBUG_USB      1
#define MCBA_PARAM_DEBUG_CAN      2
#define MCBA_IS_USB_DEBUG()       (debug & MCBA_PARAM_DEBUG_USB)
#define MCBA_IS_CAN_DEBUG()       (debug & MCBA_PARAM_DEBUG_CAN)

#define MCBA_VER_UNDEFINED 0xFF
#define MCBA_VER_USB_MAJOR 2
#define MCBA_VER_USB_MINOR 0
#define MCBA_VER_CAN_MAJOR 2
#define MCBA_VER_CAN_MINOR 3
#define MCBA_VER_REQ_USB   1
#define MCBA_VER_REQ_CAN   2

#define MCBA_USB_SID0_SID2_MASK    0x7
#define MCBA_USB_SID3_SID10_MASK   0x7F8
#define MCBA_USB_EID0_EID7_MASK    0x7f800
#define MCBA_USB_EID8_EID15_MASK   0x7f80000
#define MCBA_USB_EID16_EID17_MASK  0x18000000
#define MCBA_USB_DLC_MASK          0xf
#define MCBA_USB_RTR_MASK          0x40
#define MCBA_USB_EXID_MASK         0x80


#define MCBA_USB_SID0_SID2_SHIFT   5
#define MCBA_USB_SID3_SID10_SHIFT  0
#define MCBA_USB_EID0_EID7_SHIFT   0
#define MCBA_USB_EID8_EID15_SHIFT  0
#define MCBA_USB_EID16_EID17_SHIFT 0
#define MCBA_USB_SRR_SHIFT         4
#define MCBA_USB_DLC_SHIFT         0
#define MCBA_USB_RTR_SHIFT         6


#define MCBA_CAN_EID16_EID17_MASK  0x3
#define MCBA_CAN_SID3_SID10_SHIFT  3
#define MCBA_CAN_EID0_EID7_SHIFT   11
#define MCBA_CAN_EID8_EID15_SHIFT  19
#define MCBA_CAN_EID16_EID17_SHIFT 27

#define MCBA_CAN_RTR_MASK          0x40000000
#define MCBA_CAN_EXID_MASK         0x80000000


#define MCBA_USB_SET_SIDL(can_id)\
(((can_id & MCBA_USB_SID0_SID2_MASK) << MCBA_USB_SID0_SID2_SHIFT) |\
 ((can_id & MCBA_CAN_EXID_MASK)? MCBA_USB_EXID_MASK : 0) |\
 ((can_id & MCBA_USB_EID16_EID17_MASK) << MCBA_USB_EID16_EID17_SHIFT))

#define MCBA_USB_SET_SIDH(can_id)\
((can_id & MCBA_USB_SID3_SID10_MASK) << MCBA_USB_SID3_SID10_SHIFT)

#define MCBA_USB_SET_EIDL(can_id)\
((can_id & MCBA_USB_EID0_EID7_MASK) << MCBA_USB_EID0_EID7_SHIFT)

#define MCBA_USB_SET_EIDH(can_id)\
((can_id & MCBA_USB_EID8_EID15_MASK) << MCBA_USB_EID8_EID15_SHIFT)

#define MCBA_USB_SET_DLC(can_id, dlc)\
((can_id & MCBA_CAN_RTR_MASK)? ((dlc & MCBA_USB_DLC_MASK) | MCBA_USB_RTR_MASK) : (dlc & MCBA_USB_DLC_MASK))

#define MCBA_CAN_GET_SID(usb_msg)\
(((usb_msg.sidl >> MCBA_USB_SID0_SID2_SHIFT) & MCBA_USB_SID0_SID2_MASK) |\
  (usb_msg.sidh << MCBA_CAN_SID3_SID10_SHIFT) |\
  ((usb_msg.dlc & MCBA_USB_RTR_MASK)? MCBA_CAN_RTR_MASK : 0))

#define MCBA_CAN_GET_EID(usb_msg)\
(((usb_msg.sidl >> MCBA_USB_SID0_SID2_SHIFT) & MCBA_USB_SID0_SID2_MASK) |\
  (usb_msg.sidh << MCBA_CAN_SID3_SID10_SHIFT) |\
  (usb_msg.eidl << MCBA_CAN_EID0_EID7_SHIFT) |\
  (usb_msg.eidh << MCBA_CAN_EID8_EID15_SHIFT) |\
  ((usb_msg.sidl & MCBA_CAN_EID16_EID17_MASK) << MCBA_CAN_EID16_EID17_SHIFT) |\
  ((usb_msg.dlc & MCBA_USB_RTR_MASK)? MCBA_CAN_RTR_MASK : 0) |\
  MCBA_CAN_EXID_MASK)

#define MCBA_CAN_SET_ID(usb_msg)\
((usb_msg.dlc & MCBA_USB_RTR_MASK)? MCBA_CAN_GET_EID(usb_msg) : MCBA_CAN_GET_SID(usb_msg))

struct mcba_usb_ctx {
    struct mcba_priv *priv;
    u32 ndx;
    u8 dlc;
    bool can;
};

/* Structure to hold all of our device specific stuff */
struct mcba_priv {
    struct can_priv can; /* must be the first member */
    struct sk_buff *echo_skb[MCBA_MAX_TX_URBS];
    struct mcba_usb_ctx tx_context[MCBA_MAX_TX_URBS];

    struct usb_device *udev;
    struct net_device *netdev;
    struct usb_anchor tx_submitted;
    struct usb_anchor rx_submitted;
    struct can_berr_counter bec;
    u8 pic_usb_sw_ver_major;
    u8 pic_usb_sw_ver_minor;
    u8 pic_can_sw_ver_major;
    u8 pic_can_sw_ver_minor;
    u8 termination_state;
};

/* command frame */
struct __packed mcba_usb_msg_can {
    u8 cmdId;
    u8 eidh;
    u8 eidl;
    u8 sidh;
    u8 sidl;
    u8 dlc;
    u8 data[8];
    u8 timestamp[4];
    u8 checksum;
};

/* command frame */
struct __packed mcba_usb_msg {
    u8 cmdId;
    u8 unused[18];
};

struct __packed mcba_usb_msg_keep_alive_usb {
    u8 cmd_id;
    u8 termination_state;
    u8 soft_ver_major;
    u8 soft_ver_minor;
    u8 unused[15];
};

struct __packed mcba_usb_msg_keep_alive_can {
    u8 cmd_id;
    u8 tx_err_cnt;
    u8 rx_err_cnt;
    u8 rx_buff_ovfl;
    u8 tx_bus_off;
    u8 can_bitrate_hi;
    u8 can_bitrate_lo;
    u8 rx_lost_lo;
    u8 rx_lost_hi;
    u8 can_stat;
    u8 soft_ver_major;
    u8 soft_ver_minor;
    u8 debug_mode;
    u8 test_complete;
    u8 test_result;
    u8 unused[4];
};

struct __packed mcba_usb_msg_change_bitrate {
    u8 cmd_id;
    u8 bitrate_hi;
    u8 bitrate_lo;
    u8 unused[16];
};

struct __packed mcba_usb_msg_terminaton {
    u8 cmdId;
    u8 termination;
    u8 unused[17];
};

struct __packed mcba_usb_msg_fw_ver {
    u8 cmdId;
    u8 pic;
    u8 unused[17];
};

/* Required by can-dev however not for the sake of driver as CANBUS is USB based */
static const struct can_bittiming_const mcba_bittiming_const = {
        .name = "mcba_usb",
        .tseg1_min = 1,
        .tseg1_max = 8,
        .tseg2_min = 1,
        .tseg2_max = 8,
        .sjw_max = 4,
        .brp_min = 2,
        .brp_max = 128,
        .brp_inc = 2,
};

static netdev_tx_t mcba_usb_xmit(struct mcba_priv *priv, struct mcba_usb_msg *usb_msg, struct sk_buff *skb);
static void mcba_usb_xmit_cmd(struct mcba_priv *priv, struct mcba_usb_msg *usb_msg);
static void mcba_usb_xmit_read_fw_ver(struct mcba_priv *priv, u8 pic);
static void mcba_usb_xmit_termination(struct mcba_priv *priv, u8 termination);
static inline void mcba_init_ctx(struct mcba_priv *priv);
