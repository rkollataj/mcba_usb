#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>

/* driver constants */
#define MAX_RX_URBS         20
#define MAX_TX_URBS         20
#define USB_BUFF_SIZE       19

#define MCBA_USB_EP_IN      1
#define MCBA_USB_EP_OUT     1

/* vendor and product id */
#define MODULE_NAME          "mcba_usb"
#define MCBA_VENDOR_ID      0x04d8
#define MCBA_PRODUCT_ID     0x0a30

/* Not required by driver itself as CANBUS is USB based */
/* It seems to be requried by CANBUS                    */
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

/* table of devices that work with this driver */
static const struct usb_device_id mcba_usb_table[] = {
    { USB_DEVICE(MCBA_VENDOR_ID, MCBA_PRODUCT_ID) },
    { }                 /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, mcba_usb_table);

struct mcba_urb_ctx {
    struct mcba_priv *priv;

    u32 echo_index;
    u8 dlc;
};

/* Structure to hold all of our device specific stuff */
struct mcba_priv {
    struct can_priv can; /* must be the first member */
//    struct sk_buff *echo_skb[MAX_TX_URBS];

    struct usb_device *udev;
    struct net_device *netdev;

    atomic_t active_tx_urbs;
    struct usb_anchor tx_submitted;
    struct mcba_urb_ctx tx_contexts[MAX_TX_URBS];

    struct usb_anchor rx_submitted;

    struct can_berr_counter bec;

    u8 *cmd_msg_buffer;

    struct mutex write_lock;

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

static ssize_t termination_show(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, "%d\n", 66);
}

static ssize_t termination_store(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
//        sscanf(buf, "%du", &foo);
        return count;
}


static struct device_attribute termination_attr = {
    .attr = {
        .name = "termination",
        .mode = 0666 },
    .show	= termination_show,
    .store	= termination_store
};

static void mcba_usb_process_keep_alive_usb(struct mcba_priv *priv, struct mcba_usb_msg_keep_alive_usb *msg)
{
//    printk("Termination %hhu, ver_maj %hhu, soft_min %hhu\n", msg->termination_state, msg->soft_ver_major, msg->soft_ver_minor);

    priv->pic_usb_sw_ver_major = msg->soft_ver_major;
    priv->pic_usb_sw_ver_minor = msg->soft_ver_minor;
    priv->termination_state = msg->termination_state;
}

static void mcba_usb_process_keep_alive_can(struct mcba_priv *priv, struct mcba_usb_msg_keep_alive_can *msg)
{
//    printk("tx_err_cnt %hhu, rx_err_cnt %hhu, rx_buff_ovfl %hhu, tx_bus_off %hhu, "
//            "can_bitrate %hu, rx_lost %hu, can_stat %hhu, soft_ver %hhu.%hhu, "
//            "debug_mode %hhu, test_complete %hhu, test_result %hhu\n",
//           msg->tx_err_cnt, msg->rx_err_cnt, msg->rx_buff_ovfl, msg->tx_bus_off,
//           ((msg->can_bitrate_hi << 8) + msg->can_bitrate_lo), ((msg->rx_lost_hi >> 8) + msg->rx_lost_lo),
//           msg->can_stat, msg->soft_ver_major, msg->soft_ver_minor,
//           msg->debug_mode, msg->test_complete, msg->test_result);

    priv->bec.txerr = msg->tx_err_cnt;
    priv->bec.rxerr = msg->rx_err_cnt;

    priv->pic_can_sw_ver_major = msg->soft_ver_major;
    priv->pic_can_sw_ver_minor = msg->soft_ver_minor;
}

static void mcba_usb_process_rx(struct mcba_priv *priv, struct mcba_usb_msg *msg)
{
    switch(msg->cmdId)
    {
    case MBCA_CMD_I_AM_ALIVE_FROM_CAN:
        mcba_usb_process_keep_alive_can(priv, (struct mcba_usb_msg_keep_alive_can *)msg);
        break;

    case MBCA_CMD_I_AM_ALIVE_FROM_USB:
        mcba_usb_process_keep_alive_usb(priv, (struct mcba_usb_msg_keep_alive_usb *)msg);
        break;

    default:
        netdev_warn(priv->netdev, "Unsupported msg (0x%hhX)", msg->cmdId);
        // Unsupported message
        break;
    }
}


/* Callback for reading data from device
 *
 * Check urb status, call read function and resubmit urb read operation.
 */
static void mcba_usb_read_bulk_callback(struct urb *urb)
{
        struct mcba_priv *priv = urb->context;
        struct net_device *netdev;
        int retval;
        int pos = 0;

        netdev = priv->netdev;

        if (!netif_device_present(netdev))
                return;

        switch (urb->status) {
        case 0: /* success */
                break;

        case -ENOENT:
        case -ESHUTDOWN:
                return;

        default:
                netdev_info(netdev, "Rx URB aborted (%d)\n",
                         urb->status);
                goto resubmit_urb;
        }

        while (pos < urb->actual_length) {
                struct mcba_usb_msg *msg;

                if (pos + sizeof(struct mcba_usb_msg) > urb->actual_length) {
                        netdev_err(priv->netdev, "format error\n");
                        break;
                }

                msg = (struct mcba_usb_msg *)(urb->transfer_buffer + pos);
                mcba_usb_process_rx(priv, msg);

                pos += sizeof(struct mcba_usb_msg);
        }

resubmit_urb:
        usb_fill_bulk_urb(urb, priv->udev,
                          usb_rcvbulkpipe(priv->udev, MCBA_USB_EP_OUT),
                          urb->transfer_buffer, USB_BUFF_SIZE,
                          mcba_usb_read_bulk_callback, priv);

        retval = usb_submit_urb(urb, GFP_ATOMIC);

        if (retval == -ENODEV)
                netif_device_detach(netdev);
        else if (retval)
                netdev_err(netdev,
                        "failed resubmitting read bulk urb: %d\n", retval);
}

/* Start USB device */
static int mcba_usb_start(struct mcba_priv *priv)
{
    struct net_device *netdev = priv->netdev;
    int err, i;

    for (i = 0; i < MAX_RX_URBS; i++) {
            struct urb *urb = NULL;
            u8 *buf;

            /* create a URB, and a buffer for it */
            urb = usb_alloc_urb(0, GFP_KERNEL);
            if (!urb) {
                    netdev_err(netdev, "No memory left for URBs\n");
                    err = -ENOMEM;
                    break;
            }

            buf = usb_alloc_coherent(priv->udev, USB_BUFF_SIZE, GFP_KERNEL,
                                     &urb->transfer_dma);
            if (!buf) {
                    netdev_err(netdev, "No memory left for USB buffer\n");
                    usb_free_urb(urb);
                    err = -ENOMEM;
                    break;
            }

            usb_fill_bulk_urb(urb, priv->udev,
                              usb_rcvbulkpipe(priv->udev,
                                              MCBA_USB_EP_IN),
                              buf, USB_BUFF_SIZE,
                              mcba_usb_read_bulk_callback, priv);
            urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
            usb_anchor_urb(urb, &priv->rx_submitted);

            err = usb_submit_urb(urb, GFP_KERNEL);
            if (err) {
                    usb_unanchor_urb(urb);
                    usb_free_coherent(priv->udev, USB_BUFF_SIZE, buf,
                                      urb->transfer_dma);
                    usb_free_urb(urb);
                    break;
            }

            /* Drop reference, USB core will take care of freeing it */
            usb_free_urb(urb);
    }

    /* Did we submit any URBs */
    if (i == 0) {
            netdev_warn(netdev, "couldn't setup read URBs\n");
            return err;
    }

    /* Warn if we've couldn't transmit all the URBs */
    if (i < MAX_RX_URBS)
            netdev_warn(netdev, "rx performance may be slow\n");

//    err = usb_8dev_cmd_open(priv);
    if (err)
            goto failed;

    priv->can.state = CAN_STATE_ERROR_ACTIVE;

    return 0;

failed:
    if (err == -ENODEV)
            netif_device_detach(priv->netdev);

    netdev_warn(netdev, "couldn't submit control: %d\n", err);

    return err;
}

static void mcba_usb_write_bulk_callback(struct urb *urb)
{
    struct mcba_priv *priv = urb->context;
    struct net_device *netdev;

    BUG_ON(!priv);

    netdev = priv->netdev;

    /* free up our allocated buffer */
    usb_free_coherent(urb->dev, urb->transfer_buffer_length,
              urb->transfer_buffer, urb->transfer_dma);

    atomic_dec(&priv->active_tx_urbs);



//    if (!netif_device_present(netdev))
//        return;

    if (urb->status)
        netdev_info(netdev, "Tx URB aborted (%d)\n",
             urb->status);

//    netdev->stats.tx_packets++;
//    netdev->stats.tx_bytes += context->dlc;

//    can_get_echo_skb(netdev, context->echo_index);

//    can_led_event(netdev, CAN_LED_EVENT_TX);

//    /* Release context */
//    context->echo_index = MAX_TX_URBS;

//    netif_wake_queue(netdev);
}

/* Send data to device */
static void mcba_usb_xmit(struct mcba_priv *priv, struct mcba_usb_msg *usb_msg)
{
//	struct net_device_stats *stats = &netdev->stats;
//	struct can_frame *cf = (struct can_frame *) skb->data;
//	struct usb_8dev_tx_msg *msg;
    struct urb *urb;
//	struct usb_8dev_tx_urb_context *context = NULL;
    u8 *buf;
    int i, err;
//	size_t size = sizeof(struct usb_8dev_tx_msg);

    /* create a URB, and a buffer for it, and copy the data to the URB */
    urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!urb) {
        netdev_err(priv->netdev, "No memory left for URBs\n");
        goto nomem;
    }

    buf = usb_alloc_coherent(priv->udev, USB_BUFF_SIZE, GFP_ATOMIC,
                 &urb->transfer_dma);
    if (!buf) {
        netdev_err(priv->netdev, "No memory left for USB buffer\n");
        goto nomembuf;
    }

    memcpy(buf, usb_msg, USB_BUFF_SIZE);

//    for (i = 0; i < MAX_TX_URBS; i++) {
//        if (priv->tx_contexts[i].echo_index == MAX_TX_URBS) {
//            context = &priv->tx_contexts[i];
//            break;
//        }
//    }

    /* May never happen! When this happens we'd more URBs in flight as
     * allowed (MAX_TX_URBS).
     */
//    if (!context)
//        goto nofreecontext;

//    context->priv = priv;
//    context->echo_index = i;
//    context->dlc = cf->can_dlc;

    usb_fill_bulk_urb(urb, priv->udev,
              usb_sndbulkpipe(priv->udev, MCBA_USB_EP_OUT),
              buf, USB_BUFF_SIZE, mcba_usb_write_bulk_callback, priv);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &priv->tx_submitted);

//    can_put_echo_skb(skb, netdev, context->echo_index);

    atomic_inc(&priv->active_tx_urbs);

    err = usb_submit_urb(urb, GFP_ATOMIC);
    if (unlikely(err))
        goto failed;
//    else if (atomic_read(&priv->active_tx_urbs) >= MAX_TX_URBS)
//        /* Slow down tx path */
//        netif_stop_queue(netdev);

    /* Release our reference to this URB, the USB core will eventually free
     * it entirely.
     */
    usb_free_urb(urb);

//    return NETDEV_TX_OK;

    return;

nofreecontext:
    usb_free_coherent(priv->udev, USB_BUFF_SIZE, buf, urb->transfer_dma);
    usb_free_urb(urb);

    netdev_warn(priv->netdev, "couldn't find free context");

    return NETDEV_TX_BUSY;

failed:
//    can_free_echo_skb(netdev, context->echo_index);

    usb_unanchor_urb(urb);
    usb_free_coherent(priv->udev, USB_BUFF_SIZE, buf, urb->transfer_dma);

    atomic_dec(&priv->active_tx_urbs);

    if (err == -ENODEV)
        netif_device_detach(priv->netdev);
    else
        netdev_warn(priv->netdev, "failed tx_urb %d\n", err);

nomembuf:
    usb_free_urb(urb);

nomem:
//    dev_kfree_skb(skb);
//    stats->tx_dropped++;

//    return NETDEV_TX_OK;
    return;
}

static void mcba_usb_xmit_change_bitrate(struct mcba_priv *priv, u16 bitrate)
{
    struct mcba_usb_msg_change_bitrate usb_msg;

    usb_msg.cmd_id =  MBCA_CMD_CHANGE_BIT_RATE;
    usb_msg.bitrate_hi = (0xff00 & bitrate) >> 8;
    usb_msg.bitrate_lo = (0xff & bitrate);

    mcba_usb_xmit(priv, (struct mcba_usb_msg *)&usb_msg);
}

/* Open USB device */
static int mcba_usb_open(struct net_device *netdev)
{
    struct mcba_priv *priv = netdev_priv(netdev);
    int err;

    printk("%s\n", __FUNCTION__);

    /* common open */
    err = open_candev(netdev);
    if (err)
        return err;

    can_led_event(netdev, CAN_LED_EVENT_OPEN);

    /* finally start device */
    err = mcba_usb_start(priv);
    if (err) {
        if (err == -ENODEV)
            netif_device_detach(priv->netdev);

        netdev_warn(netdev, "couldn't start device: %d\n",
             err);

        close_candev(netdev);

        return err;
    }

    netif_start_queue(netdev);

    return 0;
}

static void mcba_urb_unlink(struct mcba_priv *priv)
{
    int i;

    usb_kill_anchored_urbs(&priv->rx_submitted);

    usb_kill_anchored_urbs(&priv->tx_submitted);
    atomic_set(&priv->active_tx_urbs, 0);

    for (i = 0; i < MAX_TX_URBS; i++)
        priv->tx_contexts[i].echo_index = MAX_TX_URBS;
}

/* Close USB device */
static int mcba_usb_close(struct net_device *netdev)
{
    struct mcba_priv *priv = netdev_priv(netdev);
    int err = 0;

    printk("%s\n", __FUNCTION__);

    /* Send CLOSE command to CAN controller */
//    err = usb_8dev_cmd_close(priv);
//    if (err)
//        netdev_warn(netdev, "couldn't stop device");

    priv->can.state = CAN_STATE_STOPPED;

    netif_stop_queue(netdev);

    /* Stop polling */
    mcba_urb_unlink(priv);

    close_candev(netdev);

    can_led_event(netdev, CAN_LED_EVENT_STOP);

    return err;
}

/* Set network device mode
 *
 * Maybe we should leave this function empty, because the device
 * set mode variable with open command.
 */
static int mcba_net_set_mode(struct net_device *netdev, enum can_mode mode)
{
//        struct mcba_priv *priv = netdev_priv(netdev);
        int err = 0;

        printk("%s\n", __FUNCTION__);

        switch (mode) {
        case CAN_MODE_START:
//                err = usb_8dev_cmd_open(priv);
//                if (err)
//                        netdev_warn(netdev, "couldn't start device");
//                break;

        default:
                return -EOPNOTSUPP;
        }

        return err;
}

static int mcba_net_get_berr_counter(const struct net_device *netdev,
                                     struct can_berr_counter *bec)
{
    struct mcba_priv *priv = netdev_priv(netdev);

    bec->txerr = priv->bec.txerr;
    bec->rxerr = priv->bec.rxerr;

    return 0;
}

static const struct net_device_ops mcba_netdev_ops = {
    .ndo_open = mcba_usb_open,
    .ndo_stop = mcba_usb_close,
//    .ndo_start_xmit = usb_8dev_start_xmit,
//    .ndo_change_mtu = can_change_mtu,
};

static void mcba_net_calc_bittiming(u32 sjw, u32 prop, u32 seg1, u32 seg2, u32 brp, struct can_bittiming *bt)
{
    bt->sjw = sjw;
    bt->prop_seg = prop;
    bt->phase_seg1 = seg1;
    bt->phase_seg2 = seg2;
    bt->brp = brp;
    /* nanoseconds expected */
    bt->tq = (bt->brp * 1000)/(MCBA_CAN_CLOCK/1000000);
    bt->bitrate = 1000000000/((bt->sjw + bt->prop_seg + bt->phase_seg1 + bt->phase_seg2)*bt->tq);
    bt->sample_point = ((bt->sjw + bt->prop_seg + bt->phase_seg1)*1000)/(bt->sjw + bt->prop_seg + bt->phase_seg1 + bt->phase_seg2);
}

/* Microchip CANBUS has hardcoded bittiming values by default. This fucntion sends
 * request via USB to change the speed and align bittiming values for presentation purposes only
 */
static int mcba_net_set_bittiming(struct net_device *netdev)
{
    struct mcba_priv *priv = netdev_priv(netdev);
    struct can_bittiming *bt = &priv->can.bittiming;

    switch(bt->bitrate)
    {
    case MCBA_BITRATE_20_KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 5, 8, 6, 100, bt);
        mcba_usb_xmit_change_bitrate(priv, 20);
        break;

    case MCBA_BITRATE_33_3KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 8, 48, bt);
        mcba_usb_xmit_change_bitrate(priv, 33);
        break;

    case MCBA_BITRATE_50KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 7, 4, 40, bt);
        mcba_usb_xmit_change_bitrate(priv, 50);
        break;

    case MCBA_BITRATE_80KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 8, 20, bt);
        mcba_usb_xmit_change_bitrate(priv, 80);
        break;

    case MCBA_BITRATE_83_3KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 7, 20, bt);
        mcba_usb_xmit_change_bitrate(priv, 83);
        break;

    case MCBA_BITRATE_100KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 1, 5, 3, 40, bt);
        mcba_usb_xmit_change_bitrate(priv, 100);
        break;

    case MCBA_BITRATE_125KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 3, 8, 8, 16, bt);
        mcba_usb_xmit_change_bitrate(priv, 125);
        break;

    case MCBA_BITRATE_150KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 6, 4, 14, bt);
        mcba_usb_xmit_change_bitrate(priv, 150);
        break;

    case MCBA_BITRATE_175KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 6, 4, 12, bt);
        mcba_usb_xmit_change_bitrate(priv, 175);
        break;

    case MCBA_BITRATE_200KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 8, 8, bt);
        mcba_usb_xmit_change_bitrate(priv, 200);
        break;

    case MCBA_BITRATE_225KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 5, 8, bt);
        mcba_usb_xmit_change_bitrate(priv, 225);
        break;

    case MCBA_BITRATE_250KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 3, 8, 8, 8, bt);
        mcba_usb_xmit_change_bitrate(priv, 250);
        break;

    case MCBA_BITRATE_275KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 7, 6, bt);
        mcba_usb_xmit_change_bitrate(priv, 275);
        break;

    case MCBA_BITRATE_300KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 5, 6, bt);
        mcba_usb_xmit_change_bitrate(priv, 300);
        break;

    case MCBA_BITRATE_500KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 3, 8, 8, 4, bt);
        mcba_usb_xmit_change_bitrate(priv, 500);
        break;

    case MCBA_BITRATE_625KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 1, 4, 2, 8, bt);
        mcba_usb_xmit_change_bitrate(priv, 625);
        break;

    case MCBA_BITRATE_800KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 8, 8, 8, 2, bt);
        mcba_usb_xmit_change_bitrate(priv, 800);
        break;

    case MCBA_BITRATE_1000KBPS_40MHZ:
        /* bittiming aligned with default Microchip CANBUS firmware */
        mcba_net_calc_bittiming(1, 3, 8, 8, 2, bt);
        mcba_usb_xmit_change_bitrate(priv, 1000);
        break;

    default:
        netdev_err(netdev, "Unsupported bittrate (%u). Use one of: 20000, "
                   "33333, 50000, 80000, 83333, 100000, 125000, 150000, "
                   "175000, 200000, 225000, 250000, 275000, 300000, 500000, "
                   "625000, 800000, 1000000\n", bt->bitrate);

        return -EINVAL;
    }

    return 0;
}

static int mcba_usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct net_device *netdev;
    struct mcba_priv *priv;
    int i, err = 0;//-ENOMEM;
//    u32 version;
//    char buf[18];
    struct usb_device *usbdev = interface_to_usbdev(intf);

    dev_info(&intf->dev, "%s: Microchip CAN BUS analizer connected\n", MODULE_NAME);

    netdev = alloc_candev(sizeof(struct mcba_priv), MAX_TX_URBS);
    if (!netdev) {
        dev_err(&intf->dev, "Couldn't alloc candev\n");
        return -ENOMEM;
    }

    priv = netdev_priv(netdev);

    priv->udev = usbdev;
    priv->netdev = netdev;

    priv->can.state = CAN_STATE_STOPPED;
    priv->can.clock.freq = MCBA_CAN_CLOCK;
    priv->can.bittiming_const = &mcba_bittiming_const;
    priv->can.do_set_mode = mcba_net_set_mode;
    priv->can.do_get_berr_counter = mcba_net_get_berr_counter;
    priv->can.do_set_bittiming = mcba_net_set_bittiming;
    priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
                      CAN_CTRLMODE_LISTENONLY |
                      CAN_CTRLMODE_ONE_SHOT;

    netdev->netdev_ops = &mcba_netdev_ops;

    netdev->flags |= IFF_ECHO; /* we support local echo */

    init_usb_anchor(&priv->rx_submitted);

    init_usb_anchor(&priv->tx_submitted);
    atomic_set(&priv->active_tx_urbs, 0);

    for (i = 0; i < MAX_TX_URBS; i++)
        priv->tx_contexts[i].echo_index = MAX_TX_URBS;

    priv->cmd_msg_buffer = kzalloc(sizeof(struct mcba_usb_msg),
                      GFP_KERNEL);
    if (!priv->cmd_msg_buffer)
        goto cleanup_candev;

    usb_set_intfdata(intf, priv);

    SET_NETDEV_DEV(netdev, &intf->dev);

    mutex_init(&priv->write_lock);

    err = register_candev(netdev);
    if (err) {
        netdev_err(netdev,
            "couldn't register CAN device: %d\n", err);
        goto cleanup_cmd_msg_buffer;
    }

    err = device_create_file(&netdev->dev, &termination_attr);
    if (err)
        goto cleanup_cmd_msg_buffer;

    return 0;

cleanup_cmd_msg_buffer:
    kfree(priv->cmd_msg_buffer);

cleanup_candev:
    free_candev(netdev);

    return err;
}

/* Called by the usb core when driver is unloaded or device is removed */
static void mcba_usb_disconnect(struct usb_interface *intf)
{
    struct mcba_priv *priv = usb_get_intfdata(intf);

    usb_set_intfdata(intf, NULL);

    if (priv) {
        netdev_info(priv->netdev, "device disconnected\n");

        unregister_netdev(priv->netdev);
        free_candev(priv->netdev);

        mcba_urb_unlink(priv);
    }

    device_remove_file(&priv->netdev->dev, &termination_attr);
}

static struct usb_driver mcba_usb_driver = {
        .name =		MODULE_NAME,
        .probe =	mcba_usb_probe,
        .disconnect =	mcba_usb_disconnect,
        .id_table =	mcba_usb_table,
};

module_usb_driver(mcba_usb_driver);

MODULE_AUTHOR("Remigiusz Kołłątaj <remigiusz.kollataj@mobica.com>");
MODULE_LICENSE("GPL v2");
