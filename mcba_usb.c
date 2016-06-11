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
#define MAX_RX_URBS           20
#define MAX_TX_URBS           20
#define RX_BUFFER_SIZE        64

#define MCBA_USB_EP_IN   1
#define MCBA_USB_EP_OUT  1

/* vendor and product id */
#define MODULE_NAME          "mcba_usb"
#define MCBA_VENDOR_ID      0x04d8
#define MCBA_PRODUCT_ID     0x0a30

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
    struct sk_buff *echo_skb[MAX_TX_URBS];

    struct usb_device *udev;
    struct net_device *netdev;

    atomic_t active_tx_urbs;
    struct usb_anchor tx_submitted;
    struct mcba_urb_ctx tx_contexts[MAX_TX_URBS];

    struct usb_anchor rx_submitted;

    struct can_berr_counter bec;

    u8 *cmd_msg_buffer;

    struct mutex write_lock;
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
    u8 data[18];
};

/* TODO: align bit timing */
static const struct can_bittiming_const mcba_bittiming_const = {
        .name = "usb_8dev",
        .tseg1_min = 1,
        .tseg1_max = 16,
        .tseg2_min = 1,
        .tseg2_max = 8,
        .sjw_max = 4,
        .brp_min = 1,
        .brp_max = 1024,
        .brp_inc = 1,
};

static void mcba_usb_process_rx(struct mcba_priv *priv, struct mcba_usb_msg *msg)
{

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
                          urb->transfer_buffer, RX_BUFFER_SIZE,
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

            buf = usb_alloc_coherent(priv->udev, RX_BUFFER_SIZE, GFP_KERNEL,
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
                              buf, RX_BUFFER_SIZE,
                              mcba_usb_read_bulk_callback, priv);
            urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
            usb_anchor_urb(urb, &priv->rx_submitted);

            err = usb_submit_urb(urb, GFP_KERNEL);
            if (err) {
                    usb_unanchor_urb(urb);
                    usb_free_coherent(priv->udev, RX_BUFFER_SIZE, buf,
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

/* Open USB device */
static int mcba_usb_open(struct net_device *netdev)
{
    struct mcba_priv *priv = netdev_priv(netdev);
    int err;

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
//	struct usb_8dev_priv *priv = netdev_priv(netdev);

//	bec->txerr = priv->bec.txerr;
//	bec->rxerr = priv->bec.rxerr;

        return 0;
}

static const struct net_device_ops mcba_netdev_ops = {
    .ndo_open = mcba_usb_open,
    .ndo_stop = mcba_usb_close,
//    .ndo_start_xmit = usb_8dev_start_xmit,
//    .ndo_change_mtu = can_change_mtu,
};

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
    priv->can.clock.freq = 40000000;
    priv->can.bittiming_const = &mcba_bittiming_const;
    priv->can.do_set_mode = mcba_net_set_mode;
    priv->can.do_get_berr_counter = mcba_net_get_berr_counter;
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

//    err = usb_8dev_cmd_version(priv, &version);
//    if (err) {
//        netdev_err(netdev, "can't get firmware version\n");
//        goto cleanup_unregister_candev;
//    } else {
//        netdev_info(netdev,
//             "firmware: %d.%d, hardware: %d.%d\n",
//             (version>>24) & 0xff, (version>>16) & 0xff,
//             (version>>8) & 0xff, version & 0xff);
//    }

    devm_can_led_init(netdev);

    return 0;

//cleanup_unregister_candev:
//    unregister_netdev(priv->netdev);

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
