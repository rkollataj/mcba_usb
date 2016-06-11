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
struct __packed mcba_usb_cmd {
    u8 begin;
    u8 channel; /* unkown - always 0 */
    u8 command; /* command to execute */
    u8 opt1;    /* optional parameter / return value */
    u8 opt2;    /* optional parameter 2 */
    u8 data[10];    /* optional parameter and data */
    u8 end;
};

/* Start USB device */
static int mcba_usb_start(struct mcba_priv *priv)
{

    return 0;
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

//    priv->can.state = CAN_STATE_STOPPED;
//    priv->can.clock.freq = USB_8DEV_ABP_CLOCK;
//    priv->can.bittiming_const = &usb_8dev_bittiming_const;
//    priv->can.do_set_mode = usb_8dev_set_mode;
//    priv->can.do_get_berr_counter = usb_8dev_get_berr_counter;
//    priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
//                      CAN_CTRLMODE_LISTENONLY |
//                      CAN_CTRLMODE_ONE_SHOT;
//
    netdev->netdev_ops = &mcba_netdev_ops;

    netdev->flags |= IFF_ECHO; /* we support local echo */

    init_usb_anchor(&priv->rx_submitted);

    init_usb_anchor(&priv->tx_submitted);
    atomic_set(&priv->active_tx_urbs, 0);

    for (i = 0; i < MAX_TX_URBS; i++)
        priv->tx_contexts[i].echo_index = MAX_TX_URBS;

    priv->cmd_msg_buffer = kzalloc(sizeof(struct mcba_usb_cmd),
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
