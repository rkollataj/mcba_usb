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
#define USB_MC_NAME           "usb_microchip_canbus"
#define USB_MC_VENDOR_ID      0x04d8
#define USB_MC_PRODUCT_ID     0x0a30

/* table of devices that work with this driver */
static const struct usb_device_id usb_mc_table[] = {
    { USB_DEVICE(USB_MC_VENDOR_ID, USB_MC_PRODUCT_ID) },
    { }                 /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, usb_mc_table);

struct usb_mc_tx_urb_context {
    struct usb_mc_priv *priv;

    u32 echo_index;
    u8 dlc;
};

/* Structure to hold all of our device specific stuff */
struct usb_mc_priv {
    struct can_priv can; /* must be the first member */
    struct sk_buff *echo_skb[MAX_TX_URBS];

    struct usb_device *udev;
    struct net_device *netdev;

    atomic_t active_tx_urbs;
    struct usb_anchor tx_submitted;
    struct usb_mc_tx_urb_context tx_contexts[MAX_TX_URBS];

    struct usb_anchor rx_submitted;

    struct can_berr_counter bec;

    u8 *cmd_msg_buffer;

    struct mutex usb_mc_cmd_lock;
};

/* command frame */
struct __packed usb_mc_cmd_msg {
    u8 begin;
    u8 channel; /* unkown - always 0 */
    u8 command; /* command to execute */
    u8 opt1;    /* optional parameter / return value */
    u8 opt2;    /* optional parameter 2 */
    u8 data[10];    /* optional parameter and data */
    u8 end;
};

/* Start USB device */
static int usb_mc_start(struct usb_mc_priv *priv)
{

    return 0;
}


/* Open USB device */
static int usb_mc_open(struct net_device *netdev)
{
    struct usb_mc_priv *priv = netdev_priv(netdev);
    int err;

    /* common open */
    err = open_candev(netdev);
    if (err)
        return err;

    can_led_event(netdev, CAN_LED_EVENT_OPEN);

    /* finally start device */
    err = usb_mc_start(priv);
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

static void unlink_all_urbs(struct usb_mc_priv *priv)
{
    int i;

    usb_kill_anchored_urbs(&priv->rx_submitted);

    usb_kill_anchored_urbs(&priv->tx_submitted);
    atomic_set(&priv->active_tx_urbs, 0);

    for (i = 0; i < MAX_TX_URBS; i++)
        priv->tx_contexts[i].echo_index = MAX_TX_URBS;
}

/* Close USB device */
static int usb_mc_close(struct net_device *netdev)
{
    struct usb_mc_priv *priv = netdev_priv(netdev);
    int err = 0;

    /* Send CLOSE command to CAN controller */
//    err = usb_8dev_cmd_close(priv);
//    if (err)
//        netdev_warn(netdev, "couldn't stop device");

    priv->can.state = CAN_STATE_STOPPED;

    netif_stop_queue(netdev);

    /* Stop polling */
    unlink_all_urbs(priv);

    close_candev(netdev);

    can_led_event(netdev, CAN_LED_EVENT_STOP);

    return err;
}

static const struct net_device_ops usb_mc_netdev_ops = {
    .ndo_open = usb_mc_open,
    .ndo_stop = usb_mc_close,
//    .ndo_start_xmit = usb_8dev_start_xmit,
//    .ndo_change_mtu = can_change_mtu,
};

static int usb_mc_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct net_device *netdev;
    struct usb_mc_priv *priv;
    int i, err = 0;//-ENOMEM;
//    u32 version;
//    char buf[18];
    struct usb_device *usbdev = interface_to_usbdev(intf);

    dev_info(&intf->dev, "%s: Microchip CAN BUS analizer connected\n", USB_MC_NAME);

    netdev = alloc_candev(sizeof(struct usb_mc_priv), MAX_TX_URBS);
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
    netdev->netdev_ops = &usb_mc_netdev_ops;

    netdev->flags |= IFF_ECHO; /* we support local echo */

    init_usb_anchor(&priv->rx_submitted);

    init_usb_anchor(&priv->tx_submitted);
    atomic_set(&priv->active_tx_urbs, 0);

    for (i = 0; i < MAX_TX_URBS; i++)
        priv->tx_contexts[i].echo_index = MAX_TX_URBS;

    priv->cmd_msg_buffer = kzalloc(sizeof(struct usb_mc_cmd_msg),
                      GFP_KERNEL);
    if (!priv->cmd_msg_buffer)
        goto cleanup_candev;

    usb_set_intfdata(intf, priv);

    SET_NETDEV_DEV(netdev, &intf->dev);

    mutex_init(&priv->usb_mc_cmd_lock);

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
static void usb_mc_disconnect(struct usb_interface *intf)
{
    struct usb_mc_priv *priv = usb_get_intfdata(intf);

    usb_set_intfdata(intf, NULL);

    if (priv) {
        netdev_info(priv->netdev, "device disconnected\n");

        unregister_netdev(priv->netdev);
        free_candev(priv->netdev);

        unlink_all_urbs(priv);
    }
}

static struct usb_driver usb_mc_driver = {
	.name =		    USB_MC_NAME,
	.probe =	    usb_mc_probe,
	.disconnect =	usb_mc_disconnect,
	.id_table =	    usb_mc_table,
};

module_usb_driver(usb_mc_driver);

MODULE_AUTHOR("Remigiusz Kołłątaj <remigiusz.kollataj@mobica.com>");
MODULE_DESCRIPTION("CAN driver for Microchip CAN BUS Analyzer");
MODULE_LICENSE("GPL v2");
