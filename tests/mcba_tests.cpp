#include "gtest/gtest.h"

#define u8 uint8_t
#define u32 uint32_t
#define __packed
#define netdev_tx_t void
struct can_priv{};
struct usb_anchor{};
struct can_berr_counter{};

#include "../mcba_usb.h"


TEST(canIDConvertion, standardId)
{
    uint32_t canIdConverted = 0;
    mcba_usb_msg_can usb_msg;
    usb_msg.dlc = 0;

    for(uint32_t canId = 1; canId < 2048; ++canId)
    {
        usb_msg.sidl = MCBA_USB_SET_SIDL(canId);
        usb_msg.sidh = MCBA_USB_SET_SIDH(canId);
        usb_msg.eidl = MCBA_USB_SET_EIDL(canId);
        usb_msg.eidh = MCBA_USB_SET_EIDH(canId);

        canIdConverted = MCBA_CAN_SET_ID(usb_msg);

        EXPECT_EQ(canId, canIdConverted);
    }
}

TEST(canIDConvertion, standardIdRTR)
{
    uint32_t canIdConverted = 0;
    uint32_t canId = 0;
    mcba_usb_msg_can usb_msg;
    usb_msg.dlc = MCBA_USB_DLC_RTR_MASK;

    for(uint32_t i = 1; canId < 2048; ++canId)
    {
        canId = i | MCBA_CAN_RTR_MASK;

        usb_msg.sidl = MCBA_USB_SET_SIDL(canId);
        usb_msg.sidh = MCBA_USB_SET_SIDH(canId);
        usb_msg.eidl = MCBA_USB_SET_EIDL(canId);
        usb_msg.eidh = MCBA_USB_SET_EIDH(canId);

        canIdConverted = MCBA_CAN_SET_ID(usb_msg);

        EXPECT_EQ(canId, canIdConverted);
    }
}

TEST(canIDConvertion, extendedId)
{
    uint32_t canIdConverted = 0;
    uint32_t canId = 0;
    mcba_usb_msg_can usb_msg;
    usb_msg.dlc = 0;

    for(uint32_t i = 1; i < 0x20000000; ++i)
    {
        canId = i | MCBA_CAN_EXID_MASK;

        usb_msg.sidl = MCBA_USB_SET_SIDL(canId);
        usb_msg.sidh = MCBA_USB_SET_SIDH(canId);
        usb_msg.eidl = MCBA_USB_SET_EIDL(canId);
        usb_msg.eidh = MCBA_USB_SET_EIDH(canId);

        canIdConverted = MCBA_CAN_SET_ID(usb_msg);

        EXPECT_EQ(canId, canIdConverted);
    }
}

TEST(canIDConvertion, extendedIdRTR)
{
    uint32_t canIdConverted = 0;
    uint32_t canId = 0;
    mcba_usb_msg_can usb_msg;
    usb_msg.dlc = MCBA_USB_DLC_RTR_MASK;

    for(uint32_t i = 1; i < 0x20000000; ++i)
    {
        canId = i | MCBA_CAN_RTR_MASK | MCBA_CAN_EXID_MASK;

        usb_msg.sidl = MCBA_USB_SET_SIDL(canId);
        usb_msg.sidh = MCBA_USB_SET_SIDH(canId);
        usb_msg.eidl = MCBA_USB_SET_EIDL(canId);
        usb_msg.eidh = MCBA_USB_SET_EIDH(canId);

        canIdConverted = MCBA_CAN_SET_ID(usb_msg);

        EXPECT_EQ(canId, canIdConverted);
    }
}
