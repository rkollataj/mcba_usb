#include "gtest/gtest.h"

#define u8 uint8_t
#define u32 uint32_t
#define __packed
#define netdev_tx_t void
struct can_priv{};
struct usb_anchor{};
struct can_berr_counter{};

struct can_frame {
    uint32_t can_id;
};

#include "../mcba_usb.h"

TEST(canIDConvertion, standardId)
{
    uint32_t canIdConverted = 0;
    mcba_usb_msg_can usb_msg;
    struct can_frame cf;

    for(uint32_t i = 1; i < 2048; ++i)
    {
        cf.can_id = i;

        usb_msg.sidl = MCBA_SET_S_SIDL(cf.can_id);
        usb_msg.sidh = MCBA_SET_S_SIDH(cf.can_id);
        usb_msg.eidl = 0;
        usb_msg.eidh = 0;

        canIdConverted = MCBA_CAN_GET_SID((&usb_msg));

        EXPECT_EQ(cf.can_id, canIdConverted);
    }
}

TEST(canIDConvertion, standardIdRTR)
{
    uint32_t canIdConverted = 0;
    mcba_usb_msg_can usb_msg;
    struct can_frame cf;

    for(uint32_t i = 1; i < 2048; ++i)
    {
        cf.can_id = i | MCBA_CAN_RTR_MASK;

        usb_msg.sidl = MCBA_SET_S_SIDL(cf.can_id);
        usb_msg.sidh = MCBA_SET_S_SIDH(cf.can_id);
        usb_msg.eidl = 0;
        usb_msg.eidh = 0;
        usb_msg.dlc = MCBA_DLC_RTR_MASK;

        canIdConverted = MCBA_CAN_GET_SID((&usb_msg));

        if(MCBA_RX_IS_RTR((&usb_msg)))
            canIdConverted |= MCBA_CAN_RTR_MASK;

        EXPECT_EQ(cf.can_id, canIdConverted);
    }
}

TEST(canIDConvertion, extendedId)
{
    uint32_t canIdConverted = 0;
    mcba_usb_msg_can usb_msg;
    struct can_frame cf;

    for(uint32_t i = 1; i < 0x20000000; ++i)
    {
        cf.can_id = i | MCBA_CAN_EXID_MASK;

        usb_msg.sidl = MCBA_SET_E_SIDL(cf.can_id);
        usb_msg.sidh = MCBA_SET_E_SIDH(cf.can_id);
        usb_msg.eidl = MCBA_SET_EIDL(cf.can_id);
        usb_msg.eidh = MCBA_SET_EIDH(cf.can_id);

        canIdConverted = MCBA_CAN_GET_EID((&usb_msg));

        if(MCBA_RX_IS_EXID((&usb_msg)))
            canIdConverted |= MCBA_CAN_EXID_MASK;

        EXPECT_EQ(cf.can_id, canIdConverted);
    }
}

TEST(canIDConvertion, extendedIdRTR)
{
    uint32_t canIdConverted = 0;
    mcba_usb_msg_can usb_msg;
    struct can_frame cf;

    for(uint32_t i = 1; i < 0x20000000; ++i)
    {
        cf.can_id = i | MCBA_CAN_RTR_MASK | MCBA_CAN_EXID_MASK;

        usb_msg.sidl = MCBA_SET_E_SIDL(cf.can_id);
        usb_msg.sidh = MCBA_SET_E_SIDH(cf.can_id);
        usb_msg.eidl = MCBA_SET_EIDL(cf.can_id);
        usb_msg.eidh = MCBA_SET_EIDH(cf.can_id);
        usb_msg.dlc = MCBA_DLC_RTR_MASK;

        canIdConverted = MCBA_CAN_GET_EID((&usb_msg));

        if(MCBA_RX_IS_EXID((&usb_msg)))
            canIdConverted |= MCBA_CAN_EXID_MASK;

        if(MCBA_RX_IS_RTR((&usb_msg)))
            canIdConverted |= MCBA_CAN_RTR_MASK;

        EXPECT_EQ(cf.can_id, canIdConverted);
    }
}
