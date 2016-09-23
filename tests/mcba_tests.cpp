#include <gtest/gtest.h>
#include <stdarg.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>
#include <future>

#define u8 uint8_t
#define u32 uint32_t
#define __packed
#define netdev_tx_t void
struct can_priv{};
struct usb_anchor{};
struct can_berr_counter{};

#include "../mcba_usb.h"

int openCANSocket(const char *canName)
{
    int s;
    struct ifreq ifr;
    struct sockaddr_can addr;

    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strncpy(ifr.ifr_name, canName, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

    if (!ifr.ifr_ifindex) {
        perror("if_nametoindex");
        return 1;
    }

    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    return s;
}

int writeCAN(int canFd, canid_t id, u8 dlc, ...)
{
    va_list vl;
    va_start(vl, dlc);
    struct can_frame frame;

    frame.can_id  = id;
    frame.can_dlc = dlc;

    for (int i=0; i < dlc; i++)
    {
        frame.data[i] = (u8) va_arg(vl, int);
    }

    va_end(vl);

    return write(canFd, &frame, sizeof(struct can_frame));
}

int readCAN(int canFd, struct can_frame *frame)
{
    fd_set rfds;
    struct timeval tv;
    int retval;

    FD_ZERO(&rfds);
    FD_SET(canFd, &rfds);

    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    retval = select(canFd+1, &rfds, NULL, NULL, &tv);

    EXPECT_GT(retval, -1);

    if (retval)
    {
        retval = read(canFd, frame, sizeof(struct can_frame));
    }

    return retval;
}

void configureCAN(const char *canName, int speed)
{
    char buff[100];

    sprintf(buff, "sudo ip link set %s down", canName);
    EXPECT_EQ(0, system(buff));

    sprintf(buff, "sudo ip link set %s type can bitrate %d", canName, speed);
    EXPECT_EQ(0, system(buff));

    sprintf(buff, "sudo ip link set %s up && sleep 1", canName);
    EXPECT_EQ(0, system(buff));
}


int getTermination(const char *interface)
{
    FILE *f = 0;
    char buff[100];
    char term = -1;

    sprintf(buff, "/sys/class/net/%s/termination", interface);

    f = fopen(buff, "r");
    EXPECT_NE(0, *((int *)f));

    // get initial can1 termination value
    EXPECT_EQ(1, fread(&term, 1, 1, f));
    EXPECT_NE(-1, term);

    fclose(f);

    return term;
}

void setTermination(const char *interface, char val)
{
    FILE *f = 0;
    char buff[100];

    sprintf(buff, "/sys/class/net/%s/termination", interface);

    f = fopen(buff, "w");
    EXPECT_NE(0, *((int *)f));

    // get initial can1 termination value
    EXPECT_EQ(1, fwrite(&val, 1, 1, f));

    fclose(f);
}

//TEST(canIDConvertion, standardId)
//{
//    uint32_t canIdConverted = 0;
//    mcba_usb_msg_can usb_msg;
//    struct can_frame cf;

//    for(uint32_t i = 1; i < 2048; ++i)
//    {
//        cf.can_id = i;

//        usb_msg.sidl = MCBA_SET_S_SIDL(cf.can_id);
//        usb_msg.sidh = MCBA_SET_S_SIDH(cf.can_id);
//        usb_msg.eidl = 0;
//        usb_msg.eidh = 0;

//        canIdConverted = MCBA_CAN_GET_SID((&usb_msg));

//        EXPECT_EQ(cf.can_id, canIdConverted);
//    }
//}

//TEST(canIDConvertion, standardIdRTR)
//{
//    uint32_t canIdConverted = 0;
//    mcba_usb_msg_can usb_msg;
//    struct can_frame cf;

//    for(uint32_t i = 1; i < 2048; ++i)
//    {
//        cf.can_id = i | MCBA_CAN_RTR_MASK;

//        usb_msg.sidl = MCBA_SET_S_SIDL(cf.can_id);
//        usb_msg.sidh = MCBA_SET_S_SIDH(cf.can_id);
//        usb_msg.eidl = 0;
//        usb_msg.eidh = 0;
//        usb_msg.dlc = MCBA_DLC_RTR_MASK;

//        canIdConverted = MCBA_CAN_GET_SID((&usb_msg));

//        if(MCBA_RX_IS_RTR((&usb_msg)))
//            canIdConverted |= MCBA_CAN_RTR_MASK;

//        EXPECT_EQ(cf.can_id, canIdConverted);
//    }
//}

//TEST(canIDConvertion, extendedId)
//{
//    uint32_t canIdConverted = 0;
//    mcba_usb_msg_can usb_msg;
//    struct can_frame cf;

//    for(uint32_t i = 1; i < 0x20000000; ++i)
//    {
//        cf.can_id = i | MCBA_CAN_EXID_MASK;

//        usb_msg.sidl = MCBA_SET_E_SIDL(cf.can_id);
//        usb_msg.sidh = MCBA_SET_E_SIDH(cf.can_id);
//        usb_msg.eidl = MCBA_SET_EIDL(cf.can_id);
//        usb_msg.eidh = MCBA_SET_EIDH(cf.can_id);

//        canIdConverted = MCBA_CAN_GET_EID((&usb_msg));

//        if(MCBA_RX_IS_EXID((&usb_msg)))
//            canIdConverted |= MCBA_CAN_EXID_MASK;

//        EXPECT_EQ(cf.can_id, canIdConverted);
//    }
//}

//TEST(canIDConvertion, extendedIdRTR)
//{
//    uint32_t canIdConverted = 0;
//    mcba_usb_msg_can usb_msg;
//    struct can_frame cf;

//    for(uint32_t i = 1; i < 0x20000000; ++i)
//    {
//        cf.can_id = i | MCBA_CAN_RTR_MASK | MCBA_CAN_EXID_MASK;

//        usb_msg.sidl = MCBA_SET_E_SIDL(cf.can_id);
//        usb_msg.sidh = MCBA_SET_E_SIDH(cf.can_id);
//        usb_msg.eidl = MCBA_SET_EIDL(cf.can_id);
//        usb_msg.eidh = MCBA_SET_EIDH(cf.can_id);
//        usb_msg.dlc = MCBA_DLC_RTR_MASK;

//        canIdConverted = MCBA_CAN_GET_EID((&usb_msg));

//        if(MCBA_RX_IS_EXID((&usb_msg)))
//            canIdConverted |= MCBA_CAN_EXID_MASK;

//        if(MCBA_RX_IS_RTR((&usb_msg)))
//            canIdConverted |= MCBA_CAN_RTR_MASK;

//        EXPECT_EQ(cf.can_id, canIdConverted);
//    }
//}

//TEST(Configuration, SpeedSettings)
//{
//    const int bitrateSet[] = {20000, 33333, 50000, 80000, 83333, 100000,
//                             125000, 150000, 175000, 200000, 225000, 250000,
//                            275000, 300000, 500000, 625000, 800000, 1000000};

//    const int bitrateGet[] = {20000, 33333, 50000, 80000, 83333, 100000,
//                             125000, 150375, 175438, 200000, 227272, 250000,
//                            277777, 303030, 500000, 625000, 800000, 1000000};
//    char buff[100];


//    for(int i = 0; i < 18; ++i)
//    {
//        configureCAN("can0", bitrateSet[i]);

//        sprintf(buff, "sudo ip -d link show can0 | grep %d > /dev/null", bitrateGet[i]);
//        EXPECT_EQ(0, system(buff));

//        sleep(1);
//    }

//    EXPECT_EQ(0, system("sudo ip link set can0 down"));
//}

//TEST(Configuration, Termination)
//{
//    FILE *f = 0;
//    char initValue = -1;
//    char value = -1;
//    const char *termPath = "/sys/class/net/can0/termination";
//    char buff[100];

//    sprintf(buff, "ls %s > /dev/null", termPath);
//    EXPECT_EQ(0, system(buff));

//    initValue = getTermination("can0");

//    setTermination("can0", '1');
//    value = -1;
//    value = getTermination("can0");
//    EXPECT_EQ('1', value);

//    setTermination("can0", '0');
//    value = -1;
//    value = getTermination("can0");
//    EXPECT_EQ('0', value);

//    setTermination("can0", '1');
//    value = -1;
//    value = getTermination("can0");
//    EXPECT_EQ('1', value);

//    for(u8 i = 0; i < '0'; ++i)
//    {
//        setTermination("can0", i);
//        value = -1;
//        value = getTermination("can0");
//        EXPECT_EQ('1', value);
//    }

//    for(u8 i = '2'; i > 0; ++i)
//    {
//        setTermination("can0", i);
//        value = -1;
//        value = getTermination("can0");
//        EXPECT_EQ('1', value);
//    }

//    setTermination("can0", initValue);
//    value = -1;
//    value = getTermination("can0");
//    EXPECT_EQ(initValue, value);
//}


int canReadThread(const char* ifname)
{
    int canFd = 0;
    can_frame frame;
    int retVal;
    canid_t i = 0;

//    configureCAN(ifname, 1000000);
    canFd = openCANSocket(ifname);

    do
    {
	retVal = readCAN(canFd, &frame);

	if(retVal)
	{
	    EXPECT_EQ(i++, frame.can_id);
	}
    }
    while(retVal);

    close(canFd);

    return i;
}

int canWriteThread(const char* ifname, const int cnt)
{
    int canFd = 0;
    canid_t i = 0;
    int dataSent = 0;

//    configureCAN(ifname, 1000000);
    canFd = openCANSocket(ifname);

    for(i = 0; i <= cnt; ++i)
    {
	dataSent = writeCAN(canFd, i, 8, 1, 2, 3, 4, 5, 6, 7, 8);

	usleep(1000);

	EXPECT_EQ(dataSent, CAN_MTU);
    }

    close(canFd);

    return i;
}

int canReadThreadExt(const char* ifname)
{
    int canFd = 0;
    can_frame frame;
    canid_t i = 0x1fffffff;
    int retVal;

//    configureCAN(ifname, 1000000);
    canFd = openCANSocket(ifname);

    do
    {
	retVal = readCAN(canFd, &frame);

	if(retVal)
	{
	    EXPECT_EQ(i-- | CAN_EFF_FLAG, frame.can_id);
	}
    }
    while(retVal);

    close(canFd);

    return i;
}

int canWriteThreadExt(const char* ifname, canid_t cnt)
{
    int canFd = 0;
    canid_t i = 0;
    int dataSent = 0;

//    configureCAN(ifname, 1000000);
    canFd = openCANSocket(ifname);

    for(i = cnt; i >= 0; --i)
    {
	dataSent = writeCAN(canFd, i | CAN_EFF_FLAG, 8, 1, 2, 3, 4, 5, 6, 7, 8);

	usleep(1000);

	EXPECT_EQ(dataSent, CAN_MTU);
    }

    close(canFd);

    return i;
}

//TEST(StressTest, 1on1)
//{
//    int can0_term = -1;
//    int can1_term = -1;
//    const int testCnt = 0x7ff;

////    can0_term = getTermination("can0");
////    can1_term = getTermination("can1");

////    setTermination("can0", '1');
////    setTermination("can1", '0');

//    std::future<int> readRet = std::async(&canReadThread, "can0");
//    std::future<int> writeRet = std::async(&canWriteThread, "can1", testCnt);

//    EXPECT_EQ(testCnt+1, writeRet.get());
//    EXPECT_EQ(testCnt+1, readRet.get());

////    //bring back original termination
////    setTermination("can0", can0_term);
////    setTermination("can1", can1_term);
//}

TEST(StressTest, 1on1ext)
{
    const canid_t testCnt = 0x1fffffff;

//    can0_term = getTermination("can0");
//    can1_term = getTermination("can1");

//    setTermination("can0", '1');
//    setTermination("can1", '0');

    std::future<int> readRet = std::async(&canReadThreadExt, "can0");
    std::future<int> writeRet = std::async(&canWriteThreadExt, "slcan0", testCnt);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet.get());

//    //bring back original termination
//    setTermination("can0", can0_term);
//    setTermination("can1", can1_term);
}
