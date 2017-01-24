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

#define MCBA_CAN_EID8_EID15_SHIFT 8
#define MCBA_CAN_EID16_EID17_SHIFT 16
#define MCBA_CAN_E_SID0_SID2_SHIFT 18
#define MCBA_CAN_E_SID3_SID10_SHIFT 21

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

    sprintf(buff, "sudo ip link set %s up && usleep 300000", canName);
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

int canReadThread(const char* ifname, u32 flags)
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
	    EXPECT_EQ(i++ | flags, frame.can_id);
	}
    }
    while(retVal);

    close(canFd);

    return i;
}

int canReadThreadNoId(const char* ifname)
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

        EXPECT_GE(retVal, 0);

        i++;
    }
    while(retVal);

    close(canFd);

    return i;
}

int canWriteThread(const char* ifname, const int cnt, u32 flags)
{
    int canFd = 0;
    canid_t i = 0;
    int dataSent = 0;

//    configureCAN(ifname, 1000000);
    canFd = openCANSocket(ifname);

    for(i = 0; i <= cnt; ++i)
    {
	dataSent = writeCAN(canFd, i | flags, 8, 1, 2, 3, 4, 5, 6, 7, 8);

	usleep(1500);

	EXPECT_EQ(dataSent, CAN_MTU);
    }

    close(canFd);

    return i;
}

int canReadThreadExt(const char* ifname, canid_t start, canid_t shift, u32 flags)
{
    int canFd = 0;
    can_frame frame;
    canid_t i = start;
    canid_t shifted = 0;
    int retVal;

//    configureCAN(ifname, 1000000);
    canFd = openCANSocket(ifname);

    do
    {
        retVal = readCAN(canFd, &frame);

        if(retVal)
        {
            shifted = i++ << shift;

	    EXPECT_EQ(shifted | flags, frame.can_id);
        }

    } while(retVal);

    close(canFd);

    return i;
}

int canWriteThreadExt(const char* ifname, canid_t start, canid_t cnt, canid_t shift, u32 flags)
{
    int canFd = 0;
    canid_t i = 0;
    canid_t shifted = 0;
    int dataSent = 0;

//    configureCAN(ifname, 1000000);
    canFd = openCANSocket(ifname);

    for(i = start; i <= cnt; ++i)
    {
        shifted = i << shift;

	dataSent = writeCAN(canFd, shifted | flags, 8, 1, 2, 3, 4, 5, 6, 7, 8);

        usleep(500);

        EXPECT_EQ(dataSent, CAN_MTU);
    }

    close(canFd);

    return i;
}

/***********************************************
 *
 *  can0 - Microchip CAN BUS Analyzer
 *  can1 - Other SocketCAN
 *
 ***********************************************/

TEST(can_id_rcv, sid)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThread, "can0", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can1", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet.get());
}

TEST(can_id_rcv, eid_sid3_sid10)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_E_SID3_SID10_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv, eid_sid0_sid2)
{
    const canid_t start = 1;
    const canid_t cnt = 7;
    const canid_t shift = MCBA_CAN_E_SID0_SID2_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv, eid_eid16_eid17)
{
    const canid_t start = 1;
    const canid_t cnt = 3;
    const canid_t shift = MCBA_CAN_EID16_EID17_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv, eid_eid8_eid15)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_EID8_EID15_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv, eid_eid0_eid7)
{
    const canid_t start = 0;
    const canid_t cnt = 0xff;
    const canid_t shift = 0;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv_rtr, sid)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThread, "can0", CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThread, "can1", testCnt, CAN_RTR_FLAG);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet.get());
}

TEST(can_id_rcv_rtr, eid_sid3_sid10)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_E_SID3_SID10_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv_rtr, eid_sid0_sid2)
{
    const canid_t start = 1;
    const canid_t cnt = 7;
    const canid_t shift = MCBA_CAN_E_SID0_SID2_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv_rtr, eid_eid16_eid17)
{
    const canid_t start = 1;
    const canid_t cnt = 3;
    const canid_t shift = MCBA_CAN_EID16_EID17_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv_rtr, eid_eid8_eid15)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_EID8_EID15_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_rcv_rtr, eid_eid0_eid7)
{
    const canid_t start = 0;
    const canid_t cnt = 0xff;
    const canid_t shift = 0;

    std::future<int> readRet = std::async(&canReadThreadExt, "can0", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can1", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd, sid)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThread, "can1", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can0", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet.get());
}


TEST(can_id_snd, eid_sid3_sid10)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_E_SID3_SID10_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd, eid_sid0_sid2)
{
    const canid_t start = 1;
    const canid_t cnt = 7;
    const canid_t shift = MCBA_CAN_E_SID0_SID2_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd, eid_eid16_eid17)
{
    const canid_t start = 1;
    const canid_t cnt = 3;
    const canid_t shift = MCBA_CAN_EID16_EID17_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd, eid_eid8_eid15)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_EID8_EID15_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd, eid_eid0_eid7)
{
    const canid_t start = 0;
    const canid_t cnt = 0xff;
    const canid_t shift = 0;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd_rtr, sid)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThread, "can1", CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThread, "can0", testCnt, CAN_RTR_FLAG);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet.get());
}


TEST(can_id_snd_rtr, eid_sid3_sid10)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_E_SID3_SID10_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd_rtr, eid_sid0_sid2)
{
    const canid_t start = 1;
    const canid_t cnt = 7;
    const canid_t shift = MCBA_CAN_E_SID0_SID2_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd_rtr, eid_eid16_eid17)
{
    const canid_t start = 1;
    const canid_t cnt = 3;
    const canid_t shift = MCBA_CAN_EID16_EID17_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd_rtr, eid_eid8_eid15)
{
    const canid_t start = 1;
    const canid_t cnt = 0xff;
    const canid_t shift = MCBA_CAN_EID8_EID15_SHIFT;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(can_id_snd_rtr, eid_eid0_eid7)
{
    const canid_t start = 0;
    const canid_t cnt = 0xff;
    const canid_t shift = 0;

    std::future<int> readRet = std::async(&canReadThreadExt, "can1", start, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);
    std::future<int> writeRet = std::async(&canWriteThreadExt, "can0", start, cnt, shift, CAN_EFF_FLAG | CAN_RTR_FLAG);

    EXPECT_EQ(cnt+1, writeRet.get());
    EXPECT_EQ(cnt+1, readRet.get());
}

TEST(stress_snd, 1on1)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThread, "can1", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can0", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet.get());
}

TEST(stress_snd, 1on2)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet1 = std::async(&canReadThread, "can1", 0);
    std::future<int> readRet2 = std::async(&canReadThread, "can1", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can0", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet1.get());
    EXPECT_EQ(testCnt+1, readRet2.get());
}

TEST(stress_snd, 1on4)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet1 = std::async(&canReadThread, "can1", 0);
    std::future<int> readRet2 = std::async(&canReadThread, "can1", 0);
    std::future<int> readRet3 = std::async(&canReadThread, "can1", 0);
    std::future<int> readRet4 = std::async(&canReadThread, "can1", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can0", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet1.get());
    EXPECT_EQ(testCnt+1, readRet2.get());
    EXPECT_EQ(testCnt+1, readRet3.get());
    EXPECT_EQ(testCnt+1, readRet4.get());
}

TEST(stress_snd, 2on1)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThreadNoId, "can1");
    std::future<int> writeRet1 = std::async(&canWriteThread, "can0", testCnt, 0);
    std::future<int> writeRet2 = std::async(&canWriteThread, "can0", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet1.get());
    EXPECT_EQ(testCnt+1, writeRet2.get());
    EXPECT_EQ((testCnt+1)*2+1, readRet.get());
}

TEST(stress_snd, 4on1)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThreadNoId, "can1");
    std::future<int> writeRet1 = std::async(&canWriteThread, "can0", testCnt, 0);
    std::future<int> writeRet2 = std::async(&canWriteThread, "can0", testCnt, 0);
    std::future<int> writeRet3 = std::async(&canWriteThread, "can0", testCnt, 0);
    std::future<int> writeRet4 = std::async(&canWriteThread, "can0", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet1.get());
    EXPECT_EQ(testCnt+1, writeRet2.get());
    EXPECT_EQ(testCnt+1, writeRet3.get());
    EXPECT_EQ(testCnt+1, writeRet4.get());
    EXPECT_EQ((testCnt+1)*4+1, readRet.get());
}

TEST(stress_rcv, 1on1)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThread, "can0", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can1", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet.get());
}

TEST(stress_rcv, 1on2)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet1 = std::async(&canReadThread, "can0", 0);
    std::future<int> readRet2 = std::async(&canReadThread, "can0", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can1", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet1.get());
    EXPECT_EQ(testCnt+1, readRet2.get());
}

TEST(stress_rcv, 1on4)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet1 = std::async(&canReadThread, "can0", 0);
    std::future<int> readRet2 = std::async(&canReadThread, "can0", 0);
    std::future<int> readRet3 = std::async(&canReadThread, "can0", 0);
    std::future<int> readRet4 = std::async(&canReadThread, "can0", 0);
    std::future<int> writeRet = std::async(&canWriteThread, "can1", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet.get());
    EXPECT_EQ(testCnt+1, readRet1.get());
    EXPECT_EQ(testCnt+1, readRet2.get());
    EXPECT_EQ(testCnt+1, readRet3.get());
    EXPECT_EQ(testCnt+1, readRet4.get());
}

TEST(stress_rcv, 2on1)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThreadNoId, "can0");
    std::future<int> writeRet1 = std::async(&canWriteThread, "can1", testCnt, 0);
    std::future<int> writeRet2 = std::async(&canWriteThread, "can1", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet1.get());
    EXPECT_EQ(testCnt+1, writeRet2.get());
    EXPECT_EQ((testCnt+1)*2+1, readRet.get());
}

TEST(stress_rcv, 4on1)
{
    const int testCnt = 0x7ff;

    std::future<int> readRet = std::async(&canReadThreadNoId, "can0");
    std::future<int> writeRet1 = std::async(&canWriteThread, "can1", testCnt, 0);
    std::future<int> writeRet2 = std::async(&canWriteThread, "can1", testCnt, 0);
    std::future<int> writeRet3 = std::async(&canWriteThread, "can1", testCnt, 0);
    std::future<int> writeRet4 = std::async(&canWriteThread, "can1", testCnt, 0);

    EXPECT_EQ(testCnt+1, writeRet1.get());
    EXPECT_EQ(testCnt+1, writeRet2.get());
    EXPECT_EQ(testCnt+1, writeRet3.get());
    EXPECT_EQ(testCnt+1, writeRet4.get());
    EXPECT_EQ((testCnt+1)*4+1, readRet.get());
}

TEST(dlc_snd, dlc)
{
    int dataSent = 0;
    can_frame frame;
    int dataRcv;

    int canFdSnd = openCANSocket("can0");
    int canFdRcv = openCANSocket("can1");

    for(u8 i = 0; i <= 8; ++i)
    {
	dataSent = writeCAN(canFdSnd, 0xff, i, 1, 2, 3, 4, 5, 6, 7, 8);

	EXPECT_EQ(dataSent, CAN_MTU);

	dataRcv = readCAN(canFdRcv, &frame);

	EXPECT_GE(dataRcv, CAN_MTU);
	EXPECT_EQ(frame.can_dlc, i);
    }

    close(canFdSnd);
    close(canFdRcv);
}

TEST(dlc_rcv, dlc)
{
    int dataSent = 0;
    can_frame frame;
    int dataRcv;

    int canFdSnd = openCANSocket("can1");
    int canFdRcv = openCANSocket("can0");

    for(u8 i = 0; i <= 8; ++i)
    {
	dataSent = writeCAN(canFdSnd, 0xff, i, 1, 2, 3, 4, 5, 6, 7, 8);

	EXPECT_EQ(dataSent, CAN_MTU);

	dataRcv = readCAN(canFdRcv, &frame);

	EXPECT_GE(dataRcv, CAN_MTU);
	EXPECT_EQ(frame.can_dlc, i);
    }

    close(canFdSnd);
    close(canFdRcv);
}

TEST(data_snd, data)
{
    int dataSent = 0;
    can_frame frame;
    int dataRcv;
    u8 data[] = {0, 1, 2, 3, 4, 5, 6, 7};

    int canFdSnd = openCANSocket("can0");
    int canFdRcv = openCANSocket("can1");

    for(u32 i = 0; i <= 255; ++i)
    {
	dataSent = writeCAN(canFdSnd, 0xff, 8, data[0] + i, data[1] + i, data[2] + i, data[3] + i, data[4] + i, data[5] + i, data[6] + i, data[7] + i);

	EXPECT_EQ(dataSent, CAN_MTU);

	dataRcv = readCAN(canFdRcv, &frame);

	EXPECT_GE(dataRcv, CAN_MTU);
	EXPECT_EQ(frame.data[0], (u8)(data[0] + i));
	EXPECT_EQ(frame.data[1], (u8)(data[1] + i));
	EXPECT_EQ(frame.data[2], (u8)(data[2] + i));
	EXPECT_EQ(frame.data[3], (u8)(data[3] + i));
	EXPECT_EQ(frame.data[4], (u8)(data[4] + i));
	EXPECT_EQ(frame.data[5], (u8)(data[5] + i));
	EXPECT_EQ(frame.data[6], (u8)(data[6] + i));
	EXPECT_EQ(frame.data[7], (u8)(data[7] + i));
    }

    close(canFdSnd);
    close(canFdRcv);
}

TEST(data_rcv, data)
{
    int dataSent = 0;
    can_frame frame;
    int dataRcv;
    u8 data[] = {0, 1, 2, 3, 4, 5, 6, 7};

    int canFdSnd = openCANSocket("can1");
    int canFdRcv = openCANSocket("can0");

    for(u32 i = 0; i <= 255; ++i)
    {
	dataSent = writeCAN(canFdSnd, 0xff, 8, data[0] + i, data[1] + i, data[2] + i, data[3] + i, data[4] + i, data[5] + i, data[6] + i, data[7] + i);

	EXPECT_EQ(dataSent, CAN_MTU);

	dataRcv = readCAN(canFdRcv, &frame);

	EXPECT_GE(dataRcv, CAN_MTU);
	EXPECT_EQ(frame.data[0], (u8)(data[0] + i));
	EXPECT_EQ(frame.data[1], (u8)(data[1] + i));
	EXPECT_EQ(frame.data[2], (u8)(data[2] + i));
	EXPECT_EQ(frame.data[3], (u8)(data[3] + i));
	EXPECT_EQ(frame.data[4], (u8)(data[4] + i));
	EXPECT_EQ(frame.data[5], (u8)(data[5] + i));
	EXPECT_EQ(frame.data[6], (u8)(data[6] + i));
	EXPECT_EQ(frame.data[7], (u8)(data[7] + i));
    }

    close(canFdSnd);
    close(canFdRcv);
}

TEST(Configuration, SpeedSettings)
{
    const int bitrate[] = {20000, 33333, 50000, 80000, 83333, 100000,
			     125000, 150000, 175000, 200000, 225000, 250000,
			    275000, 300000, 500000, 625000, 800000, 1000000};

    char buff[100];


    for(int i = 0; i < 18; ++i)
    {
	configureCAN("can0", bitrate[i]);

	sprintf(buff, "sudo ip -d link show can0 | grep %d > /dev/null", bitrate[i]);
	EXPECT_EQ(0, system(buff));

	sleep(1);
    }

    EXPECT_EQ(0, system("sudo ip link set can0 down"));
}

TEST(Configuration, Termination)
{
    char buff[100];

    sprintf(buff, "sudo ~Proj/iproute2/ip/ip link set can0 type can termination 0");
    EXPECT_EQ(0, system(buff));
    sleep(1);

    sprintf(buff, "sudo ~Proj/iproute2/ip/ip -d link show can0 | grep 'termination 0' > /dev/null");
    EXPECT_EQ(0, system(buff));
    
    sprintf(buff, "sudo ~Proj/iproute2/ip/ip link set can0 type can termination 120");
    EXPECT_EQ(0, system(buff));
    sleep(1);

    sprintf(buff, "sudo ~Proj/iproute2/ip/ip -d link show can0 | grep 'termination 120' > /dev/null");
    EXPECT_EQ(0, system(buff));

    sprintf(buff, "sudo ~Proj/iproute2/ip/ip link set can0 type can termination 0");
    EXPECT_EQ(0, system(buff));
    sleep(1);

    sprintf(buff, "sudo ~Proj/iproute2/ip/ip -d link show can0 | grep 'termination 0' > /dev/null");
    EXPECT_EQ(0, system(buff));
    
    sprintf(buff, "sudo ip link set can0 up");
    EXPECT_EQ(0, system(buff));
}
