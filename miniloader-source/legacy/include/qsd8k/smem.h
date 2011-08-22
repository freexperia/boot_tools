#ifndef MSM7K_SMEM_H
#define MSM7K_SMEM_H

struct smd_half_channel {
        unsigned state;
        unsigned char fDSR;
        unsigned char fCTS;
        unsigned char fCD;
        unsigned char fRI;
        unsigned char fHEAD;
        unsigned char fTAIL;
        unsigned char fSTATE;
        unsigned char fUNUSED;
        unsigned tail;
        unsigned head;
};

#define SMD_SS_CLOSED            0x00000000
#define SMD_SS_OPENING           0x00000001
#define SMD_SS_OPENED            0x00000002
#define SMD_SS_FLUSHING          0x00000003
#define SMD_SS_CLOSING           0x00000004
#define SMD_SS_RESET             0x00000005
#define SMD_SS_RESET_OPENING     0x00000006

typedef struct smd_channel {
        volatile struct smd_half_channel *send;
        volatile struct smd_half_channel *recv;
        unsigned char *send_buf;
        unsigned char *recv_buf;
        unsigned buf_size;
        unsigned type;
}smd_channel_t;

void * smem_get_entry(unsigned int id, unsigned int *size);
void smd_irq_handler(unsigned n);
void smsm_irq_handler(unsigned n);

#define RPCROUTER_VERSION                       1

#define RPCROUTER_PID_LOCAL			1
#define RPCROUTER_PID_REMOTE			0

#define RPCROUTER_CLIENT_BCAST_ID		0xffffffff
#define RPCROUTER_ROUTER_ADDRESS		0xfffffffe

#define RPCROUTER_CTRL_CMD_DATA                 1
#define RPCROUTER_CTRL_CMD_HELLO                2
#define RPCROUTER_CTRL_CMD_BYE                  3
#define RPCROUTER_CTRL_CMD_NEW_SERVER           4
#define RPCROUTER_CTRL_CMD_REMOVE_SERVER        5
#define RPCROUTER_CTRL_CMD_REMOVE_CLIENT        6
#define RPCROUTER_CTRL_CMD_RESUME_TX            7
#define RPCROUTER_CTRL_CMD_EXIT                 8
#define RPCROUTER_CTRL_CMD_PING                 9

struct rr_header {
        unsigned version;
        unsigned type;
        unsigned src_pid;
        unsigned src_cid;
        unsigned confirm_rx;
        unsigned size;
        unsigned dst_pid;
        unsigned dst_cid;
};

union rr_control_msg {
        unsigned cmd;
        struct {
                unsigned cmd;
                unsigned prog;
                unsigned vers;
                unsigned pid;
                unsigned cid;
        } srv;
        struct {
                unsigned cmd;
                unsigned pid;
                unsigned cid;
        } cli;
};

void dump_rr_header(const struct rr_header*hdr);
#endif
