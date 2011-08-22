/*
 * Copyright (c) 2008, Google Inc.
 * All rights reserved.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the 
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <qsd8k/smd_private.h>
#include <qsd8k/smem.h>
#include <boot/boot.h>

struct smem_heap_info
{
	unsigned initialized;
	unsigned free_offset;
	unsigned heap_remaining;
	unsigned reserved;
};

struct smem_heap_entry
{
	unsigned allocated;
	unsigned offset;
	unsigned size;
	unsigned reserved;
};

struct smem_proc_comm
{
	unsigned command;
	unsigned status;
	unsigned data1;
	unsigned data2;
};

#define SMD_HEAP_SIZE 512

struct smem_shared
{
	struct smem_proc_comm proc_comm[4];
	unsigned version[32];
	struct smem_heap_info heap_info;
	struct smem_heap_entry heap_toc[SMD_HEAP_SIZE];
};	
	
#define SZ_DIAG_ERR_MSG 0xC8
#define ID_HEAP_INFO    3

static void smem_init_alloc_tbl(void);
void dump_smem_alloc_tbl(void);
void dump_smem_ch(int ch);

static volatile unsigned* smd_ver;
static struct smem_shared *shared = (void*) 0x00100000; //phys
//volatile static struct smem_shared *shared = (void*) 0xE0100000;
void smem_init(void)
{
    int i;
    unsigned ver_size;
    dprintf("--- smem info ---\n");
    dprintf("heap: init=%x free=%x remain=%x\n",
            shared->heap_info.initialized,
            shared->heap_info.free_offset,
            shared->heap_info.heap_remaining);
    for(i=0; i<4; i++)
        dprintf("proc_comm[%d]: command=%x status=%x data1=%x data2=%x\n",
                i, shared->proc_comm[i].command,
                shared->proc_comm[i].status,
                shared->proc_comm[i].data1,
                shared->proc_comm[i].data2);
    smd_ver =
           (unsigned *)smem_get_entry( SMEM_VERSION_SMD, &ver_size );
    dprintf("Versions size=%x\n", ver_size);
    smem_init_alloc_tbl();
}

void * smem_get_entry(unsigned int id, unsigned int *size)
{
    struct smem_heap_entry *he;
    if (!shared->heap_info.initialized)
            return 0;

    he = &(shared->heap_toc[id]);
    if(!he->allocated || he->size == 0)
            return 0;

    if (size)
            *size = he->size;
    return ((void *) shared) + he->offset;
}

void dump_smem_info_n(int item)
{
    struct smem_heap_entry *he;

    he = &shared->heap_toc[item];
    cprintf("%x: alloc=%x offset=%x size=%x\n",
                item, he->allocated, he->offset, he->size);
}

void dump_smem_info(void)
{
    unsigned n;
    struct smem_heap_entry *he;

    he = shared->heap_toc;
    for(n = 0; n < SMD_HEAP_SIZE; n++) {
        if(he->allocated) {
            cprintf("%x: alloc=%x offset=%x size=%x\n",
                    n, he->allocated, he->offset, he->size);
        }
        he++;
    }
}

#define MSM_CSR_BASE 0xAC100000
#define MSM_TRIG_A2M_INT(n) (writel(1, MSM_CSR_BASE + 0x400 + (n) * 4))
void smem_reboot(void)
{
    unsigned* states;
    int states_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    states[SMSM_APPS_STATE] |= SMSM_SYSTEM_REBOOT; //acpu rebooting
    MSM_TRIG_A2M_INT(5);
}

unsigned smem_get_version(int n)
{
    return smd_ver[n];
}

void dump_smem_version_info(void)
{
    cprintf("  QDSP6     :%d.%d\n",
        smd_ver[VERSION_QDSP6] >> 16, smd_ver[VERSION_QDSP6] & 0xffff);
    cprintf("  APPS_SBL  :%d.%d\n",
        smd_ver[VERSION_APPS_SBL] >> 16, smd_ver[VERSION_APPS_SBL] & 0xffff);
    cprintf("  MODEM_SBL :%d.%d\n",
        smd_ver[VERSION_MODEM_SBL] >> 16, smd_ver[VERSION_MODEM_SBL] & 0xffff);
    cprintf("  APPS      :%d.%d\n",
        smd_ver[VERSION_APPS] >> 16, smd_ver[VERSION_APPS] & 0xffff);
    cprintf("  MODEM     :%d.%d\n",
        smd_ver[VERSION_MODEM] >> 16, smd_ver[VERSION_MODEM] & 0xffff);
}

struct smd_alloc_elm {
        char name[20];
        unsigned cid;
        unsigned type;
        unsigned ref_count;
};
static struct smd_alloc_elm *tbl;
static int tblnum;

#define SMD_CHANNEL_TYPE(x) ((x) & 0x000000FF)
#define SMD_XFER_TYPE(x)    (((x) & 0x00000F00) >> 8)

static void smem_init_alloc_tbl(void)
{
    unsigned tbl_size;
    tbl =
           (void*)smem_get_entry( SMEM_CHANNEL_ALLOC_TBL, &tbl_size );
    tblnum = tbl_size / sizeof(struct smd_alloc_elm);
    dprintf("alloc_tbl buf=%x size=%x num=%d\n", tbl, tbl_size, tblnum);
    //dump_smem_alloc_tbl();
    //dump_smem_ch(2);
}

void dump_smem_alloc_tbl(void)
{
    int n;
    for (n = 0; n < tblnum; n++) {
        if (!tbl[n].ref_count)
                continue;
        cprintf("name=%s cid=%d ch type=%d xfer type=%d ref_count=%d\n", 
                tbl[n].name[0] ? tbl[n].name : "#UNKNOWN",
                tbl[n].cid,
                SMD_CHANNEL_TYPE(tbl[n].type),
                SMD_XFER_TYPE(tbl[n].type),
                tbl[n].ref_count);
    }
}

void dump_smem_state(void)
{
    unsigned* states;
    unsigned states_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    cprintf("%s\n", __FUNCTION__);
    cprintf("APPS_STATE=%x\n", states[SMSM_APPS_STATE]);
    cprintf("MODEMSTATE=%x\n", states[SMSM_MODEM_STATE]);
    cprintf("Q6_STATE=%x\n", states[SMSM_Q6_STATE]);
    cprintf("APPS_DEM=%x\n", states[SMSM_APPS_DEM]);
    cprintf("MODEM_DEM=%x\n", states[SMSM_MODEM_DEM]);
    cprintf("Q6_DEM=%x\n", states[SMSM_Q6_DEM]);
    cprintf("POWER_MASTER_DEM=%x\n", states[SMSM_POWER_MASTER_DEM]);
    cprintf("TIME_MASTER_DEM=%x\n", states[SMSM_TIME_MASTER_DEM]);
}

static const char *chstate(unsigned n)
{
    switch (n) {
    case SMD_SS_CLOSED:        return "CLOSED";
    case SMD_SS_OPENING:       return "OPENING";
    case SMD_SS_OPENED:        return "OPENED";
    case SMD_SS_FLUSHING:      return "FLUSHING";
    case SMD_SS_CLOSING:       return "CLOSING";
    case SMD_SS_RESET:         return "RESET";
    case SMD_SS_RESET_OPENING: return "ROPENING";
    default:                   return "UNKNOWN";
    }
}

static void dump_ch(int n,
                  struct smd_half_channel *s,
                  struct smd_half_channel *r)
{
        return cprintf(
                "ch%d:"
                " %s(%d/%d) %c%c%c%c%c%c%c <->"
                " %s(%d/%d) %c%c%c%c%c%c%c", n,
                chstate(s->state), s->tail, s->head,
                s->fDSR ? 'D' : 'd',
                s->fCTS ? 'C' : 'c',
                s->fCD ? 'C' : 'c',
                s->fRI ? 'I' : 'i',
                s->fHEAD ? 'W' : 'w',
                s->fTAIL ? 'R' : 'r',
                s->fSTATE ? 'S' : 's',
                chstate(r->state), r->tail, r->head,
                r->fDSR ? 'D' : 'd',
                r->fCTS ? 'R' : 'r',
                r->fCD ? 'C' : 'c',
                r->fRI ? 'I' : 'i',
                r->fHEAD ? 'W' : 'w',
                r->fTAIL ? 'R' : 'r',
                r->fSTATE ? 'S' : 's'
                );
}

void dump_smem_ch(int ch)
{
    void* info;
    char* shared_fifo;
    unsigned info_size = 0;
    unsigned fifo_size = 0;
    info =
           (void *)smem_get_entry( ID_SMD_CHANNELS + ch, &info_size);

    cprintf("channel%d ch_info=%x size=%x\n",ch, info, info_size);
    if(info){
        dump_ch(ch, (struct smd_half_channel*)info,
                ((struct smd_half_channel*)info)+1);
        cprintf(" buf=%x\n", (((struct smd_half_channel*)info)+2));
    }

    shared_fifo =
           (char *)smem_get_entry( SMEM_SMD_FIFO_BASE_ID + ch, &fifo_size);
    if(shared_fifo){
        cprintf("ch=%d send_buf=%x recv_buf=%x size=%x\n",
                ch, shared_fifo, shared_fifo+fifo_size/2, fifo_size);
    }else
        cprintf("channel%d not allocated\n", ch);
}

static inline void notify_other_smd(unsigned ch_type)
{
    MSM_TRIG_A2M_INT(0);
}

static int ch_is_open(struct smd_channel *ch)
{
        return (ch->recv->state == SMD_SS_OPENED ||
                ch->recv->state == SMD_SS_FLUSHING)
                && (ch->send->state == SMD_SS_OPENED);
}

static unsigned ch_read_buffer(struct smd_channel *ch, void **ptr)
{
        unsigned head = ch->recv->head;
        unsigned tail = ch->recv->tail;
        *ptr = (void *) (ch->recv_buf + tail);

        if (tail <= head)
                return head - tail;
        else
                return ch->buf_size - tail;
}

static void ch_read_done(struct smd_channel *ch, unsigned count)
{
        ch->recv->tail = (ch->recv->tail + count) & (ch->buf_size - 1);
        ch->send->fTAIL = 1;
}

static int ch_read(struct smd_channel *ch, void *_data, int len)
{
        void *ptr;
        unsigned n;
        unsigned char *data = _data;
        int orig_len = len;

        while (len > 0) {
                n = ch_read_buffer(ch, &ptr);
                if (n == 0)
                        break;

                if (n > len)
                        n = len;
                if (_data)
                        memcpy(data, ptr, n);

                data += n;
                len -= n;
                ch_read_done(ch, n);
        }

        return orig_len - len;
}

int smd_stream_read(smd_channel_t *ch, void *data, int len)
{
        int r;

        if (len < 0)
                return -1;

        r = ch_read(ch, data, len);
        if (r > 0)
                notify_other_smd(ch->type);

        return r;
}

int smd_stream_read_avail(struct smd_channel *ch)
{
        return (ch->recv->head - ch->recv->tail) & (ch->buf_size - 1);
}

static unsigned ch_write_buffer(struct smd_channel *ch, void **ptr)
{
        unsigned head = ch->send->head;
        unsigned tail = ch->send->tail;
        *ptr = (void *) (ch->send_buf + head);

        if (head < tail) {
                return tail - head - 1;
        } else {
                if (tail == 0)
                        return ch->buf_size - head - 1;
                else
                        return ch->buf_size - head;
        }
}

static void ch_write_done(struct smd_channel *ch, unsigned count)
{
        ch->send->head = (ch->send->head + count) & (ch->buf_size - 1);
        ch->send->fHEAD = 1;
}

int smd_stream_write(smd_channel_t *ch, const void *_data, int len)
{
        void *ptr;
        const unsigned char *buf = _data;
        unsigned xfer;
        int orig_len = len;

        if (len < 0)
                return -1;
        else if (len == 0)
                return 0;

        while ((xfer = ch_write_buffer(ch, &ptr)) != 0) {
                if (!ch_is_open(ch))
                        break;
                if (xfer > len)
                        xfer = len;
                memcpy(ptr, buf, xfer);
                ch_write_done(ch, xfer);
                len -= xfer;
                buf += xfer;
                if (len == 0)
                        break;
        }

        if (orig_len - len)
                notify_other_smd(ch->type);

        return orig_len - len;
}

int smd_stream_write_avail(struct smd_channel *ch)
{
        return (ch->buf_size - 1) -
                ((ch->send->head - ch->send->tail) & (ch->buf_size - 1));
}

struct smd_channel rpc_ch;
static unsigned last_pm = 0;

void rpc_analyze(void)
{
    int i;
    int send_i = 0;
    unsigned* rpc_sendbuf = (void*)rpc_ch.send_buf;
    struct rr_header* hdr;
    unsigned* data;
    for(i=0; i<0x1000; i++){
        hdr = (void*)&rpc_sendbuf[i];
        if(hdr->version == RPCROUTER_VERSION && hdr->size == 0x14){
            send_i = i;
            break;
        }
        if(hdr->version == RPCROUTER_VERSION && hdr->size == 0x2c){
            send_i = i;
            break;
        }
    }
    hdr = (void*)&rpc_sendbuf[send_i];
    while(hdr->version == RPCROUTER_VERSION){
        if(hdr->type == 1){ // CMD_DATA
            data = (void*)(hdr+1);
            if(last_pm < data[0]) last_pm = data[0];
        }
        hdr = (void*)(((unsigned*)hdr) + ((hdr->size/4) + 8));
    }
}

void rpc_init(void)
{
    struct rpc_channel {
        struct smd_half_channel s;
        struct smd_half_channel r;
    }* rpc_smemch;
    unsigned info_size;
    char* shared_fifo;
    int ch = 2;
    rpc_smemch =
           (void *)smem_get_entry( ID_SMD_CHANNELS + ch, &info_size);
    rpc_ch.send = &rpc_smemch->s;
    rpc_ch.recv = &rpc_smemch->r;

    shared_fifo =
           (char *)smem_get_entry( SMEM_SMD_FIFO_BASE_ID + ch, &info_size);
    rpc_ch.send_buf = shared_fifo;
    rpc_ch.recv_buf = shared_fifo + info_size/2;
    rpc_ch.buf_size = info_size/2;
    rpc_ch.type = 0;

    rpc_analyze();
}

struct rr_header req;
static struct rr_header reply;
static int rpc_status;
static char rpc_data[0x100];

__inline unsigned cpu_to_be32(unsigned i)
{
    unsigned ret;
    ret = (i >> 24) & 0xff;
    ret |= (i >> 8) & 0xff00;
    ret |= (i << 8) & 0xff0000;
    ret |= (i << 24) & 0xff000000;
    return ret;
}

void dump_rr_header(const struct rr_header* hdr)
{
    cprintf("rpc version=%d type=%x src=%x:%x confirm_rx=%x"
            " size=%x dst=%x:%x\n", hdr->version, hdr->type,
            hdr->src_pid, hdr->src_cid, hdr->confirm_rx, hdr->size,
            hdr->dst_pid, hdr->dst_cid);
}

int rpc_reply_event(void)
{
    int need;
    unsigned* rpc_replydata = (unsigned*)rpc_data;
    unsigned xid;

    if(req.confirm_rx){
        unsigned data[5];
        reply.type = RPCROUTER_CTRL_CMD_RESUME_TX;
        reply.version = RPCROUTER_VERSION;
        reply.src_pid = RPCROUTER_PID_LOCAL;
        reply.src_cid = RPCROUTER_ROUTER_ADDRESS;
        reply.dst_pid = RPCROUTER_PID_REMOTE;
        reply.dst_cid = RPCROUTER_ROUTER_ADDRESS;
        reply.confirm_rx = 0;
        reply.size = 0x14;
        data[0] = RPCROUTER_CTRL_CMD_RESUME_TX;
        data[1] = RPCROUTER_PID_LOCAL;
        data[2] = req.dst_cid;
        data[3] = RPCROUTER_PID_REMOTE;
        data[4] = 1;
        //dump_rr_header(&reply);
        //memdump(data, 5);
        smd_stream_write(&rpc_ch, &reply, sizeof(reply));
        smd_stream_write(&rpc_ch, data, sizeof(data));
    }
    if(req.type != RPCROUTER_CTRL_CMD_DATA) return 0;
    if(req.src_pid != 0) return 0;
    if(req.src_cid != 1) return 0;
    xid = rpc_replydata[1];
    if(rpc_replydata[2] != 0) return 0; // request
    if(rpc_replydata[3] != 0x02000000) return 0; // rpc_version
    if(rpc_replydata[4] != 0x15000030) return 0; // DOG_KEEPALIVE
    if(rpc_replydata[6] != 0x02000000) return 0; // BEACON
    need = sizeof(reply) + 0x1c;
    reply.type = RPCROUTER_CTRL_CMD_DATA;
    reply.version = RPCROUTER_VERSION;
    reply.src_cid = req.dst_cid;
    reply.src_pid = req.dst_pid;
    reply.dst_cid = req.src_cid;
    reply.dst_pid = req.src_pid;
    reply.confirm_rx = 0;
    reply.size = 0x1c;
    last_pm = last_pm + 0x10000;
    last_pm &= ~0xffff;
    last_pm |= 0x18;
    rpc_replydata[0] = last_pm;
    rpc_replydata[1] = xid;
    rpc_replydata[2] = 0x01000000; // reply
    rpc_replydata[3] = 0; // accepted
    rpc_replydata[4] = 0;
    rpc_replydata[5] = 0;
    rpc_replydata[6] = 0;
    //dump_rr_header(&reply);
    //memdump(rpc_replydata, reply.size/4);
    if(smd_stream_write_avail(&rpc_ch) >= need){
        smd_stream_write(&rpc_ch, &reply, sizeof(reply));
        smd_stream_write(&rpc_ch, rpc_replydata, reply.size);
    }
    return 0;
}

int rpc_poll_event(void)
{
    for(;;){
        if(rpc_status==0 && smd_stream_read_avail(&rpc_ch) >= sizeof(req)){
            smd_stream_read(&rpc_ch, (void*)&req, sizeof(req));
            //dump_rr_header(&req);
            rpc_status = 1;
        }else
        if(rpc_status==1 && smd_stream_read_avail(&rpc_ch) >= req.size){
            smd_stream_read(&rpc_ch, (void*)rpc_data, req.size);
            //memdump(rpc_data, req.size>>2);
            rpc_reply_event();
            rpc_status = 0;
        }else
            break;
    }
    return 0;
}

void smd_irq_handler(unsigned n)
{
    rpc_poll_event();
}

void smsm_irq_handler(unsigned n)
{
    unsigned* states;
    unsigned apps_state;
    unsigned mdm_state;
    int states_size;
    states =
           (void *)smem_get_entry(ID_SHARED_STATE, &states_size);
    if(states == 0){
        cprintf("<SM NO STATE>\n");
    }
    apps_state = states[SMSM_APPS_STATE];
    mdm_state = states[SMSM_MODEM_STATE];
    cprintf("<SM %x %x>\n", apps_state, mdm_state);
    if(apps_state & SMSM_RESET){
        cprintf("smsm reset acked\n");
        apps_state &= ~SMSM_RESET;
    }else if(mdm_state & SMSM_RESET){
        cprintf("!! modem crashed !!\n");
        //apps_state |= SMSM_RESET;
        states[SMSM_APPS_STATE] |= SMSM_RESET | SMSM_MODEM_WAIT;
        MSM_TRIG_A2M_INT(5);
        return;
    }else{
        cprintf("init\n");
        apps_state |= SMSM_INIT;
        if(mdm_state & SMSM_SMDINIT)
            apps_state |= SMSM_SMDINIT;
        if(mdm_state & SMSM_RPCINIT)
            apps_state |= SMSM_RPCINIT;
        if((apps_state & (SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT)) ==
            (SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT))
            apps_state |= SMSM_RUN;
    }
    if(apps_state != states[SMSM_APPS_STATE]){
        states[SMSM_APPS_STATE] = apps_state;
        MSM_TRIG_A2M_INT(5);
    }
}

